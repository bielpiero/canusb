#!/usr/bin/env python3

import serial
import threading
import math
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist

from canusb.msg import KinematicModelIncrement
from typing import Optional
import struct


class CanUsb:
  def __init__(self,
               port: str = "/dev/ttyACM0",
               baudrate: int = 921600,
               timeout: float = 0.1):
    self.port = port
    self.baudrate = baudrate
    self.timeout = timeout
    self.ser: Optional[serial.Serial] = None

    self.read_thread = None
    self.running = None

    self.current_enc_a = None
    self.current_enc_b = None

    self.last_enc_a = None
    self.last_enc_b = None

    self.x = 0.0
    self.y = 0.0
    self.th = 0.0

    self.lock = threading.Lock()

    #publishers
    self.pose_pub = rospy.Publisher('/pose', Pose2D, queue_size=10)
    self.inc_pub = rospy.Publisher('/kinematic_model_increment', KinematicModelIncrement, queue_size=10)
    
    pose = Pose2D()
    pose.x = self.x
    pose.y = self.y
    pose.theta = self.th
    self.pose_pub.publish(pose)
    
    inc = KinematicModelIncrement()
    inc.d_ang = 0.0
    inc.d_lin = 0.0
    self.inc_pub.publish(inc)

    # incrementos acumulados pendientes de integrar
    self.pending_d_lin = 0.0
    self.pending_d_yaw = 0.0
    
    self.new_packet_a = False
    self.new_packet_b = False
    
    #subscribers
    self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_callback)
    self.update_pose_sub = rospy.Subscriber('/update_pose', Pose2D, self._update_pose_callback)

    #configuración de la silla
    self.wheel_radius = rospy.get_param("~wheel_radius", 0.165)
    self.wheel_base = rospy.get_param("~wheel_base", 0.525)
    self.counts_per_rev = rospy.get_param("~counts_per_rev", 64000)
    self.ticks_per_m = self._compute_ticks_per_m()
    self.k_param = 7.8125
    
    self.loop_mode = 65 # 65 -> Open Loop, 67 Closed Loop
    self.pc_mode = 0
    print(f"ticks_per_m; {self.ticks_per_m}")
  
  def _cmd_vel_callback(self, msg: Twist):
    #TODO: to send speed commands to motors
    v = msg.linear.x
    w = msg.angular.z
    
    vr = v + self.wheel_base / 2 * w
    vl = v - self.wheel_base / 2 * w
    
    wr = vr / self.wheel_radius
    wl = vl / self.wheel_radius   
    
    print(f"wr: {wr}, wl: {wl}")
    
    data_r = int(wr * self.k_param)
    data_l = int(wl * self.k_param)
    
    data_r = max(min(data_r, 127), -127)
    data_l = max(min(data_l, 127), -127)
    
    if data_r < 0:
      data_r = 256 + data_r
      
    if data_l < 0:
      data_l = 256 + data_l
    
    print(f"data_l: {data_l}, data_r: {data_r}")
    
    flag = 0x3E7C
    id = 288
    frame = struct.pack('HHBBBBBBBB', int(flag), id, data_r, data_l, 0, 0, 65, 0, 0, 0)
    self.write(frame)
    
  def _compute_ticks_per_m(self):
    perimeter = 2.0 * math.pi * self.wheel_radius
    return perimeter / float(self.counts_per_rev)

  def open(self):
    if self.ser and self.ser.is_open:
      try:
        self.ser.close()
      except Exception:
        pass
      self.ser = None
      
    self.ser = serial.Serial(
      port=self.port,
      baudrate=self.baudrate,
      timeout=self.timeout
    )
    print("USB-CAN port Openned")

  def close(self):
    if self.ser and self.ser.is_open:
      self.ser.close()
      self.ser = None

  def write(self, data: bytes):
    if not self.ser or not self.ser.is_open:
      raise RuntimeError("Puerto serie no está abierto")
    self.ser.write(bytes(data))

  def read(self, nbytes: int = 1) -> bytes:
    if not self.ser or not self.ser.is_open:
      raise RuntimeError("Puerto serie no está abierto")
    return self.ser.read(nbytes)

  def read_frame(self):
    while True:
      try:
        msg = self.ser.read(12)
      except Exception as e:
        rospy.logwarn(f"message: {e}")
      
      if len(msg) != 12:
        continue
      
      if (msg[1] << 8)+msg[0] != 15996:  # 0x3e7c
        break
      
      return msg

  def decode(self, frame: bytes) -> Optional[dict]:

    can_id = (frame[3] << 8) + frame[2]

    data_a = struct.pack('B', frame[4]) + struct.pack('B', frame[5]) + struct.pack('B', frame[6]) + struct.pack('B', frame[7])
    data_b = struct.pack('B', frame[8]) + struct.pack('B', frame[9]) + struct.pack('B', frame[10]) + struct.pack('B', frame[11])

    #print(f"frame -> msg_id: {can_id}, data_a: {data_a}, data_b: {data_b}")
    
    results = {}
    results["msg_id"] = can_id
    results["data_a"] = data_a
    results["data_b"] = data_b
      
    return results

  def start_reading(self):
    if self.read_thread and self.read_thread.is_alive():
      return
    
    self.running = True
    self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
    self.read_thread.start()

    self.pose_thread = threading.Thread(target=self._pose_integration_loop, daemon=True)
    self.pose_thread.start()

  def stop_reading(self):
    self.running = False
    if self.read_thread:
      self.read_thread.join(timeout=1.0)
      self.read_thread = None

  def _read_loop(self):
    while self.running and not rospy.is_shutdown():
      try:
        frame = self.read_frame()
        if frame:
          decoded = self.decode(frame)
          if decoded:
            self._handle_message(decoded)
      except serial.SerialException:
        break
      except Exception as e:
        rospy.logwarn(f"error in read loop {e}")

  def update_odometry(self, enc_a = None, enc_b=None):
    publish_inc = False
    d_lin = 0.0
    d_yaw = 0.0
    with self.lock:
      if enc_a is not None:
        self.current_enc_a = enc_a
        self.new_packet_a = True
      if enc_b is not None:
        self.current_enc_b = enc_b
        self.new_packet_b = True
        
      if self.new_packet_a == False or self.new_packet_b == False:
        return
        
      if self.last_enc_a is None:
        self.last_enc_a = self.current_enc_a
        self.last_enc_b = self.current_enc_b
        self.new_packet_a = False
        self.new_packet_b = False
        return
      
      #print(f"current_enc_a: {self.current_enc_a}, last_enc_a: {self.last_enc_a}, current_enc_b: {self.current_enc_b}, last_enc_b: {self.last_enc_b}")
        
      delta_a = self.current_enc_a - self.last_enc_a
      delta_b = self.current_enc_b - self.last_enc_b
      
      #print(f"delta_a: {delta_a}, delta_b: {delta_b}")
      
      self.last_enc_a = self.current_enc_a
      self.last_enc_b = self.current_enc_b
      self.new_packet_a = False
      self.new_packet_b = False

      d_right = delta_a * self.ticks_per_m
      d_left = delta_b * self.ticks_per_m
      
      #print(f"d_left: {d_left}, d_right: {d_right}")

      d_lin = (d_left + d_right) * 0.5
      d_yaw = (d_right - d_left) / self.wheel_base

      self.pending_d_lin += d_lin
      self.pending_d_yaw += d_yaw
      publish_inc = True
    
    if publish_inc:
      kmi = KinematicModelIncrement()
      kmi.d_ang = d_yaw
      kmi.d_lin = d_lin
      self.inc_pub.publish(kmi)

  def _update_pose_callback(self, msg: Pose2D):
    with self.lock:
      rospy.loginfo("---------> Pose Received: ", msg)
      self.x = msg.x
      self.y = msg.y
      self.th = msg.theta
      self.th = (self.th + math.pi) % (2.0 * math.pi) - math.pi

      self.pending_d_lin = 0.0
      self.pending_d_yaw = 0.0

      if self.current_enc_a is not None:
          self.last_enc_a = self.current_enc_a
      if self.current_enc_b is not None:
          self.last_enc_b = self.current_enc_b

  def _pose_integration_loop(self):
    rate = rospy.Rate(50)  # 50 Hz, ajustable

    while self.running and not rospy.is_shutdown():
      with self.lock:
        d_lin = self.pending_d_lin
        d_yaw = self.pending_d_yaw

        self.pending_d_lin = 0.0
        self.pending_d_yaw = 0.0

        if abs(d_lin) > 0.0 or abs(d_yaw) > 0.0:
            self.x += d_lin * math.cos(self.th + 0.5 * d_yaw)
            self.y += d_lin * math.sin(self.th + 0.5 * d_yaw)
            self.th += d_yaw
            self.th = (self.th + math.pi) % (2.0 * math.pi) - math.pi

        pose = Pose2D()
        pose.x = self.x
        pose.y = self.y
        pose.theta = self.th

      self.pose_pub.publish(pose)
      rate.sleep()
  
  def _handle_message(self, msg: dict):
    msg_id = msg["msg_id"]
    data_a = msg["data_a"]
    data_b = msg["data_b"]

    if msg_id == 0x101: #encoder motor A
      enc_a = struct.unpack('I', data_a)[0]
      if enc_a >= 2147483648:
        enc_a = enc_a-4294967296
      
      self.update_odometry(enc_a=enc_a)

    elif msg_id == 0x102: #encoder motor B
      enc_b = struct.unpack('I', data_a)[0]
      if enc_b >= 2147483648:
        enc_b = enc_b-4294967296
      
      self.update_odometry(enc_b=enc_b)
      
    elif msg_id == 273: #pc mode
      (self.pc_mode,) = struct.unpack('B', data_a[0:1])
      flag = 0x3E7C
      id = 288
      frame = struct.pack('HHBBBBBBBB', int(flag), id, 0, 0, 0, 0, self.loop_mode, 0, 0, 0)
      self.write(frame)
      
      
def main():
  rospy.init_node("can_usb_comm", anonymous=False)
  
  port = rospy.get_param("~port", "/dev/ttyACM0")
  baud = rospy.get_param("~baud", 921600)
  timeout = rospy.get_param("~timeout", 0.1)

  can = CanUsb(port, baud, timeout)

  can.open()
  can.start_reading()

  rospy.loginfo("CAN-USB communication started")
  rospy.spin()

  can.stop_reading()
  can.close()

if __name__ == "__main__":
  main()
