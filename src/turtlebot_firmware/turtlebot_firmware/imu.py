#!/usr/bin/env python3
import rclpy
import smbus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

_DEV_ADDR = 0x68

_MAP = {
    'pwr': 0x6B,
    'div': 0x19,
    'cfg': 0x1A,
    'g_cfg': 0x1B,
    'int_en': 0x38,
    'ax': 0x3B,
    'ay': 0x3D,
    'az': 0x3F,
    'gx': 0x43,
    'gy': 0x45,
    'gz': 0x47
}

class ImuSensorPublisher(Node):
    def __init__(self):
        super().__init__('imu_sensor_publisher')
        
        self._hw_ready = False
        self._i2c_bus = None
        self._setup_hardware()

        self._data_emitter = self.create_publisher(Imu, '/imu/out', qos_profile=qos_profile_sensor_data)
        self._current_frame = Imu()
        self._current_frame.header.frame_id = 'base_footprint'
        
        self.create_timer(0.01, self._process_sensor_tick)

    def _setup_hardware(self):
        try:
            self._i2c_bus = smbus.SMBus(1)
            self._i2c_bus.write_byte_data(_DEV_ADDR, _MAP['div'], 7)
            self._i2c_bus.write_byte_data(_DEV_ADDR, _MAP['pwr'], 1)
            self._i2c_bus.write_byte_data(_DEV_ADDR, _MAP['cfg'], 0)
            self._i2c_bus.write_byte_data(_DEV_ADDR, _MAP['g_cfg'], 24)
            self._i2c_bus.write_byte_data(_DEV_ADDR, _MAP['int_en'], 1)
            self._hw_ready = True
        except OSError:
            self._hw_ready = False
        
    def _fetch_axis_value(self, start_reg):
        msb = self._i2c_bus.read_byte_data(_DEV_ADDR, start_reg)
        lsb = self._i2c_bus.read_byte_data(_DEV_ADDR, start_reg + 1)
        raw_val = (msb << 8) | lsb
        return raw_val if raw_val < 32768 else raw_val - 65536

    def _process_sensor_tick(self):
        try:
            if not self._hw_ready:
                self._setup_hardware()
                if not self._hw_ready:
                    return
            
            a_x = self._fetch_axis_value(_MAP['ax'])
            a_y = self._fetch_axis_value(_MAP['ay'])
            a_z = self._fetch_axis_value(_MAP['az'])
            
            g_x = self._fetch_axis_value(_MAP['gx'])
            g_y = self._fetch_axis_value(_MAP['gy'])
            g_z = self._fetch_axis_value(_MAP['gz'])
            
            self._current_frame.linear_acceleration.x = a_x / 1670.13
            self._current_frame.linear_acceleration.y = a_y / 1670.13
            self._current_frame.linear_acceleration.z = a_z / 1670.13
            
            self._current_frame.angular_velocity.x = g_x / 939.65
            self._current_frame.angular_velocity.y = g_y / 939.65
            self._current_frame.angular_velocity.z = g_z / 939.65

            self._current_frame.header.stamp = self.get_clock().now().to_msg()
            self._data_emitter.publish(self._current_frame)
            
        except OSError:
            self._hw_ready = False

def start_execution():
    rclpy.init()
    sensor_node = ImuSensorPublisher()
    try:
        rclpy.spin(sensor_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    start_execution()