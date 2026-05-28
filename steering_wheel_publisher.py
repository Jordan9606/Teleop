#!/usr/bin/env python3
import sys
import os
sys.path.append('/opt/ros/foxy/local/lib/python3.8/site-packages')
sys.path.append('/opt/ros/foxy/local/lib/python3.8/dist-packages')
sys.path.append('/opt/ros/foxy/lib/python3.8/site-packages')
sys.path.append('/home/host/.local/lib/python3.8/site-packages')

autoware_path = '/ros2_ws/autoware/install/autoware_auto_control_msgs/lib/python3.8/site-packages'
if autoware_path not in sys.path:
    sys.path.append(autoware_path)

# 2. Add the standard ROS paths (just in case)
sys.path.append('/opt/ros/foxy/lib/python3.8/site-packages')

# 3. Add the library path for the C++ typesupport (Fixes the "type_support is null" error)
os.environ['LD_LIBRARY_PATH'] = os.environ.get('LD_LIBRARY_PATH', '') + ':/ros2_ws/autoware/install/autoware_auto_control_msgs/lib'

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data 
from sensor_msgs.msg import Joy
from evdev import InputDevice, categorize, ecodes
import threading
from autoware_auto_control_msgs.msg import AckermannControlCommand

# Replace 'event15' with the number you found earlier
dev_path = '/dev/input/event5'
MAX_STEER_RAD = 0.4189  # Standard max angle

class wheel_publisher(Node):
    def __init__(self):
        super().__init__('steering_wheel_pub')

        # publish on node
        self.joy_pub = self.create_publisher(Joy, 'steering_wheel_joy',10)
        self.ackermann_pub = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd', 10)

        # Internal state
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0, 0.0, 0.0] # Steering, Throttle, Brake
        self.joy_msg.buttons = [0] * 5 # Init. 5 buttons

        # Wheel encoder thread
        self.thread = threading.Thread(target=self.steering_wheel_encoder, daemon=True)
        self.thread.start()
        self.get_logger().info('Steering wheel node started')

    def map_value(self, value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def publish_msgs(self):
        # 1. Publish Raw Joy for debugging
        self.joy_pub.publish(self.joy_msg)

        # 2. Construct and Publish Autoware Ackermann Message
        ack_msg = AckermannControlCommand()
        
        # Timestamp is critical for Autoware controllers
        ack_msg.stamp = self.get_clock().now().to_msg()
        
        # Lateral: Map -1.0/1.0 to -0.4189/0.4189 radians
        # Note: Check if your wheel needs a sign flip (left should be positive in ROS)
        ack_msg.lateral.steering_tire_angle = self.joy_msg.axes[0] * MAX_STEER_RAD
        
        # Longitudinal: Basic mapping of throttle to speed (Example: 0.0 to 5.0 m/s)
        # In a real setup, throttle usually maps to acceleration, but speed is easier for testing
        target_speed = self.joy_msg.axes[1] * 2.0  # Max 2m/s
        ack_msg.longitudinal.speed = target_speed
        
        # Brake: If braking, set acceleration to negative or speed to 0
        if self.joy_msg.axes[2] > 0.1:
            ack_msg.longitudinal.speed = 0.0
            ack_msg.longitudinal.acceleration = -3.0 # Deceleration in m/s^2
        else:
            ack_msg.longitudinal.acceleration = 1.0 # Constant small acceleration

        self.ackermann_pub.publish(ack_msg)

    def steering_wheel_encoder(self):
        try:
            device = InputDevice(dev_path)

            for event in device.read_loop():
            
                # --- PEDALS & STEERING ---
                if event.type == ecodes.EV_ABS:
                    absevent = categorize(event)
                    code = ecodes.bytype[absevent.event.type][absevent.event.code]
            
                    if code == 'ABS_X':
                        normalized_steering = self.map_value(absevent.event.value, 0, 65535, 1.0, -1.0)
                        self.joy_msg.axes[0] = normalized_steering
                    elif code == 'ABS_Y':
                        normalized_gas = self.map_value(absevent.event.value, 255, 0, 0.0, 1.0)
                        self.joy_msg.axes[1] = normalized_gas
                    elif code == 'ABS_Z':
                        normalized_brake = self.map_value(absevent.event.value, 255, 0, 0.0, 1.0)
                        self.joy_msg.axes[2] = normalized_brake

                    # Publish msg
                    # self.publisher.publish(self.joy_msg)
                    self.publish_msgs()

                # --- BUTTONS ---
                # elif event.type == ecodes.EV_KEY:
                #     keyevent = categorize(event)
                #     if keyevent.keystate == 1: # Only print on "Press"
                #         print(f"Button {keyevent.keycode} pushed!")

        except PermissionError:
            print(f"Permission denied! Run: chmod 666 {dev_path} on the host.")
        except FileNotFoundError:
            print(f"Device {dev_path} not found. Check your Docker mapping.")
        except KeyboardInterrupt:
            print("\nStopping...")

def main(args=None):
    rclpy.init(args=args)
    steering_wheel_node = wheel_publisher()

    try:
        rclpy.spin(steering_wheel_node)
    except KeyboardInterrupt:
        pass
    steering_wheel_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# Message can be received with "ros2 topic echo /steering_wheel_joy"
