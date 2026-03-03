#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import DynamicJointState

def main():
    import sys
    import time
    rclpy.init()
    node = rclpy.create_node('dynamic_joint_state_listener')
    
    line_count = {'count': 0}
    
    def callback(msg):
        try:
            if line_count['count'] == 0:
                print("wheel-velocity;hinge-position")
                
            if line_count['count'] < 100:
                if len(msg.interface_values) > 5:
                    val0 = msg.interface_values[0].values
                    val5 = msg.interface_values[5].values
                    # Print only the first value from each array, semi-colon separated
                    first_val0 = val0[1] if len(val0) > 0 else 'NaN'
                    first_val5 = val5[0] if len(val5) > 0 else 'NaN'
                    print(f"{first_val0};{first_val5}")
                    line_count['count'] += 1
                    
                    if line_count['count'] >= 100:
                        node.destroy_node()
                        rclpy.shutdown()
        except Exception as e:
            print(f"Error: {e}")

    node.create_subscription(
        DynamicJointState,
        '/racecar/dynamic_joint_states',
        callback,
        10
    )

    try:
        print("Listening to /racecar/dynamic_joint_states...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
