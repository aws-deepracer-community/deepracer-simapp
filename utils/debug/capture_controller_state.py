#!/usr/bin/env python3
"""
Captures PID controller state values from the right front wheel velocity controller.
Records all fields from dof_states[0] including reference, output, error, and PID terms.
"""
import rclpy
from rclpy.node import Node
import sys

def main():
    rclpy.init()
    node = rclpy.create_node('controller_state_listener')
    
    line_count = {'count': 0}
    header_printed = {'value': False}
    
    def callback(msg):
        try:
            # Print header on first message
            if not header_printed['value']:
                if hasattr(msg, 'dof_states') and len(msg.dof_states) > 0:
                    dof = msg.dof_states[0]
                    fields = list(dof.get_fields_and_field_types().keys())
                    print(";".join(["timestamp"] + fields))
                    header_printed['value'] = True
                else:
                    print("Error: Message does not contain dof_states", file=sys.stderr)
                    return
                
            if line_count['count'] < 200:
                # Extract timestamp from header
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                
                # Extract all values from dof_states[0]
                if hasattr(msg, 'dof_states') and len(msg.dof_states) > 0:
                    dof = msg.dof_states[0]
                    values = [str(timestamp)]
                    
                    # Dynamically extract all field values
                    for field_name in dof.get_fields_and_field_types().keys():
                        value = getattr(dof, field_name, 'N/A')
                        values.append(str(value))
                    
                    print(";".join(values))
                    
                line_count['count'] += 1
                
                if line_count['count'] >= 200:
                    print(f"\nCaptured {line_count['count']} samples.", file=sys.stderr)
                    node.destroy_node()
                    rclpy.shutdown()
                    
        except Exception as e:
            print(f"Error in callback: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc(file=sys.stderr)

    # Import the controller state message type
    from control_msgs.msg import MultiDOFStateStamped

    subscription = node.create_subscription(
        MultiDOFStateStamped,
        '/racecar/right_front_wheel_velocity_controller/controller_state',
        callback,
        10
    )

    try:
        print("Listening to /racecar/right_front_wheel_velocity_controller/controller_state...", file=sys.stderr)
        print("Will capture 200 samples from dof_states[0]...", file=sys.stderr)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
