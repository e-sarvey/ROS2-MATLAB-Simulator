import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from control_msgs.action import FollowJointTrajectory
import paho.mqtt.client as mqtt
import time

class SimulatedArmActionServer(Node):
    """
    A ROS2 action server that interacts with a simulated robot arm through MQTT.
    MQTT client name, angles topic, status topic, and action name can be set as ROS parameters.
    """

    def __init__(self):
        super().__init__('simulated_arm_server')

        # Declare parameters for MQTT client name, topics, and action name
        self.declare_parameter('mqtt_client_name', 'SimulatedArmClient')
        self.declare_parameter('angles_topic', 'robot_joint_angles')
        self.declare_parameter('status_topic', 'robot_status')
        self.declare_parameter('action_name', 'follow_joint_trajectory')  # Add action_name parameter

        # Fetch parameters for MQTT client and topics
        mqtt_client_name = self.get_parameter('mqtt_client_name').get_parameter_value().string_value
        self.angles_topic = self.get_parameter('angles_topic').get_parameter_value().string_value
        self.status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        action_name = self.get_parameter('action_name').get_parameter_value().string_value  # Get action_name

        self.get_logger().info(f'MQTT Client: {mqtt_client_name}')
        self.get_logger().info(f'Angles Topic: {self.angles_topic}')
        self.get_logger().info(f'Status Topic: {self.status_topic}')
        self.get_logger().info(f'Action Name: {action_name}')

        # Initialize MQTT client and set up callbacks
        self.mqtt_client = mqtt.Client(mqtt_client_name)
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_client.on_connect = self.on_mqtt_connect

        # Variables to store action state
        self.current_goal_handle = None  # Keep track of the current goal handle
        self.action_done = False         # Flag to indicate action completion
        self.reconnecting = False        # Reconnection flag

        # Create an action server with the specified action name
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,  # Use the dynamic action name
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback
        )

        # Initial connection attempt
        self.connect_to_broker()

        self.get_logger().info("Simulated Arm Action Server initialized.")

    def connect_to_broker(self):
        """Attempt to connect to the MQTT broker."""
        try:
            self.mqtt_client.connect("10.243.82.33", 1883, 60)
            # Start the MQTT network loop in a background thread
            self.mqtt_client.loop_start()
            self.get_logger().info("Connected to MQTT broker.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
            self.reconnecting = True

    def goal_callback(self, goal_request):
        """Callback to decide whether to accept or reject a goal."""
        self.get_logger().info('Received a goal request.')
        if self.reconnecting:
            self.get_logger().warn('Currently reconnecting to MQTT broker, rejecting the goal.')
            return GoalResponse.REJECT
        else:
            self.get_logger().info('Accepting the goal request.')
            return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Handles the execution of an accepted goal. Sends joint angles via MQTT and waits for "done" status.
        """
        self.get_logger().info('Executing goal...')

        self.current_goal_handle = goal_handle
        self.action_done = False  # Reset action done flag

        # Extract trajectory points and send them to MQTT
        trajectory_points = goal_handle.request.trajectory.points
        for point in trajectory_points:
            joint_angles = point.positions
            self.send_joint_angles_to_arm(joint_angles)

        # Subscribe to the status topic
        self.mqtt_client.subscribe(self.status_topic)

        # Log message and wait for "done" status
        self.get_logger().info('Waiting for "done" message from the arm...')

        # Simple loop to wait for the action to be completed
        while not self.action_done:
            time.sleep(0.1)  # Sleep briefly to avoid busy waiting

        # When action is done
        self.get_logger().info('Action completed successfully.')
        result = FollowJointTrajectory.Result()
        result.error_code = 0  # Indicate success
        goal_handle.succeed()

        self.current_goal_handle = None
        return result

    def send_joint_angles_to_arm(self, joint_angles):
        """Sends joint angles to the configured MQTT topic."""
        if self.reconnecting:
            self.get_logger().warn('Unable to send joint angles while reconnecting to MQTT broker.')
            return

        angles_str = ','.join(map(str, joint_angles))
        try:
            self.mqtt_client.publish(self.angles_topic, angles_str)
            self.get_logger().info(f'Sent joint angles to {self.angles_topic}: {angles_str}')
        except Exception as e:
            self.get_logger().error(f"Failed to publish joint angles: {e}")
            self.reconnecting = True

    def on_mqtt_message(self, client, userdata, msg):
        """Callback for MQTT messages."""
        status = msg.payload.decode()
        self.get_logger().info(f'Received status: {status}')

        if status == "received":
            self.get_logger().info('Received "received" status from the arm.')
            # Optionally, you can publish feedback here if needed
            if self.current_goal_handle:
                feedback = FollowJointTrajectory.Feedback()
                # Populate feedback if necessary
                self.current_goal_handle.publish_feedback(feedback)

        if status == "done" and self.current_goal_handle:
            self.get_logger().info('Received "done" status. Completing the goal.')
            self.action_done = True  # Set action done flag

    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback for handling MQTT disconnections."""
        self.get_logger().warn(f"Disconnected from MQTT broker (code {rc}). Reconnecting...")
        self.reconnecting = True
        self.reconnect_to_broker()

    def reconnect_to_broker(self):
        """Attempts to reconnect to the MQTT broker."""
        while self.reconnecting:
            try:
                self.get_logger().info("Attempting to reconnect to MQTT broker...")
                self.mqtt_client.reconnect()
                self.reconnecting = False
                self.get_logger().info("Reconnected to MQTT broker.")
                self.mqtt_client.subscribe(self.status_topic)
            except Exception as e:
                self.get_logger().error(f"Reconnection failed: {e}")
                time.sleep(5)  # Wait before retrying

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback for handling successful MQTT connections."""
        self.get_logger().info(f"Successfully connected to MQTT broker (code {rc}).")
        self.mqtt_client.subscribe(self.status_topic)

    def shutdown(self):
        """Shutdown the MQTT client when the node is terminated."""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()


def main(args=None):
    rclpy.init(args=args)
    simulated_arm_action_server = SimulatedArmActionServer()

    try:
        rclpy.spin(simulated_arm_action_server)
    except KeyboardInterrupt:
        simulated_arm_action_server.get_logger().info("Shutting down Simulated Arm Action Server.")
    finally:
        simulated_arm_action_server.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()