import roslibpy
import time, sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from threading import Lock

rosbridge_IP = '192.168.147.142'
rosbridge_port = 9090

latest_cmd_vel = {'lin_x': 0.0, 'ang_z':0.0}
vel_lock = Lock()

class CoppeliaSimBridge:
    def __init__(self):
        self.client = None
        self.sim = None
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            print("Connected to CoppeliaSim via ZeroMQ API")

            self.sim.setStepping(True)

            if self.sim.getSimulationState() != self.sim.simulation_stopped:
                print("Simulation was running. Stopping it...")
                self.sim.stopSimulation()
                while self.sim.getSimulationState() != self.sim.simulation_stopped:
                    time.sleep(0.01)

            print("Starting Simulation...")
            self.sim.startSimulation()
            print("Simulation Started")

            self.robot_handle = self.sim.getObject('/PioneerP3DX')
            self.left_motor_handle = self.sim.getObject('/PioneerP3DX/leftMotor')
            self.right_motor_handle = self.sim.getObject('/PioneerP3DX/rightMotor')

            print("Motor and robot handles obtained.")

        except Exception as e:
            if self.client:
                self.close()
            raise Exception(f"Failed connecting to CoppeliaSim or initializing: {e}")

    def set_robot_vel(self, lin_x, ang_z):
        wheel_radius = 0.0975 # in meters
        wheel_dist = 0.331 #in meters

        left_speed = (lin_x - (ang_z * wheel_dist / 2)) / wheel_radius
        right_speed = (lin_x + (ang_z * wheel_dist / 2)) / wheel_radius
        self.sim.setJointTargetVelocity(self.left_motor_handle, left_speed)
        self.sim.setJointTargetVelocity(self.right_motor_handle, right_speed)

    def get_robot_pose(self):
        # Get object position and orientation
        position = self.sim.getObjectPosition(self.robot_handle, -1)
        orientation = self.sim.getObjectOrientation(self.robot_handle, -1)

        return {
            'x': position[0],
            'y': position[1],
            'theta': orientation[2]  # yaw angle
        }

    def close(self):
        # Stop simulation if it's running when closing
        if self.sim and self.sim.getSimulationState() != self.sim.simulation_stopped:
            print("Stopping simulation before closing connection...")
            self.sim.stopSimulation()
            while self.sim.getSimulationState() != self.sim.simulation_stopped:
                time.sleep(0.01) # Small sleep to ensure it stops
            print("Simulation stopped.")

        # In ZeroMQ API, closing the client is usually handled implicitly
        # when the client object goes out of scope, or you can explicitly
        # call client.disconnect() if available and necessary.
        # For RemoteAPIClient, it manages its connection lifecycle.
        print("CoppeliaSimBridge closed.")

def main():
    print("Starting ROSBridge Client...")
    client = roslibpy.Ros(host=rosbridge_IP, port=rosbridge_port)
    try:
        client.run() # This runs in a separate thread
        print("ROSBridge client connected.")
    except Exception as e:
        print(f"Failed to connect to ROSBridge: {e}")
        sys.exit(1)

    sim_bridge = None
    try:
        sim_bridge = CoppeliaSimBridge()
        def on_cmd_vel(message):
            linear = message['linear']
            angular = message['angular']
            with vel_lock:
                latest_cmd_vel['lin_x'] = linear['x']
                latest_cmd_vel['ang_z'] = angular['z']
            print(f"Received cmd_vel: linear_x={linear['x']}, angular_z={angular['z']}")

        cmd_vel_listener = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')
        cmd_vel_listener.subscribe(on_cmd_vel)
        print("Subscribed to /cmd_vel topic.")

        robot_pose_publisher = roslibpy.Topic(client, '/robot_pose', 'geometry_msgs/Pose2D')
        print("Created /robot_pose publisher.")

        while True:
        # Advance the simulation by one step when in synchronous mode
            sim_bridge.sim.step()

            with vel_lock:
                lin_x = latest_cmd_vel['lin_x']
                ang_z = latest_cmd_vel['ang_z']

            sim_bridge.set_robot_vel(lin_x, ang_z)
            # Get and publish robot pose
            pose = sim_bridge.get_robot_pose()
            msg = roslibpy.Message({'x': pose['x'], 'y': pose['y'], 'theta': pose['theta']})
            robot_pose_publisher.publish(msg)

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Ctrl+C pressed. Shutting down.")
    except Exception as e:
        print(f"An error occurred in main loop: {e}")

    finally:
        if 'cmd_vel_listener' in locals() and cmd_vel_listener.is_subscribed:
            print("Unsubscribing from /cmd_vel...")
            cmd_vel_listener.unsubscribe()
        if 'robot_pose_publisher' in locals() and robot_pose_publisher.is_advertised:
            print("Unadvertising /robot_pose...")
            robot_pose_publisher.unadvertise()
        if client and client.is_connected:
            print("Terminating ROSBridge client...")
            client.terminate()
        if sim_bridge:
            print("Closing CoppeliaSim bridge...")
            sim_bridge.close()
        print("Script finished.")

if __name__ == '__main__':
    main()