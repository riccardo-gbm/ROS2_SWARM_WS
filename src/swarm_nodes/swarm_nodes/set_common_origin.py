from functools import partial

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from geographic_msgs.msg import GeoPointStamped
from rclpy.qos import qos_profile_sensor_data

class SetCommonOrigin(Node):

    def __init__(self):
        super().__init__("set_common_origin")
        self.declare_parameter("drone_amount", "")
        self.declare_parameter("sim", True)
        
        self.client_list = []
        self.publisher_list = []
        self.futures = []
        self.new_origin = GeoPointStamped()

        self.drone_amount = self.get_parameter("drone_amount").value
        self.sim = self.get_parameter("sim").value

        if not self.sim: 
            
            self.gps_subscriber = self.create_subscription(GeoPointStamped, f"/drone1/mavros/global_position/gp_origin", self.gp_callback, qos_profile_sensor_data)

            for i in range(1, self.drone_amount + 1):
                self.cli = self.create_client(CommandLong, f"/drone{str(i)}/mavros/cmd/command")
                self.client_list.append(self.cli)
        else:
            
            self.gps_subscriber = self.create_subscription(GeoPointStamped, "/mavros/global_position/gp_origin", self.gp_callback, 10)
            self.cli = self.create_client(CommandLong, f"/mavros/cmd/command")
            self.client_list.append(self.cli)

        self.req = CommandLong.Request()

    def gp_callback(self, msg):
        self.new_origin.header.stamp = self.get_clock().now().to_msg()
        self.new_origin.header.frame_id = "map"
        self.new_origin.position.latitude = msg.position.latitude
        self.new_origin.position.longitude = msg.position.longitude
        self.new_origin.position.altitude = msg.position.altitude
        
    def send_request(self, command, param5=0.0, param6=0.0, param7=0.0):

        self.req.command = command
        self.req.param5 = param5
        self.req.param6 = param6
        self.req.param7 = param7

        for client in self.client_list:
            self.future = client.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            self.futures.append(self.future.result())
        return self.futures

def main(args=None):
    try:
        rclpy.init(args=args)
        set_common_origin = SetCommonOrigin()
        responses = set_common_origin.send_request(command=611, param5=set_common_origin.new_origin.position.latitude,param6=set_common_origin.new_origin.position.longitude,param7=set_common_origin.new_origin.position.altitude)
    
        for response in responses:
            set_common_origin.get_logger().info(f"Drone ID: {responses.index(response)}, Succes: {response.success}, Result: {response.result}")
    except Exception as error:
        set_common_origin.get_logger().error(f"Exception in SetCommonOrigin: {error}")
    finally:
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()