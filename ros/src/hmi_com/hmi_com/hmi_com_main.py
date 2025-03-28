from json import dumps

import rclpy
from hmi_com_interface.srv import NodeInfoSrv
from rclpy.node import Node
from std_msgs.msg import String

CALLBACK_TIME = 5

class HmiCom(Node):
    def __init__(self) -> None:
        super().__init__('hmi_com')
        self.publisher = self.create_publisher(String, 'node_info_pub', 10)
        self.service = self.create_service(NodeInfoSrv, 'node_info_srv', self.handle_node_info_srv)

        self.timer = self.create_timer(CALLBACK_TIME, self.timer_callback)

    def timer_callback(self) -> None:
        msg = String()
        msg.data = self.get_node_info()
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def handle_node_info_srv(self, request, response) -> any:
        response.data = self.get_node_info()
        self.get_logger().info('Service: "%s"' % response.data)
        return response

    def get_node_info(self) -> str:
        """
        Gets all active nodes and their topics:\n
        ``publishers, subscribers, service_servers, and service_clients.``
        
        Returns:
            node_info: JSON formatted ``str`` containing node data.
        """

        node_info = {}  # Final return value
    
        try:
            nodes = self.get_node_names_and_namespaces()
            
            for node_name, namespace in nodes:

                node_data = {
                    'publishers':self.parse_node_data(self.get_publisher_names_and_types_by_node(node_name, namespace)),
                    'subscribers':self.parse_node_data(self.get_subscriber_names_and_types_by_node(node_name, namespace)),
                    'service_servers':self.parse_node_data(self.get_service_names_and_types_by_node(node_name, namespace)),
                    'service_clients':self.parse_node_data(self.get_client_names_and_types_by_node(node_name, namespace))
                }

                node_info[f"{namespace}{node_name}"] = node_data

            return dumps(node_info)

        except Exception as e:
            return dumps({"Error while getting node info": str(e)})
        
    def parse_node_data(self, data: list[tuple[str, list[str]]]) -> list[dict[str, str]]:
        try:
            return [{"topic":name, "type":data_types[0] if data_types else "/unknown"} for name, data_types in data]
        except Exception as e:
            return [{"Error while parsing": str(e)}]
        
def main(args=None) -> None:
    rclpy.init(args=args)
    hmi_com_node = HmiCom()
    rclpy.spin(hmi_com_node)
    hmi_com_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()