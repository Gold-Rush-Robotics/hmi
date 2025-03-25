import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hmi_com_interface.srv import NodeInfoSrv
from json import dumps

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
            node_data: JSON formatted ``str`` containing node data.
        """

        node_data = {}  # Final return value
        valid_titles_template = {
            'publishers': [],
            'subscribers': [],
            'service_servers': [],
            'service_clients': [],
        }

        try:
            nodes = self.get_node_names_and_namespaces()
            
            for node_name, namespace in nodes:
                valid_titles = valid_titles_template.copy()  # Copy for each node.

                # Get topic info and parse.
                valid_titles['publishers'] = self.parse_topics("topic", self.get_publisher_names_and_types_by_node(node_name, namespace))
                valid_titles['subscribers'] = self.parse_topics("topic", self.get_subscriber_names_and_types_by_node(node_name, namespace))
                valid_titles['service_servers'] = self.parse_topics("service", self.get_service_names_and_types_by_node(node_name, namespace))
                valid_titles['service_clients'] = self.parse_topics("service", self.get_client_names_and_types_by_node(node_name, namespace))

                node_data[f"{namespace}{node_name}"] = valid_titles

            return dumps(node_data)

        except Exception as e:
            return dumps({"Error while getting node info": str(e)})
        
    def parse_topics(self, key_name: str, data: list[tuple[str, list[str]]]) -> list[dict[str, str]]:
        try:
            return [{key_name:topic, "type":data_types[0] if data_types else "Unknown"} for topic, data_types in data]
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