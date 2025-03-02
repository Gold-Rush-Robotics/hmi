import rclpy
from rclpy.node import Node
import subprocess
from std_msgs.msg import String
from json import dumps
CALLBACK_TIME = 5

class NodeInfo(Node):
    
    def __init__(self) -> None:
        super().__init__('node_info')
        self.publisher_ = self.create_publisher(String, 'node_info_publisher', 10)

        self.timer = self.create_timer(CALLBACK_TIME, self.timer_callback)

    def timer_callback(self) -> None:
        msg = String()
        msg.data = get_node_info()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def get_node_info() -> str:
    """
    Gets all nodes currently active and gets all of the 
    topics from those nodes.
    """
    node_data = {} # Final return value

    valid_titles = {'Publishers:': [], 'Subscribers:': [], 'Service Servers:': [],
     'Service Clients:': [], 'Action Servers:': [], 'Action Clients:': []}  # Titles wanted with arrays to store topics
    # invalid_topics = {'/node_info_publisher', '/rosout', '/parameter_events'} 

    try:
        nodes = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE) # Gets all current running nodes
        nodes_decode = nodes.stdout.decode('utf-8').strip().split('\n') # Decodes using charset and split each node

        for node in nodes_decode:

            node_info = subprocess.run(['ros2', 'node', 'info', node], stdout=subprocess.PIPE) # Gets info on current node
            node_info_decode = node_info.stdout.decode('utf-8').strip().split('\n') # Decodes using charset and splits info
            node_name = node_info_decode[0] # Node name

            """
            Example of ros2 node info:

            /node_publisher
            Subscribers:

            Publishers:
                /node_data: std_msgs/msg/String
                /parameter_events: rcl_interfaces/msg/ParameterEvent
                /rosout: rcl_interfaces/msg/Log
            Service Servers:
                /node_publisher/describe_parameters: rcl_interfaces/srv/DescribeParameters
                /node_publisher/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
                /node_publisher/get_parameters: rcl_interfaces/srv/GetParameters
                /node_publisher/list_parameters: rcl_interfaces/srv/ListParameters
                /node_publisher/set_parameters: rcl_interfaces/srv/SetParameters
                /node_publisher/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
            Service Clients:

            Action Servers:

            Action Clients:
            """
            
            current_array = None
            for line in node_info_decode:
                line = line.strip()
                if not line: # If line is empty
                    continue
                
                if current_array is None or line in valid_titles.keys(): # If there is no array selected or the line is a valid key
                    array = valid_titles.get(line, None) 
                    if array is not None:
                        current_array = array 
                    continue
                
                current_topic = line.split(':') # Split each topic EX: (/topic: std_msgs/msg/String)

                # if current_topic[0] in invalid_topics:
                #     continue
            
                current_array.append({'topic': current_topic[0], 'type': current_topic[1].strip()}) # If is valid, add to currently selected array
            
            node_data[node_name] = {title.lower().replace(' ', '_').removesuffix(':'): valid_titles[title] for title in valid_titles.keys()} # Add all valid_titles arrays to node_data using nodes name as key
            valid_titles = {title: [] for title in valid_titles.keys()} # Clear all arrays before next node
        return dumps(node_data)

    except Exception as e:
        return dumps({"Error while parsing,": str(e)})

def main(args : any = None) -> None:
    rclpy.init(args=args)

    node_info = NodeInfo()
    
    rclpy.spin(node_info)

    node_info.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()