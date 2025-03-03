import rclpy
import subprocess
from rclpy.node import Node
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

    valid_titles = {'Publishers:': [], 
                    'Subscribers:': [], 
                    'Service Servers:': [],
                    'Service Clients:': [], 
                    'Action Servers:': [], 
                    'Action Clients:': []}  
    
    # invalid_topics = {'/node_info_publisher', '/rosout', '/parameter_events'} 

    try:
        nodes = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE) # Gets all current running nodes
        nodes_decode = nodes.stdout.decode('utf-8').strip().split('\n') # Decodes using charset and split each node

        for node in nodes_decode:

            node_info = subprocess.run(['ros2', 'node', 'info', node], stdout=subprocess.PIPE) # Gets info on current node
            node_info_decode = node_info.stdout.decode('utf-8').strip().split('\n')
            node_name = node_info_decode[0]
            
            current_array = None
            for line in node_info_decode:
                line = line.strip()
                if not line: # If line is empty
                    continue
                
                if current_array is None or line in valid_titles.keys():
                    current_array = valid_titles.get(line, None) 
                    continue
                
                topic, topic_type = line.split(':', 1) # Split each topic EX: (/topic: std_msgs/msg/String)

                # if topic in invalid_topics:
                #     continue
            
                current_array.append({'topic': topic, 'type': topic_type.strip()}) # If is valid, add to currently selected array
            
            node_data[node_name] = {title.lower().replace(' ', '_').removesuffix(':'): valid_titles[title] for title in valid_titles.keys()} # Add all valid_titles arrays to node_data using node_name as key
            valid_titles = {title: [] for title in valid_titles.keys()} # Clear all arrays before next node
        return dumps(node_data) # Use dumps(node_data, indent=4) for better formatting

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