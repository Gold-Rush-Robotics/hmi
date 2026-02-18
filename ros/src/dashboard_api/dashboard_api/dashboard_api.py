from json import dumps
from rclpy.node import Node
from std_msgs.msg import String

class DashboardApi():
    def __init__(self, node: Node) -> None:
        self.node = node
        self.publisher = node.create_publisher(String, "hmi_dashboard_data", 10)

    def _publish_dashboard(self, msg: String) -> None:
        self.publisher.publish(msg)
        self.node.get_logger().info(f"Published: {msg.data}")

    def _make_msg(self, _id: str, screen: str, _type: str, data: dict) -> String:
        msg = String()
        data = {
            "id": _id,
            "screen": screen,
            "type": _type,
            "data": data
        }
        msg.data = dumps(data)
        return msg

    def card(
        self,
        _id: str,
        screen: str,
        title: str,
        content: str=None
    ) -> None:
        data = {
            "title": title,
            "content": content
        }
        msg = self._make_msg(_id, screen, "card", data)
        self._publish_dashboard(msg)

    def color(
        self,
        _id: str,
        screen: str,
        title: str,
        color: str,
        content: str=None
    ) -> None:
        data = {
            "title": title,
            "content": content,
            "color": color
        }
        msg = self._make_msg(_id, screen, "color", data)
        self._publish_dashboard(msg)
    
    def bar(
        self,
        _id: str,
        screen: str,
        title: str,
        value: int,
        content: str=None,
        min_value: int=0,
        max_value: int=1
    ) -> None:
        data = {
            "title": title,
            "content": content,
            "value": value,
            "min": min_value,
            "max": max_value
        }
        msg = self._make_msg(_id, screen, "bar", data)
        self._publish_dashboard(msg)
