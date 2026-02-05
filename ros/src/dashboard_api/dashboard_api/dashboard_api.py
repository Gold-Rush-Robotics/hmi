from json import dumps
from rclpy.node import Node
from dashboard_interface.msg import Dashboard

class DashboardApi():
    def __init__(self, node: Node, _id: int) -> None:
        self.node = node
        self.id = _id
        self.publisher = node.create_publisher(Dashboard, "hmi_dashboard_data", 10)

    def _publish_dashboard(self, msg: Dashboard) -> None:
        self.publisher.publish(msg)
        self.node.get_logger().info(f"Published {msg.id} of type {msg.type}: {msg.data}")

    def _make_msg(self, screen: str, _type: str, data: dict) -> Dashboard:
        msg = Dashboard()
        msg.id = self.id
        msg.screen = screen
        msg.type = _type
        msg.data = dumps(data)
        return msg

    def card(self, screen: str, title: str, content: str) -> None:
        data = {
            "title": title,
            "content": content
        }
        msg = self._make_msg(screen, "card", data)
        self._publish_dashboard(msg)

    def color(self, screen: str, title: str, content: str, color: str) -> None:
        data = {
            "title": title,
            "content": content,
            "color": color
        }
        msg = self._make_msg(screen, "color", data)
        self._publish_dashboard(msg)
    
    def bar(self, screen: str, title: str, content: str, value: int, min_value: int=0, max_value: int=1) -> None:
        data = {
            "title": title,
            "content": content,
            "value": value,
            "min": min_value,
            "max": max_value
        }
        msg = self._make_msg(screen, "bar", data)
        self._publish_dashboard(msg)

_dashboard: DashboardApi | None = None

def init(node: Node, _id: int=0) -> None:
    global _dashboard
    if _dashboard is not None:
        raise RuntimeError("Dashboard already initialized")
    _dashboard = DashboardApi(node, _id)


def get() -> DashboardApi:
    if _dashboard is None:
        raise RuntimeError("Dashboard not initialized. Call init() first.")
    return _dashboard


def card(screen: str, title: str, content: str) -> None:
    get().card(screen, title, content)


def color(screen: str, title: str, content: str, color: str):
    get().color(screen, title, content, color)


def bar(screen: str, title: str, content: str, value: int, min_value: int=0, max_value: int=1) -> None:
    get().bar(screen, title, content, value, min_value, max_value)
