# scripts/ros2_policy_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import torch
from train_bc import PolicyNet

class PolicyNode(Node):
    def __init__(self):
        super().__init__("policy_node")
        self.pub = self.create_publisher(Float32MultiArray, "actions", 10)
        self.model = PolicyNet(10, 7)
        self.model.load_state_dict(torch.load("data/processed/policy.pt"))
        self.model.eval()
        self.timer = self.create_timer(0.1, self.step)

    def step(self):
        obs = torch.randn(1, 10)  # dummy tactile features
        with torch.no_grad():
            act = self.model(obs).numpy().flatten()
        msg = Float32MultiArray(data=act.tolist())
        self.pub.publish(msg)
        self.get_logger().info(f"Action: {act}")

def main():
    rclpy.init()
    node = PolicyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
