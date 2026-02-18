import rclpy
from rclpy.node import Node

# Import Interfaces ที่เราสร้างไว้
from patrol_interfaces.msg import PatrolStatus
from patrol_interfaces.srv import ResetCmd

class PatrolLoggerNode(Node):
    def __init__(self):
        super().__init__('patrol_logger_py_node')

        # 1. Subscriber: คอยฟังสถานะจาก C++ Node
        self.subscription = self.create_subscription(
            PatrolStatus,
            '/patrol/status',
            self.listener_callback,
            10
        )

        # 2. Service Server: รอรับคำสั่ง Reset (จุดที่จะใส่ Bug Security ในอนาคต)
        self.srv = self.create_service(
            ResetCmd, 
            '/patrol/reset_cmd', 
            self.reset_callback
        )
        
        self.get_logger().info('✅ Patrol Logger Node is Ready (Logging & Command Service)')

    def listener_callback(self, msg):
        # หน้าที่: แค่รับมาแล้วโชว์ (ในอนาคตเราจะแกล้งใส่ File Leak ตรงนี้)
        self.get_logger().info(f'🤖 [Robot ID: {msg.id}] Location: ({msg.location.x:.1f}, {msg.location.y:.1f}) | Status: "{msg.status}"')

    def reset_callback(self, request, response):
        # หน้าที่: รับคำสั่ง Reset
        # ใน Main Branch: เราเขียน Logic แบบปลอดภัย (Safe)
        self.get_logger().info(f'📥 Received Command: "{request.command_type}" for Node: "{request.target_node}"')

        if request.command_type == "restart":
            self.get_logger().warn("⚠️ System is restarting...")
            response.success = True
            response.message = "System restart initiated safely."
        elif request.command_type == "shutdown":
             self.get_logger().error("🛑 System is shutting down...")
             response.success = True
             response.message = "Shutdown sequence started."
        else:
            response.success = False
            response.message = f"Unknown command: {request.command_type}"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PatrolLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()