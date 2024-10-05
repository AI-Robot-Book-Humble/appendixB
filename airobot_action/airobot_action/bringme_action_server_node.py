import time, random
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from airobot_interfaces.action import StringCommand  # カスタムアクション定義のインポート


class BringmeActionServer(Node):
    def __init__(self):
        super().__init__('bringme_action_server')
        self._action_server = ActionServer(
            self, StringCommand, 'command', 
            execute_callback=self.execute_callback            
        )
        self.food = ['apple', 'banana', 'candy']

    def execute_callback(self, goal_handle):
        feedback = StringCommand.Feedback()
        count = random.randint(10, 20)

        while count > 0:
            self.get_logger().info(f"フィードバック送信中: 残り{count}[s]")     
            feedback.process = f'{count}'
            goal_handle.publish_feedback(feedback)  
            count -= 1  
            time.sleep(1)

        result = StringCommand.Result()
        for item in self.food:
            if item in goal_handle.request.command:
                result.answer =f'はい，{item}です．'
                goal_handle.succeed()
                self.get_logger().info(f"ゴールの結果: {result.answer}")
                return result
        
        result.answer = f'{goal_handle.request.command}を見つけることができませんでした．'
        goal_handle.succeed()
        self.get_logger().info(f"ゴールの結果: {result.answer}")
        return result

def main(args=None):
    rclpy.init(args=args)
    bringme_action_server = BringmeActionServer()
    rclpy.spin(bringme_action_server)
    bringme_action_server.destroy_node()
    rclpy.shutdown()