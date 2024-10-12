import time, random
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from airobot_interfaces.action import StringCommand  # カスタムアクション定義のインポート


class BringmeActionServer(Node):
    def __init__(self):
        super().__init__('bringme_action_server')
        self._action_server = ActionServer(
            self, StringCommand, 'command', 
            execute_callback=self.execute_callback,
        )
        self.food = ['apple', 'banana', 'candy']

    def execute_callback(self, goal_handle):
        feedback = StringCommand.Feedback()
        count = random.randint(5, 10)

        while count > 0:
            self.get_logger().info(f'フィードバック送信中: 残り{count}[s]')     
            feedback.process = f'{count}'
            goal_handle.publish_feedback(feedback)  
            count -= 1  
            time.sleep(1)

        item = goal_handle.request.command
        result = StringCommand.Result()
        if item in self.food:
            result.answer =f'はい，{item}です．'
        else:
            result.answer = f'{item}を見つけることができませんでした．'
        goal_handle.succeed()
        self.get_logger().info(f'ゴールの結果: {result.answer}')
        return result


def main():
    rclpy.init()
    bringme_action_server = BringmeActionServer()
    print('サーバ開始')
    try:
        rclpy.spin(bringme_action_server)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()
    print('サーバ終了')
