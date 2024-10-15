import time, random
from threading import Lock
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import StringCommand  # カスタムアクション定義のインポート


class BringmeActionServer(Node):
    def __init__(self):
        super().__init__('bringme_action_server')
        self.goal_handle = None    # 処理中のゴールの情報を保持する変数
        self.goal_lock = Lock()    # 二重実行させないためのロック変数
        self.execute_lock = Lock() # 二重実行させないためのロック変数
        self._action_server = ActionServer(
            self, StringCommand, 'command', 
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.food = ['apple', 'banana', 'candy']

    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:               # ブロック内を二重実行させない
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('前の処理を中断')
                self.goal_handle.abort()
            self.goal_handle = goal_handle # ゴール情報の更新
        goal_handle.execute()              # ゴール処理の実行

    def execute_callback(self, goal_handle):
        with self.execute_lock:            # ブロック内を二重実行させない
            feedback = StringCommand.Feedback()
            result = StringCommand.Result()
            count = random.randint(5, 10)

            while count > 0:
                if not goal_handle.is_active:
                    self.get_logger().info('中断処理')
                    return result

                if goal_handle.is_cancel_requested:
                    self.get_logger().info('キャンセル処理')
                    goal_handle.canceled()
                    return result

                self.get_logger().info(f'フィードバック送信中: 残り{count}[s]')     
                feedback.process = f'{count}'
                goal_handle.publish_feedback(feedback)  
                count -= 1  
                time.sleep(1)

            item = goal_handle.request.command
            if item in self.food:
                result.answer =f'はい，{item}です．'
            else:
                result.answer = f'{item}を見つけることができませんでした．'
            goal_handle.succeed()
            self.get_logger().info(f'ゴールの結果: {result.answer}')
            return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('キャンセル受信')
        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    bringme_action_server = BringmeActionServer()
    print('サーバ開始')
    try:
        rclpy.spin(bringme_action_server, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()
    print('サーバ終了')
