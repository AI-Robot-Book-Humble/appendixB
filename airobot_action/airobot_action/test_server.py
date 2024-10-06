from threading import Lock
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import StringCommand


class CountServer(Node):
    def __init__(self):
        super().__init__('count_server')
        self.goal_handle = None
        self.goal_lock = Lock()
        self.execute_lock = Lock()
        self.action_server = ActionServer(
            self,
            StringCommand,
            'command',
            self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            # 処理中にキャンセルや新たなゴールを受け付けるには，ReentrantCallbackGroupが必要
            callback_group=ReentrantCallbackGroup(),
        )

    # goal_callbackは省略（デフォルトのコールバックが使われる）

    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            # 同時に2つ以上のゴールを処理しないように排他制御
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('前の処理を中止')
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        with self.execute_lock:
            # 同時に2つ以上の実行をしないように排他制御
            self.get_logger().info('実行...')
            result = StringCommand.Result()
            result.answer = 'NG'
            try:
                count = int(goal_handle.request.command)
            except ValueError:
                self.get_logger().warn(f'command: \'{goal_handle.request.command}\'は不適切')
                goal_handle.abort()
                return result

            feedback = StringCommand.Feedback()
            while count > 0:
                if not goal_handle.is_active:
                    self.get_logger().info('中止')
                    return result

                if goal_handle.is_cancel_requested:
                    self.get_logger().info('キャンセル')
                    goal_handle.canceled()
                    return result

                feedback.process = f'{count}'
                self.get_logger().info(f'フィードバック: \'{feedback.process}\'')
                goal_handle.publish_feedback(feedback)            
                count -= 1
                time.sleep(1)
            self.get_logger().info('完了')
            goal_handle.succeed()
            result.answer = 'OK'
            return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('キャンセル受信')
        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    node = CountServer()
    print('サーバ開始')
    try:
        # 処理中にキャンセルや新たなゴールを受け付けるには，MultiThreadedExecutorが必要
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()
    print('サーバ終了')
