import os
import sys
import readline  # input()に履歴機能を追加するために必要
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args
from action_msgs.msg import GoalStatus
from airobot_interfaces.action import StringCommand


class TestClient(Node):
    def __init__(self, action_name):
        super().__init__('test_client')
        self.get_logger().info(f'{action_name}のクライアントを起動します')
        self.goal_handle = None  # 処理中のゴールの情報を保持する変数
        self.action_client = ActionClient(
            self, StringCommand, action_name)
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('アクションサーバ無効，待機中...')

    def send_goal(self, command):
        self.get_logger().info(f'ゴール送信: {command}')
        goal_msg = StringCommand.Goal()
        goal_msg.command = command
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'フィードバック: \'{feedback_msg.feedback.process}\'')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('ゴールは拒否されました')
            return
        self.goal_handle = goal_handle  # ゴールの情報を更新
        self.get_logger().info('ゴールは受け付けられました')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'結果: {result.answer}')
        else:
            self.get_logger().info(f'失敗ステータス: {status}')
        self.goal_handle = None  # ゴール情報をリセット

    def cancel(self):
        if self.goal_handle is None:
            self.get_logger().info('キャンセル対象なし')
            return
        self.get_logger().info('キャンセル')
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('キャンセル成功')
            self.goal_handle = None  # ゴール情報をリセット
        else:
            self.get_logger().info('キャンセル失敗')


def main():
    action_name = 'command'                # アクション名のデフォルト値
    args = remove_ros_args(args=sys.argv)  # コマンドラインの引数からROS用を取り除く
    if len(args) >= 2:                     # 引数の1番目をアクション名に設定
        action_name = args[1]
    history_path = '.history' + '_' + action_name.replace('/', '_')
    if os.path.isfile(history_path):
        readline.read_history_file(history_path)
    print('''使い方：
  文字列を入力しEnter → StringCommandのゴールcommandとして送信
  Enter → キャンセル
  exit Enter → プログラム終了
  サーバが実行中に次のゴールを送ることができる
  行履歴・行編集の機能あり''')

    rclpy.init()
    node = TestClient(action_name)
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    threading.excepthook = lambda x: ()
    thread.start()

    try:
        while True:
            command = input('command: ')
            if command == '':        # Enterキーだけを押した場合はキャンセル
                node.cancel()
            elif command == 'exit':  # exitの場合は終了
                break
            else:                    # それ以外は，文字列をそのままゴールとして送信
                node.send_goal(command)
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()

    readline.write_history_file(history_path)
