import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from airobot_interfaces.action import StringCommand

class BringmeActionClient(Node):
    def __init__(self):  # コンストラクタ
        super().__init__('bringme_action_client')
        # アクションクライアントを初期化
        self._action_client = ActionClient(self, StringCommand, 'command')

    def send_goal(self, order):  # ゴールの送信        
        goal_msg = StringCommand.Goal()  # ゴールメッセージの作成
        goal_msg.command = order        
        self._action_client.wait_for_server()  # サーバーが準備できるまで待機
        # ゴールを送信し、フィードバックや結果を非同期で処理
        return self._action_client.send_goal_async(  
            goal_msg, feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):  # フィードバックを受け取り進捗を表示
        self.get_logger().info(f'フィードバック受信中: 残り{feedback_msg.feedback.process}[s]')


def main(args=None):
    rclpy.init(args=args)
    bringme_action_client = BringmeActionClient()
    order = input('何を取ってきますか？')

    future = bringme_action_client.send_goal(order)  # ゴールを送信しFutureオブジェクトを取得    
    rclpy.spin_until_future_complete(bringme_action_client, future)  # ゴール送信が完了するまで待機    
    goal_handle = future.result()  # ゴールハンドルの取得

    if not goal_handle.accepted:
        bringme_action_client.get_logger().info('ゴールは拒否されました')
    else:
        bringme_action_client.get_logger().info('ゴールが承認されました')        
        result_future = goal_handle.get_result_async()  # 結果を非同期で取得
        # 結果が完了するまで待機
        rclpy.spin_until_future_complete(bringme_action_client, result_future)        
        result = result_future.result().result  # 結果を取得        
        bringme_action_client.get_logger().info(f'ゴールの結果: {result.answer}')  # ノードが終了したら破棄
    
    bringme_action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
