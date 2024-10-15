# アクションのプログラム例

## 概要

- この本で共通の[StringCommandアクション型](https://github.com/AI-Robot-Book-Humble/chapter2/blob/main/airobot_interfaces/action/StringCommand.action)を使う．
- アクションサーバが，ゴールの処理中にキャンセルや新たなゴールを受け付けるプログラムの例
- アクションクライアントが，ゴールの処理中にキャンセルや新たなゴールを送ることのできるプログラムの例．
- Ubuntu 22.04, ROS Humbleで作成・確認．

## インストール

- ROSのワークスペースを`~/airobot_ws`とする．
  ```
  cd ~/airobot_ws/src
  ```

- このパッケージを含むリポジトリを入手
  ```
  cd ~/airobot_ws/src
  git clone https://github.com/AI-Robot-Book-Humble/appendixB
  ```

- アクションのインタフェースを定義しているパッケージを含むリポジトリを入手
  ```
  git clone https://github.com/AI-Robot-Book-Humble/chapter2
  ```

- パッケージをビルド
  ```
  cd ~/airobot_ws
  colcon build --symlink-install
  source install/setup.bash
  ```

## 実行

- 端末1（アクションサーバ）
  ```
  cd ~/airobot_ws
  source install/setup.bash
  ros2 run airobot_action new_bringme_action_server_node
  ```

- 端末2（アクションクライアント）
  ```
  cd ~/airobot_ws
  source install/setup.bash
  ros2 run airobot_action test_client
  ```

## ヘルプ

## 著者

升谷 保博

## 履歴

- 2024-10-15: 公開

## ライセンス

Copyright (c) 2024 MASUTANI Yasuhiro  
All rights reserved.  
This project is licensed under the Apache License 2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献

- [rclpy Actions](https://docs.ros2.org/foxy/api/rclpy/api/actions.html)
- [Understanding actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- [Writing an action server and client (Python)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
  - キャンセルや処理中のゴールの受付について説明されていない．
- [ROS1 の SimpleActionServer を ROS2 で書き換える](https://qiita.com/nasu_onigiri/items/783d7ee77556528e5a52)
- [minimal_action_server パッケージ](https://github.com/ros2/examples/tree/humble/rclpy/actions/minimal_action_server)
  - 一度に１個だけのゴールを処理するサーバ [server_single_goal](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_single_goal.py)
  - 受け付けたゴールをキューに入れるサーバ[server_queue_goals](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_queue_goals.py)
  - 複数のゴールを並行処理するサーバ[server](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server.py)
  - 複数のゴールを延期して並行処理するサーバ[server_defer](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_defer.py)
  - ノードのクラスを作らないサーバ [server_not_composable](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_not_composable.py)
- [minimal_action_client パッケージ](https://github.com/ros2/examples/tree/humble/rclpy/actions/minimal_action_client)
  - 最低限のクライアント [client](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client.py)
  - ゴールを送ってからキャンセルを送るクライアント[client_cancel](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client_cancel.py)
  - 並行処理 [client_asyncio](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client_asyncio.py)
  - ノードのクラスを作らないクライアント [client_not_composable](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client_not_composable.py)

