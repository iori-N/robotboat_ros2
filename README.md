# minix_ws

水草刈取り用自律艇（USV）の ROS 2 ワークスペース。

## 環境

- Ubuntu: Ubuntu 24.04.4 LTS
- ROS 2: jazzy
- 制御PC: NEO-Z150-Aero (MINIX)

## 再構築

```bash
mkdir -p ~/minix_ws && cd ~/minix_ws
git clone git@github.com:iori-N/minix_ws.git .
vcs import src < minix_ws.repos
colcon build --symlink-install
```

パッケージ一覧と各リポジトリのURLは `minix_ws.repos` を参照。
`iori-N/` のものは自作・改造済み、それ以外は上流のまま。

---

# 自律航行ボート ROS2プロジェクト

RTK-GNSSによる高精度測位を用いた自律走行地上ローバーシステム。

詳細なシステム構成・パッケージ仕様・設計規約については [doc/system-overview.md](doc/system-overview.md) を参照。

## 必要環境

| 項目 | バージョン |
|------|-----------|
| ROS2 | Jazzy |
| Gazebo | Harmonic (シミュレーション使用時) |
| Python | 3 |
| OS | Ubuntu 24.04 (推奨) |

### 追加依存

- **pyproj**: `~/minix_ws/.venv` 内にインストール済み (rosdep管理外)
  - 使用パッケージ: `lg580p_gnss_driver`, `look_ahead_control`, `ublox_gnss_driver` (legacy)
- **pyserial**: `lg580p_gnss_driver`, `maestro_driver`, `ublox_gnss_driver` (legacy) で使用

## ソースコードの取得

依存パッケージのインストール、ソース取得、venv作成、`pyproj` インストール、ビルドまでまとめて実行する場合:

```bash
git clone https://github.com/IgarashiBME/robotboat_ros2.git ~/minix_ws
cd ~/minix_ws
tools/setup_workspace.sh
```

ビルドを後で行う場合:

```bash
tools/setup_workspace.sh --no-build
```

手動で取得する場合:

```bash
git clone https://github.com/IgarashiBME/robotboat_ros2.git ~/minix_ws
cd ~/minix_ws
vcs import --recursive src < robotboat.repos
```

既に `vcs import` 済みで `mavlink_ros2_bridge/include/c_library_v2/` が未取得の場合:

```bash
git -C src/mavlink_ros2_bridge submodule update --init --recursive
```

## ビルド

```bash
# 1. ROS2環境のセットアップ
source /opt/ros/jazzy/setup.bash

# 2. pyproj用venvの有効化
source ~/minix_ws/.venv/bin/activate

# 3. bme_common_msgs を先にビルド (他パッケージが依存)
cd ~/minix_ws
colcon build --symlink-install --packages-select bme_common_msgs
source install/setup.bash

# 4. 残りのパッケージをビルド
colcon build --symlink-install
source install/setup.bash
```

### 単体パッケージのビルド

```bash
colcon build --packages-select <package_name>
```

## 起動

### 実機

```bash
source /opt/ros/jazzy/setup.bash
source ~/minix_ws/install/setup.bash

ros2 launch lg580p_gnss_driver lg580p_gnss.launch.py
# legacy (u-blox 2受信機構成): ros2 launch ublox_gnss_driver ublox_gnss.launch.py
ros2 launch ntrip_client ntrip_client.launch.py
ros2 launch mavlink_ros2_bridge mavlink_bridge.launch.py
ros2 launch look_ahead_control look_ahead_following.launch.py
ros2 run maestro_driver maestro_driver_node
```

### シミュレーション

```bash
source /opt/ros/jazzy/setup.bash
source ~/minix_ws/install/setup.bash

ros2 launch bme_simplerover_gazebo sim.launch.py
ros2 launch mavlink_ros2_bridge mavlink_bridge.launch.py
ros2 launch look_ahead_control look_ahead_following.launch.py
```

### systemd による自動起動

実機での運用では systemd を利用して全ノードをOS起動時に自動実行できる。
詳細は [doc/deployment.md](doc/deployment.md) を参照。

## 運用

QGroundControl を使用したミッション作成・自律航行の操作手順については [doc/operation.md](doc/operation.md) を参照。
