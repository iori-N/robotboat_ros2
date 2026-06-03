# 自律走行ローバー ROS2 ワークスペース

RTK-GNSSによる高精度測位を用いた自律走行地上ローバーシステム。
ROS2 Jazzy + Gazebo Harmonic で構成される。

## 目次

- [1. プロジェクト概要](#1-プロジェクト概要)
- [2. システムアーキテクチャ](#2-システムアーキテクチャ)
- [3. パッケージ一覧と役割](#3-パッケージ一覧と役割)
- [4. トピック・データフロー](#4-トピックデータフロー)
- [5. 座標系規約](#5-座標系規約)
- [6. ビルド手順](#6-ビルド手順)
- [7. 実機での起動手順](#7-実機での起動手順)
- [8. シミュレーションでの起動手順](#8-シミュレーションでの起動手順)
- [9. 設計上の注意点](#9-設計上の注意点)

---

## 1. プロジェクト概要

本プロジェクトは、RTK-GNSS測位とPure Pursuit経路追従制御により自律走行する地上ローバーのソフトウェアスタックである。

主な特徴:

- **RTK-GNSS測位**: u-blox F9シリーズ受信機2台 (RTKローバー + Moving Base) によるcm級測位・ヘディング取得
- **NTRIP補正**: NTRIPサーバーからRTCM補正データを受信しRTK Fixを実現
- **QGroundControl連携**: MAVLink v2プロトコルでQGCと通信し、ミッション(ウェイポイント)計画・パラメータ調整・テレメトリ表示を実現。ArduPilot地上ローバーをエミュレート
- **Pure Pursuit制御**: Look-ahead + PI クロストラックエラー補正による経路追従
- **サーボ制御**: Pololu Maestroサーボコントローラ経由でモーター駆動
- **Gazeboシミュレーション**: 実機GNSSの代替として地上真値からGnssSolutionを生成し、実機と同一の制御コードで動作検証可能

---

## 2. システムアーキテクチャ

### 全体データフロー

```
+--------------------+
| QGroundControl     |
| (GCS)              |
+--------+-----------+
         | MAVLink UDP
         | TX:14550 / RX:14551
         v
+--------------------+       +-------------------+
| mavlink_ros2_bridge|       | ntrip_client      |
| (C++)              |       | (Python)          |
|                    |       |                   |
| /mav/mission  -----+--+   | NTRIPサーバー     |
| /mav/modes    -----+  |   |   ↓ TCP 2101      |
| /mav/joystick -----+  |   | /ntrip_rtcm ------+---+
|                    |  |   | ← /gngga          |   |
| ← /auto_log       |  |   +-------------------+   |
| ← /gnss/solution   |  |                           |
+--------------------+  |   +-------------------+   |
                        |   | ublox_gnss_driver |   |
                        |   | (Python)          |   |
                        |   |                   |   |
                        |   | rtk_rover_node    |   |
                        |   |  u-blox RTK ←-----+---+
                        |   |  /gnss/solution   |  RTCM書込
                        |   |  /pvt, /hpposllh  |
                        |   |  /gngga, /utm     |
                        |   |                   |
                        |   | moving_base_node  |
                        |   |  u-blox MB        |
                        |   |  /gnss_odom       |
                        |   |  /heading_imu     |
                        |   |  /relposned       |
                        |   +-------------------+
                        |
                        v
              +--------------------+
              | look_ahead_control |
              | (Python, 10Hz)     |
              |                    |
              | ← /gnss/solution   |
              | ← /gnss_odom       |
              |                    |
              | Pure Pursuit +     |
              | PI CTE補正         |
              |                    |
              | /rc_pwm -----------+--+
              | /cmd_vel           |  |
              | /auto_log          |  |
              +--------------------+  |
                                      v
                            +--------------------+
                            | maestro_driver     |
                            | (Python)           |
                            |                    |
                            | ← /rc_pwm          |
                            | USB Serial →       |
                            | Pololu Maestro     |
                            |   → モーター       |
                            +--------------------+
```

### シミュレーション構成

実機のGNSSパイプラインを以下で置換する:

```
+--------------------+
| Gazebo Harmonic    |
| (スキッドステア)    |
|                    |
| ← /cmd_vel         |
| /ground_truth/odom |
+--------+-----------+
         |
         v
+--------------------+
| fake_gnss_node     |
| (Python)           |
|                    |
| 地上真値 → UTM変換 |
| /gnss/solution     |
+--------------------+
         |
         v
  look_ahead_control
  (実機と同一コード)
```

---

## 3. パッケージ一覧と役割

| パッケージ | 言語 | ビルド | 役割 |
|-----------|------|--------|------|
| `bme_common_msgs` | C++ | ament_cmake | カスタムメッセージ定義 |
| `ublox_gnss_driver` | Python | ament_python | u-blox GNSS UBXプロトコルドライバ |
| `ntrip_client` | Python | ament_python | NTRIP v2クライアント (RTK補正) |
| `mavlink_ros2_bridge` | C++ | ament_cmake | MAVLink/QGCブリッジ |
| `look_ahead_control` | Python | ament_python | Pure Pursuit経路追従制御 |
| `maestro_driver` | Python | ament_python | Pololu Maestroサーボドライバ |
| `bme_simplerover_gazebo` | C++/Python | ament_cmake | Gazeboシミュレーション |

### 3.1 bme_common_msgs

u-blox UBXプロトコル (u-blox F9 HPG 1.50) に基づくカスタムメッセージ定義パッケージ。

**定義メッセージ一覧**:

| メッセージ | UBXクラス/ID | 用途 |
|-----------|-------------|------|
| `PVT.msg` | 0x01 0x07 | 位置・速度・時刻 (Navigation PVT) |
| `HPPOSLLH.msg` | 0x01 0x14 | 高精度緯度経度高度 |
| `RELPOSNED.msg` | 0x01 0x3C | Moving Base相対位置 (NED) |
| `GnssSolution.msg` | — | 統合GNSS解 (受信機非依存) |
| `UTMHP.msg` | — | UTM座標 + 高精度GNSS |
| `AutoLog.msg` | — | 自律走行テレメトリ |
| `MavModes.msg` | — | MAVLinkモード情報 |

`GnssSolution.msg` は受信機に依存しない統合メッセージで、以下のフィールドを含む:
- 測位状態 (`position_rtk_status`: 0=RTKなし, 1=Float, 2=Fixed)
- 高精度緯度経度 (度)
- UTM座標 (easting/northing, メートル)
- ヘディング状態・角度
- 精度情報 (水平・垂直, mm単位)

### 3.2 ublox_gnss_driver

u-blox GNSS受信機のシリアルドライバ。UBXバイナリプロトコルを解析する。

**ノード**:

| ノード | 受信機 | 主な出力 |
|--------|--------|---------|
| `rtk_rover_node` | RTKローバー | `/gnss/solution`, `/pvt`, `/hpposllh`, `/gngga`, `/utm` |
| `moving_base_node` | Moving Base | `/gnss_odom`, `/heading_imu`, `/relposned` |

- `rtk_rover_node` は `/ntrip_rtcm` を購読し、受信したRTCMデータをシリアルポートに書き込む
- `moving_base_node` は相対位置ベクトル (NE) からENUヨー角を算出し、QGCパラメータ `gnss_ofs_fwd`, `gnss_ofs_left`, `gnss_ofs_up`, `gnss_ofs_yaw` によるアンテナ位置から代表点へのオフセットを適用して `/gnss/solution`, `/gnss_odom`, `/heading_imu` を配信
- `/gnss/solution` の緯度経度・UTM座標・方位は補正後代表点を表す。生GNSSデータは `/pvt`, `/hpposllh`, `/relposned`, `/navsatfix`, `/utm`, `/gngga` に残す
- GNSSオフセットの保存値は `~/.ros/mavlink_bridge_params.yaml` が優先され、未保存時は launch 引数の初期値を使用
- UBX解析は `ubx_parser.py` でステートレスに実装 (同期バイト検出 → Fletcher-8チェックサム検証 → struct.unpackでデータクラスに展開)

**パラメータ**:
- `port`: シリアルデバイスパス (例: `/dev/serial/by-id/usb-u-blox...`)
- `baudrate`: RTKローバー 115200, Moving Base 38400
- `frame_id`: TFフレーム名

### 3.3 ntrip_client

NTRIP v2プロトコルでRTK補正データ (RTCM) を取得するクライアント。

- NTRIPサーバーにTCP接続し、RTCMストリームを `/ntrip_rtcm` (UInt8MultiArray) として配信
- `/gngga` (String) を購読し、NMEA GGA文をサーバーに送信 (VRS等の位置フィードバック)
- ソケットエラー時は2秒間隔で自動再接続

**パラメータ** (`config/ntrip_params.yaml`):
- `ntrip_address`: サーバーホスト名
- `ntrip_port`: ポート (標準: 2101)
- `ntrip_username` / `ntrip_password`: 認証情報
- `ntrip_mountpoint`: 補正ストリーム名

### 3.4 mavlink_ros2_bridge

MAVLink v2 UDPブリッジ。QGroundControlに対してArduPilot地上ローバーをエミュレートする。

- **車両ID**: MAV_TYPE_GROUND_ROVER, sysid=1
- **UDP**: ローカルRX 14551 → GCS TX 14550
- `/gnss/solution` と `/auto_log` を購読し、GPS_RAW_INT, ATTITUDE, LOCAL_POSITION_NED等のMAVLinkメッセージとしてGCSへ送信
- GCSからのミッションダウンロード結果を `/mav/mission` (Float64MultiArray) として配信
- ARM/DISARM状態、モード変更を `/mav/modes` (MavModes) として配信
- GCSからのジョイスティック入力を `/mav/joystick` (TwistStamped) として配信

**QGCチューニングパラメータ** (MAVLink PARAM_SETで変更可能):
- 制御系: `Kp`, `Kcte`, `Ki`, `Kd`, `look_ahead`, `pivot_threshold`, `wp_arrival_dist` 等
- 出力系: `throttle_range`, `pivot_range`, `driver_mix`, `pwm_center/min/max`, `steering_reverse`, `left_motor_reverse`, `right_motor_reverse` 等
- `~/.ros/mavlink_bridge_params.yaml` に永続化

### 3.5 look_ahead_control

Pure Pursuit経路追従制御ノード。PIクロストラックエラー (CTE) 補正付き。

- 10Hzの制御ループをメインスレッドで実行
- `/mav/mission` からウェイポイントを取得、`/mav/modes` でARM + ミッション開始を待機
- `/gnss/solution` または `/gnss_odom` から現在位置を取得 (`odom_source` パラメータで選択)
- 制御出力: `/rc_pwm` (UInt16MultiArray, [ch1, ch2]) と `/cmd_vel` (Twist)
- テレメトリ: `/auto_log` (AutoLog)

**制御パラメータ**:
- `Kp`: 比例ステアリングゲイン
- `Kcte`: クロストラックエラーゲイン
- `Ki` / `Kd`: 積分・微分ゲイン
- `look_ahead`: Look-ahead距離 (m)
- `pivot_threshold`: ピボットターン閾値 (度, デフォルト 40)
- `wp_arrival_dist`: ウェイポイント到達判定距離 (m, デフォルト 0.1)
- `driver_mix`: 0.0=ノード内でL/R差動ミキシング, 1.0=パススルー

**状態遷移**:
1. ウェイポイント受信待ち
2. ARM + ミッション開始待ち
3. 経路追従制御実行 (座標変換 → 方位角計算 → PIステアリング → PWM出力)
4. ウェイポイント到達で次点へ進行、全完了で停止

### 3.6 maestro_driver

Pololu Maestroサーボコントローラのドライバ。ROS2 RC PWMをMaestro Compact Protocolに変換する。

- `/rc_pwm` (BestEffort QoS, depth=1) を購読
- PWMマイクロ秒 × 4 = 1/4マイクロ秒ターゲット値としてUSBシリアル送信
- フェイルセーフ: 0.5秒間メッセージ途絶で中立PWM (1500us) を送信
- シャットダウン時もフェイルセーフPWMを送信
- シリアル切断時は1秒間隔で自動再接続

**パラメータ**:
- `serial_port`: Maestroデバイスパス
- `ch1_channel` / `ch2_channel`: Maestroチャンネル番号
- `failsafe_timeout`: タイムアウト秒数 (デフォルト 0.5)
- `pwm_min` / `pwm_max`: PWMクランプ範囲 (1000-2000us)

### 3.7 bme_simplerover_gazebo

Gazebo Harmonicによるスキッドステアローバーシミュレーション。

- **URDFモデル** (`urdf/rover.urdf.xacro`): 4輪スキッドステア (base_footprint → base_link → 4 wheels)
- **Gazeboプラグイン**:
  - DiffDrive: `/cmd_vel` → 車輪駆動
  - OdometryPublisher: `/ground_truth/odom` (物理エンジン直接、スリップなし)
  - JointStatePublisher: `/joint_states`
- **ros_gz_bridge** (`config/bridge.yaml`): ROS2-Gazebo間トピック変換
- **fake_gnss_node**: `/ground_truth/odom` → `/gnss/solution` (UTM座標に変換)
- シミュレーションの `/gnss/solution` には、現時点では実機用GNSSオフセットパラメータ `gnss_ofs_*` を反映しない
- **teleop_key_node**: キーボード操作 (w/a/s/d/q/e)

---

## 4. トピック・データフロー

### 主要トピック一覧

| トピック | 型 | Publisher | Subscriber | 説明 |
|---------|---|----------|-----------|------|
| `/gnss/solution` | `GnssSolution` | rtk_rover_node / fake_gnss_node | mavlink_bridge, look_ahead_control, moving_base_node | 統合GNSS解 |
| `/pvt` | `PVT` | rtk_rover_node | — | UBX-NAV-PVT |
| `/hpposllh` | `HPPOSLLH` | rtk_rover_node | — | 高精度位置 |
| `/gngga` | `String` | rtk_rover_node | ntrip_client | NMEA GGA文 |
| `/ntrip_rtcm` | `UInt8MultiArray` | ntrip_client | rtk_rover_node | RTCM補正データ |
| `/relposned` | `RELPOSNED` | moving_base_node | — | 相対位置 (NED) |
| `/heading_imu` | `Imu` | moving_base_node | — | ヘディング (四元数) |
| `/gnss_odom` | `Odometry` | moving_base_node | look_ahead_control | GNSS測位 + ヘディング |
| `/utm` | `Odometry` | rtk_rover_node | — | UTM座標オドメトリ |
| `/mav/mission` | `Float64MultiArray` | mavlink_bridge | look_ahead_control | ミッションWP [seq, total, cmd, lat, lon] |
| `/mav/modes` | `MavModes` | mavlink_bridge | look_ahead_control | ARM/モード状態 |
| `/mav/joystick` | `TwistStamped` | mavlink_bridge | — | GCSジョイスティック |
| `/mav/mission_set_current` | `UInt16` | mavlink_bridge | — | アクティブWPインデックス |
| `/rc_pwm` | `UInt16MultiArray` | look_ahead_control | maestro_driver | RC PWM [ch1, ch2] |
| `/cmd_vel` | `Twist` | look_ahead_control / teleop | Gazebo DiffDrive | 速度指令 |
| `/auto_log` | `AutoLog` | look_ahead_control | mavlink_bridge | テレメトリ |
| `/ground_truth/odom` | `Odometry` | Gazebo | fake_gnss_node | シミュレーション地上真値 |

### データフロー図 (RTKパイプライン)

```
NTRIPサーバー ──TCP──> ntrip_client ──/ntrip_rtcm──> rtk_rover_node ──シリアル書込──> u-blox
                         ^                              |
                         |                              v
                     /gngga (GGA位置フィードバック)    /gnss/solution
```

### データフロー図 (制御パイプライン)

```
QGC ──MAVLink UDP──> mavlink_bridge ──/mav/mission, /mav/modes──> look_ahead_control
                         ^                                             |
                         |                                             v
                     /auto_log, /gnss/solution                     /rc_pwm
                                                                       |
                                                                       v
                                                                maestro_driver
                                                                       |
                                                                       v
                                                                    モーター
```

---

## 5. 座標系規約

本プロジェクトでは **ENU座標系** (REP-103準拠) を採用する。

| 軸 | 方向 |
|---|------|
| X | 東 (East) |
| Y | 北 (North) |
| Z | 上 (Up) |

**ヨー角**:
- 東 = 0度
- 北 = 90度
- 反時計回り (CCW) が正

**MAVLinkとの変換**:
MAVLinkはNED座標系 (North-East-Down) を使用するため、`mavlink_ros2_bridge` 内でENU⇔NED変換を行う。

**UTM座標**:
- `pyproj` を使用して緯度経度からUTM座標に変換
- UTMゾーンは初回受信時に自動決定しキャッシュ

---

## 6. ビルド手順

### 前提条件

- ROS2 Jazzy
- Gazebo Harmonic (シミュレーション使用時)
- Python 3 + pyproj (venv内)

### ビルド手順

```bash
# 1. ROS2環境のセットアップ
source /opt/ros/jazzy/setup.bash

# 2. pyproj用venvの有効化
#    (ublox_gnss_driver, look_ahead_control のビルドに必要)
source ~/ros2_ws/.venv/bin/activate

# 3. bme_common_msgs を先にビルド (他パッケージが依存)
cd ~/ros2_ws
colcon build --packages-select bme_common_msgs
source install/setup.bash

# 4. 残りのパッケージをビルド
colcon build
source install/setup.bash
```

**注意事項**:
- `bme_common_msgs` は他パッケージのメッセージ依存元のため、必ず先にビルドすること
- `pyproj` は `rosdep` ではなく venv にインストールされている。`package.xml` に追加しないこと
- pyproj依存パッケージのビルドにはvenv pythonが必要:
  ```bash
  /home/kikai/ros2_ws/.venv/bin/python3 -m colcon build --packages-select <package_name>
  ```

### 単体パッケージのビルド

```bash
colcon build --packages-select <package_name>
```

### テスト (ament lint)

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```

機能テストは存在せず、ament lint (flake8, pep257, copyright) のみ。

---

## 7. 実機での起動手順

各ノードを別ターミナルで起動する。

```bash
# 共通: 環境セットアップ (各ターミナルで実行)
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 1. GNSS ドライバ (RTKローバー + Moving Base)
ros2 launch ublox_gnss_driver ublox_gnss.launch.py

# 2. NTRIP クライアント (RTK補正データ)
ros2 launch ntrip_client ntrip_client.launch.py

# 3. MAVLink ブリッジ (QGroundControl連携)
ros2 launch mavlink_ros2_bridge mavlink_bridge.launch.py

# 4. 経路追従制御
ros2 launch look_ahead_control look_ahead_following.launch.py

# 5. サーボドライバ
ros2 run maestro_driver maestro_driver_node
```

**運用フロー**:
1. 上記ノードを全て起動
2. QGroundControlでミッション (ウェイポイント) を作成・アップロード
3. QGCからミッション開始 + ARM
4. ローバーがウェイポイントを順に追従走行
5. 全ウェイポイント到達で自動停止・DISARM

---

## 8. シミュレーションでの起動手順

```bash
# 共通: 環境セットアップ
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 1. Gazeboシミュレーション起動
#    (ローバーモデル + fake_gnss_node + ros_gz_bridge を含む)
ros2 launch bme_simplerover_gazebo sim.launch.py

# 2. MAVLink ブリッジ
ros2 launch mavlink_ros2_bridge mavlink_bridge.launch.py

# 3. 経路追従制御
ros2 launch look_ahead_control look_ahead_following.launch.py

# (オプション) キーボード操作
ros2 run bme_simplerover_gazebo teleop_key_node
```

シミュレーションでは `fake_gnss_node` が Gazeboの地上真値オドメトリ (`/ground_truth/odom`) を `GnssSolution` に変換するため、実機のGNSSドライバ・NTRIPクライアント・サーボドライバは不要。

---

## 9. 設計上の注意点

### PWM規約

- 中立: 1500us
- 範囲: 1000-2000us
- データ形式: `UInt16MultiArray` の `data[0]=ch1`, `data[1]=ch2`
- `driver_mix=0.0` の場合、`look_ahead_control` がL/R差動ミキシングを実施
- `driver_mix=1.0` の場合、ドライバ側でミキシング (パススルー)

### フェイルセーフ

- **maestro_driver**: `/rc_pwm` が0.5秒途絶すると中立PWM (1500us) を自動送信。シャットダウン時もフェイルセーフ動作
- **ublox_gnss_driver**: シリアル切断時は自動再接続
- **ntrip_client**: ソケットエラー時は2秒間隔で自動再接続

### ArduPilotモード定数

MAVLinkプロトコルで使用されるArduPilotのモード定数:

| 状態 | base_mode | custom_mode |
|------|-----------|-------------|
| Armed + Guided | 217 | — |
| Disarmed + Guided | 89 | — |
| Auto | — | 10 |

### パラメータ永続化

- `mavlink_ros2_bridge` はQGCから設定されたパラメータを `~/.ros/mavlink_bridge_params.yaml` に保存
- `look_ahead_control` は起動時にこのファイルからパラメータを読み込む
- 実行中のパラメータ同期は `/parameter_events` トピック経由で行われる

### Pythonスレッディングパターン

全Pythonノードで共通のパターンを採用:
- **メインスレッド**: ブロッキングI/O (シリアル読み取り) または制御ループ
- **デーモンスレッド**: `rclpy.spin()` によるROS2コールバック処理

### メッセージ命名規則 (ROS2 Jazzy)

- ファイル名: PascalCase (例: `PVT.msg`)
- フィールド名: snake_case (例: `lon_deg`)
- 違反するとビルドエラーになる

### MAVLink Cライブラリ

MAVLink Cライブラリは `mavlink_ros2_bridge` の git submodule として `mavlink_ros2_bridge/include/c_library_v2/` に配置される。ワークスペース取得時は `vcs import --recursive src < robotboat.repos` で submodule も取得すること。CMakeではSYSTEMヘッダとしてインクルードされる。

### QGroundControl連携時の注意

- `MISSION_COUNT` は `mission_type == 0` でフィルタすること (fence/rallyのカウントで上書きされる問題を回避)
- `SET_MODE` と `COMMAND_LONG` の両方でモード変更が来る可能性があり、ARM状態を保持する必要がある
- ブリッジのパラメータスコープはブリッジ内のみ。下流ノードへは `/parameter_events` で同期
