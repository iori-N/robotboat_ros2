# systemd によるノード自動起動

systemd を利用して、ローバーの全 ROS2 ノードをOS起動時に自動実行する仕組み。
関連ファイルは全て `deploy/` ディレクトリに格納されている。

GNSSドライバは u-blox 2受信機構成(`ublox_gnss_driver`)から Quectel LG580P
単一受信機構成(`lg580p_gnss_driver`)に移行済み。`ros2-ublox` は
`setup_services.sh` により既定で無効化されるが、ロールバック用にユニット
ファイル・起動スクリプトは残置している(`deploy/README.md`参照)。

## ファイル構成

```
deploy/
├── env.sh                        # 共通環境設定
├── setup_services.sh             # インストーラスクリプト
├── scripts/
│   ├── launch_lg580p.sh          # LG580P GNSSドライバ起動
│   ├── launch_ublox.sh           # (legacy) u-blox GNSSドライバ起動
│   ├── launch_ntrip.sh           # NTRIPクライアント起動
│   ├── launch_mavlink.sh         # MAVLinkブリッジ起動
│   ├── launch_look_ahead.sh      # 経路追従制御起動
│   └── launch_maestro.sh         # サーボドライバ起動
└── systemd/
    ├── ros2-lg580p.service
    ├── ros2-ublox.service        # (legacy, 既定では無効)
    ├── ros2-ntrip.service
    ├── ros2-mavlink.service
    ├── ros2-look-ahead.service
    └── ros2-maestro.service
```

## サービス一覧

| サービス名 | 起動スクリプト | 対応ノード |
|-----------|--------------|-----------|
| `ros2-lg580p` | `launch_lg580p.sh` | lg580p_gnss_driver (LG580P, 位置+ヘディング統合) |
| `ros2-ntrip` | `launch_ntrip.sh` | ntrip_client (RTK補正データ) |
| `ros2-mavlink` | `launch_mavlink.sh` | mavlink_ros2_bridge (QGC連携) |
| `ros2-look-ahead` | `launch_look_ahead.sh` | look_ahead_control (経路追従制御) |
| `ros2-maestro` | `launch_maestro.sh` | maestro_driver (サーボ制御) |

`ros2-ublox` (legacy, ublox_gnss_driver: RTKローバー + Moving Base の
2ノード構成) は既定で無効化されている。ロールバック手順は
`deploy/README.md` を参照。

## 仕組み

### env.sh (共通環境設定)

全起動スクリプトから `source` される共通設定:

- ROS2 Jazzy のセットアップ (`/opt/ros/jazzy/setup.bash`)
- ワークスペースのセットアップ (`~/minix_ws/install/setup.bash`)
- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (ノード検出をローカルに限定)
- `ROS_DOMAIN_ID=0`

### 起動スクリプト (scripts/)

各ノードに対応する起動スクリプト。共通の構造:

1. `env.sh` を `source` して ROS2 環境を読み込む
2. (pyproj が必要なノードのみ) venv を有効化し `PYTHONPATH` を設定
3. `exec` で `ros2 launch` を実行 (PID を引き継ぎ、systemd のプロセス管理に適合)

venv が必要なスクリプト:
- `launch_lg580p.sh` — pyproj 使用
- `launch_look_ahead.sh` — pyproj 使用
- `launch_ublox.sh` (legacy) — pyproj 使用

### systemd ユニットファイル (systemd/)

全サービスに共通の設定:

| 項目 | 値 |
|------|---|
| `User` | kariko |
| `Restart` | always |
| `RestartSec` | 5秒 |
| `After` | network-online.target |
| `StandardOutput/Error` | journal |
| `TimeoutStopSec` | 10秒 |

## インストール

```bash
cd ~/minix_ws/deploy
sudo bash setup_services.sh
```

`setup_services.sh` が行う処理:

1. `env.sh` と `scripts/*.sh` に実行権限を付与
2. legacy `ros2-ublox` を `disable --now` し、`/etc/systemd/system/` のシンボリックリンクを削除 (存在する場合)
3. サービスファイルを `/etc/systemd/system/` にシンボリックリンク
4. `systemctl daemon-reload`
5. 全サービスを `enable` (OS起動時に自動実行)
6. ユーザー `kariko` が `dialout` グループに属しているか確認 (シリアルポートアクセスに必要)

## 操作方法

### 全サービスの起動・停止

```bash
# 起動
sudo systemctl start ros2-lg580p ros2-ntrip ros2-mavlink ros2-look-ahead ros2-maestro

# 停止
sudo systemctl stop ros2-lg580p ros2-ntrip ros2-mavlink ros2-look-ahead ros2-maestro
```

### 個別サービスの操作

```bash
# 状態確認
systemctl status ros2-mavlink

# ログ表示 (リアルタイム)
journalctl -u ros2-mavlink -f

# 再起動
sudo systemctl restart ros2-mavlink
```

### サービスの無効化

```bash
sudo systemctl disable ros2-lg580p ros2-ntrip ros2-mavlink ros2-look-ahead ros2-maestro
```

## 前提条件

- ワークスペースがビルド済みであること (`~/minix_ws/install/` が存在)
- ユーザー `kariko` が `dialout` グループに属していること (シリアルデバイスアクセス用)
  ```bash
  sudo usermod -aG dialout kariko
  # 反映にはログアウト・ログインが必要
  ```
- pyproj が `~/minix_ws/.venv` にインストール済みであること
