# systemd 自動起動サービス

自律走行ローバーの6つの ROS2 ノードを systemd で起動時に自動実行する。

GNSSドライバは u-blox 2受信機構成(`ublox_gnss_driver`)から Quectel LG580P
単一受信機構成(`lg580p_gnss_driver`)に移行済み。`setup_services.sh` は
`ros2-lg580p` を有効化し、旧 `ros2-ublox` は自動的に無効化・停止される
(ロールバック用にユニットファイル・起動スクリプト自体は残置)。

## ファイル構成

| ファイル | 役割 |
|------|------|
| `env.sh` | 共通環境設定: ROS2 + ワークスペースの source、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` |
| `scripts/launch_*.sh` | 各ノードの起動スクリプト（現行6つ + legacy `launch_ublox.sh`）。lg580p と look_ahead (と legacy ublox) は venv も activate。全て `exec` で PID を引き継ぎ |
| `systemd/ros2-*.service` | systemd ユニットファイル（現行6つ: lg580p/ntrip/mavlink/look-ahead/maestro/power-logger。旧 `ros2-ublox.service` はロールバック用に残置、既定では無効） |
| `setup_services.sh` | インストーラ: 権限付与、旧`ros2-ublox`の無効化、シンボリックリンク、daemon-reload、enable、dialout 確認 |

## インストール方法

```bash
cd ~/minix_ws/deploy && sudo bash setup_services.sh
```

## テスト方法

```bash
# 単体テスト
sudo systemctl start ros2-mavlink
journalctl -u ros2-mavlink -f

# 全サービス起動/停止
sudo systemctl start ros2-lg580p ros2-ntrip ros2-mavlink ros2-look-ahead ros2-maestro ros2-power-logger
sudo systemctl stop ros2-lg580p ros2-ntrip ros2-mavlink ros2-look-ahead ros2-maestro ros2-power-logger
```

## u-blox 構成へのロールバック

`ros2-ublox.service` と `scripts/launch_ublox.sh` は削除していない。手動で戻す場合:

```bash
sudo systemctl disable --now ros2-lg580p
sudo ln -sf ~/minix_ws/deploy/systemd/ros2-ublox.service /etc/systemd/system/ros2-ublox.service
sudo systemctl daemon-reload
sudo systemctl enable --now ros2-ublox
```
