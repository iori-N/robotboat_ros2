# 運用手順

QGroundControl (QGC) を使用した自律航行の操作手順。

## 自律航行の流れ

### 1. ノードの起動

全ノードを起動する。手動起動またはsystemdによる自動起動については [README.md](../README.md) および [deployment.md](deployment.md) を参照。

### 2. QGCでミッション作成

QGCの Plan ビューでウェイポイントミッションを作成する。

- ウェイポイントコマンドは `MAV_CMD_NAV_WAYPOINT` (16) を使用
- 作成したミッションをアップロードする

### 3. ミッション開始

1. QGCからモードを Auto に設定
2. ARM を実行
3. ミッションが開始され、ウェイポイントを順に追従走行する

### 4. ミッション完了

全ウェイポイントに到達すると自動的に停止・DISARM される。

## QGCパラメータ調整

QGCの Parameters ビューから以下のパラメータをリアルタイムに調整できる。変更値は `~/.ros/mavlink_bridge_params.yaml` に永続化される。

### 制御パラメータ

| パラメータ | 説明 | デフォルト |
|-----------|------|-----------|
| `Kp` | 比例ステアリングゲイン | — |
| `Kcte` | クロストラックエラーゲイン | — |
| `Ki` | 積分ゲイン | — |
| `Kd` | 微分ゲイン | — |
| `look_ahead` | Look-ahead距離 (m) | — |
| `pivot_threshold` | ピボットターン閾値 (度) | 40 |
| `wp_arrival_dist` | ウェイポイント到達判定距離 (m) | 0.1 |

### 出力パラメータ

| パラメータ | 説明 | デフォルト |
|-----------|------|-----------|
| `throttle_range` | スロットル範囲 | — |
| `pivot_range` | ピボット範囲 | — |
| `driver_mix` | ミキシングモード (0.0=ノード内, 1.0=パススルー) | 0.0 |
| `pwm_center` | PWM中立値 (us) | 1500 |
| `pwm_min` | PWM最小値 (us) | 1000 |
| `pwm_max` | PWM最大値 (us) | 2000 |
| `steering_reverse` | ステアリング反転 | 0.0 |
| `throttle_reverse` | 前後方向のスロットル反転 | 0.0 |
| `left_motor_reverse` | 左モーター個別反転 (differential時) | 0.0 |
| `right_motor_reverse` | 右モーター個別反転 (differential時) | 0.0 |
