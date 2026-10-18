# INA228 2台のROS記録

## 起動とビルド

ノード・実行ファイルは `power_logger_node`、launchは `power_logger.launch.py`。
systemdのサービスや起動スクリプトの変更は不要。新しいメッセージ型を使うため、
変更後は両パッケージをビルドし、環境を読み直して既存サービスを再起動する。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select bme_common_msgs power_logger
source install/setup.bash
ros2 launch power_logger power_logger.launch.py
```

`port`・`baudrate` のlaunch引数は従来通り。ノードの
`session_publish_period` パラメータは既定5秒（正の有限値）。

## 記録内容

- `/power_log`：`bme_common_msgs/msg/Ina228Measurement`。
  CSVの1行を1メッセージにする。`address` は整数64または65。
  `seq`・`read_start_us`・`read_duration_us`・`epoch` はuint64、
  `status`・`diag` はuint16のまま保存する。
  `bus_voltage` はV、`current` はA、`power` はW、`energy` はJ。
  電流と電力だけCSVから1/1000に換算し、追加校正しない。
  電流の符号、IC電力、異常行、NaNを保持する。`diag != 0` は異常条件ではない。
- `/power_logger/raw`：`PowerLoggerRaw`。受信時刻、セッションID、文字列、
  改行まで受信できたかを保存する。正常行・コメント・空行・不正行も含む。
  UTF-8不正バイトは置換文字にするため、バイト列の完全な複製ではない。
- `/power_logger/session`：`PowerLoggerSession`。開始時刻、開始理由と、
  受信したコメント・ブート行を即時および5秒ごとに配信する。
  測定CSVに設定文字列を挿入しない。設定をESP32へ書き戻す処理もない。

起動設定のスナップショットは最大256行。超過は `settings_truncated` で示し、
全文はrawトピックに流す。`settings_observed` はコメント等の受信有無であり、
必要な設定がすべて取得できたことや実機設定の正しさを保証しない。

## セッションと時刻

`stamp` は完成行をROS側で処理した時刻。ESP32の `read_start_us` を保持し、
正確なADC変換時刻やROS時刻への同期済み時刻とはみなさない。
2台は同じseqで結合せず、ESP32取得時刻で比較する。

セッションIDはUUID。開始理由は次のいずれか。

| reason | 意味 |
|---|---|
| `receiver_start_midstream` | ノード起動。途中受信も許容 |
| `serial_reconnect_unknown_boot` | シリアル再接続。実際のESP32再起動は断定しない |
| `esp32_boot_message` | `rst:` または `ESP-ROM:` 行を検出 |
| `sensor_counter_regression` | 同じセンサーのseqまたは取得時刻が逆行 |

`# INIT` やepochの変化だけではセッションを変更しない。
逆行は受信側で保守的に区切るもので、原因を再起動と断定しない。
ブート文字列が複数回出れば短いセッションができる場合がある。
再接続後は前セッションの設定を引き継がず、未取得として扱う。

## rosbagと解析

既存のARM＋AUTO時の `ros2 bag record -a` を利用する。
後から録画を開始しても、録画中にセッション情報が再配信されれば取得できる。
ノードが起動する前の設定出力や、録画前のraw履歴は復元できない。
再配信より短い録画ではセッション情報を取得できない場合がある。
ローカルファイル保存は行わない。

測定CSVには `/power_log` のみを書き出し、raw・sessionは別に扱う。
旧 `PowerLog.msg` は残すが、`/power_log` の型は変更される。
旧型を指定する購読・解析プログラムは新型へ対応させる必要がある。
リポジトリ内には旧型を読む専用解析プログラムは見つからなかった。
`reference/pc_logger.py` は旧4列仕様の参考資料として残し、新仕様には使わない。

解析では `status == 0` かつ必要な値が有限である測定を使う。
エネルギー差分は同じ `(session_id, address, epoch)` 内で計算し、
積算値の減少は異常として扱う。積算値を各行で合計しない。
整数の識別情報をCSV解析時にfloatへ変換しない。

## 動作確認

```bash
ros2 topic echo /power_log
ros2 topic echo /power_logger/session
ros2 topic echo /power_logger/raw
ros2 bag info <保存したbagのディレクトリ>
```

2台のaddress、符号・単位、異常状態、セッションIDを確認する。
実機の設定確認にはsessionの受信行を参照する。
