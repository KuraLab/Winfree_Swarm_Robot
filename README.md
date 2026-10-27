# Winfree_Swarm_Robot

Winfree 型パルス結合振動子による群ロボット制御の実機コード一式（倉林研究室）。
ESP32 ベースの全方向移動ロボットを、局所的・匿名の IR 結合だけで等間隔に並べるための
ファームウェア、中継サーバ、ダッシュボードを含む。

## 構成

```
firmware/
  Winfree_leader/Winfree_leader.ino        リーダー用
  Winfree_follower/Winfree_follower.ino    フォロワー用
  secrets.h.example                                     Wi-Fi / OTA 設定のひな形
host/
  bridge_server.py            UDP テレメトリ → WebSocket 中継、CSV ログ、OTA、パラメータ配布
  dashboard.html              ブラウザ制御 UI（bridge_server.py が配信）
  robots_config.example.json  MAC→ラベル・役割・役割別デフォルトパラメータ
analysis/                     ログ解析スクリプト置き場（未追加）
hardware/                     基板・CAD・BOM 置き場（未追加）
docs/                         仕様書置き場（未追加）
```

## 環境

- Arduino IDE + ESP32 ボードパッケージ 2.x（2.0.17 を使用）
- ライブラリ: FastLED
- ホスト側: Python 3 と `pip install aiohttp`

## セットアップ

1. 各スケッチフォルダに `firmware/secrets.h.example` を `secrets.h` としてコピーし、
   Wi-Fi の SSID とパスワード（複数可）、leader の OTA パスワードを書く。
   `secrets.h` は Git に入らない。
2. `host/robots_config.example.json` を `host/robots_config.json` にコピーし、
   `espota_path` とロボットの MAC 下位3バイトを自分の環境に合わせる。
   未登録のロボットはテレメトリ受信時に自動追加される。
3. スケッチをコンパイルして書き込む。初回は USB、以後は OTA。
   Arduino IDE ではスケッチ名とフォルダ名が一致している必要がある（上の構成のまま使う）。
4. `cd host && python3 bridge_server.py` を実行し、同じ LAN のブラウザで
   `http://<PCのIP>:8080` を開く。

運用は無線のみで行う。シリアルモニタ・シリアルコマンドは使わない。

## 通信

| 用途 | 方向 | ポート |
|---|---|---|
| テレメトリ（CSV 1行、UDP broadcast） | ロボット → PC | 4210 |
| コマンド（RUN/STOP は broadcast、パラメータはユニキャスト） | PC → ロボット | 4211 |
| ダッシュボード（HTTP / WebSocket） | ブラウザ ↔ PC | 8080 |

テレメトリは leader が41フィールド、follower が77フィールドで、`bridge_server.py` が
パケット長で判別する。フィールド定義は `host/bridge_server.py` の `FIELDS_COMMON` を参照。

## UDP コマンド

`RUN` / `START`（起動）、`STOP`（停止）、それ以外は `<文字> <値>` 形式。
実装は各 `.ino` の該当 `switch` が正で、コメントと食い違う場合は実装を優先する。

| コマンド | leader | follower |
|---|---|---|
| `o` `k` `t` | 固有角振動数ω、結合強度κ、時定数τ | 同左 |
| `g` | 定速駆動 RPM | 速度ゲイン K_P_RPM |
| `P` `I` `D` | PID ゲイン | 同左 |
| `L` | 速度指令の一次遅れ τ_filt | 同左 |
| `F` | なし | 立ち下がり側の τ_filt_fall |
| `K` | なし | ダンパ項ゲイン K_D |
| `Z` | なし | PRC 形状の切替 |
| `f` | LiDAR の CONF_MIN | 同左 |
| `q` | LiDAR スキャン周波数 | 同左 |
| `y` | ヨー保持ゲイン | 同左 |
| `w` | 直進トリム（M3） | なし |
| `s` `G` `x` `a` `j` `v` `p` | なし | D_SAFE、CBF_GAMMA、距離減衰指数、軸固定、横ゲイン、受光閾値、位相 |
| `r` `e` `c` `m` | LED 虹色、IRビーコン、再較正 | LED 虹色、IRビーコン、再較正、モータ有効 |
| `0` | 走行停止（running=false） | 手動モータ試験の解除 |
| `1` `2` `3` | なし | 手動モータ試験（モータ1〜3） |

役割別の推奨値は `host/robots_config.example.json` の `role_defaults` にあり、
ダッシュボードの「現在のリーダー・フォロワー設定を更新」で保存できる。
再起動を検知すると `bridge_server.py` が自動で再送する。

## 注意

- ダッシュボードと `bridge_server.py` は認証無しで、同じ LAN の誰でも OTA や停止指令を送れる。
  研究室内の閉じたネットワークで使うこと。
- follower のファームは OTA パスワード無し（leader は有り）。
- `.bin` と実験ログ CSV はリポジトリに入れない。
