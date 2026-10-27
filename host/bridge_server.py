#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bridge_server.py — Winfree群ロボット ダッシュボード中継サーバー（直列実験対応版）

役割:
  1) 各ロボットがUDP broadcast(port 4210)で送るテレメトリを受信し、
     ブラウザへWebSocketで中継する（ブラウザは生UDPを受信できないため）。
  2) 受信CSVログを従来のPythonモニタと同様に残す。
  3) ダッシュボードからのコマンドを受けて:
       - 一斉起動: UDP broadcast "RUN"（互換のため"START"も送信）を打つ
       - 一斉停止: UDP broadcast "STOP" を打つ（発光・移動停止。キャリブレーション値は保持）
       - パラメータ変更: 対象ロボットのIP:CMD_PORT(4211)へユニキャストでコマンド文字列を送る
       - OTA: espota.py をサブプロセスで叩いて.binを書き込み、進捗をWebSocketで返す

  直列実験用に、テレメトリのフィールド数を34→37→40→41に拡張:
    34: actual_frame_dt_ms（フォロワーの実測フレーム周期[ms]。FRAME_DT調整用の診断値）
    35: chain_fwd_dist_m（前方(0°)ゾーンの最近傍距離[m]、直列実験用。未検出時99.9）
    36: chain_back_dist_m（後方(180°)ゾーンの最近傍距離[m]、直列実験用。未検出時99.9）
    37: chain_fwd_conf_avg（前方ゾーンの生bin_conf平均。CONF_MIN判定前の値。データ無ければ-1）
    38: chain_back_conf_avg（後方ゾーンの生bin_conf平均。同上）
    39: conf_min_current（そのロボットが現在使っているCONF_MINの値。fコマンドで変更可）
    40: yaw_est_deg（3輪エンコーダのデッドレコニングで推定した機体ヨー角[deg]。
        RUN受信時0リセットの相対角。IMU非搭載のためYAW_HOLD_KPでの補正診断用）

  さらにfollowerのみ41→77に拡張（センサ非対称性の診断用。cal_V/raw_Vという概念自体を
  持たないleaderは41フィールドのまま）:
    41-52: rawV0-11（各chの生電圧。cal_V/V_base正規化前の実測値そのもの）
    53-64: calV0-11（各chの自己反射/2体較正キャリブ値）
    65-76: baseV0-11（各chのベースライン）
  leader(41)とfollower(77)はUDPパケットのフィールド数で自動判別する
  （FIELDS_LEADER / FIELDS_FOLLOWER）。CSVログは列を77列に揃え、leader由来の行は
  rawV/calV/baseV列を空欄で出力する。

必要パッケージ: pip install aiohttp --break-system-packages

起動:
  python3 bridge_server.py
  → http://<このPCのIP>:8080 をロボットと同じLAN上のブラウザで開く

設定ファイル: robots_config.json （同じディレクトリに置く。無ければ初回起動時に空で自動生成）
"""

import asyncio
import csv
import json
import re
import socket
import sys
import time
from pathlib import Path

from aiohttp import web

# ================================================================
#  設定
# ================================================================
BASE_DIR      = Path(__file__).parent.resolve()
CONFIG_PATH   = BASE_DIR / "robots_config.json"
LOG_DIR       = BASE_DIR / "logs"
UPLOAD_DIR    = BASE_DIR / "uploads"
DASHBOARD_HTML = BASE_DIR / "dashboard.html"

UDP_TELEMETRY_PORT = 4210   # ロボット→サーバー（既存のbroadcast telemetry、及びRUN/STOP broadcast送信先）
UDP_CMD_PORT        = 4211   # サーバー→ロボット（パラメータ変更コマンド、ユニキャスト）
HTTP_PORT           = 8080

STALE_SEC = 5.0   # この秒数テレメトリが来なければ「オフライン」とみなす（winfree_sensor_monitor.pyのalive判定(5.0s)に合わせた）

# テレメトリ41フィールドの定義。winfree_sensor_monitor.py由来の32フィールドに、
# IRゲートで実体確認済みの相手ロボット距離(peer_bearing_deg, peer_dist_m)、
# さらにセンサフレーム周期の実測値(actual_frame_dt_ms、FRAME_DT調整用の診断値)、
# 直列実験用の前方/後方チェーン距離(chain_fwd_dist_m, chain_back_dist_m)、
# LiDARの生confidence平均とCONF_MIN現在値(chain_fwd_conf_avg, chain_back_conf_avg,
# conf_min_current。反射材の効果検証・検出不安定域の診断用)、
# IMU非搭載のため3輪エンコーダのデッドレコニングで推定した機体ヨー角(yaw_est_deg。
# RUN受信時0リセットの相対角、YAW_HOLD_KPでの補正診断用)を追加。
# nearest_*は障害物全般（壁等も含む）、peer_*はロボットだと確認できたものだけ。
# leader/follower共通の先頭41フィールド。
FIELDS_COMMON = [
    "mac", "t_robot", "phi", "active", "sumV",
    "V0", "V1", "V2", "V3", "V4", "V5", "V6", "V7", "V8", "V9", "V10", "V11",
    "raw_sumV",
    "rpm0", "rpm1", "rpm2",
    "tgt0", "tgt1", "tgt2",
    "pwm0", "pwm1", "pwm2",
    "n_tracks_alive", "nearest_bearing_deg", "nearest_dist_m", "cbf_active", "gate_reject_cnt",
    "peer_bearing_deg", "peer_dist_m",
    "actual_frame_dt_ms",
    "chain_fwd_dist_m", "chain_back_dist_m",
    "chain_fwd_conf_avg", "chain_back_conf_avg", "conf_min_current",
    "yaw_est_deg",
]
assert len(FIELDS_COMMON) == 41, "ファームウェアのsendTelemetry()共通部と同じ41フィールドである必要があります"

# followerのみ追加で送ってくる36フィールド（各ch0-11の生電圧・キャリブ値・ベースライン）。
# leaderはcal_V/raw_Vという概念自体を持たないため送信しない。
FIELDS_FOLLOWER_EXTRA = (
    [f"rawV{j}" for j in range(12)] +
    [f"calV{j}" for j in range(12)] +
    [f"baseV{j}" for j in range(12)]
)
assert len(FIELDS_FOLLOWER_EXTRA) == 36

FIELDS_LEADER   = FIELDS_COMMON                       # 41フィールド
FIELDS_FOLLOWER = FIELDS_COMMON + FIELDS_FOLLOWER_EXTRA  # 77フィールド
# CSV/ダッシュボード側で扱う列の全体集合（重複無しの和集合、順序はfollowerの並びを優先）
FIELDS = FIELDS_FOLLOWER

LOG_DIR.mkdir(exist_ok=True)
UPLOAD_DIR.mkdir(exist_ok=True)


# ================================================================
#  設定ファイル（robots_config.json）
# ================================================================
def load_config():
    if not CONFIG_PATH.exists():
        default = {"espota_path": "", "robots": {}, "role_defaults": {"leader": {}, "follower": {}}}
        CONFIG_PATH.write_text(json.dumps(default, indent=2, ensure_ascii=False), encoding="utf-8")
        return default
    try:
        return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    except Exception as e:
        print(f"[CONFIG][!] robots_config.json の読み込みに失敗: {e} → デフォルト値で続行")
        return {"espota_path": "", "robots": {}}


def save_config(cfg):
    CONFIG_PATH.write_text(json.dumps(cfg, indent=2, ensure_ascii=False), encoding="utf-8")


config = load_config()
config.setdefault("role_defaults", {"leader": {}, "follower": {}})
config["role_defaults"].setdefault("leader", {})
config["role_defaults"].setdefault("follower", {})


def robot_meta(mac_id: str) -> dict:
    """robots_config.json内の該当ロボット設定を返す。未登録なら自動追加して返す。
    role未指定のロボットは安全側に倒してfollower扱いにする（leaderは明示設定が必要）。
    lidar_hz: LiDARスキャン周波数の手動指定[Hz]（未設定=None なら、ラベル中の番号nから
    自動算出したデフォルトを使う。詳細はauto_lidar_hz_for_label()参照）。"""
    entry = config["robots"].get(mac_id)
    if entry is None:
        entry = {"label": mac_id, "ota_password": "", "role": "follower", "lidar_hz": None}
        config["robots"][mac_id] = entry
        save_config(config)
    entry.setdefault("role", "follower")
    entry.setdefault("lidar_hz", None)
    return entry


# ================================================================
#  LiDAR周波数の自動割当（"Robot n"のnに基づく混信対策）
#    LD19のスキャン周波数仕様は5〜13Hz。奇数番目=10Hz・偶数番目=13Hzと交互に
#    振ることで、物理的に隣接するロボット同士（n と n+1）は必ず仕様内で
#    最大の3Hz差になり、混信（ビート）の周期を最短化できる。
#    n はラベル文字列中の最初の数字列から抽出する（dashboard.htmlの並び替え
#    ロジック /(\d+)/ と同じ規約。"Robot01"→1, "Robot5"→5）。
# ================================================================
_LABEL_NUM_RE = re.compile(r"(\d+)")


def extract_robot_number(label: str):
    """ラベル文字列から番号n(int)を抽出する。数字が無ければNoneを返す。"""
    if not label:
        return None
    m = _LABEL_NUM_RE.search(label)
    return int(m.group(1)) if m else None


# LiDARスキャン周波数の自動割当パラメータ。
#   下限7Hz: LD19仕様は5〜13Hzだが、BIN_MAX_AGE_TICKS(ファームウェア側、150ms)の
#            鮮度判定と両立するには1回転が150ms以内＝約6.67Hz以上必要。
#            それを下回ると毎回転で鮮度切れの隙間が生じうるため、余裕を見て7Hzを下限にする。
#   上限13Hz: LD19仕様の上限そのもの。
#   7〜13Hzの整数7段階＝現在の最大編成数(7台)とちょうど一致し、番号nが1〜7の範囲なら
#   全ロボットで重複なく1台1Hzずつ割り当てられる。8台目以降は7で周回し重複が発生しうる
#   （物理的な台数を増やす場合はLIDAR_HZ_MIN/MAXの幅を広げて対応すること）。
LIDAR_HZ_MIN = 7.0
LIDAR_HZ_MAX = 13.0
LIDAR_HZ_STEPS = int(LIDAR_HZ_MAX - LIDAR_HZ_MIN) + 1   # =7


def auto_lidar_hz_for_label(label: str):
    """ラベルの番号nから自動的にLiDAR周波数[Hz]を決める。抽出できなければNone
    （呼び出し側はファームウェア既定のMACハッシュ値にフォールバックさせる）。
    n=1..LIDAR_HZ_STEPS の範囲なら全ロボットでユニークな値になる。"""
    n = extract_robot_number(label)
    if n is None:
        return None
    return LIDAR_HZ_MIN + float((n - 1) % LIDAR_HZ_STEPS)


def resolve_lidar_hz(meta: dict):
    """そのロボットが実際に使うべきLiDAR周波数を決定する。
    手動指定(lidar_hz)があれば最優先、無ければラベル番号からの自動算出値、
    それも取れなければNone（ファームウェア既定=MACハッシュに任せる）を返す。"""
    if meta.get("lidar_hz") is not None:
        return meta["lidar_hz"], False   # (値, auto=False手動)
    auto_hz = auto_lidar_hz_for_label(meta.get("label", ""))
    return auto_hz, True                  # (値 or None, auto=True自動)


def check_lidar_hz_collisions():
    """robots_config.json登録済みの全ロボットについてresolve_lidar_hz()を計算し、
    同じ値になっているペアが無いかを検査してコンソールに警告する。
    rename/set_lidar_hz/起動時など、割当が変わりうるタイミングで毎回呼ぶこと。
    n が LIDAR_HZ_STEPS(=7)を超えて周回した場合や、手動指定と自動算出が偶然
    衝突した場合など、番号だけでは防ぎきれないケースをここで実際に検出する。"""
    by_hz = {}
    for mac_id, meta in config["robots"].items():
        hz, _ = resolve_lidar_hz(meta)
        if hz is None:
            continue
        by_hz.setdefault(hz, []).append(meta.get("label", mac_id))
    collided = {hz: labels for hz, labels in by_hz.items() if len(labels) > 1}
    if collided:
        for hz, labels in collided.items():
            print(f"[LIDAR-HZ][!] 周波数{hz}Hzが重複しています: {', '.join(labels)}  "
                  f"→ 手動指定(lidar_hz)で片方をずらしてください")
    return collided


# ================================================================
#  ロボット状態（メモリ上、mac_id → dict）
# ================================================================
robots = {}          # mac_id -> {"ip":..., "last_seen": float, "data": {...}}
ws_clients = set()    # 接続中のブラウザWebSocket


async def broadcast_ws(msg: dict):
    if not ws_clients:
        return
    text = json.dumps(msg, ensure_ascii=False)
    dead = []
    for ws in ws_clients:
        try:
            await ws.send_str(text)
        except Exception:
            dead.append(ws)
    for ws in dead:
        ws_clients.discard(ws)


# ================================================================
#  CSVロガー（ダッシュボードの「ログ開始/停止」ボタンで制御する）
#    winfree_sensor_monitor.pyと同じファイル名規則・ヘッダ順（開始するたびに1ファイル）。
#    末尾に'ip'列だけ追加しているが、先頭からの列順・列名は完全互換。
#    プロセス起動時点では記録しない（logging_active=False）。ダッシュボードの
#    「● ログ開始」ボタンを押した時点から新しいCSVファイルへの書き込みが始まる。
# ================================================================
CSV_HEADER = ["utc_time"] + FIELDS + ["ip"]
_csv_file = None
_csv_writer = None
logging_active = False


def open_csv_logger():
    global _csv_file, _csv_writer
    fname = LOG_DIR / time.strftime("winfree_log_%Y%m%d_%H%M%S.csv")
    _csv_file = open(fname, "a", newline="", encoding="utf-8")
    _csv_writer = csv.writer(_csv_file)
    _csv_writer.writerow(CSV_HEADER)
    _csv_file.flush()
    print(f"[LOG] writing to {fname}")
    return _csv_writer


def start_logging() -> str:
    """新しいCSVファイルを開いて記録を開始する。既に記録中なら一旦停止してから開き直す
    （実験を区切ってログファイルを分けたい場合、開始ボタンを押し直すだけでよい）。"""
    global logging_active
    if logging_active:
        stop_logging()
    open_csv_logger()
    logging_active = True
    fname = str(_csv_file.name) if _csv_file else ""
    print(f"[LOG] logging STARTED -> {fname}")
    return fname


def stop_logging():
    """記録を停止し、ファイルをクローズする。"""
    global logging_active, _csv_file, _csv_writer
    logging_active = False
    if _csv_file:
        try:
            _csv_file.close()
        except Exception:
            pass
        print(f"[LOG] logging STOPPED -> {_csv_file.name}")
    _csv_file = None
    _csv_writer = None


# ================================================================
#  UDPテレメトリ受信（asyncio DatagramProtocol）
# ================================================================
class TelemetryProtocol(asyncio.DatagramProtocol):
    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        try:
            text = data.decode("utf-8", errors="ignore").strip()
        except Exception:
            return
        if not text or text.startswith("BT_MAC:"):
            return  # PS4ペアリング用のBT MAC通知などは無視
        if text in ("RUN", "START", "STOP"):
            return  # 自分自身が送ったRUN/STOP broadcastがループバックしてくる場合があるので無視

        parts = text.split(",")
        # leader(41フィールド)とfollower(77フィールド。raw_V/cal_V/base_V追加分含む)を
        # パケット長で判別する。どちらの長さにも一致しないパケットはサイレントにdropする
        # （ロボット側ファームウェアのバージョンがまだ古い＝フィールド数が少ない場合もここで弾かれる。
        #   ダッシュボードにロボットが出てこない場合は、まずこの不一致を疑うこと）
        if len(parts) == len(FIELDS_FOLLOWER):
            fields_for_packet = FIELDS_FOLLOWER
        elif len(parts) == len(FIELDS_LEADER):
            fields_for_packet = FIELDS_LEADER
        else:
            return

        record = {}
        for name, raw in zip(fields_for_packet, parts):
            if name == "mac":
                record[name] = raw
            else:
                try:
                    record[name] = float(raw)
                except ValueError:
                    record[name] = 0.0
        # leaderパケット由来でrawV/calV/baseVが無い場合はCSV列を揃えるためNoneで埋める
        # （空欄で出力。dashboard.html側もこのキーが無い/Noneの場合は表示をスキップする）
        for name in FIELDS_FOLLOWER_EXTRA:
            record.setdefault(name, None)

        mac_id = record["mac"]
        ip = addr[0]
        now = time.time()

        meta = robot_meta(mac_id)
        entry = robots.setdefault(mac_id, {})
        entry["ip"] = ip
        entry["last_seen"] = now
        entry["data"] = record
        entry["label"] = meta.get("label", mac_id)
        entry["role"] = meta.get("role", "follower")

        # 再起動検知: t_robot(ロボット側millis()/1000。起動からの経過秒)が前回受信時より
        # 大きく巻き戻っていたら再起動と判断する。ノイズ(パケット順序入れ替わり等)で
        # 誤検知しないよう1秒以上の巻き戻りのみを対象にする。
        # 再起動直後はlidar_hz_override(RAM上の値)が消えて既定のMACハッシュ値に戻っている
        # ため、resolve_lidar_hz()（ラベル番号からの自動算出 or 手動指定）を無線のみで
        # 自動的に再送する。
        prev_t_robot = entry.get("last_t_robot")
        this_t_robot = record.get("t_robot", 0.0)
        rebooted = (prev_t_robot is not None) and (this_t_robot < prev_t_robot - 1.0)
        entry["last_t_robot"] = this_t_robot
        if rebooted:
            hz, is_auto = resolve_lidar_hz(meta)
            if hz is not None:
                send_param(mac_id, f"q {hz}")
                src = "自動(ラベル番号)" if is_auto else "手動指定"
                print(f"[LIDAR-HZ] {mac_id} 再起動を検知 → {hz}Hz を自動再送（{src}）")

            # 保存済みの役割別デフォルトパラメータも同時に再送する。
            # これにより「電源を入れ直したらパラメータを1個ずつ手動で設定し直す」作業が不要になる
            # （ダッシュボードの「現在のリーダー・フォロワー設定を更新」ボタンで事前に保存しておくこと）。
            # datagram_received()は同期コールバックなのでawaitできないため、
            # 間隔を空けた送信(send_params_sequential)はバックグラウンドタスクとして起動する。
            role = meta.get("role", "follower")
            defaults = config.get("role_defaults", {}).get(role, {})
            if defaults:
                asyncio.create_task(send_params_sequential(mac_id, defaults))
                print(f"[PARAM-DEFAULT] {mac_id}({role}) 再起動を検知 → 保存済みパラメータ{len(defaults)}件を自動再送（順次送信開始）")

        if logging_active and _csv_writer is not None:
            try:
                utc_time = time.strftime("%Y-%m-%d %H:%M:%S")
                row = [record.get(f) for f in FIELDS]  # FIELDS=FIELDS_FOLLOWER（全列の和集合）
                row = ["" if v is None else v for v in row]
                _csv_writer.writerow([utc_time] + row + [ip])
                _csv_file.flush()   # クラッシュ/kill時のログ消失を防ぐため毎回flush（7台×10Hz程度なら負荷は無視できる）
            except Exception as e:
                print(f"[CSV][!] 書き込み失敗: {e}")

        asyncio.create_task(broadcast_ws({
            "type": "telemetry",
            "robot": mac_id,
            "label": entry["label"],
            "role": entry["role"],
            "ip": ip,
            "recv_time": now,
            "data": record,
        }))


def udp_send(payload: bytes, ip: str, port: int, broadcast: bool = False):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if broadcast:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    try:
        s.sendto(payload, (ip, port))
    finally:
        s.close()


def send_start_broadcast():
    # RUN/STOPはUDP_CMD_PORT(4211)へ送る。UDP_TELEMETRY_PORT(4210)は全ロボットが
    # 10Hzでテレメトリを流し合っている高トラフィックなポートなので、そこに相乗りさせると
    # ロボット台数が増えるほど各ロボットの受信キューでRUN/STOPパケットが埋もれて
    # 届かなくなる（1台なら問題にならず、2台以上で症状が出ていたのはこれが原因）。
    # "RUN"が正式コマンド。"START"は旧ファームウェアとの互換のため併送する。
    udp_send(b"RUN", "255.255.255.255", UDP_CMD_PORT, broadcast=True)
    udp_send(b"START", "255.255.255.255", UDP_CMD_PORT, broadcast=True)
    print(f"[CMD] RUN/START broadcast 送信 (port {UDP_CMD_PORT})")


def send_stop_broadcast():
    udp_send(b"STOP", "255.255.255.255", UDP_CMD_PORT, broadcast=True)
    print(f"[CMD] STOP broadcast 送信 (port {UDP_CMD_PORT})")


def send_param(mac_id: str, cmd_line: str) -> bool:
    entry = robots.get(mac_id)
    if not entry:
        print(f"[CMD][!] 未知のロボット: {mac_id}")
        return False
    udp_send(cmd_line.encode("utf-8"), entry["ip"], UDP_CMD_PORT)
    print(f"[CMD] -> {mac_id}({entry['ip']}): {cmd_line}")
    return True


# UDPはACK無しの一方向通信なので、同じロボットへ間隔ゼロで大量のパラメータコマンドを
# 送りつけると、ESP32側の受信バッファ/処理が追いつかず一部を静かに取りこぼすことがある
# （例: 軸固定(a)やtau_filt(L)のように送信順が後ろの方のコマンドが欠落し、
#   本人は正しい値を送ったつもりでもロボット側には反映されない）。
# これを避けるため、1台に対して複数パラメータをまとめて送る箇所は必ずこのヘルパー経由にし、
# コマンド間に短い間隔を空けて順番に送る。
async def send_params_sequential(mac_id: str, params: dict, delay_ms: float = 40.0) -> int:
    sent = 0
    for key, val in params.items():
        if send_param(mac_id, f"{key} {val}"):
            sent += 1
        await asyncio.sleep(delay_ms / 1000.0)
    return sent


# ================================================================
#  OTA（espota.py をサブプロセスで実行）
# ================================================================
async def trigger_ota(mac_id: str, bin_path: str):
    entry = robots.get(mac_id)
    if not entry:
        await broadcast_ws({"type": "ota_done", "robot": mac_id, "success": False,
                             "message": "ロボットが見つかりません（テレメトリ未受信）"})
        return False

    espota_path = config.get("espota_path", "")
    if not espota_path or not Path(espota_path).exists():
        await broadcast_ws({"type": "ota_done", "robot": mac_id, "success": False,
                             "message": "espota_path が robots_config.json に設定されていません"})
        return False

    meta = robot_meta(mac_id)
    password = meta.get("ota_password", "")

    cmd = [sys.executable, espota_path, "-i", entry["ip"], "-p", "3232", "-f", bin_path]
    if password:
        cmd += ["-a", password]

    await broadcast_ws({"type": "ota_progress", "robot": mac_id,
                         "line": f"開始: {' '.join(cmd)}"})

    try:
        proc = await asyncio.create_subprocess_exec(
            *cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT
        )
        async for raw_line in proc.stdout:
            line = raw_line.decode("utf-8", errors="ignore").rstrip()
            if line:
                await broadcast_ws({"type": "ota_progress", "robot": mac_id, "line": line})
        rc = await proc.wait()
    except Exception as e:
        await broadcast_ws({"type": "ota_done", "robot": mac_id, "success": False,
                             "message": f"実行エラー: {e}"})
        return False

    ok = (rc == 0)
    await broadcast_ws({"type": "ota_done", "robot": mac_id, "success": ok,
                         "message": "完了" if ok else f"失敗 (exit={rc})"})
    return ok


async def trigger_ota_batch(mac_ids: list, bin_path: str):
    # WiFi混雑を避けるため1台ずつ順番に書き込む（並列にしない）
    for mac_id in mac_ids:
        await trigger_ota(mac_id, bin_path)


# ================================================================
#  WebSocket / HTTP ハンドラ
# ================================================================
async def handle_index(request):
    resp = web.FileResponse(DASHBOARD_HTML)
    # dashboard.htmlを更新した後にブラウザキャッシュで古い版が表示され続けるのを防ぐ
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
    return resp


async def handle_ws(request):
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    ws_clients.add(ws)
    print(f"[WS] クライアント接続 (計{len(ws_clients)})")

    # 接続直後に現在の全ロボットの最新状態を送る（再読込時の初期表示用）
    for mac_id, entry in robots.items():
        await ws.send_str(json.dumps({
            "type": "telemetry",
            "robot": mac_id,
            "label": entry.get("label", mac_id),
            "role": entry.get("role", "follower"),
            "ip": entry.get("ip"),
            "recv_time": entry.get("last_seen"),
            "data": entry.get("data", {}),
        }, ensure_ascii=False))

    # 接続直後に現在のログ記録状態も送る（再読込時にボタンの有効/無効を正しく復元するため）
    await ws.send_str(json.dumps({
        "type": "logging_status",
        "active": logging_active,
        "filename": str(_csv_file.name) if _csv_file else None,
    }, ensure_ascii=False))

    # 接続直後に保存済みの役割別デフォルトパラメータも送る（再読込時に「更新」ボタンの
    # 基準値をサーバー保存済みの値に揃えるため）。
    await ws.send_str(json.dumps({
        "type": "role_defaults",
        "leader": config.get("role_defaults", {}).get("leader", {}),
        "follower": config.get("role_defaults", {}).get("follower", {}),
    }, ensure_ascii=False))

    try:
        async for msg in ws:
            if msg.type != web.WSMsgType.TEXT:
                continue
            try:
                payload = json.loads(msg.data)
            except Exception:
                continue
            await handle_ws_message(payload)
    finally:
        ws_clients.discard(ws)
        print(f"[WS] クライアント切断 (残{len(ws_clients)})")
    return ws


async def handle_ws_message(payload: dict):
    t = payload.get("type")
    if t == "start_all":
        send_start_broadcast()

    elif t == "stop_all":
        send_stop_broadcast()

    elif t == "start_logging":
        fname = start_logging()
        await broadcast_ws({"type": "logging_status", "active": True, "filename": fname})

    elif t == "stop_logging":
        stop_logging()
        await broadcast_ws({"type": "logging_status", "active": False, "filename": None})

    elif t == "set_param":
        send_param(payload["robot"], payload["cmd"])

    elif t == "rename":
        mac_id = payload["robot"]
        meta = robot_meta(mac_id)
        meta["label"] = payload.get("label", mac_id)
        save_config(config)
        entry = robots.get(mac_id)
        if entry:
            entry["label"] = meta["label"]
        await broadcast_ws({"type": "renamed", "robot": mac_id, "label": meta["label"]})

        # ラベル変更で自動割当のLiDAR周波数が変わりうるので、手動指定が無ければ
        # 新しい値を即座に無線で反映する（次回再起動を待たない）。
        hz, is_auto = resolve_lidar_hz(meta)
        if entry and entry.get("ip") and hz is not None:
            send_param(mac_id, f"q {hz}")
        await broadcast_ws({"type": "lidar_hz_changed", "robot": mac_id,
                             "lidar_hz": meta.get("lidar_hz"), "effective_hz": hz, "auto": is_auto})
        collided = check_lidar_hz_collisions()
        if collided:
            await broadcast_ws({"type": "lidar_hz_collision", "collisions": collided})

    elif t == "set_role":
        mac_id = payload["robot"]
        role = payload.get("role", "follower")
        if role not in ("leader", "follower"):
            role = "follower"
        meta = robot_meta(mac_id)
        meta["role"] = role
        save_config(config)
        entry = robots.get(mac_id)
        if entry:
            entry["role"] = role
        await broadcast_ws({"type": "role_changed", "robot": mac_id, "role": role})

    elif t == "set_lidar_hz":
        # 通常はラベル番号(n)からの自動割当で十分だが、必要なら手動で明示上書きできる。
        # hz=null/空を送るとクリアして自動割当(ラベル番号ベース)に戻る。
        mac_id = payload["robot"]
        hz_raw = payload.get("hz")
        meta = robot_meta(mac_id)
        if hz_raw is None or hz_raw == "":
            meta["lidar_hz"] = None
        else:
            try:
                meta["lidar_hz"] = float(hz_raw)
            except (TypeError, ValueError):
                return
        save_config(config)
        entry = robots.get(mac_id)
        hz, is_auto = resolve_lidar_hz(meta)
        if entry and entry.get("ip") and hz is not None:
            send_param(mac_id, f"q {hz}")
        await broadcast_ws({"type": "lidar_hz_changed", "robot": mac_id,
                             "lidar_hz": meta.get("lidar_hz"), "effective_hz": hz, "auto": is_auto})
        collided = check_lidar_hz_collisions()
        if collided:
            await broadcast_ws({"type": "lidar_hz_collision", "collisions": collided})

    elif t == "ota":
        asyncio.create_task(trigger_ota(payload["robot"], payload["file"]))

    elif t == "ota_batch":
        asyncio.create_task(trigger_ota_batch(payload["robots"], payload["file"]))

    elif t == "save_role_defaults":
        # ダッシュボードの「現在のリーダー・フォロワー設定を更新」ボタンから呼ばれる。
        # payload["leader"]/payload["follower"] は {コマンド文字: 値} の辞書。
        # ロボット個別ではなく役割(leader/follower)単位で保存するため、以後電源を入れ直した
        # ロボットは、再起動検知(rebooted判定)のタイミングで自動的にこの値を再送される。
        leader_vals   = payload.get("leader", {})   or {}
        follower_vals = payload.get("follower", {}) or {}
        config["role_defaults"]["leader"]   = {str(k): v for k, v in leader_vals.items()}
        config["role_defaults"]["follower"] = {str(k): v for k, v in follower_vals.items()}
        save_config(config)
        print(f"[PARAM-DEFAULT] 保存: leader={len(leader_vals)}件 follower={len(follower_vals)}件")
        await broadcast_ws({
            "type": "role_defaults_saved",
            "leader_count": len(leader_vals),
            "follower_count": len(follower_vals),
            "saved_at": time.time(),
        })

    elif t == "apply_role_defaults":
        # 保存済みの役割別デフォルトを、現在オンラインの全ロボットへ即座に反映する
        # （再起動を待たずに、既に繋がっている機体にも同じ設定を配りたい場合用）。
        # 同一ロボットへの複数パラメータはsend_params_sequential()で間隔を空けて送る
        # （ロボットが違えば宛先IPも別なので、ロボットをまたいだ並列実行はOK）。
        tasks = []
        applied = 0
        for mac_id, entry in robots.items():
            if not entry.get("ip"):
                continue
            meta = robot_meta(mac_id)
            role = meta.get("role", "follower")
            defaults = config.get("role_defaults", {}).get(role, {})
            if defaults:
                tasks.append(send_params_sequential(mac_id, defaults))
                applied += 1
        if tasks:
            await asyncio.gather(*tasks)
        print(f"[PARAM-DEFAULT] 即時反映: {applied}台")
        await broadcast_ws({"type": "role_defaults_applied", "robot_count": applied})


async def handle_upload(request):
    reader = await request.multipart()
    field = await reader.next()
    if field is None or not field.filename:
        return web.json_response({"error": "ファイルがありません"}, status=400)
    filename = Path(field.filename).name  # ディレクトリトラバーサル対策
    save_path = UPLOAD_DIR / filename
    with open(save_path, "wb") as f:
        while True:
            chunk = await field.read_chunk()
            if not chunk:
                break
            f.write(chunk)
    return web.json_response({"path": str(save_path), "name": filename})


async def handle_robots_list(request):
    # 現在把握しているロボット一覧（未接続でもconfigに登録済みのものは含める）
    out = {}
    for mac_id, meta in config["robots"].items():
        entry = robots.get(mac_id, {})
        hz, is_auto = resolve_lidar_hz(meta)
        out[mac_id] = {
            "label": meta.get("label", mac_id),
            "role": meta.get("role", "follower"),
            "ip": entry.get("ip"),
            "last_seen": entry.get("last_seen"),
            "has_ota_password": bool(meta.get("ota_password")),
            "lidar_hz": meta.get("lidar_hz"),
            "lidar_hz_effective": hz,
            "lidar_hz_auto": is_auto,
        }
    return web.json_response(out)


# ================================================================
#  起動
# ================================================================
async def main():
    # 起動時点では記録しない。ダッシュボードの「● ログ開始」ボタンを押すまでCSVは作られない。
    loop = asyncio.get_running_loop()

    check_lidar_hz_collisions()   # robots_config.json登録済みロボット間の周波数重複を起動時に確認

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("0.0.0.0", UDP_TELEMETRY_PORT))
    sock.setblocking(False)
    await loop.create_datagram_endpoint(lambda: TelemetryProtocol(), sock=sock)
    print(f"[UDP] telemetry受信: 0.0.0.0:{UDP_TELEMETRY_PORT}")

    app = web.Application()
    app.router.add_get("/", handle_index)
    app.router.add_get("/ws", handle_ws)
    app.router.add_post("/upload", handle_upload)
    app.router.add_get("/api/robots", handle_robots_list)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", HTTP_PORT)
    await site.start()

    # このPCのLAN内IPを表示（複数NIC環境では目安）
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))
        local_ip = probe.getsockname()[0]
        probe.close()
    except Exception:
        local_ip = "127.0.0.1"

    print(f"[HTTP] ダッシュボード: http://{local_ip}:{HTTP_PORT}")
    print("[LOG] CSV記録は停止中です。ダッシュボードの「● ログ開始」ボタンで開始してください。")
    if not config.get("espota_path"):
        print("[!] robots_config.json の espota_path が未設定です。OTA機能を使うには設定してください。")

    await asyncio.Event().wait()  # 永久待機


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n終了します。")
    finally:
        if _csv_file:
            _csv_file.close()
