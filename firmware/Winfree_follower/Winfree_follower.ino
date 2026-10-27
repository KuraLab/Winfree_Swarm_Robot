/*
 * Winfree_CTRL_v2_follower.ino  —  ハードウェアタイマー駆動版
 *
 * ■ 位相更新式（提案手法 M1_Hotta_20260501.pdf p.7/p.10 Fig.1/p.12 準拠）:
 *      φ̇ = ω + κ · Z⁺(φ) · Σ_j V_j ,   Z⁺(φ) = −min(sin φ, 0) ≥ 0
 *    → 受光時に「加速」する（進んだ位相のロボットのみを観測する設計）
 *
 * ■ 速度制御則:
 *      ẋ = k_P · Z⁺(φ) · ê_補間
 *    → RPMターゲット → 一次遅れフィルタ(τ_filt, 5ms周期) → FF+PID(5ms固定周期) → PWM(スロー・ディケイ)
 *    一次遅れは τ_filt・du_filt/dt + u_filt = pid_target の離散化（τ_filt=0で無効・従来通り瞬時反映）。
 *    Winfreeパルス駆動に起因する速度指令の矩形的な立ち上がり/立ち下がりを滑らかにする目的。
 *    平衡点（DCゲイン）は変えず、駆動窓の角を丸めるだけなので理論上は等間隔収束の目標距離自体は
 *    変わらない（過渡応答・PWM飽和率にのみ影響）。'L'コマンドで時定数を変更可能。
 *    常時アナログ受光量の重み付き重心（旧STEER_ANALOG）を採用し、隣接chのローブ重なりから
 *    30°間を連続補間することで真正面への正確な方位決定をIRのみで完結。
 *    速度の絶対的な大きさは、純粋な方向ベクトル（単位ベクトル）に対してゲイン K_P_RPM 側で
 *    一元管理する理論形に一本化。
 *
 * ■ センサ正規化（起動時2フェーズ・キャリブレーション）:
 *      V_j = (1/6) · (raw_j − base_j) / cal_j   （0 ≤ V_j ≤ 1/6 にクランプ、合計max=2）
 *      Phase A: IR点灯 + WS2812B赤 → 自己反射平均 = cal_j（固定）
 *      Phase B: IR消灯 + WS2812B青 → ベースライン平均 = base_j（初期値）
 *      base_j は運転中、IR消灯期間のみ非対称EMAで追従更新
 *
 * ■ スケジューリング（ハードウェアタイマー0, 1kHz tick が唯一の周期駆動源）:
 *      1ms   : MUX 1ch読み取り（前tickでアドレス設定 → 約1msの整定確保）
 *      12ms  : 1フレーム完了 → メディアンフィルタ → V_j → 位相RK4(Δt=0.012固定)
 *              → IR発光更新 → 速度指令
 *      5ms   : エンコーダ(PCNT)読取 → FF+PID → PWM出力（Δt=0.005固定)
 *      40ms  : WS2812B 位相レインボー（変化時のみ show）
 *      100ms : シリアル状態出力 + UDPテレメトリ(37フィールド)
 *
 * ■ 特徴機能:
 *      - RUN/STOP一斉制御: ダッシュボードからのUDPコマンド("RUN"/"START"/"STOP")で
 *        全機同時に発光・移動を停止/再開できる。STOP中もcal_V/V_base/phiは保持されるため、
 *        「パラメータ調整 → 一斉停止 → 一斉起動」を無線のみ・再キャリブレーション不要で行える。
 *      - チェーン距離測定: 前方(0°)・後方(180°)を別ゾーンで独立に最近傍トラックを探すことで、
 *        隣接するすべてのペアの距離を漏れなく計測する。
 *      - 実測フレーム周期(actual_frame_dt_ms)をテレメトリに追加（FRAME_DT調整用の診断値）。
 *      - 2D自由運動の確保: 直列拘束（軸投影）を完全に撤廃し、全方位への自由な運動特性を確保。
 *
 * ■ 短パルス除去:
 *      各chのサンプル周期は12ms。LD19の1掃引あたり照射時間(約8.3ms)は
 *      高々1サンプルにしか掛からないため、メディアン3で完全に除去できる。
 *
 * ■ エンコーダ: PCNTハードウェア4逓倍デコード + グリッチフィルタ(12.8µs)
 *      ENC1: GPIO21/4 → Motor1   ENC2: GPIO36/39 → Motor2 (※外部プルアップ必要)
 *      ENC3: GPIO34/17 → Motor3
 *
 * ■ Pin map (new PCB):
 *      MUX_SIG GPIO35(ADC1_CH7)  MUX_S0 GPIO16  MUX_S1 GPIO5  MUX_S2 GPIO18  MUX_S3 GPIO19
 *      IR_CTRL GPIO25 (LEDC非使用・digitalWrite専用)   LED_DIN GPIO26
 *      MOT1: GPIO32/33 (LEDC ch0/1)   MOT2: GPIO27/14 (ch2/3)   MOT3: GPIO13/12 (ch4/5)
 *      モータ搭載位置 M1=90°(J3) M2=210°(J7) M3=330°(J11)、駆動方向 180°/60°/300°
 *      センサ角度: ch_j → (j×30°)  ※実機検証済み (ch2→180°, ch6→60°, ch10→300°)
 *
 * ■ Serial commands:
 *      o/k/t/g   omega / kappa / tau / K_P_RPM
 *      P/I/D     Kp / Ki / Kd
 *      L <s>     tau_filt（速度指令の一次遅れ時定数[s]、立ち上がり側。0=無効）
 *      F <s>     tau_filt_fall（同、立ち下がり側。0=無効。Lより大きくすると、指令が
 *                消えても長く回転を保ったまま滑らかに減速する非対称フィルタになる）
 *      v <val>   検出閾値 V_ON（V_OFF は 0.6×V_ON に連動）
 *      p <rad>   phi強制    ?  ステータス
 *      c [0/1]   フルキャリブ再実行
 *                0 or 省略: 1体運用（静止のままIR ON 5秒→IR OFF 5秒で自己反射の平均を測定。従来通り）
 *                1        : 2体運用（自機IRはOFFのままその場回転し、相方ロボットが
 *                           ビーコンモード('e 1')で発光しているIRの受信量を各chごとに
 *                           最大値追跡）
 *      e <0/1>   ビーコンモード（他ロボットの2体キャリブレーション用にIRを常時点灯。移動なし）
 *      x <0.1-8> V_DECAY_EXP（既定1.0）。センサ正規化比ratio=(raw-base)/cal_Vにratio^xを
 *                適用し、距離減衰を急峻にする。大きいほど遠距離だけ急激に弱まる。
 *      y <gain>  YAW_HOLD_KP（既定1.0=有効。0で無効化可）。3輪エンコーダによるデッドレコニングヨー角推定から
 *                目標0°との誤差をP制御で補正し機体の向きを一定に保つ。
 *      1/2/3 <rpm>  モータ1/2/3 目標RPM直接指定
 *      0            全停止
 *
 * ■ UDPブロードキャストコマンド（CMD_PORTで一括受信）:
 *      RUN / START   全機同時に発光・移動を再開（cal_V/V_base/phiは保持されたまま）
 *      STOP          全機同時に発光・移動を停止（スタンバイ、再キャリブレーション不要）
 */

#include <Arduino.h>
#include <FastLED.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <ArduinoOTA.h>
#include <esp_system.h>
#include <math.h>
// ★ハードウェアドライバはファイル先頭でinclude（Arduino IDEの自動プロトタイプ
//   挿入により pcnt_unit_t 未定義エラーが出る問題への対策）
#include "driver/pcnt.h"
#include "driver/adc.h"

// ================================================================
//  WiFi / OTA
// ================================================================
// Wi-Fi候補は secrets.h に分離（Gitに含めない）。
// secrets.h.example をコピーして secrets.h を作り、値を書き換えること。
#include "secrets.h"
#define UDP_PORT  4210
#define CMD_PORT  4211      // ダッシュボードからのパラメータ変更コマンド受信専用（telemetryのUDP_PORTとは分離）

WiFiUDP    udp;
WiFiUDP    cmdUdp;
bool       wifi_ok    = false;
volatile bool ota_active = false;
char       mac_id[8];
char       ota_hostname[24];
IPAddress  broadcastIP;

// ================================================================
//  Pins — Sensor / IR / LED
// ================================================================
#define MUX_S0   16
#define MUX_S1    5
#define MUX_S2   18
#define MUX_S3   19
#define MUX_SIG  35            // ADC1_CH7（WiFi併用可はADC1のみ）
#define IR_CTRL  25            // digitalWrite専用。絶対にledcAttachPinしないこと
                               // （LEDC接続後のdigitalWriteは無視される既知問題）
#define LED_DIN  26
#define NUM_LEDS 12

// ================================================================
//  Pins — Motors (DRV8835 IN/IN, MODE=GND)
// ================================================================
#define MOT1_IN1  32
#define MOT1_IN2  33
#define MOT2_IN1  27
#define MOT2_IN2  14
#define MOT3_IN1  13
#define MOT3_IN2  12           // 起動時LOWで安全（strappingピン）

static const int CH_IN1[3] = { 0, 2, 4 };   // LEDCチャネル
static const int CH_IN2[3] = { 1, 3, 5 };
#define MOT_FREQ   20000       // 20kHz（2kHz時代のWS2812Bちらつき問題対策済み値）
#define MOT_BITS   8

// ================================================================
//  Pins — Encoders (PCNTハードウェアデコード)
// ================================================================
#define ENC1_A  21
#define ENC1_B   4
#define ENC2_A  36             // input-only, 外部プルアップ必要
#define ENC2_B  39             // input-only, 外部プルアップ必要
#define ENC3_A  34             // input-only
#define ENC3_B  17

#define GEAR_RATIO   50
#define ENC_PPR       7
#define ENC_DECODE    4        // PCNT 4逓倍
static const float COUNTS_PER_OUT_REV = (float)(ENC_PPR * ENC_DECODE * GEAR_RATIO); // 1400
static const int   ENC_POL[3] = { 1, 1, 1 };  // [!] 4逓倍化後の極性を実機確認すること
static const pcnt_unit_t ENC_UNIT[3] = { PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2 };

// ================================================================
//  Pins — LiDAR (LD19)
// ================================================================
#define LIDAR_RX_PIN     23    // LD19 Tx → ESP32 RX（受信のみ、UART1）
#define LIDAR_PWM_PIN    22    // 外部速度制御（デシンク用）
#define LIDAR_PWM_CH      6    // ch0-5はモータ6ch使用済み。ch6は独立(30kHz可)
#define LIDAR_PWM_FREQ 30000

// [!] 混信/検出率の非対称性を切り分けるための診断用スイッチ。
//     trueにすると全機同じHzに固定し、MACベースのデシンク割当てを無効化する。
//     検証が終わったら必ずfalseに戻してOTA書き込みすること。
#define LIDAR_HZ_FORCE_SAME  false
#define LIDAR_HZ_FORCED_VALUE 10.0f
// LD19スペック上、スキャン周波数は5〜13Hz（通常10Hz）。既定のmac[5]%4は10〜13Hzの
// 4段階でデシンクを試みるが、台数が増えると衝突しうるため、根本対策として
// bridge_server.py側からロボット番号(Robot n のn)に基づいて明示指定できるようにする。
float lidar_hz_override = -1.0f;   // >=0ならこの値を強制使用（'q'コマンドで設定、起動時デフォルトは無効=-1）

// ================================================================
//  スケジューリング定数（ハードウェアタイマー1kHz tickが基準）
// ================================================================
#define TICK_HZ            1000
#define N_SENSORS          12
#define FRAME_TICKS        N_SENSORS            // 12ms/フレーム
static const float FRAME_DT = 0.012f;           // 位相積分Δt（固定）
#define PID_TICKS          5                    // 5ms
static const float PID_DT   = 0.005f;           // PIDΔt（固定）
#define LED_TICKS          10                   // 25Hz
#define TELEM_TICKS        100                  // 10Hz
#define MAX_CATCHUP_TICKS  24                   // 遅延吸収の上限（超過分は破棄）

// ================================================================
//  Winfree パラメータ
// ================================================================
float omega   = TWO_PI * 1.0f;
float kappa   = 10.0f;
float tau     = 0.9f;      // 発光窓 cosφ>τ（0.9→0.2に拡幅済みの運用値）
float K_P_RPM = 100.0f;    // 速度ゲイン[rpm]（b=1/3・Z⁺=1で 150rpm 相当）
#define CMD_RPM_LIMIT 240.0f

// 2値化検出閾値（ヒステリシス付き）
// [変更] 各基板のセンサ電圧の最大値を 1/3 → 1/6 に変更（合計max = 1/6×12 = 2）。
//   V_ON/V_OFFは最大値に対する比率（0.5/0.25）で定義しているため自動で追従し、
//   2値化検出の挙動自体は変わらない。ただし位相式のκ·sumVと速度則のb_jは
//   スケールが半分になるため、従来と同じ実効結合・同じ速度にしたい場合は
//   κを2倍('k')、K_P_RPMを2倍('g')にすること（意図的にスケールを変える場合は不要）。
static const float V_MAX_CH = 1.0f / 3.0f;
float V_ON  = V_MAX_CH * 0.25f;       // V_j がこれ以上で検出ON
float V_OFF = V_MAX_CH * 0.125f;       // これ以下で検出OFF

// ================================================================
//  PID パラメータ
// ================================================================
float Kp = 15.0f;
float Ki = 5.0f;
float Kd = 0.0f;
#define FF_MAX_RPM    240.0f   // フィードフォワード基準（無負荷最高回転数）
#define MOT_DEADBAND  25
#define RPM_EMA_A     0.5f

// 速度指令の一次遅れフィルタ（M dx/dt + D... 相当。τ_filt·du_filt/dt + u_filt = pid_target）。
// Winfreeパルス駆動によるカクついた速度指令を滑らかにする目的。τ_filt=0で無効（従来通り瞬時反映）。
// パルス幅(実測70〜140ms)・周期(400〜500ms @2〜2.4Hz)より十分小さく取ること（目安20〜30ms）。
// 大きすぎるとパルスのピークに到達する前に次のパルスが来て実効速度が下がる。
float tau_filt = 0.03f;   // [s]（立ち上がり側。|pid_target|が増える方向）

// 立ち下がり側（|pid_target|が減る方向、0への収束を含む）専用の時定数。
// tau_filtと分けることで「応答は速いまま、指令が消えても長く回転を保つ」という
// 非対称フィルタ（アタック/リリース的な設計）を実現する。既定はtau_filtと同じ挙動になるよう
// 小さめにしておき（後方互換）、'F'コマンドで大きくすると立ち下がりだけゆっくりになる。
float tau_filt_fall = 0.03f;   // [s]（立ち下がり側）

// ================================================================
//  キャリブレーション / フィルタ定数
// ================================================================
#define CAL_SETTLE_MS   500    // 各フェーズの整定待ち
#define CAL_AVG_MS     5000    // 各フェーズの平均化時間
#define CAL_MIN        0.01f   // cal_V下限（ゼロ割防止、約41LSB相当）
// [!] 意図的に0.000f固定にしてある（凍結=base_Vは起動時Phase Bの値から変化しない）。
//     直列実験ではロボット間距離が数秒スケールで変化するため、下手にEMAで追従させると
//     「近づく→受信漏れをbase_jが環境光と誤認→raw-baseの差が縮む」という自己相殺が起き、
//     raw_j∝1/d²の重み付け（位相式が前提とする距離依存性）を歪めてしまう。
//     そのため非対称EMAの仕組み自体は残しつつ、値を0にして事実上無効化している。
//     元のτ≈0.6s/6sは環境光ドリブト対策として設計された値だが、この実験の距離変化と
//     時間スケールが被るため使わないこと。環境光変動が気になる場合はEMAで追従させず、
//     実験開始前に毎回startupCalibration()（Phase B）をやり直す運用でカバーする。
#define BASE_ALPHA_DN  0.000f   // ベースライン下降側EMA係数（現在は無効化、0固定）
#define BASE_ALPHA_UP  0.000f   // ベースライン上昇側EMA係数（現在は無効化、0固定）
#define IR_OFF_GUARD_TICKS 10  // 自己IR消灯後のブランキング延長[ms]（減衰テール対策）
#define ADC_OVERSAMPLE 4

// ================================================================
//  LiDAR / CBF パラメータ
//   運用レンジを50cmまでに限定（CBFは近距離障害物回避専用のため）:
//   - LD19の±45mm精度保証域(0.3〜12m)にD_SAFEが収まる
//   - 角度分解能0.8°に対し相手筐体(38.6mm角)の見込み角が0.5mで約4.4°(≈5〜6点)
//   - ピッチ角(max2°)の浮きも0.5mで約17mmに収まり2mよりはるかに安全側
#define LIDAR_PKT_LEN        47
float CONF_MIN = 60.0f;   // confidence下限（無線調整可。距離0.45m超の運用で頻繁に閾値を割るため90→70に引き下げ。
                          //   下げすぎると壁等の誤検出が増えるため、IRゲート併用が前提）
#define LIDAR_D_MIN_MM       50
#define LIDAR_D_MAX_MM       800     // 27cm運用 + 追跡ヒステリシス分の余裕
#define LIDAR_R_TARGET_M  0.0275f    // 相手LiDAR表面→中心の半径補正
#define LIDAR_MOUNT_OFFSET_DEG -90.0f // 要実機較正（既知方向に対向させて確認）
#define ROBOT_TARGET_DIAM_MAX  0.08f
#define BIN_MAX_AGE_TICKS   150     // 150ms以上古いビンは無効
#define MAX_TRACKS            60
#define TRACK_MISS_LIMIT      5     // 連続ミス許容スキャン数（100ms/スキャンで約500ms）。
                                    // 0.45m運用ではconfidenceが閾値付近で頻繁に瞬断するため、
                                    // 旧値2(約200ms)だと一瞬のドロップで即ロストしていた。
                                    // 5に緩めて瞬断を吸収しつつ、実際に離れた場合の追従遅延を許容範囲に収める。
#define GATE_DEG           35.0f
#define WHEEL_RADIUS_M    0.019f    // [!] 実測値に合わせること
static const float RPM2MPS = TWO_PI * WHEEL_RADIUS_M / 60.0f;

// ================================================================
//  ヨー角推定・補正（IMU非搭載のため、3輪オムニのエンコーダ回転数から
//  デッドレコニングで推定する）
//    MOT_ANGLE_DEGが120°等間隔配置であることを利用すると、3輪の速度を
//    単純合算した時点で並進(vx,vy)成分は打ち消し合い、回転成分だけが残る:
//      ω = (w1+w2+w3) / (3・CHASSIS_R_M)   [rad/s]
//    ωをPID_DTごとに積分してyaw_estとし、目標ヨー角(=RUN開始時点の0)との
//    誤差をP制御で3輪に「同じ量だけ」上乗せすることで、並進方向を変えずに
//    向きだけを補正する（3輪同一の上乗せ=純回転になることは上式の逆から導ける）。
// ================================================================
#define CHASSIS_R_M       0.092f   // 実測値: 92mm（ロボット中心→駆動輪接地点の距離）
float    YAW_HOLD_KP = 1.0f;       // ヨー保持のPゲイン[rad/s per rad]。既定で有効化（暫定値、符号・大きさは実機で要調整）。
                                    // 0にすると従来通り無効化できる（'y 0'）。
                                    // 符号・大きさは実機で要調整（'y'コマンドで無線設定）。
float    yaw_est = 0.0f;           // 推定ヨー角[rad]（RUN受信時に0リセット、以後デッドレコニングで積分）

float D_SAFE        = 0.25f;        // [m] 中心間安全距離

// 直列軸拘束（復活）: Winfree由来の速度指令(vx,vy)をCHAIN_FRONT_DEGの軸方向に
// 投影し、軸外(横)成分をCHAIN_LATERAL_GAIN倍だけ残す。0=完全ロック（横ずれの
// 自己修正が効かない代わりに軸から絶対にずれない）、1=ロック無効（撤去前の
// 2D自由運動と同じ）。既定0.3は「軸方向を主体にしつつ、ある程度は横ずれを
// 自分で戻せる」中間設定。CHAIN_AXIS_LOCK=falseで完全無効化も可能。
bool  CHAIN_AXIS_LOCK    = true;
float CHAIN_LATERAL_GAIN = 0.3f;

// ================================================================
//  位相感受関数(PRC) Z(φ) の切り替え
//    false（既定）: 従来のsin型  Z(φ) = max(-sin(φ), 0) = -min(sinφ, 0)
//    true         : floor型    Z(φ) = floor(φ/π)(2π-φ)   (0≤φ<2π)
//                   [0,π)では0、[π,2π)では(2π-φ)で π→0へ線形減少（Z'(φ)=-1で一定）。
//                   活性域(π,2π)全体で復元力の傾きが一定になり、sin型の
//                   不安定枝（φ→2πでZ'→+1に転じる領域）を排除する狙い。
// ================================================================
bool  PRC_SELECT = false;

// ================================================================
//  ダンパ項 k_D・(dI/dt)・Z(φ)
//    I = sumVをφ∈(π,2π)（活性域）の間だけ積分した値（1周期分）。
//    dI/dt = 直前周期のIから今周期のIへの変化量を周期長で割ったもの。
//    周期完了（φが2πをまたいでwrapし、(π,2π)区間を抜けた瞬間）まで
//    サンプルホールドし、その間は前回値を使い続ける（生sumVの微分は
//    サンプル&ホールド波形のスパイクを拾うため不採用、と設計段階で決定済み）。
//    実効ゲイン K_P_RPM − K_D・dI_dt_hold は0未満にクランプし、逆走指令を防ぐ。
// ================================================================
float    K_D             = 0.0f;   // ダンパゲイン（既定0=無効。'K'コマンドで設定）
float    I_accum         = 0.0f;   // 現周期の積算値（φ∈(π,2π)の間だけ加算中）
float    I_prev_cycle    = 0.0f;   // 直前周期の確定積算値
float    dI_dt_hold      = 0.0f;   // サンプルホールドされたdI/dt
uint32_t I_cycle_start_tick = 0;   // 現周期の開始tick（周期長計測用。0=未初期化）

// センサ正規化にかける距離減衰の指数。ratio=(raw-base)/cal_Vに対しratio^V_DECAY_EXPを
// 適用する。1.0=従来通り（raw∝1/d²をそのまま使用）。値を大きくするほど遠距離側の
// 減衰が急峻になる（実効的に1/d^(2*V_DECAY_EXP)相当）。近距離(ratio≈1)はべき乗しても
// ほぼ変化しないため、主に「遠い相手の寄与を弱める」方向にのみ効く。'x'コマンドで無線調整可。
float V_DECAY_EXP    = 1.0f;
float CBF_GAMMA      = 1.0f;        // α(h)=γh の γ [1/s]
#define CBF_RANGE_MAX_M   0.30f     // [m] CBFを適用する距離の上限。これより遠いトラックは無視（速度介入しない）
#define CBF_CHAIN_ZONE_HALF_DEG 30.0f  // [deg] CHAIN_AXIS_LOCK時、CBF対象を前方/後方ゾームに限定する半角
                                        // （CHAIN_ZONE_HALF_DEGとは別定数。距離測定用ゾーンとCBF安全ゾームは
                                        //   独立に調整できるようにするため）

// ================================================================
//  操舵モード（直列実験の問題①③対策）
//   従来は速度方向をIRセンサの2値重心（Σb_j·ê_j、30°量子化）で決めていたため、
//   (a) 30°刻みの粗さで「真正面」に正確に向かえず横へじわじわずれる（問題③）、
//   (b) 横にずれた結果さらに先頭のロボットの光まで見えてしまい、重心が2体の
//       中間方向へ引っ張られて隊列が崩れる（問題①）、が起きていた。
//   ①はモードに依らず「最近傍トラック限定＋前方半平面フィルタ」（下記）で対策し、
//   ③（量子化）はSTEER_MODEで方向の決め方を選べるようにした:
//
//   STEER_BINARY(0): 従来の2値重心 Σb_j·ê_j（30°量子化。比較基準用）
//   STEER_TRACK (1): 最近傍LiDAR確定トラックの実測方位（±1°級）。IRは実体確認と
//                    駆動タイミングにのみ使う。IR光学系の較正誤差に依存しないのが利点。
//   STEER_ANALOG(2): 各chのアナログ受光量による重み付き重心 ΣV_lin_j·ê_j の「方向」を
//                    単位ベクトル化して使う（隣接chの受光ローブの重なりで30°間を連続補間）。
//                    方向決定がIRのみで完結する（LiDARトラックの瞬断に影響されない）。
//                    重みにはV_DECAY_EXPべき乗前の線形値V_linを使う（べき乗は隣接ch間の
//                    比を急峻化し重心補間の直線性を歪めるため、方向とは分離する）。
//                    [!] 補間精度は隣接chの感度較正の一致度に依存する。ch間の較正残差が
//                        そのまま角度バイアスになるため、2体キャリブレーション('c 1')推奨。
//



// ================================================================
//  直列チェーン距離測定（実験診断用）
//   前方(180°)・後方(0°)を別ゾーンで探すことで、単純な最近傍1点だけでは
//   失われる（例: d(1,2)<d(2,3)のとき2,3間が出てこない）問題を回避する。
//   0°はSENSOR_COS/SINの座標規約（ch0=0°）に合わせている。
//   [!] 実機検証の結果、機体座標0°側が物理的な「後方」、180°側が「前方」に
//       対応することが全機共通で確認されたため、CHAIN_FRONT_DEG/CHAIN_BACK_DEG
//       を入れ替えている（ダッシュボードの前方/後方表示・gap(t)グラフが逆に
//       見える問題の修正）。LIDAR_MOUNT_OFFSET_DEGや一般の障害物検出・CBFには
//       影響しない（このチェーン距離測定専用の定数のため）。
//   CHAIN_FRONT_DEGはCHAIN_AXIS_LOCK有効時、速度指令を拘束する軸の向きとしても
//   兼用する（下記computeVelocityTargets()参照）。
// ================================================================
#define CHAIN_FRONT_DEG   180.0f
#define CHAIN_BACK_DEG      0.0f
#define CHAIN_ZONE_HALF_DEG 30.0f   // 前後それぞれ±40°の扇形内で最近傍を探す

// ================================================================
//  状態
// ================================================================
volatile uint32_t g_tick = 0;         // ★唯一の周期駆動源（1kHz ISRで加算）
hw_timer_t* tick_timer = nullptr;

float    phi = 0.0f;
bool     ir_on = false;
uint32_t ir_off_guard_end = 0;

uint16_t raw_frame[N_SENSORS];        // 今フレームのADC生値
float    med_buf[N_SENSORS][3];       // メディアン3リングバッファ
uint8_t  med_idx = 0;
bool     med_filled = false;

float    cal_V [N_SENSORS];           // 自己反射キャリブ値（固定）
float    V_base[N_SENSORS];           // ベースライン（非対称EMAで更新）
float    raw_V [N_SENSORS];           // 生電圧（ADC生値/4095、メディアン3フィルタ後・ベースライン減算前。
                                      //   cal_V/V_baseによる正規化を一切受けていない実測値。診断用）
float    V     [N_SENSORS];           // 正規化センサ電圧 [0, 1/6]（V_DECAY_EXPべき乗後。位相式・検出用）
float    V_lin [N_SENSORS];           // 同、べき乗前の線形値 [0, 1/6]（STEER_ANALOGの方向重み用。
                                      //   べき乗は隣接ch間の比を急峻化し重心補間を歪めるため分離する）
bool     det   [N_SENSORS];           // 2値化検出状態（ヒステリシス）
float    sumV = 0.0f, raw_sumV = 0.0f;
int      n_detect = 0;
float    actual_frame_dt_ms = 0.0f;   // 実測フレーム周期[ms]（FRAME_DT調整用の診断値）

float    pid_target[3]      = {0, 0, 0};
float    pid_target_filt[3] = {0, 0, 0};   // pid_targetへ一次遅れフィルタを適用した後の値（PID/FFはこちらを参照）
float    actual_rpm[3]   = {0, 0, 0};
float    pid_integral[3] = {0, 0, 0};
float    pid_prev_err[3] = {0, 0, 0};
int16_t  enc_prev[3]     = {0, 0, 0};

// [バグ修正] PCNTの単発グリッチ対策（leaderと同一の対策）。
// 実測でフォロワーのyaw_estが単発グリッチにより一瞬149.53°まで跳ね、
// YAW_HOLD_KPがそれを打ち消そうと3輪に-120rpm相当を叩き込んで
// PWMが±255に2秒近く張り付く（＝モータの異音）事例を確認した。
static const int ENC_GLITCH_MAX_COUNT =
    (int)(COUNTS_PER_OUT_REV * CMD_RPM_LIMIT / 60.0f * PID_DT * 5.0f + 0.5f);
int      mot_pwm[3]      = {0, 0, 0};
float    vel_cx = 0.0f, vel_cy = 0.0f;

uint32_t dropped_ticks = 0;           // ブロッキングで破棄したtick数（診断用）
bool     manual_override = false;     // true時はcomputeVelocityTargets()がpid_targetを書き換えない

// [!] LiDARベンチテスト等でロボットを一切動かしたくない時は false にしてOTA書き込みする。
//     シリアルコマンドではなくここを手で書き換える運用（USB接続でロボットに触れる必要をなくすため）。
bool     MOTORS_ENABLED = true;

// 直列等間隔実験用のRUN/STOP状態。ダッシュボードからのUDPブロードキャスト("RUN"/"STOP")で
// 切り替える。STOP中は発光・移動を止めるが、再起動しない限りcal_V/V_baseは保持されるため、
// 「パラメータ調整→同時停止→同時再開」が無線のみ・再キャリブレーション不要で完結する。
bool     running = false;

// ビーコンモード: trueの間はWinfree位相・RUN/STOP状態に関係なくIRを常時点灯し続ける。
// 他ロボットの「2体キャリブレーション」（自機IR OFF + 回転して"相方ロボットからの"IR受信量の
// 最大値を測定するモード）用に、相方ロボットを一時的に固定発光源として使うためのもの。
// ダッシュボードの「選択した2機で相互キャリブレーション」から自動制御される内部状態で、
// 個別のパラメータUIとしては公開しない（'e'コマンド自体はデバッグ用に残す）。
bool     ir_force_on = false;

// ビーコンのON/OFFに合わせてWS2812Bを赤/青に固定表示するための上書きフラグ。
// LED_OV_NONE時は通常のir_on/cbf_activeベースの表示に戻る。
enum LedOverrideMode : uint8_t { LED_OV_NONE = 0, LED_OV_RED = 1, LED_OV_BLUE = 2 };
LedOverrideMode led_override = LED_OV_NONE;

// 位相虹色表示モード（既定有効）。phi=0で赤、2π/3で青、4π/3で緑、2πで赤に戻る
// 連続的な色相環をWS2812Bで表示する。'r 0'コマンドで従来のIR同期青パルス表示に戻せる。
bool led_rainbow_mode = true;

// ================================================================
//  LiDAR / CBF 状態
// ================================================================
struct Track { bool alive; float bearing; float dist_m; uint8_t hit; uint8_t miss; uint8_t width_deg; };
Track trk[MAX_TRACKS] = {};

// chain測定専用のロック状態: 一度掴んだトラックを、生きている限り手放さないための
// 継続性チェック用（壁など別トラックへ1スキャンごとに飛び移る現象への対策）。
struct ZoneLock { int trk_idx = -1; float last_dist = -1.0f; };
ZoneLock chainFwdLock, chainBackLock;

uint16_t bin_dist[360];
uint8_t  bin_conf[360];
uint32_t bin_tick[360];

float    lidar_hz_target = 10.0f;     // setup()でMAC下位ビットから10/11/12Hzを割当て（デシンク）
float    lidar_duty      = 0.50f;
uint16_t lidar_speed_dps = 0;         // パケットのspeedフィールド（PWM閉ループ用）

uint8_t  cbf_active      = 0;         // テレメトリ用: 直近フレームでCBFが介入したか
uint8_t  gate_reject_cnt = 0;         // テレメトリ用: 直近フレームでゲート棄却したIR検出数
float    robot_dist_m    = 99.9f;     // IRで実体確認済みの相手ロボットまでの距離[m]（未検出時99.9）
float    robot_bearing_deg = 0.0f;    // 同、方位[deg]（未検出時0.0）

// ================================================================
//  Omni ジオメトリ
// ================================================================
static float SENSOR_COS[N_SENSORS], SENSOR_SIN[N_SENSORS];
static const float MOT_ANGLE_DEG[3] = { 210.0f, 90.0f, 330.0f }; // 駆動方向
static float MOT_COS[3], MOT_SIN[3];

// ================================================================
//  WS2812B (FastLED)
// ================================================================
CRGB leds[NUM_LEDS];
static const CRGB COLOR_OFF   = CRGB::Black;
static const CRGB COLOR_OTA   = CRGB(0, 80, 255);
static const CRGB COLOR_CAL_A = CRGB(255, 0, 0);   // Phase A: 赤
static const CRGB COLOR_CAL_B = CRGB(0, 0, 255);   // Phase B: 青
static const CRGB COLOR_CBF   = CRGB(255, 0, 0);   // CBF介入中: 赤（IR点灯/消灯より優先表示）

// 位相虹色マッピング用アンカー色（赤→青→緑→赤の順で3等分、120°ずつ）。
static const CRGB RAINBOW_ANCHOR[3] = { CRGB(255, 0, 0), CRGB(0, 0, 255), CRGB(0, 255, 0) };

// ================================================================
//  1kHz tick ISR（整数加算のみ。float演算・非IRAM関数の呼出は禁止）
// ================================================================
void IRAM_ATTR onTick1kHz() {
    g_tick++;
}

// ================================================================
//  Motor low-level（統一スロー・ディケイ、6ch全てLEDC）
// ================================================================
void setupMotors() {
    const int pins[3][2] = { {MOT1_IN1, MOT1_IN2}, {MOT2_IN1, MOT2_IN2}, {MOT3_IN1, MOT3_IN2} };
    for (int i = 0; i < 3; i++) {
        ledcSetup(CH_IN1[i], MOT_FREQ, MOT_BITS); ledcAttachPin(pins[i][0], CH_IN1[i]);
        ledcSetup(CH_IN2[i], MOT_FREQ, MOT_BITS); ledcAttachPin(pins[i][1], CH_IN2[i]);
        ledcWrite(CH_IN1[i], 0); ledcWrite(CH_IN2[i], 0);   // 空転
    }
    Serial.println("[MOT] init: IN/IN slow-decay  M1:32/33  M2:27/14  M3:13/12  20kHz");
}

// spd: -255..255。正転=IN1常時H・IN2にPWM(反転)、逆転はその逆。
// ON区間=駆動 / OFF区間=ブレーキ の統一スロー・ディケイ。
// ※ledcWrite(ch,255)は255/256デューティ（約0.4%の欠け）だが実用上問題なし
void setMotorPWM(int motor, int spd) {
    spd = constrain(spd, -255, 255);
    // スティクション補償: 微小指令はデッドバンド値へスナップ（意図的停止0は除く）
    if      (spd > 0 && spd <  MOT_DEADBAND) spd =  MOT_DEADBAND;
    else if (spd < 0 && spd > -MOT_DEADBAND) spd = -MOT_DEADBAND;
    mot_pwm[motor] = spd;

    if (spd > 0) {
        ledcWrite(CH_IN1[motor], 255);
        ledcWrite(CH_IN2[motor], 255 - spd);
    } else if (spd < 0) {
        ledcWrite(CH_IN2[motor], 255);
        ledcWrite(CH_IN1[motor], 255 + spd);
    } else {
        ledcWrite(CH_IN1[motor], 0);
        ledcWrite(CH_IN2[motor], 0);          // 空転停止
    }
}

// ★積分は保持する（駆動窓をまたいだ蓄積が短い駆動窓での収束に不可欠）
void stopAllMotors() {
    for (int i = 0; i < 3; i++) {
        pid_target[i]      = 0.0f;
        pid_target_filt[i] = 0.0f;   // 明示的な停止は即座に反映（フィルタの残留を持ち越さない）
        pid_prev_err[i]    = 0.0f;
        // pid_integral[i] はリセットしない
    }
    setMotorPWM(0, 0); setMotorPWM(1, 0); setMotorPWM(2, 0);
}

// ================================================================
//  Encoder (PCNT 4逓倍 + グリッチフィルタ)
// ================================================================
static void setupEncoderPCNT(pcnt_unit_t unit, int gpioA, int gpioB) {
    pcnt_config_t c = {};
    c.unit          = unit;
    c.counter_h_lim =  32767;
    c.counter_l_lim = -32768;
    // ch0: pulse=A, ctrl=B
    c.channel        = PCNT_CHANNEL_0;
    c.pulse_gpio_num = gpioA;
    c.ctrl_gpio_num  = gpioB;
    c.pos_mode       = PCNT_COUNT_DEC;
    c.neg_mode       = PCNT_COUNT_INC;
    c.lctrl_mode     = PCNT_MODE_REVERSE;
    c.hctrl_mode     = PCNT_MODE_KEEP;
    pcnt_unit_config(&c);
    // ch1: pulse=B, ctrl=A
    c.channel        = PCNT_CHANNEL_1;
    c.pulse_gpio_num = gpioB;
    c.ctrl_gpio_num  = gpioA;
    c.pos_mode       = PCNT_COUNT_INC;
    c.neg_mode       = PCNT_COUNT_DEC;
    pcnt_unit_config(&c);

    pcnt_set_filter_value(unit, 1023);   // 1023 APBサイクル ≈ 12.8µs未満のパルスを除去
    pcnt_filter_enable(unit);            //（erratum 3.11の80ns擾乱・PWMノイズ対策）
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

void setupEncoders() {
    pinMode(ENC1_A, INPUT); pinMode(ENC1_B, INPUT);
    pinMode(ENC2_A, INPUT); pinMode(ENC2_B, INPUT);   // 36/39は外部プルアップ前提
    pinMode(ENC3_A, INPUT); pinMode(ENC3_B, INPUT);
    setupEncoderPCNT(ENC_UNIT[0], ENC1_A, ENC1_B);
    setupEncoderPCNT(ENC_UNIT[1], ENC2_A, ENC2_B);
    setupEncoderPCNT(ENC_UNIT[2], ENC3_A, ENC3_B);
    Serial.printf("[ENC] PCNT x4 decode  CPR(out)=%.0f  filter=12.8us  PID_dt=%dms\n",
                  COUNTS_PER_OUT_REV, PID_TICKS);
    Serial.println("[!] ENC_POL[3]を実機確認: 正転指令でrpmが正になること（4逓倍化後）");
}

// ================================================================
//  PID update（PID_TICKS=5msごと、Δt=PID_DT固定）
// ================================================================
void updatePID() {
    // LiDAR受信等でloop()が遅延すると、updatePID()の実際の呼び出し間隔が
    // PID_DT(5ms)からずれることがある。固定PID_DTで割ると、その分だけ
    // rpm_rawが水増し/過小評価され、PIDが暴れる（leaderで実測確認済みの原因）。
    // ここで実際の経過時間dtを計測し、固定PID_DTの代わりに使う。
    static uint32_t last_pid_run_tick = 0;
    uint32_t now_tick = g_tick;
    float dt = (last_pid_run_tick == 0) ? PID_DT
                                         : (float)(int32_t)(now_tick - last_pid_run_tick) * 0.001f;
    if (dt < 0.001f) dt = 0.001f;   // 異常に小さいdt（連続呼び出し等）を防ぐ下限
    last_pid_run_tick = now_tick;

    for (int i = 0; i < 3; i++) {
        int16_t cnt;
        pcnt_get_counter_value(ENC_UNIT[i], &cnt);
        int16_t d16 = (int16_t)(cnt - enc_prev[i]);   // int16差分でラップも正しく処理
        enc_prev[i] = cnt;
        if (abs((int)d16) > ENC_GLITCH_MAX_COUNT) d16 = 0;   // スプリアスなグリッチを捨てる
        float rpm_raw = (float)(d16 * ENC_POL[i]) / COUNTS_PER_OUT_REV / dt * 60.0f;
        actual_rpm[i] += RPM_EMA_A * (rpm_raw - actual_rpm[i]);

        if (!MOTORS_ENABLED) {
            // ベンチテスト用の凍結: pid_targetの値に関わらず無条件でPWM=0
            setMotorPWM(i, 0);
            pid_prev_err[i]    = 0.0f;
            pid_target_filt[i] = 0.0f;
            continue;
        }

        // 速度指令に非対称の一次遅れフィルタを適用（M ẍ+D ẋ=u 相当）。デッドバンド判定・誤差・
        // フィードフォワードは以降すべてフィルタ後の値pid_target_filt[i]を参照する。
        // |pid_target|が増える方向（立ち上がり）はtau_filt（速い）、減る方向（立ち下がり、
        // 0への収束を含む）はtau_filt_fall（長め）を使う。これにより応答の速さを保ったまま、
        // 指令が消えた後も長く回転を保つ（見かけ上の慣性を持たせる）ことができる。
        bool rising = fabsf(pid_target[i]) > fabsf(pid_target_filt[i]);
        float tau_use = rising ? tau_filt : tau_filt_fall;
        float alpha_filt = (tau_use > 1e-4f) ? (1.0f - expf(-dt / tau_use)) : 1.0f;
        pid_target_filt[i] += alpha_filt * (pid_target[i] - pid_target_filt[i]);

        // [変更] 完全一致(==0.0f)ではなく微小しきい値で停止判定する。
        //   YAW_HOLD_KP有効時はヨー補正の微小な残差が常にpid_targetに乗るため、
        //   ==0.0f判定だと駆動窓外でもPIDが動き続け、デッドバンドスナップで
        //   最低PWM(25)が出て微速クリープ・モータ唸りが発生する。0.5rpm未満は
        //   実質ゼロとみなして従来通り停止させる（積分は凍結保持のまま）。
        //   フィルタ後の値で判定することで、駆動窓終了時もPWMが指令のように瞬時に
        //   0へ飛ばず、フィルタの時定数どおり滑らかに減速してから停止する。
        if (fabsf(pid_target_filt[i]) < 0.5f) {
            // 非駆動窓: PWM=0で停止。積分は凍結保持（リセットしない）
            setMotorPWM(i, 0);
            pid_prev_err[i] = 0.0f;
            continue;
        }
        float err = pid_target_filt[i] - actual_rpm[i];
        float ilim = (Ki > 1e-6f) ? (200.0f / Ki) : 0.0f;   // 積分寄与を±200PWMに制限
        pid_integral[i] = constrain(pid_integral[i] + err * dt, -ilim, ilim);
        float derr = (err - pid_prev_err[i]) / dt;
        pid_prev_err[i] = err;

        float ff  = pid_target_filt[i] / FF_MAX_RPM * 255.0f;    // フィードフォワード
        float out = ff + Kp * err + Ki * pid_integral[i] + Kd * derr;
        setMotorPWM(i, constrain((int)out, -255, 255));
    }

    // ヨー角推定（デッドレコニング）: 120°等間隔配置なので3輪合算で並進成分が
    // 打ち消し合い、回転角速度だけが残る。MOTORS_ENABLED=falseでも、
    // actual_rpm自体はエンコーダの実測値なので推定は継続する
    // （手で回した場合の追従確認等、ベンチテストにも使えるようにするため）。
    float wheel_sum_mps = (actual_rpm[0] + actual_rpm[1] + actual_rpm[2]) * RPM2MPS;
    float yaw_rate = wheel_sum_mps / (3.0f * CHASSIS_R_M);
    yaw_est += yaw_rate * dt;
    yaw_est = fmodf(yaw_est + PI, TWO_PI);
    if (yaw_est < 0.0f) yaw_est += TWO_PI;
    yaw_est -= PI;   // [-π, π]に正規化
}

// ================================================================
//  Omni setup（センサ角度は実機検証済みマッピング: ch_j → -j×30°）
// ================================================================
void setupOmni() {
    for (int j = 0; j < N_SENSORS; j++) {
        float a = -(float)j * 30.0f * DEG_TO_RAD;
        SENSOR_COS[j] = cosf(a);
        SENSOR_SIN[j] = sinf(a);
    }
    for (int i = 0; i < 3; i++) {
        float a = MOT_ANGLE_DEG[i] * DEG_TO_RAD;
        MOT_COS[i] = cosf(a);
        MOT_SIN[i] = sinf(a);
        Serial.printf("[OMNI] Motor%d 駆動方向 %5.1f°\n", i + 1, MOT_ANGLE_DEG[i]);
    }
}

// ================================================================
//  位相ダイナミクス（RK4、Δt=FRAME_DT固定）
//    φ̇ = ω + κ·Z⁺(φ)·sumV,  Z⁺(φ) = −min(sinφ,0) ≥ 0  → 受光で加速
// ================================================================
static inline float Zplus(float p) {
    if (!PRC_SELECT) {
        // 従来のsin型: Z(φ) = max(-sin(φ), 0) = -min(sinφ, 0)
        float s = sinf(p);
        return (s < 0.0f) ? -s : 0.0f;
    } else {
        // floor型: Z(φ) = floor(φ/π)(2π-φ)  (0≤φ<2π)
        // [0,π)では floor(φ/π)=0 → Z=0
        // [π,2π)では floor(φ/π)=1 → Z=2π-φ（π→0へ線形減少、Z'(φ)=-1一定）
        float k = floorf(p / PI);
        return k * (TWO_PI - p);
    }
}
static inline float phaseRate(float p, float S) {
    return omega + kappa * Zplus(p) * S;
}
void phaseStepRK4(float dt, float S) {
    float k1 = phaseRate(phi, S);
    float k2 = phaseRate(phi + 0.5f * dt * k1, S);
    float k3 = phaseRate(phi + 0.5f * dt * k2, S);
    float k4 = phaseRate(phi + dt * k3, S);
    phi += dt * (k1 + 2.0f * k2 + 2.0f * k3 + k4) / 6.0f;
    phi = fmodf(phi, TWO_PI);
    if (phi < 0.0f) phi += TWO_PI;
}


void computeVelocityTargets() {
    float cx = 0.0f, cy = 0.0f;
    int nd = 0;
    static uint8_t on_cnt[N_SENSORS] = {0};
    gate_reject_cnt = 0;
    cbf_active = 0;                                // 早期return時に古い値が残らないよう先頭でリセット
    robot_dist_m = 99.9f;                          // 同上（このフレームでIR確認できなければ「未検出」に戻す）
    robot_bearing_deg = 0.0f;

    // 持続確認: 3フレーム(36ms)連続ONで初めて検出（LiDAR迷光は最大1〜2フレームしか持続しない）
    for (int j = 0; j < N_SENSORS; j++) {
        if (!det[j]) {
            if (V[j] >= V_ON) { if (++on_cnt[j] >= 3) det[j] = true; }
            else on_cnt[j] = 0;
        } else if (V[j] <= V_OFF) { det[j] = false; on_cnt[j] = 0; }
    }

    // 最も近い、LiDAR実体確認済みのtrackを1つだけ選ぶ（直列実験で2体目より奥の
    // 1体目の光まで混ざって方向が中間に引っ張られ、回り込みが起きる問題への対策）。
    // 常時有効（旧LIDAR_GATE_ENトグルは既定offのまま使われていなかったため撤去）。
    int nearest_trk = -1; float nearest_d = 1e9f;
    for (int j = 0; j < N_SENSORS; j++) {
        if (!det[j]) continue;
        int matched = nearestTrackForSensor(j);
        if (matched < 0) continue;
        
        if (trk[matched].dist_m < nearest_d) {
            nearest_d = trk[matched].dist_m;
            nearest_trk = matched;
        }
    }
    if (nearest_trk >= 0) {
        robot_dist_m = trk[nearest_trk].dist_m;
        robot_bearing_deg = trk[nearest_trk].bearing * RAD_TO_DEG;
    }

    // 方向計算(cx,cy)は、閾値 V_OFF を超えているすべての受光チャンネルから算出します。
    for (int j = 0; j < N_SENSORS; j++) {
        // 重心補間(アナログ量子化解消)のため、det確定chに加え、ヒステリシス確定前の弱い応答(V_OFF超のローブの「肩」)も含める
        bool candidate = (V_lin[j] > V_OFF);
        if (!candidate) continue;
        
        // 重みにはべき乗前の線形アナログ値 V_lin[j] を使用して、IRのみによる連続的な方向決定を行います
        float w = V_lin[j];
        cx += w * SENSOR_COS[j];
        cy += w * SENSOR_SIN[j];
        if (det[j]) nd++;   // nd（駆動タイミングとテレメトリのn_detect）はdet確定chのみ数える
    }

    // 重心方向ベクトルの完全正規化（純粋な単位ベクトル化）を行い、速度のスケール調整はK_P_RPM側に一任する理論形を適用
    if (nd > 0) {
        float norm = sqrtf(cx * cx + cy * cy);
        if (norm > 1e-6f) {
            cx = cx / norm;
            cy = cy / norm;
        }
    }
    vel_cx = cx; vel_cy = cy; n_detect = nd;

    if (manual_override) return;   // 手動モータテスト中はWinfree側で上書きしない

    // Winfree由来の速度は駆動窓内・IR検出時のみ非ゼロ。駆動窓外や未検出時は0として、
    // その先のapplyCBF()には必ず通す（CBFをWinfreeの位相タイミングから独立させるため）。
    float Zp = Zplus(phi);
    bool  winfree_drive = (Zp > 0.0f && nd > 0);
    // ダンパ項: 実効ゲイン = K_P_RPM − K_D・(dI/dt)。dI/dtはsumVの1周期変化量
    // （φ∈(π,2π)積分のサンプルホールド値、processFrame()で更新）。
    // 逆走指令を防ぐため実効ゲインは0未満にクランプする（K_D=0なら従来通りK_P_RPMのみ）。
    float damper_gain = K_P_RPM - K_D * dI_dt_hold;
    if (damper_gain < 0.0f) damper_gain = 0.0f;
    float vx = winfree_drive ? damper_gain * Zp * cx : 0.0f;
    float vy = winfree_drive ? damper_gain * Zp * cy : 0.0f;

    // 直列軸拘束（復活）: (vx,vy)をCHAIN_FRONT_DEG方向の軸成分と軸外(横)成分に
    // 分解し、横成分だけCHAIN_LATERAL_GAIN倍に減衰させる。軸成分はそのまま
    // （速度の絶対的な大きさはK_P_RPM側で管理する理論形を崩さない）。
    if (CHAIN_AXIS_LOCK) {
        static const float AXIS_X = cosf(CHAIN_FRONT_DEG * DEG_TO_RAD);
        static const float AXIS_Y = sinf(CHAIN_FRONT_DEG * DEG_TO_RAD);
        float along = vx * AXIS_X + vy * AXIS_Y;
        float lat_x = vx - along * AXIS_X;
        float lat_y = vy - along * AXIS_Y;
        vx = along * AXIS_X + lat_x * CHAIN_LATERAL_GAIN;
        vy = along * AXIS_Y + lat_y * CHAIN_LATERAL_GAIN;
    }

    applyCBF(vx, vy);   // 常時評価。Winfreeが0を指令していてもD_SAFEより近ければ後退方向の補正が入る

    // ヨー保持補正: 目標ヨー角(0)との誤差をP制御し、3輪に「同じ量だけ」回転成分を
    // 上乗せする（120°等間隔配置では、3輪同一量の上乗せ=並進方向を変えない純回転に
    // なる。導出はCHASSIS_R_M定義部のコメント参照）。YAW_HOLD_KP=0で無効化可（既定は有効）。
    float yaw_err = 0.0f - yaw_est;              // 目標0との誤差（yaw_estは[-π,π]）
    float omega_cmd = YAW_HOLD_KP * yaw_err;      // [rad/s]
    float yaw_corr_rpm = (CHASSIS_R_M * omega_cmd) / RPM2MPS;   // 3輪共通の上乗せ分[RPM]

    // 飽和は3輪一様スケール（per-wheelクランプは速度方向を変えCBF保証を壊す）
    float wheel[3], wmax = 0.0f;
    for (int i = 0; i < 3; i++) {
        wheel[i] = vx * MOT_COS[i] + vy * MOT_SIN[i] + yaw_corr_rpm;
        wmax = fmaxf(wmax, fabsf(wheel[i]));
    }
    float sc = (wmax > CMD_RPM_LIMIT) ? (CMD_RPM_LIMIT / wmax) : 1.0f;
    for (int i = 0; i < 3; i++) pid_target[i] = wheel[i] * sc;
}

// ================================================================
//  Sensor（1ch/1ms パイプライン + メディアン3 + 正規化）
// ================================================================
static inline void setMuxChannel(uint8_t ch) {
    digitalWrite(MUX_S0, (ch >> 0) & 1);
    digitalWrite(MUX_S1, (ch >> 1) & 1);
    digitalWrite(MUX_S2, (ch >> 2) & 1);
    digitalWrite(MUX_S3, (ch >> 3) & 1);
}

static inline uint16_t readAdcAveraged() {
    uint32_t acc = 0;
    for (int s = 0; s < ADC_OVERSAMPLE; s++) acc += adc1_get_raw(ADC1_CHANNEL_7);
    return (uint16_t)(acc / ADC_OVERSAMPLE);
}

static inline float median3(float a, float b, float c) {
    float mx = fmaxf(a, b), mn = fminf(a, b);
    return fmaxf(mn, fminf(mx, c));
}

void processFrame();

// 1msごとに1ch読む。アドレスは前tickで設定済み → 約1msの整定を確保
// （1MΩ負荷のフォトトランジスタでもチャネル間クロストークが乗らない）
void sensorTick() {
    static uint8_t cur_ch = 0;
    raw_frame[cur_ch] = readAdcAveraged();
    uint8_t done = cur_ch;
    cur_ch = (cur_ch + 1) % N_SENSORS;
    setMuxChannel(cur_ch);
    if (done == N_SENSORS - 1) processFrame();
}

// フレーム完了（12msごと。実測周期はUDPテレメトリのactual_frame_dt_msで確認可能）:
// フィルタ → V_j → 位相 → 発光 → 速度指令
void processFrame() {
    static uint32_t last_frame_us = 0;
    uint32_t now_frame_us = micros();
    if (last_frame_us != 0) {
        actual_frame_dt_ms = (float)(now_frame_us - last_frame_us) / 1000.0f;
    }
    last_frame_us = now_frame_us;

    bool blanked = ir_on || ((int32_t)(g_tick - ir_off_guard_end) < 0);

    if (!blanked) {
        float s = 0.0f, rs = 0.0f;
        for (int j = 0; j < N_SENSORS; j++) {
            float x = (float)raw_frame[j] / 4095.0f;
            med_buf[j][med_idx] = x;
            float m = med_filled ? median3(med_buf[j][0], med_buf[j][1], med_buf[j][2]) : x;
            raw_V[j] = m;   // 生電圧（cal_V/V_baseによる正規化前。診断用テレメトリ用）

            // ベースライン: 非対称EMA（下降側は速く、上昇側は遅く追従）
            // → 近傍ロボットのIRパルス(デューティ約44%)を床として除外しつつ環境光変動に追従
            float a = (m < V_base[j]) ? BASE_ALPHA_DN : BASE_ALPHA_UP;
            V_base[j] += a * (m - V_base[j]);

            float num = m - V_base[j];
            float ratio = num / cal_V[j];
            // べき乗は負値やゼロ付近でpowf()が不安定になるため、先に0未満をクランプしてから適用。
            // V_DECAY_EXP=1.0なら従来と完全に同じ（ratioそのまま、powf呼び出しもスキップ）。
            float ratio_clamped = (ratio < 0.0f) ? 0.0f : ratio;
            float ratio_decayed = (V_DECAY_EXP == 1.0f) ? ratio_clamped : powf(ratio_clamped, V_DECAY_EXP);
            float vr  = V_MAX_CH * ratio_decayed;
            // raw_sumVは較正ズレ検知用の診断値なので、べき乗前(線形)のratioで計算し、
            // 「べき乗をかける前は本来どれくらい強い信号だったか」を見えるようにしておく。
            rs += V_MAX_CH * ratio;
            // STEER_ANALOG用: べき乗前の線形値もクランプして保持（方向重み専用）
            float vlin = V_MAX_CH * ratio_clamped;
            if (vlin > V_MAX_CH) vlin = V_MAX_CH;
            V_lin[j] = vlin;
            float v = vr;
            if (v < 0.0f) v = 0.0f;
            if (v > V_MAX_CH) v = V_MAX_CH;             // 各基板max=1/6 → 合計max=2
            V[j] = v;
            s += v;
        }
        med_idx = (med_idx + 1) % 3;
        if (med_idx == 0) med_filled = true;
        sumV = s; raw_sumV = rs;
    }
    // ブランキング中は V / sumV / det を直前値でホールド（従来設計を踏襲）

    // ビーコンモード: Winfree位相・RUN/STOP状態を無視してIRを常時点灯し続ける
    // （他ロボットの2体キャリブレーションの基準発光源として使うためのモード）。
    // 移動もさせない。running状態やphiは変化させず、モード解除後にそのまま復帰できる。
    if (ir_force_on) {
        if (!ir_on) {
            ir_on = true;
            digitalWrite(IR_CTRL, HIGH);
        }
        led_override = LED_OV_RED;
        stopAllMotors();
        vel_cx = 0.0f; vel_cy = 0.0f; n_detect = 0;
        updateLED();
        return;
    }

    if (!running) {
        // スタンバイ中: 位相を進めず、発光も止める。
        // phiはフリーズしたまま保持されるので、次回RUN受信時はその時点のphiから
        // 再開する（再キャリブレーション不要、全機同時の完全な同期起動になる）。
        if (ir_on) {
            ir_on = false;
            digitalWrite(IR_CTRL, LOW);
        }
        // モーター単体テスト中（manual_override）はrunning=falseでもPWMを止めない。
        // 従来はここで無条件にstopAllMotors()を呼んでpid_targetを0へ戻していたため、
        // updatePID()側でmanual_overrideにより設定した目標rpmが、最悪12ms(FRAME_DT)以内に
        // 強制的に打ち消され、単体テストがほぼ回らないように見える不具合があった。
        if (!manual_override) {
            stopAllMotors();
        }
        vel_cx = 0.0f; vel_cy = 0.0f; n_detect = 0;
        // 虹色モードはir_onに依存せず点灯し続けるため、STOP直後にir_onがすでに
        // falseだった場合でも消灯を反映できるよう毎フレーム呼ぶ（内部で変化検出するので軽量）。
        updateLED();
        return;
    }

    // ダンパ項用の周期積分: sumVをφ∈(π,2π)（活性域）の間だけ積算する。
    // このフレームのsumV取得時点でのφ（=位相積分前のφ）で区間判定する。
    bool in_damper_window_before = (phi > PI && phi < TWO_PI);
    if (in_damper_window_before) {
        I_accum += sumV * FRAME_DT;
    }

    // 位相積分（Δt一定。ブランキング中もホールドしたsumVで継続）
    phaseStepRK4(FRAME_DT, sumV);

    // φが2πをまたいでwrapし(π,2π)区間を抜けた瞬間 = 1周期分の積分I確定 → dI/dt更新。
    // dI_dt_holdは次にこの区間を抜けるまでサンプルホールドされる
    // （生sumVの微分はサンプル&ホールド波形のスパイクを拾うため、周期単位でのみ更新する設計）。
    bool in_damper_window_after = (phi > PI && phi < TWO_PI);
    if (in_damper_window_before && !in_damper_window_after) {
        uint32_t now_tick = g_tick;
        if (I_cycle_start_tick != 0) {
            float dt_cycle = (float)(int32_t)(now_tick - I_cycle_start_tick) * 0.001f;
            if (dt_cycle > 1e-3f) {
                dI_dt_hold = (I_accum - I_prev_cycle) / dt_cycle;
            }
        }
        I_prev_cycle = I_accum;
        I_accum = 0.0f;
        I_cycle_start_tick = now_tick;
    }

    // IR発光: f(φ) = H(cosφ − τ)
    bool na = (cosf(phi) > tau);
    if (na != ir_on) {
        ir_on = na;
        digitalWrite(IR_CTRL, ir_on ? HIGH : LOW);
        if (!ir_on) ir_off_guard_end = g_tick + IR_OFF_GUARD_TICKS;  // 減衰テール対策
    }

    computeVelocityTargets();
    updateLED();   // IR ON/OFF・CBF介入いずれの変化も同じ12msフレーム精度で即時反映
}

// ================================================================
//  LiDAR (LD19) — UART1受信 + 360ビン + クラスタ/トラック + CBF
// ================================================================

// CRC8 (poly 0x4D, init 0x00 — LD19 Development Manual準拠)
static uint8_t crc_tbl[256];
void initLidarCrc() {
    for (int i = 0; i < 256; i++) {
        uint8_t c = i;
        for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x4D) : (uint8_t)(c << 1);
        crc_tbl[i] = c;
    }
}

// LD19角度(左手系・時計回り正) → 機体座標bin。SENSOR_COS/SINと同じ座標規約に合わせる。
static inline int lidarDegToBodyBin(float lidar_deg) {
    float b = LIDAR_MOUNT_OFFSET_DEG - lidar_deg;
    b = fmodf(b, 360.0f); if (b < 0) b += 360.0f;
    return ((int)(b + 0.5f)) % 360;
}

// バイト単位の非ブロッキングパーサ（loop()から毎回呼ぶ）
void lidarPoll() {
    static uint8_t buf[LIDAR_PKT_LEN];
    static int idx = 0;
    while (Serial1.available()) {
        uint8_t c = Serial1.read();
        if (idx == 0 && c != 0x54) continue;
        if (idx == 1 && c != 0x2C) { idx = (c == 0x54) ? 1 : 0; continue; }
        buf[idx++] = c;
        if (idx < LIDAR_PKT_LEN) continue;
        idx = 0;
        uint8_t crc = 0;
        for (int i = 0; i < LIDAR_PKT_LEN - 1; i++) crc = crc_tbl[crc ^ buf[i]];
        if (crc != buf[LIDAR_PKT_LEN - 1]) continue;

        lidar_speed_dps = buf[2] | (buf[3] << 8);
        float a0 = (buf[4] | (buf[5]  << 8)) * 0.01f;
        float a1 = (buf[42]| (buf[43] << 8)) * 0.01f;
        float da = a1 - a0; if (da < 0) da += 360.0f;
        da /= 11.0f;                             // 12点間の角度補間
        for (int p = 0; p < 12; p++) {
            uint16_t d  = buf[6 + 3*p] | (buf[7 + 3*p] << 8);
            uint8_t  cf = buf[8 + 3*p];
            int bin = lidarDegToBodyBin(a0 + da * p);
            bin_dist[bin] = d; bin_conf[bin] = cf; bin_tick[bin] = g_tick;
        }
    }
}

// 追加関数（両ファイル共通）: 距離distにおいて「ロボット本体として妥当な最大角度幅」を返す。
// 壁・什器のような広い平面はこの幅を大きく超えるため、chain測定から除外する判定に使う。
float maxPlausibleWidthDeg(float dist_m) {
    if (dist_m < 0.05f) dist_m = 0.05f;
    float half = atan2f(ROBOT_TARGET_DIAM_MAX * 0.5f, dist_m) * RAD_TO_DEG;
    return half * 2.0f + 5.0f;   // +5°はLiDAR角度分解能・ブレのマージン
}

// クラスタリング + トラック更新（100msごとにloop()から）
void lidarUpdateTracks() {
    uint32_t now = g_tick;
    float   c_bear[MAX_TRACKS * 2]; float c_dist[MAX_TRACKS * 2]; uint8_t c_width[MAX_TRACKS * 2];
    int     nc = 0;
    int     run_start = -1, gap = 0;
    float   dmin = 1e9f; double sx = 0, sy = 0; int npts = 0;
    float   dmin0 = 1e9f; double sx0 = 0, sy0 = 0;   // bin0始まりの先頭クラスタの生データ（周回結合用）
    bool    first_starts_at_zero = false;
    for (int i = 0; i < 360; i++) {
        int b = i;
        bool ok = (now - bin_tick[b] < BIN_MAX_AGE_TICKS) &&
                  bin_conf[b] >= CONF_MIN &&
                  bin_dist[b] >= LIDAR_D_MIN_MM && bin_dist[b] <= LIDAR_D_MAX_MM;
        if (ok) {
            if (run_start < 0) { run_start = i; dmin = 1e9f; sx = sy = 0; npts = 0; }
            gap = 0;
            float a = b * DEG_TO_RAD;
            sx += cos(a); sy += sin(a); npts++;
            if (bin_dist[b] < dmin) dmin = bin_dist[b];
        } else if (run_start >= 0 && ++gap > 1) {
            if (npts >= 1 && nc < MAX_TRACKS * 2) {
                if (run_start == 0) { sx0 = sx; sy0 = sy; dmin0 = dmin; first_starts_at_zero = true; }
                c_bear[nc]  = atan2f((float)sy, (float)sx);
                c_dist[nc]  = dmin * 0.001f + LIDAR_R_TARGET_M;   // 最近点 + 半径補正
                c_width[nc] = (uint8_t)min(npts, 255);            // クラスタの角度幅[°]（1ビン=1°）
                nc++;
            }
            run_start = -1;
        }
    }
    // bin359まで途切れず開いたまま終わったクラスタをここで確定する。
    // bin0始まりのクラスタ(先頭)と物理的に連続している場合は周回結合し、
    // そうでなければ独立したクラスタとして追加する。
    if (run_start >= 0 && npts >= 1) {
        if (first_starts_at_zero && nc > 0) {
            double msx = sx0 + sx, msy = sy0 + sy;
            float  mdmin = fminf(dmin0, dmin);
            c_bear[0]  = atan2f((float)msy, (float)msx);
            c_dist[0]  = mdmin * 0.001f + LIDAR_R_TARGET_M;
            c_width[0] = (uint8_t)min((int)c_width[0] + npts, 255);   // 周回結合分の角度幅を合算
        } else if (nc < MAX_TRACKS * 2) {
            c_bear[nc]  = atan2f((float)sy, (float)sx);
            c_dist[nc]  = dmin * 0.001f + LIDAR_R_TARGET_M;
            c_width[nc] = (uint8_t)min(npts, 255);
            nc++;
        }
    }
    // 最近傍アソシエーション（±25° かつ ±0.3m）
    bool used[MAX_TRACKS * 2] = {};
    for (int k = 0; k < MAX_TRACKS; k++) {
        if (!trk[k].alive) continue;
        int best = -1; float bestc = 1e9f;
        for (int c = 0; c < nc; c++) {
            if (used[c]) continue;
            float dth = fabsf(atan2f(sinf(c_bear[c] - trk[k].bearing),
                                     cosf(c_bear[c] - trk[k].bearing)));
            float dd  = fabsf(c_dist[c] - trk[k].dist_m);
            if (dth < 25.0f * DEG_TO_RAD && dd < 0.3f && dth + dd < bestc) {
                bestc = dth + dd; best = c;
            }
        }
        if (best >= 0) {
            used[best] = true;
            trk[k].bearing += 0.5f * atan2f(sinf(c_bear[best] - trk[k].bearing),
                                            cosf(c_bear[best] - trk[k].bearing));
            trk[k].dist_m  += 0.5f * (c_dist[best] - trk[k].dist_m);
            trk[k].width_deg = c_width[best];     // 角度幅も最新値に更新
            trk[k].miss = 0;
            if (trk[k].hit < 250) trk[k].hit++;
        } else if (++trk[k].miss >= TRACK_MISS_LIMIT) {
            trk[k].alive = false;                 // TRACK_MISS_LIMITスキャン連続ミスで消滅
        }
    }
    // 未対応クラスタ → 新規トラック（hit>=2まではCBF/ゲートに使わない=ゴースト除去）
    for (int c = 0; c < nc; c++) {
        if (used[c]) continue;
        for (int k = 0; k < MAX_TRACKS; k++) {
            if (trk[k].alive) continue;
            trk[k] = { true, c_bear[c], c_dist[c], 1, 0, c_width[c] };
            break;
        }
    }
}

// CBFフィルタ: 違反制約への逐次射影（QP不要の閉形式）。vx/vyはK_P_RPM系のRPM相当値。
//   CBF_RANGE_MAX_M より遠いトラックは制約対象外（LiDAR自体は80cmまで拾うが、
//   介入は近づいた場合のみに限定する）。
//   CHAIN_AXIS_LOCK（直列モード）時は、さらに前方(CHAIN_FRONT_DEG)・後方(CHAIN_BACK_DEG)
//   ±CBF_CHAIN_ZONE_HALF_DEG のゾーン外のトラックも無視する（横の別列や無関係な障害物で
//   不要に減速しないため）。自由運動モード（CHAIN_AXIS_LOCK=false）では従来通り全方位対象。
void applyCBF(float& vx_rpm, float& vy_rpm) {
    float ux = vx_rpm * RPM2MPS, uy = vy_rpm * RPM2MPS;
    cbf_active = 0;
    const float front_rad = CHAIN_FRONT_DEG * DEG_TO_RAD;
    const float back_rad  = CHAIN_BACK_DEG  * DEG_TO_RAD;
    const float half_rad  = CBF_CHAIN_ZONE_HALF_DEG * DEG_TO_RAD;
    for (int pass = 0; pass < 3; pass++) {
        bool changed = false;
        for (int k = 0; k < MAX_TRACKS; k++) {
            if (!trk[k].alive || trk[k].hit < 2) continue;   // 未確定トラックは無視
            if (trk[k].dist_m > CBF_RANGE_MAX_M) continue;   // 適用範囲外は無視
            if (CHAIN_AXIS_LOCK) {
                float dth_f = fabsf(atan2f(sinf(trk[k].bearing - front_rad), cosf(trk[k].bearing - front_rad)));
                float dth_b = fabsf(atan2f(sinf(trk[k].bearing - back_rad),  cosf(trk[k].bearing - back_rad)));
                if (dth_f > half_rad && dth_b > half_rad) continue;   // 前後ゾーン外は無視
            }
            float px = cosf(trk[k].bearing), py = sinf(trk[k].bearing);
            float bnd = CBF_GAMMA * (trk[k].dist_m - D_SAFE);   // リーダー側がCBFを持たない前提でフォロワーが全責任を負う
            float a = px * ux + py * uy;
            if (a > bnd) { ux -= (a - bnd) * px; uy -= (a - bnd) * py; changed = true; cbf_active = 1; }
        }
        if (!changed) break;
    }
    vx_rpm = ux / RPM2MPS; vy_rpm = uy / RPM2MPS;
}

// 指定chの方位に最も近い確定LiDARトラック(hit>=2)を、GATE_DEG以内から1つ選んで返す
// （テレメトリのrobot_dist_m/robot_bearing_deg用。実際のIR方向決定・速度指令には使わない）。
// 戻り値: >=0  一致したtrk[]のインデックス　/　-1  GATE_DEG以内に一致するトラックが無い
int nearestTrackForSensor(int j) {
    float sa = atan2f(SENSOR_SIN[j], SENSOR_COS[j]);
    int best = -1; float best_dth = GATE_DEG * DEG_TO_RAD;
    for (int k = 0; k < MAX_TRACKS; k++) {
        if (!trk[k].alive || trk[k].hit < 2) continue;
        float dth = fabsf(atan2f(sinf(trk[k].bearing - sa), cosf(trk[k].bearing - sa)));
        if (dth <= best_dth) { best_dth = dth; best = k; }
    }
    return best;
}

#define ZONE_LOCK_JUMP_MAX_M  0.3f   // ロック継続中でも、これ以上の急激な距離変化は
                                     // 「同じtrk[]スロットの中身が別物にすり替わった」とみなす

float findNearestTrackInZone(float center_deg, float half_deg, ZoneLock& lock) {
    float center_rad = center_deg * DEG_TO_RAD;

    // 1) 現在ロック中のトラックがまだ生きていて幅も妥当なら、最優先でそれを継続採用する。
    //    ただし、lidarUpdateTracks()側のアソシエーション(±25°/±0.3m)が同じスロットの
    //    中身を別クラスタにすり替えてしまうケースがあるため、ここでも前回値からの
    //    急激なジャンプが無いか毎フレーム確認する。ジャンプしていればロックごと破棄する。
    if (lock.trk_idx >= 0 && trk[lock.trk_idx].alive && trk[lock.trk_idx].hit >= 2 &&
        trk[lock.trk_idx].width_deg <= maxPlausibleWidthDeg(trk[lock.trk_idx].dist_m)) {
        float cur = trk[lock.trk_idx].dist_m;
        float jump = (lock.last_dist > 0) ? fabsf(cur - lock.last_dist) : 0.0f;
        if (jump <= ZONE_LOCK_JUMP_MAX_M) {
            lock.last_dist = cur;
            return cur;
        }
        lock.trk_idx = -1;   // すり替わり検出 → ロック破棄し、下の再選定に回す
    }

    // 2) ロックが切れた場合のみ、ゾーン内の候補から再選定する。
    //    直前の距離に近い候補（±0.3m以内）を優先し、無関係な遠方物へいきなり飛ばないようにする。
    int   best_k = -1; float best_dd = ZONE_LOCK_JUMP_MAX_M;
    for (int k = 0; k < MAX_TRACKS; k++) {
        if (!trk[k].alive || trk[k].hit < 2) continue;
        if (trk[k].width_deg > maxPlausibleWidthDeg(trk[k].dist_m)) continue;  // 壁など広すぎる物体を除外
        float dth = fabsf(atan2f(sinf(trk[k].bearing - center_rad), cosf(trk[k].bearing - center_rad)));
        if (dth > half_deg * DEG_TO_RAD) continue;
        float dd = (lock.last_dist > 0) ? fabsf(trk[k].dist_m - lock.last_dist) : 0.0f;
        if (dd < best_dd) { best_dd = dd; best_k = k; }
    }
    if (best_k >= 0) {
        lock.trk_idx  = best_k;
        lock.last_dist = trk[best_k].dist_m;
        return lock.last_dist;
    }

    // 3) 直前値に近い候補が無ければ本当に未検出。ロックを明示的に解除して99.9を返す
    lock.trk_idx   = -1;
    lock.last_dist = -1.0f;
    return 99.9f;
}

// 指定した中心角度±half_deg以内にある、360ビンの生データ(bin_conf)の平均値を返す。
// findNearestTrackInZone()と異なりtrk[]（CONF_MIN判定を通過した確定トラック）を経由せず、
// bin_conf[]を直接走査するため、CONF_MIN閾値でフィルタされる前の生confidence値を観測できる。
// 鮮度・距離レンジのみでフィルタし、confidence自体でのフィルタはしない
// （閾値ギリギリ/未満の値も含めて実態を見るのが目的）。該当ビンが無ければ-1を返す
// （「未検出」と「たまたまconfidence=0だった」を区別するため）。
float zoneConfAvg(float center_deg, float half_deg) {
    uint32_t now = g_tick;
    long sum = 0; int n = 0;
    int c = (int)(center_deg + 0.5f);
    int half = (int)(half_deg + 0.5f);
    for (int off = -half; off <= half; off++) {
        int b = ((c + off) % 360 + 360) % 360;
        bool fresh    = (now - bin_tick[b] < BIN_MAX_AGE_TICKS);
        bool in_range = (bin_dist[b] >= LIDAR_D_MIN_MM && bin_dist[b] <= LIDAR_D_MAX_MM);
        if (fresh && in_range) { sum += bin_conf[b]; n++; }
    }
    return (n > 0) ? ((float)sum / n) : -1.0f;
}

// LiDAR速度閉ループ（100msごと。機体ごとに10/11/12Hzへ収束させ混信をデシンク）
void lidarSpeedLoop() {
    float target_dps = lidar_hz_target * 360.0f;
    if (lidar_speed_dps > 0) {
        lidar_duty += 0.00002f * (target_dps - (float)lidar_speed_dps);
        lidar_duty = constrain(lidar_duty, 0.35f, 0.60f);
    }
    ledcWrite(LIDAR_PWM_CH, (uint32_t)(lidar_duty * 255.0f));
}

// ================================================================
//  起動時2フェーズ・キャリブレーション
//    Phase A: IR点灯 + 赤 → 自己反射 cal_V[j]（固定）
//    Phase B: IR消灯 + 青 → ベースライン V_base[j]（初期値）
// ================================================================
static void scanAverage(uint32_t settle_ms, uint32_t avg_ms, float out[N_SENSORS]) {
    double acc[N_SENSORS] = {0};
    uint32_t n = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < settle_ms + avg_ms) {
        bool counting = (millis() - t0 >= settle_ms);
        for (int ch = 0; ch < N_SENSORS; ch++) {
            setMuxChannel(ch);
            delay(1);                                  // 実運転と同じ1ms整定
            uint16_t r = readAdcAveraged();
            if (counting) acc[ch] += r;
        }
        if (counting) n++;
        if (wifi_ok) ArduinoOTA.handle();
    }
    for (int j = 0; j < N_SENSORS; j++)
        out[j] = (n > 0) ? (float)(acc[j] / n) / 4095.0f : 0.0f;
}

void startupCalibration() {
    stopAllMotors();
    led_override = LED_OV_NONE;   // ビーコン由来の赤/青上書きが残っていないようにする
    float avgA[N_SENSORS], avgB[N_SENSORS];

    // --- Phase A: 赤発光 + IR点灯 → 自己反射レベル ---
    Serial.println("[CAL] Phase A: IR ON (self-reflection)...");
    fill_solid(leds, NUM_LEDS, COLOR_CAL_A); FastLED.show();
    digitalWrite(IR_CTRL, HIGH);
    scanAverage(CAL_SETTLE_MS, CAL_AVG_MS, avgA);

    // --- Phase B: 青発光 + IR消灯 → ベースライン ---
    Serial.println("[CAL] Phase B: IR OFF (baseline)...");
    digitalWrite(IR_CTRL, LOW);
    fill_solid(leds, NUM_LEDS, COLOR_CAL_B); FastLED.show();
    scanAverage(CAL_SETTLE_MS, CAL_AVG_MS, avgB);

    int floored = 0;
    for (int j = 0; j < N_SENSORS; j++) {
        V_base[j] = avgB[j];
        cal_V[j]  = avgA[j] - avgB[j];       // 正味の自己反射応答
        if (cal_V[j] < CAL_MIN) { cal_V[j] = CAL_MIN; floored++; }
        for (int k = 0; k < 3; k++) med_buf[j][k] = avgB[j];
        V[j] = 0.0f; V_lin[j] = 0.0f; raw_V[j] = 0.0f; det[j] = false;
        Serial.printf("  ch%02d: cal=%.4f  base=%.4f\n", j, cal_V[j], V_base[j]);
    }
    med_idx = 0; med_filled = true;
    sumV = 0.0f; raw_sumV = 0.0f;
    if (floored > 0)
        Serial.printf("[CAL][!] %dチャネルでcal_Vが下限クランプ（自己反射不足。周囲環境を確認）\n", floored);
    fill_solid(leds, NUM_LEDS, COLOR_OFF); FastLED.show();
    Serial.println("[CAL] Done.");
}

// ================================================================
//  2体キャリブレーション（自機のIRはOFFのまま回転し、
//  相方ロボットが発光するIRの受信最大値を各chごとに取得する）
//
//    通常のstartupCalibration()は「自機のIRを点灯して自己反射を測る」方式だが、
//    実際の運用でV_jが検出しているのは自己反射ではなく相方ロボットからのIR発光
//    である。そのため2台で行う場合は、cal_Vも「相方ロボットの発光を実際に
//    どれだけ受信できるか」を基準にする方が実態に即している。
//
//    そこでこのモードでは自機のIRは終始OFFのままにし、代わりに相方ロボットが
//    ビーコンモード（'e 1'コマンドでIR常時点灯）でいる間に、自機だけをその場で
//    回転させる。回転中に全方位を掃引しながら各chのADC最大値を追跡することで、
//    機体がどの向きでビーコンと相対していたかに依存しない「相方ロボットからの
//    受信信号の真のピーク」をcal_Vとして採用できる。
//
//    Phase B（ベースライン）は自機・周囲ともにIRが無い状態で静止して環境光を測定する。
//
//    [!] 相方のビーコンON/OFFタイミングは、ダッシュボードの「選択した2機で相互
//        キャリブレーション」が自動で協調させる（このロボット単体では相方の状態を
//        知る手段が無いため、必ずダッシュボードの当該ボタンから使うこと。手動で
//        'c 1'だけを送っても、相方が発光していなければ有効な値は測れない）。
// ================================================================
#define CAL_ROTATE_SETTLE_MS  300     // 回転開始直後の過渡（立ち上がり）を捨てる時間
#define CAL_ROTATE_MS         4700    // この間の各chの最大値をcal_V算出に使う
                                       // （SETTLE+ROTATE=5000ms。ダッシュボード側の
                                       //  待ち時間もこの合計値を前提にしているので、
                                       //  変更する場合はdashboard.htmlのCAL_ROTATE_TOTAL_MSも要修正）
#define CAL_ROTATE_PWM         100    // 回転用PWM。3輪同符号・同振幅駆動 → 対称omni配置では
                                       // 並進成分が相殺され、その場回転（純回転）になる

void startupCalibrationRotate() {
    stopAllMotors();
    led_override = LED_OV_NONE;   // ビーコン由来の赤/青上書きが残っていないようにする
    float maxA[N_SENSORS];
    for (int j = 0; j < N_SENSORS; j++) maxA[j] = 0.0f;

    // --- Phase A': 自機IRはOFFのまま、その場回転 → 相方ロボットの発光の受信ピークを追跡 ---
    Serial.println("[CAL] Phase A' (2体): own IR stays OFF + rotating,"
                    " tracking per-ch MAX of PARTNER's IR...");
    fill_solid(leds, NUM_LEDS, COLOR_CAL_A); FastLED.show();   // 進行中を示す表示色（自機IRとは無関係）
    digitalWrite(IR_CTRL, LOW);

    setMotorPWM(0, CAL_ROTATE_PWM);
    setMotorPWM(1, CAL_ROTATE_PWM);
    setMotorPWM(2, CAL_ROTATE_PWM);

    uint32_t t0 = millis();
    while (millis() - t0 < CAL_ROTATE_SETTLE_MS) {     // 立ち上がり待ち（記録しない）
        for (int ch = 0; ch < N_SENSORS; ch++) { setMuxChannel(ch); delayMicroseconds(1000); readAdcAveraged(); }
        if (wifi_ok) ArduinoOTA.handle();
    }
    uint32_t t1 = millis();
    while (millis() - t1 < CAL_ROTATE_MS) {
        for (int ch = 0; ch < N_SENSORS; ch++) {
            setMuxChannel(ch);
            delayMicroseconds(1000);
            uint16_t r = readAdcAveraged();
            float x = (float)r / 4095.0f;
            if (x > maxA[ch]) maxA[ch] = x;
        }
        if (wifi_ok) ArduinoOTA.handle();
    }

    // 回転停止。ブロッキング中に貯まった大きなエンコーダ差分が、復帰直後の
    // updatePID()で「PID_DT=5msの間に何百カウントも動いた」という誤ったrpm
    // スパイクとして計算されるのを防ぐため、エンコーダとPID状態を明示的にリセットする。
    setMotorPWM(0, 0); setMotorPWM(1, 0); setMotorPWM(2, 0);
    for (int i = 0; i < 3; i++) {
        pcnt_counter_clear(ENC_UNIT[i]);
        enc_prev[i]     = 0;
        actual_rpm[i]   = 0.0f;
        pid_integral[i] = 0.0f;
        pid_prev_err[i] = 0.0f;
    }

    // --- Phase B: 青発光 + 自機IR OFF → ベースライン（静止。既存Phase Bと同一実装） ---
    // この間は相方ロボットもビーコンOFF（ダッシュボードが自動で切る）になっている想定。
    Serial.println("[CAL] Phase B: baseline (partner should be beacon-OFF by now)...");
    digitalWrite(IR_CTRL, LOW);
    fill_solid(leds, NUM_LEDS, COLOR_CAL_B); FastLED.show();
    float avgB[N_SENSORS];
    scanAverage(CAL_SETTLE_MS, CAL_AVG_MS, avgB);

    int floored = 0;
    for (int j = 0; j < N_SENSORS; j++) {
        V_base[j] = avgB[j];
        cal_V[j]  = maxA[j] - avgB[j];       // 回転中に観測した「相方ロボット発光の受信」最大値を採用
        if (cal_V[j] < CAL_MIN) { cal_V[j] = CAL_MIN; floored++; }
        for (int k = 0; k < 3; k++) med_buf[j][k] = avgB[j];
        V[j] = 0.0f; V_lin[j] = 0.0f; raw_V[j] = 0.0f; det[j] = false;
        Serial.printf("  ch%02d: maxA=%.4f  cal=%.4f  base=%.4f\n", j, maxA[j], cal_V[j], V_base[j]);
    }
    med_idx = 0; med_filled = true;
    sumV = 0.0f; raw_sumV = 0.0f;
    if (floored > 0)
        Serial.printf("[CAL][!] %dチャネルでcal_Vが下限クランプ"
                      "（相方ロボットの発光を検出できていない可能性。配置とビーコン動作を確認）\n", floored);
    fill_solid(leds, NUM_LEDS, COLOR_OFF); FastLED.show();
    Serial.println("[CAL] Done (2体).");
}

// ================================================================
//  WS2812B（IR発光と同時に点灯、変化時のみshow。色・輝度はOTA表示と統一）
// ================================================================
// 位相φ(rad, 任意値)を虹色にマッピングする。φ=0で赤、2π/3で青、4π/3で緑、2πで赤に戻る
// 3色（赤・青・緑）を頂点とした直線RGB補間で、標準HSVの色相順（赤→緑→青）とは逆順の
// 赤→青→緑を実現している。
CRGB phaseToRainbow(float p) {
    p = fmodf(p, TWO_PI);
    if (p < 0.0f) p += TWO_PI;
    const float SEG = TWO_PI / 3.0f;                // 120°ずつ3区間
    int   idx  = (int)(p / SEG);
    if (idx > 2) idx = 2;                            // 浮動小数の丸め対策（p≈2πのとき）
    float frac = (p - idx * SEG) / SEG;               // 区間内位置 0.0〜1.0
    const CRGB& c0 = RAINBOW_ANCHOR[idx];
    const CRGB& c1 = RAINBOW_ANCHOR[(idx + 1) % 3];
    CRGB out;
    out.r = (uint8_t)(c0.r + (c1.r - c0.r) * frac);
    out.g = (uint8_t)(c0.g + (c1.g - c0.g) * frac);
    out.b = (uint8_t)(c0.b + (c1.b - c0.b) * frac);
    return out;
}

// CBF介入中(cbf_active=1)は、従来モード(青パルス)では通常表示より赤を優先する。
// 虹色モードでは位相の視認性を優先し、CBF介入中も虹色表示を維持する（安全表示は従来モードのみ）。
// ir_onだけでなくcbf_activeの変化も監視することで、Winfree駆動窓外で
// CBFだけが単独で介入したケースもLED_TICKS(40ms)周期のリフレッシュで拾える。
// led_rainbow_mode有効時はphiが毎フレーム変化するため、変化検出にphiの差分も加える。
void updateLED() {
    static bool            last_ir_on     = false;
    static uint8_t         last_cbf       = 0;
    static LedOverrideMode last_override  = LED_OV_NONE;
    static float           last_phi_shown = -999.0f;
    static bool            last_running   = false;

    bool rainbow_now = led_rainbow_mode && (led_override == LED_OV_NONE);
    bool phi_moved   = rainbow_now && (fabsf(phi - last_phi_shown) > 0.01f);

    if (!phi_moved && ir_on == last_ir_on && cbf_active == last_cbf
        && led_override == last_override && running == last_running) return;
    last_ir_on    = ir_on;
    last_cbf      = cbf_active;
    last_override = led_override;
    last_running  = running;
    if (rainbow_now) last_phi_shown = phi;

    CRGB c;
    if (led_override == LED_OV_RED)       c = COLOR_CAL_A;   // ビーコンON中: 赤（回転側Phase Aと同色）
    else if (led_override == LED_OV_BLUE) c = COLOR_CAL_B;   // ビーコンOFF後の待機: 青（回転側Phase Bと同色）
    else if (led_rainbow_mode)            c = (!running) ? COLOR_OFF : phaseToRainbow(phi);   // 停止中(STOP)は虹色モードでも消灯
    else                                   c = ir_on ? (cbf_active ? COLOR_CBF : COLOR_OTA) : COLOR_OFF;  // 従来: IR同期青パルス（CBF中は赤）
    fill_solid(leds, NUM_LEDS, c);
    FastLED.show();
}

// ================================================================
//  テレメトリ（UDP 77フィールド）※followerのみ。leaderはcal_Vを持たないため41フィールドのまま。
//   0:id 1:t 2:phi 3:active 4:sumV 5-16:V[0..11] 17:raw_sumV
//   18-20:rpm 21-23:tgt 24-26:pwm
//   27:n_tracks_alive 28:nearest_bearing_deg 29:nearest_dist_m
//   30:cbf_active 31:gate_reject_cnt
//   32:peer_bearing_deg 33:peer_dist_m（IRゲートで実体確認済みの相手ロボットの方位・距離。
//     nearest_*は障害物全般、peer_*はロボットだと確認できたものだけ。未検出時は99.9）
//   34:actual_frame_dt_ms（実測フレーム周期[ms]、FRAME_DT調整用の診断値）
//   35:chain_fwd_dist_m（前方(0°)ゾーンの最近傍距離[m]、直列実験用。未検出時99.9）
//   36:chain_back_dist_m（後方(180°)ゾーンの最近傍距離[m]、直列実験用。未検出時99.9）
//   37:chain_fwd_conf_avg（前方ゾーンの生bin_conf平均。CONF_MIN判定前の値。データ無ければ-1）
//   38:chain_back_conf_avg（後方ゾーンの生bin_conf平均。同上）
//   39:conf_min_current（このロボットが現在使っているCONF_MINの値。fコマンドで変更可）
//   40:yaw_est_deg（3輪エンコーダのデッドレコニングで推定した機体ヨー角[deg]。
//     RUN受信時に0リセットされる相対角。IMU非搭載のため誤差蓄積あり、目安値）
//   41-52:raw_V[0..11]（各chの生電圧。ADC生値/4095、メディアン3フィルタ後・cal_V/V_base
//     正規化前の実測値そのもの。飽和・非対称性の診断用）
//   53-64:cal_V[0..11]（各chの自己反射/2体較正キャリブ値。'c'/'c 1'コマンド実行時に更新）
//   65-76:base_V[0..11]（各chのベースライン。現在はEMA無効化のため起動時Phase Bの値で固定）
//   ※フィールド数を変えたら、受信側(bridge_server.py)のFIELDS定義も必ず更新すること。
//   ※leaderはraw_V/cal_V/base_Vを持たないため41フィールドのまま送信する（bridge_server.py
//     側はパケット長でleader/followerを判別する）
// ================================================================
void sendTelemetry() {
    // 最近傍の確定トラック（Serial表示・UDP共通。無ければdist=99.9で「未検出」を明示）
    int   nearest = -1; float nearest_d = 1e9f; int n_alive = 0;
    for (int k = 0; k < MAX_TRACKS; k++) {
        if (!trk[k].alive || trk[k].hit < 2) continue;
        n_alive++;
        if (trk[k].dist_m < nearest_d) { nearest_d = trk[k].dist_m; nearest = k; }
    }
    float near_bear_deg = (nearest >= 0) ? trk[nearest].bearing * RAD_TO_DEG : 0.0f;
    float near_dist_m   = (nearest >= 0) ? nearest_d : 99.9f;

    // 直列チェーン実験用: 前方/後方それぞれ独立に最近傍を測るので、
    // d(1,2)<d(2,3)のようなケースでも2-3間の距離が消えない
    float chain_fwd_dist_m  = findNearestTrackInZone(CHAIN_FRONT_DEG, CHAIN_ZONE_HALF_DEG, chainFwdLock);
    float chain_back_dist_m = findNearestTrackInZone(CHAIN_BACK_DEG,  CHAIN_ZONE_HALF_DEG, chainBackLock);

    // 反射材の効果検証用: CONF_MIN判定前の生confidence平均（前方/後方ゾーン）
    float chain_fwd_conf_avg  = zoneConfAvg(CHAIN_FRONT_DEG, CHAIN_ZONE_HALF_DEG);
    float chain_back_conf_avg = zoneConfAvg(CHAIN_BACK_DEG,  CHAIN_ZONE_HALF_DEG);

    Serial.printf("phi=%5.3f f=%d Z+=%.3f sumV=%5.3f nDet=%d c=(%+.3f,%+.3f) "
                  "tgt[%5.1f %5.1f %5.1f] rpm[%5.1f %5.1f %5.1f] pwm[%4d %4d %4d] "
                  "lidar[trk=%d near=%.2fm cbf=%d rej=%d peer=%.3fm@%.0f] "
                  "chain[fwd=%.3fm back=%.3fm] run=%d frame_dt=%.2fms yaw=%.1fdeg\n",
                  phi, (int)ir_on, Zplus(phi), sumV, n_detect, vel_cx, vel_cy,
                  pid_target[0], pid_target[1], pid_target[2],
                  actual_rpm[0], actual_rpm[1], actual_rpm[2],
                  mot_pwm[0], mot_pwm[1], mot_pwm[2],
                  n_alive, near_dist_m, cbf_active, gate_reject_cnt,
                  robot_dist_m, robot_bearing_deg,
                  chain_fwd_dist_m, chain_back_dist_m, (int)running, actual_frame_dt_ms,
                  yaw_est * RAD_TO_DEG);
    if (!wifi_ok) return;

    char pkt[900];   // raw_V/cal_V/base_V(各12ch)追加(36フィールド)分、520→900に拡張
    int len = snprintf(pkt, sizeof(pkt),
        "%s,%.3f,%.4f,%d,%.4f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.4f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,"
        "%d,%.1f,%.3f,%d,%d,%.1f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.2f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
        mac_id, (float)millis() / 1000.0f, phi, (int)ir_on, sumV,
        V[0], V[1], V[2], V[3], V[4], V[5],
        V[6], V[7], V[8], V[9], V[10], V[11],
        raw_sumV,
        actual_rpm[0], actual_rpm[1], actual_rpm[2],
        pid_target[0], pid_target[1], pid_target[2],
        mot_pwm[0], mot_pwm[1], mot_pwm[2],
        n_alive, near_bear_deg, near_dist_m, cbf_active, gate_reject_cnt,
        robot_bearing_deg, robot_dist_m,   // IR確認済み相手ロボットの方位・距離
        actual_frame_dt_ms,                // 実測フレーム周期[ms]（FRAME_DT調整用）
        chain_fwd_dist_m,                  // 前方(0°)ゾーンの最近傍距離[m]（直列実験用）
        chain_back_dist_m,                 // 後方(180°)ゾーンの最近傍距離[m]（直列実験用）
        chain_fwd_conf_avg,                // 前方ゾーンの生bin_conf平均（CONF_MIN判定前）
        chain_back_conf_avg,               // 後方ゾーンの生bin_conf平均（CONF_MIN判定前）
        CONF_MIN,                          // 現在のCONF_MIN値（fコマンドで変更可）
        yaw_est * RAD_TO_DEG,               // 推定ヨー角[deg]（RUN受信時0リセット、デッドレコニング）
        raw_V[0], raw_V[1], raw_V[2], raw_V[3], raw_V[4], raw_V[5],
        raw_V[6], raw_V[7], raw_V[8], raw_V[9], raw_V[10], raw_V[11],   // 41-52: 生電圧
        cal_V[0], cal_V[1], cal_V[2], cal_V[3], cal_V[4], cal_V[5],
        cal_V[6], cal_V[7], cal_V[8], cal_V[9], cal_V[10], cal_V[11],   // 53-64: キャリブ値
        V_base[0], V_base[1], V_base[2], V_base[3], V_base[4], V_base[5],
        V_base[6], V_base[7], V_base[8], V_base[9], V_base[10], V_base[11]); // 65-76: ベースライン
    udp.beginPacket(broadcastIP, UDP_PORT);
    udp.write((uint8_t*)pkt, len);
    udp.endPacket();
}

// ================================================================
//  Serial commands
// ================================================================
void printStatus() {
    Serial.printf("phi=%.4f  omega=%.4f(%.2fHz)  kappa=%.3f  tau=%.3f  K_P=%.1f\n",
                  phi, omega, omega / TWO_PI, kappa, tau, K_P_RPM);
    Serial.printf("PID: Kp=%.3f Ki=%.3f Kd=%.3f  FF_MAX=%.0f  V_ON=%.3f V_OFF=%.3f\n",
                  Kp, Ki, Kd, FF_MAX_RPM, V_ON, V_OFF);
    Serial.printf("tau_filt(rise)=%.4f s  tau_filt_fall=%.4f s（0=無効・即時反映）\n", tau_filt, tau_filt_fall);
    Serial.printf("MOTORS_ENABLED=%d  running=%d  dropped_ticks=%lu  ir_force_on=%d\n",
                  (int)MOTORS_ENABLED, (int)running, (unsigned long)dropped_ticks, (int)ir_force_on);
    Serial.printf("V_DECAY_EXP=%.2f（1.0=従来、raw∝1/d²のまま。大きいほど遠距離を急峻に減衰）\n",
                  V_DECAY_EXP);
    Serial.printf("yaw_est=%.2f°  YAW_HOLD_KP=%.3f（デッドレコニング推定、RUN受信時に0リセット）\n",
                  yaw_est * RAD_TO_DEG, YAW_HOLD_KP);
    for (int j = 0; j < N_SENSORS; j++)
        Serial.printf("  ch%02d: V=%.4f base=%.4f cal=%.4f det=%d\n",
                      j, V[j], V_base[j], cal_V[j], (int)det[j]);
    Serial.println("Cmds: o k t g P I D L v p a j c[0/1] e[0/1] ? | 1/2/3 <rpm> | 0");
    Serial.printf("OTA: host=%s  IP=%s\n", ota_hostname, WiFi.localIP().toString().c_str());
}

// Serial/UDP共通のコマンド処理。1行分の文字列を受け取り、既存のシリアルコマンド書式
// （例: "o 6.28"）をそのままパースする。呼び出し元(Serial/UDP)は違いを意識しない。
void processCommand(const String& line) {
    if (!line.length()) return;
    char  cmd = line[0];
    float val = (line.length() > 2) ? line.substring(2).toFloat() : 0.0f;

    switch (cmd) {
        case 'o': omega = val;
                  Serial.printf("omega = %.4f rad/s (%.4f Hz)\n", omega, omega / TWO_PI); break;
        case 'k': kappa = val;
                  Serial.printf("kappa = %.4f\n", kappa); break;
        case 't': tau = val;
                  Serial.printf("tau   = %.4f\n", tau); break;
        case 'g': K_P_RPM = val;
                  Serial.printf("K_P_RPM = %.2f\n", K_P_RPM); break;
        case 'v': V_ON = val; V_OFF = 0.6f * val;
                  Serial.printf("V_ON = %.4f  V_OFF = %.4f\n", V_ON, V_OFF); break;
        case 'p': phi = fmodf(val, TWO_PI);
                  if (phi < 0.0f) phi += TWO_PI;
                  Serial.printf("phi -> %.4f rad\n", phi); break;
        case 'P': Kp = val;
                  Serial.printf("Kp = %.4f\n", Kp); break;
        case 'I': Ki = val;
                  for (int i = 0; i < 3; i++) pid_integral[i] = 0;
                  Serial.printf("Ki = %.4f (integral reset)\n", Ki); break;
        case 'D': Kd = val;
                  Serial.printf("Kd = %.4f\n", Kd); break;
        case 'L': tau_filt = fmaxf(val, 0.0f);
                  Serial.printf("tau_filt(rise) = %.4f s (%s)\n", tau_filt,
                                (tau_filt > 1e-4f) ? "有効" : "無効・素通し"); break;
        case 'F': tau_filt_fall = fmaxf(val, 0.0f);
                  Serial.printf("tau_filt_fall = %.4f s (%s)\n", tau_filt_fall,
                                (tau_filt_fall > 1e-4f) ? "有効" : "無効・素通し"); break;
        case 's': D_SAFE = val;
                  Serial.printf("D_SAFE = %.3f m\n", D_SAFE); break;
        case 'f': CONF_MIN = val;
                  Serial.printf("CONF_MIN = %.1f\n", CONF_MIN); break;
        case 'q': lidar_hz_override = val;   // LD19仕様上5〜13Hzに収まるよう運用者(bridge_server.py)側で選ぶこと
                  lidar_hz_target   = val;   // 即時反映
                  Serial.printf("lidar_hz_target = %.1f Hz (override)\n", lidar_hz_target); break;
        case 'G': CBF_GAMMA = val;
                  Serial.printf("CBF_GAMMA = %.3f\n", CBF_GAMMA); break;
        // 'l' コマンド（LIDAR_GATE_EN）は既定offのまま使われていなかったため削除しました。
        case 'a': CHAIN_AXIS_LOCK = (val != 0.0f);
                  Serial.printf("CHAIN_AXIS_LOCK = %d\n", (int)CHAIN_AXIS_LOCK); break;
        case 'j': CHAIN_LATERAL_GAIN = constrain(val, 0.0f, 1.0f);
                  Serial.printf("CHAIN_LATERAL_GAIN = %.2f\n", CHAIN_LATERAL_GAIN); break;
        case 'Z': PRC_SELECT = (val != 0.0f);
                  Serial.printf("PRC_SELECT = %d (%s)\n", (int)PRC_SELECT,
                                PRC_SELECT ? "floor型: floor(phi/pi)*(2pi-phi)"
                                           : "sin型: max(-sin(phi),0)"); break;
        case 'K': K_D = val;
                  Serial.printf("K_D (damper) = %.4f\n", K_D); break;
        case 'x': V_DECAY_EXP = constrain(val, 0.1f, 8.0f);   // 距離減衰の指数（1=従来、大きいほど遠距離を急峻に減衰）
                  Serial.printf("V_DECAY_EXP = %.2f\n", V_DECAY_EXP); break;
        case 'y': YAW_HOLD_KP = val;   // ヨー保持Pゲイン（0=無効。符号・大きさは実機調整）
                  Serial.printf("YAW_HOLD_KP = %.3f\n", YAW_HOLD_KP); break;
        case 'm': MOTORS_ENABLED = (val != 0.0f);
                  Serial.printf("MOTORS_ENABLED = %d\n", (int)MOTORS_ENABLED); break;
        case 'r': led_rainbow_mode = (val != 0.0f);   // WS2812B: 位相虹色表示(既定1) / 0でIR同期青パルスに戻す
                  updateLED();
                  Serial.printf("led_rainbow_mode = %d\n", (int)led_rainbow_mode); break;
        case 'e': ir_force_on = (val != 0.0f);   // ビーコンモード: 他ロボットのキャリブレーション基準用にIR常時点灯
                  if (!ir_force_on) {
                      // ビーコン解除: 即座に消灯し、次の明示的な状態変化（RUN/STOP/再ビーコン/
                      // キャリブレーション開始）までWS2812Bは青を維持する
                      ir_on = false;
                      digitalWrite(IR_CTRL, LOW);
                      led_override = LED_OV_BLUE;
                  } else {
                      led_override = LED_OV_RED;
                  }
                  updateLED();
                  Serial.printf("ir_force_on = %d (beacon mode)\n", (int)ir_force_on); break;
        case 'c': if (val >= 0.5f) startupCalibrationRotate();  // "c 1": 2体（回転してmax取得）
                  else             startupCalibration();        // "c"/"c 0": 1体（従来通り静止）
                  break;   // ブロッキング分はloop側で自動リシンク
        case '1': manual_override = true;
                  pid_target[0] = constrain(val, -CMD_RPM_LIMIT, CMD_RPM_LIMIT);
                  Serial.printf("[M1] target=%.1f rpm (manual)\n", pid_target[0]); break;
        case '2': manual_override = true;
                  pid_target[1] = constrain(val, -CMD_RPM_LIMIT, CMD_RPM_LIMIT);
                  Serial.printf("[M2] target=%.1f rpm (manual)\n", pid_target[1]); break;
        case '3': manual_override = true;
                  pid_target[2] = constrain(val, -CMD_RPM_LIMIT, CMD_RPM_LIMIT);
                  Serial.printf("[M3] target=%.1f rpm (manual)\n", pid_target[2]); break;
        case '0': manual_override = false;
                  stopAllMotors();
                  Serial.println("[MOT] all stop (auto mode resumed)"); break;
        default:  printStatus(); break;
    }
}

void handleSerial() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    processCommand(line);
}

// CMD_PORTで受信する全コマンド（ダッシュボードのパラメータ変更、および一斉RUN/STOPの両方）を
// 単一のソケットで一括処理する。
// 以前はRUN/STOPをテレメトリ用ソケット(UDP_PORT)のブロードキャストで受けていたが、
// 全機のテレメトリ(10Hz×台数)が同じソケットの受信キューに殺到し、台数が増えるほど
// RUNコマンドが埋もれて届かなくなる問題があった。CMD_PORTはコマンド専用で流量が少ないため、
// ここに一本化することで台数に依存せず確実に届くようにする。
// また、以前は同じcmdUdpを2つの関数が別々にparsePacket()していたため、1周期で片方しか
// パケットを受け取れない競合があった点も、1つの関数にまとめることで解消している。
void handleCmdPort() {
    int psize = cmdUdp.parsePacket();
    if (psize <= 0) return;
    char buf[64];
    int n = cmdUdp.read(buf, sizeof(buf) - 1);
    if (n <= 0) return;
    buf[n] = '\0';
    String line = String(buf);
    line.trim();

    if (line == "RUN" || line == "START") {
        running = true;
        // [バグ修正] ビーコンモード('e 1')がONのまま残っていると、processFrame()内で
        // ir_force_onのチェックがrunningの判定より先に来るため、running=trueにしても
        // モーターが強制停止され続けて動かない事故が起きる。RUN受信時は必ず解除する。
        if (ir_force_on) {
            ir_force_on = false;
            ir_on = false;
            digitalWrite(IR_CTRL, LOW);
        }
        // ヨー角推定はデッドレコニング（積分）なので長時間で誤差が蓄積する。
        // RUN=直列走行の開始時点を「まっすぐ」の基準(0)とみなしてリセットする。
        yaw_est = 0.0f;
        led_override = LED_OV_NONE;
        Serial.println("[RUN] started");
        return;
    }
    if (line == "STOP") {
        running = false;
        // STOP時も同様にビーコンモードを解除しておく（残存を防ぐ安全策）
        if (ir_force_on) {
            ir_force_on = false;
            ir_on = false;
            digitalWrite(IR_CTRL, LOW);
        }
        led_override = LED_OV_NONE;
        Serial.println("[RUN] stopped (standby, calibration保持)");
        return;
    }

    Serial.printf("[UDP-CMD] %s\n", line.c_str());
    processCommand(line);
}

// ================================================================
//  OTA（パスワードなし）
// ================================================================
void setupOTA() {
    ArduinoOTA.setHostname(ota_hostname);
    ArduinoOTA.onStart([]() {
        ota_active = true;
        digitalWrite(IR_CTRL, LOW);      // IR_CTRLはLEDC非使用なのでdigitalWriteが有効
        stopAllMotors();
        fill_solid(leds, NUM_LEDS, COLOR_OTA); FastLED.show();
        Serial.printf("\n[OTA] Start: %s\n",
            (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
    });
    ArduinoOTA.onEnd([]() {
        fill_solid(leds, NUM_LEDS, COLOR_OFF); FastLED.show();
        Serial.println("[OTA] End. Rebooting...");
    });
    ArduinoOTA.onProgress([](unsigned int cur, unsigned int total) {
        int lit = (int)((float)cur / total * NUM_LEDS + 0.5f);
        for (int i = 0; i < NUM_LEDS; i++) leds[i] = (i < lit) ? COLOR_OTA : COLOR_OFF;
        FastLED.show();
    });
    ArduinoOTA.onError([](ota_error_t error) {
        ota_active = false;
        fill_solid(leds, NUM_LEDS, CRGB::Red); FastLED.show();
        const char* msg[] = {"", "Auth Failed", "Begin Failed", "Connect Failed", "Receive Failed", "End Failed"};
        Serial.printf("[OTA] Error[%u]: %s\n", error, (error <= OTA_END_ERROR) ? msg[error] : "Unknown");
        delay(2000);
        fill_solid(leds, NUM_LEDS, COLOR_OFF); FastLED.show();
    });
    ArduinoOTA.begin();
    Serial.printf("[OTA] Ready  host=%s  IP=%s  port=3232 (no password)\n",
                  ota_hostname, WiFi.localIP().toString().c_str());
}

// ================================================================
//  setup
// ================================================================
void setup() {
    Serial.setTxBufferSize(1024);        // 状態出力でloopをブロックしないように
    Serial.begin(115200);

    pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT); pinMode(MUX_S3, OUTPUT);
    setMuxChannel(0);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);   // GPIO35
    pinMode(IR_CTRL, OUTPUT); digitalWrite(IR_CTRL, LOW);

    FastLED.addLeds<WS2812B, LED_DIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(150);
    fill_solid(leds, NUM_LEDS, COLOR_OFF); FastLED.show();

    // --- LiDAR (LD19) ---
    initLidarCrc();
    Serial1.setRxBufferSize(16384);
    Serial1.begin(230400, SERIAL_8N1, LIDAR_RX_PIN, -1);   // 受信のみ
    ledcSetup(LIDAR_PWM_CH, LIDAR_PWM_FREQ, MOT_BITS);
    ledcAttachPin(LIDAR_PWM_PIN, LIDAR_PWM_CH);
    ledcWrite(LIDAR_PWM_CH, 128);                          // duty 50% → 外部速度制御トリガ

    setupMotors();
    setupEncoders();
    setupOmni();

    // --- WiFi（キャリブ前に接続し、以後OTAを常時受付） ---
    WiFi.mode(WIFI_STA);
    for (auto& cred : WIFI_CANDIDATES) {
        WiFi.begin(cred[0], cred[1]);
        Serial.printf("[WiFi] trying %s", cred[0]);
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 5000) {
            delay(200); Serial.print('.');
        }
        if (WiFi.status() == WL_CONNECTED) break;
        WiFi.disconnect(); Serial.println(" failed.");
    }

    uint8_t mac[6]; WiFi.macAddress(mac);
    snprintf(mac_id,       sizeof(mac_id),       "%02X%02X%02X",         mac[3], mac[4], mac[5]);
    snprintf(ota_hostname, sizeof(ota_hostname), "winfree-%02X%02X%02X", mac[3], mac[4], mac[5]);

    lidar_hz_target = LIDAR_HZ_FORCE_SAME ? LIDAR_HZ_FORCED_VALUE
                                           : (10.0f + (float)(mac[5] % 4));   // 10/11/12/13Hz（LD19仕様上限13Hz厳守）
    // 起動直後はbridge_server.py側からのq指定がまだ届いていない可能性があるため、
    // 上記MACハッシュ値を暫定として使う。'q'コマンド受信後は即座に上書きされる。
    if (lidar_hz_override >= 0.0f) lidar_hz_target = lidar_hz_override;
    Serial.printf("[LIDAR] UART1 RX=%d  target=%.0fHz (desync)  D_SAFE=%.2fm\n",
                  LIDAR_RX_PIN, lidar_hz_target, D_SAFE);

    if (WiFi.status() == WL_CONNECTED) {
        wifi_ok = true;
        WiFi.setSleep(false);            // モデムスリープ由来の約1Hz ADCスパイク対策（必須）
        udp.begin(UDP_PORT);
        cmdUdp.begin(CMD_PORT);          // ダッシュボードからのパラメータ変更コマンド受信
        uint32_t ip = (uint32_t)WiFi.localIP(), mask = (uint32_t)WiFi.subnetMask();
        broadcastIP = IPAddress(ip | ~mask);
        Serial.printf("\n[WiFi] %s  ID:%s  bcast:%s  cmd_port:%d\n",
                      WiFi.localIP().toString().c_str(), mac_id, broadcastIP.toString().c_str(), CMD_PORT);
        setupOTA();
    } else {
        Serial.println("\n[WiFi] timeout — UDP/OTA disabled");
    }

    // --- キャリブレーション（赤→青） ---
    startupCalibration();

    phi = 0.0f;

    // --- ハードウェアタイマー開始（全周期処理の駆動源。最後に起動） ---
    tick_timer = timerBegin(0, 80, true);            // 80MHz/80 = 1µs tick
    timerAttachInterrupt(tick_timer, &onTick1kHz, true);
    timerAlarmWrite(tick_timer, 1000, true);         // 1000µs = 1kHz, auto-reload
    timerAlarmEnable(tick_timer);

    Serial.println("\n=== Winfree Oscillator v2 (HW-timer scheduled, 直列実験対応) ===");
    Serial.printf("phi0=%.4f  omega=%.4f  kappa=%.4f  tau=%.4f  K_P=%.1f\n",
                  phi, omega, kappa, tau, K_P_RPM);
    Serial.printf("Frame=%dms(phase dt)  PID=%dms  V_ON=%.3f\n",
                  FRAME_TICKS, PID_TICKS, V_ON);
    Serial.println("Motors: 搭載 M1=90°(J3) M2=210°(J7) M3=330°(J11) / 駆動方向 180°/60°/300°");
    Serial.println("Sensors: ch_j → j×30°（実機検証済み）");
    Serial.println("[!] 起動直後はrunning=false（スタンバイ）。ダッシュボードの「一斉起動」で開始すること。");
}

// ================================================================
//  loop（周期処理はすべて g_tick 起点でディスパッチ。Δtは固定値を使用）
// ================================================================
void loop() {
    if (wifi_ok) ArduinoOTA.handle();
    if (ota_active) return;

    handleSerial();
    handleCmdPort();   // ダッシュボードからのパラメータ変更・一斉RUN/STOPを一括処理（非ブロッキング）
    lidarPoll();                                  // 非ブロッキング・バイト単位パース（毎ループ）

    static uint32_t sens_tick = 0, pid_tick = 0, led_tick = 0, tel_tick = 0, lidar_tick = 0;
    static bool first = true;
    uint32_t tick = g_tick;
    if (first) { sens_tick = pid_tick = led_tick = tel_tick = lidar_tick = tick; first = false; }

    // --- センサ 1ch/1ms（遅延時は上限まで追従処理、超過分は破棄してリシンク） ---
    int catchup = 0;
    while ((int32_t)(tick - sens_tick) > 0) {
        sens_tick++;
        sensorTick();
        if (++catchup >= MAX_CATCHUP_TICKS) {
            uint32_t now = g_tick;
            if ((int32_t)(now - sens_tick) > 0) {
                dropped_ticks += (now - sens_tick);
                sens_tick = now;
            }
            break;
        }
    }

    // --- PID 5ms（Δt=PID_DT固定） ---
    tick = g_tick;
    if ((int32_t)(tick - pid_tick) >= PID_TICKS) {
        pid_tick += PID_TICKS;
        updatePID();
        if ((int32_t)(tick - pid_tick) >= 4 * PID_TICKS) pid_tick = tick;  // 大幅遅延はリシンク
    }

    // --- LiDAR 10Hz: クラスタ+トラック更新 + 速度デシンクループ ---
    // ※running状態に関わらず常時実行（停止中でもチェーン距離を監視できるようにするため）
    if ((int32_t)(tick - lidar_tick) >= 100) {
        lidar_tick = tick;
        lidarUpdateTracks();
        lidarSpeedLoop();
    }

    // --- LED 25Hz ---
    if ((int32_t)(tick - led_tick) >= LED_TICKS) {
        led_tick = tick;
        updateLED();
    }

    // --- テレメトリ 10Hz ---
    if ((int32_t)(tick - tel_tick) >= TELEM_TICKS) {
        tel_tick = tick;
        sendTelemetry();
    }
}
