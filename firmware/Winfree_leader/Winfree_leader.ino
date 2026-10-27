/*
 * Winfree_LEADER_chain.ino  —  直列実験用リーダーファームウェア（軽量・HW-timer駆動版、LiDAR搭載）
 *
 * ■ このファイルについて
 *     PS4Controller（Bluetooth Classicスタック）は削除済み（flash 117%オーバーの主因だったため。
 *     緊急停止はUDPの"STOP"ブロードキャストで代替）。
 *
 *     Winfreeの位相振動子（φの積分・IR発光）は維持する。リーダーの並進運動（等速直線運動）は
 *     位相とは完全に独立したFF+PID駆動で行う。
 *
 *     LiDAR(LD19)処理はフォロワーと全く同じ実装（UART受信・CRC8・360ビン・クラスタリング・
 *     トラック追跡・前方/後方ゾーン距離測定）を移植した。以前は全てプレースホルダ(99.9)
 *     だったchain_fwd_dist_m/chain_back_dist_mが実測値になる。
 *     ただしフォロワーが持つIRゲート（lidarConfirmTrack、IR検出とLiDARトラックの突き合わせに
 *     よる相手ロボット実体確認）はリーダーには実装していないため、peer_bearing_deg/peer_dist_m
 *     は引き続きプレースホルダのまま。CBF（衝突回避）もリーダーの並進運動には未接続。
 *
 * ■ モータ駆動方式: IN/IN slow-decay（follower(IN/IN方式)と統一。GPIOは変更なし）
 *     低速指令（RPM=10〜20程度）での起動遅延・速度むらの改善のため、元のPhase/Enable
 *     （常時fast decay）方式から変更した。詳細はsetupMotors()/setMotorPWM()のコメント参照。
 *
 * ■ スケジューリング（ハードウェアタイマー1kHz tickが唯一の駆動源。フォロワーと同じ設計）:
 *     1ms   : MUX 1ch読み取り（センサ12chを12ms/周期でスキャン）
 *     12ms  : 1フレーム完了 → V_j算出 → 位相Euler積分(Δt=0.012固定) → IR発光更新
 *     5ms   : エンコーダ(PCNT)読取 → FF+PID → PWM出力（並進運動は位相と無関係）
 *     40ms  : WS2812B 発光状態表示
 *     100ms : LiDARクラスタ+トラック更新 + 速度デシンクループ（フォロワーと同一周期）
 *     100ms : シリアル状態出力 + UDPテレメトリ（bridge_server.py/dashboard.html互換の37フィールド）
 *
 * ■ UDPブロードキャストコマンド: RUN/START（起動） STOP（停止）
 * ■ Serial/UDPパラメータコマンド: o/k/t（Winfree位相） g（駆動RPM） P/I/D（PID）
 *     y（ヨー保持Pゲイン、既定2.0。'y 0'で無効） w（M3直進トリム、既定1.0。タイヤ半径差の較正用）
 *     c（再較正） ?（状態表示）
 * ■ 手動操縦（PC矢印キー、dashboard.html経由）: "M <fwd_rpm> <side_rpm>"（2値コマンド、
 *     ダッシュボードがキー押下中150ms間隔で再送）。side_rpmは横方向（平行移動）の指令で、
 *     機体の向きは変えない（3輪オムニの運動学に基づきM2=主動・M1/M3=半分で分担）。
 *     running/ir_force_onとは独立に動作する位置決め用の直接RPM指令で、以前削除した
 *     PS4Controllerの代替。最後の'M'コマンドからMANUAL_TIMEOUT_MS(400ms)以内に次が
 *     来なければ自動解除・安全停止する。RUN/STOP/'0'コマンドでも明示的に解除される。
 */

#include <Arduino.h>
#include <FastLED.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <ArduinoOTA.h>
#include <esp_system.h>
#include <math.h>
#include "driver/pcnt.h"
#include "driver/adc.h"

// ================================================================
//  WiFi / OTA
// ================================================================
// Wi-Fi候補とOTAパスワードは secrets.h に分離（Gitに含めない）。
// secrets.h.example をコピーして secrets.h を作り、値を書き換えること。
#include "secrets.h"
#define UDP_PORT       4210
#define CMD_PORT       4211    // ダッシュボードからのパラメータ変更コマンド受信専用

WiFiUDP    udp;
WiFiUDP    cmdUdp;
bool       wifi_ok    = false;
volatile bool ota_active = false;
char       mac_id[8];
char       ota_hostname[24];
IPAddress  broadcastIP;

// ================================================================
//  Pins — Sensor / IR / LED（follower同一基板を想定）
// ================================================================
#define MUX_S0   16
#define MUX_S1    5
#define MUX_S2   18
#define MUX_S3   19
#define MUX_SIG  35            // ADC1_CH7
#define IR_CTRL  25            // digitalWrite専用。ledcAttachPinしないこと
#define LED_DIN  26
#define NUM_LEDS 12

// ================================================================
//  Pins — Motors (DRV8835 IN/IN, MODE=GND。followerと同一方式に統一)
//   [変更履歴] 元々Phase/Enable方式だったが、常時fast decay（OFF区間で完全空転）
//   になる特性上、低速指令（RPM=10〜20程度）で電流リプル＝トルクむらが大きく、
//   起動が遅れたり速度が不安定になる問題が確認された。followerが元々使っている
//   IN/IN slow-decay方式（OFF区間もブレーキがかかり電流リプルが小さい）に統一し、
//   低速安定性を改善する。GPIOピン自体は変更なし（配線変更不要、ソフトのみの変更）。
// ================================================================
#define MOT1_IN1  32
#define MOT1_IN2  33
#define MOT2_IN1  27
#define MOT2_IN2  14
#define MOT3_IN1  13
#define MOT3_IN2  12

static const int CH_IN1[3] = { 0, 2, 4 };   // LEDCチャネル
static const int CH_IN2[3] = { 1, 3, 5 };
#define MOT_FREQ   20000
#define MOT_BITS   8

// ================================================================
//  Pins — Encoders (PCNTハードウェアデコード。フォロワーと同一方式)
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
static const int   ENC_POL[3] = { 1, 1, 1 };  // [!] 実機確認すること
static const pcnt_unit_t ENC_UNIT[3] = { PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2 };

// ================================================================
//  Pins — LiDAR (LD19)。followerと同一実装
// ================================================================
#define LIDAR_RX_PIN     23    // LD19 Tx → ESP32 RX（受信のみ、UART1）
#define LIDAR_PWM_PIN    22    // 外部速度制御（デシンク用）
#define LIDAR_PWM_CH      6    // ch0/2/4はモータ3ch使用済み。ch6は独立
#define LIDAR_PWM_FREQ 30000

#define LIDAR_HZ_FORCE_SAME  false
#define LIDAR_HZ_FORCED_VALUE 10.0f
// followerと同一理由。詳細はfollower側コメント参照
float lidar_hz_override = -1.0f;

// ================================================================
//  スケジューリング定数（ハードウェアタイマー1kHz tickが基準。フォロワーと共通の設計）
// ================================================================
#define TICK_HZ            1000
#define N_SENSORS          12
#define FRAME_TICKS        N_SENSORS            // 12ms/フレーム
static const float FRAME_DT = 0.012f;
#define PID_TICKS             5                 // 5ms
static const float PID_DT   = 0.005f;
#define LED_TICKS             10                // 25Hz
#define TELEM_TICKS           100               // 10Hz
#define LIDAR_TICKS           100               // 10Hz（フォロワーと同一周期）
#define MAX_CATCHUP_TICKS     24
#define IR_OFF_GUARD_TICKS    10
#define ADC_OVERSAMPLE         4
#define BASELINE_ALPHA      0.01f

// ================================================================
//  LiDAR パラメータ（followerと同一値）
// ================================================================
#define LIDAR_PKT_LEN        47
float CONF_MIN = 60.0f;   // confidence下限（無線調整可。followerと同じ理由で90→70に引き下げ）
#define LIDAR_D_MIN_MM       50
#define LIDAR_D_MAX_MM       800     // 80cmまで
#define LIDAR_R_TARGET_M  0.0275f    // 相手LiDAR表面→中心の半径補正
#define LIDAR_MOUNT_OFFSET_DEG -90.0f // 要実機較正（既知方向に対向させて確認）
#define ROBOT_TARGET_DIAM_MAX  0.08f // 要実機較正（followerと共通の初期値。個体ごとに要確認）
#define BIN_MAX_AGE_TICKS   150
#define MAX_TRACKS            60
#define TRACK_MISS_LIMIT      5     // 連続ミス許容スキャン数。followerと同一理由で2→5に緩和

// 直列チェーン距離測定（followerと同一設計。実機検証の結果、機体座標0°側が
// 物理的な「後方」・180°側が「前方」に対応することが全機共通で確認されたため、
// CHAIN_FRONT_DEG/CHAIN_BACK_DEGを入れ替えている。follower側と同じ修正）
#define CHAIN_FRONT_DEG   180.0f
#define CHAIN_BACK_DEG      0.0f
#define CHAIN_ZONE_HALF_DEG 30.0f

// ================================================================
//  Winfree パラメータ（位相振動子。並進運動とは独立）
// ================================================================
float omega = TWO_PI * 1.3f;
float kappa = 0.0f;
float tau   = 0.9f;

// ================================================================
//  並進運動パラメータ（位相と無関係。FF+PIDで等速直線運動を行う）
// ================================================================
float CONST_DRIVE_RPM = 30.0f;
#define MAX_RPM       170.0f
#define CMD_RPM_LIMIT 240.0f

// ================================================================
//  直進性補正（問題②対策）
//   (1) ヨー保持: IMU非搭載のため、3輪エンコーダのデッドレコニングでヨー角を推定する。
//       120°等間隔配置では3輪の速度を単純合算すると並進成分が打ち消し合い、
//       回転成分だけが残る:  ω = (w1+w2+w3)·RPM2MPS / (3·CHASSIS_R_M)
//       これをPID_DTごとに積分してyaw_estとし、目標0との誤差をP制御で3輪に
//       「同じ量だけ」上乗せする（3輪同一の上乗せ=並進を変えない純回転）。
//       推定も補正も同じ「3輪合算」座標で閉じているため、YAW_HOLD_KP>0であれば
//       符号は構成上必ず負帰還になる（実機で符号確認は不要。大きさのみ調整）。
//       RUN受信時にyaw_est=0リセット（走行開始時の向きを「まっすぐ」の基準とする）。
//   (2) ホイールトリム: M1/M3の実効タイヤ半径差など、エンコーダには映らない
//       系統誤差による曲がりを補正する較正パラメータ。M3の目標RPMに乗算する。
//       較正手順: トリム1.0・ヨー保持ONで走らせ、それでも一定方向へ曲がる場合に
//       'w 1.02'のように±数%を与えて直進するまで追い込む（走行距離Lで横ずれdなら
//       目安 trim ≈ 1 + 2d/L 程度から開始）。
// ================================================================
#define WHEEL_RADIUS_M  0.019f     // followerと同一値
static const float RPM2MPS = TWO_PI * WHEEL_RADIUS_M / 60.0f;
#define CHASSIS_R_M     0.092f     // 実測値: ロボット中心→駆動輪接地点の距離（followerと同一）
float YAW_HOLD_KP   = 2.0f;        // ヨー保持Pゲイン[rad/s per rad]。'y 0'で無効化可能
float yaw_est       = 0.0f;        // 推定ヨー角[rad]（RUN受信時0リセット）
float yaw_ref       = 0.0f;        // 手動操縦中のヨー保持の目標角[rad]。平行移動を始めた時点の向き
float STRAIGHT_TRIM = 1.0f;        // M3目標RPMに乗算する較正係数（'w'コマンド）

float Kp = 15.0f;   // followerと完全一致
float Ki = 5.0f;    // followerと完全一致
float Kd = 0.0f;
#define FF_MAX_RPM    240.0f
#define MOT_DEADBAND  25
#define RPM_EMA_A     0.5f
float tau_filt = 0.03f;   // [s] 速度指令の一次遅れフィルタ（followerと同一。ガタつき対策）

// ================================================================
//  状態
// ================================================================
volatile uint32_t g_tick = 0;
hw_timer_t* tick_timer = nullptr;

// --- Winfree位相振動子の状態 ---
float    phi = 0.0f;
bool     ir_on = false;
uint32_t ir_off_guard_end = 0;
uint16_t raw_frame[N_SENSORS];
float    V[N_SENSORS];
float    V_base[N_SENSORS];
float    sumV = 0.0f, raw_sumV = 0.0f;
float    actual_frame_dt_ms = 0.0f;

// --- 並進運動（FF+PID）の状態 ---
float    pid_target[3]      = {0, 0, 0};
float    pid_target_filt[3] = {0, 0, 0};   // pid_targetへ一次遅れフィルタを適用した後の値
float    actual_rpm[3]      = {0, 0, 0};
float    pid_integral[3] = {0, 0, 0};
float    pid_prev_err[3] = {0, 0, 0};
int16_t  enc_prev[3]     = {0, 0, 0};
int      mot_pwm[3]      = {0, 0, 0};

// [バグ修正] PCNTの単発グリッチ対策: PID_DT窓での物理上限
// (CMD_RPM_LIMIT=240rpmなら PID_DT=5ms→約28カウント、20ms→約112カウント)
// に対し、ノイズ等で数万カウント相当のスプリアスな差分が単発で入ることがある
// （実測でrpm=-30701という物理的にあり得ない値がyaw_estを一瞬±150°も
// 暴れさせた事例を確認）。PID_TICKS/PID_DTを何msに変えても正しい値になる
// よう、決め打ちせずCMD_RPM_LIMIT・PID_DTから計算する（マージン5倍）。
static const int ENC_GLITCH_MAX_COUNT =
    (int)(COUNTS_PER_OUT_REV * CMD_RPM_LIMIT / 60.0f * PID_DT * 5.0f + 0.5f);

uint32_t dropped_ticks = 0;
bool     running = false;

// --- 手動操縦（PCの矢印キー、'M'コマンド）の状態 ---
// Winfreeのrunning/ir_force_on状態とは独立に動作する（位置決め・取り回し用途。
// 以前削除したPS4Controllerの代替）。CMD_PORT経由で"M <fwd_rpm> <side_rpm>"を
// 受けるたびに更新・タイムスタンプ更新する。MANUAL_TIMEOUT_MSを超えて次のコマンドが
// 来なければ自動的に解除して安全停止する（ブラウザのタブを閉じた・通信切断時の暴走防止）。
bool     manual_mode        = false;
float    manual_fwd_rpm     = 0.0f;
float    manual_side_rpm    = 0.0f;   // 横移動（平行移動）指令[rpm相当]。正が「右」の想定（実機で逆なら'M'の第2引数の符号をdashboard.html側で反転すること）
float    manual_rot_rpm     = 0.0f;   // その場回転指令[rpm相当]。3輪すべてに同じ値を加算する（120°等配置の対称性により並進成分を持たない純回転になる）。正がどちら回りかは実機で要確認（逆ならdashboard.html側のQ/Eの符号を反転すること）
uint32_t manual_last_cmd_ms = 0;
#define  MANUAL_TIMEOUT_MS   400

// ビーコンモード: trueの間はWinfree位相・RUN/STOP状態に関係なくIRを常時点灯し続け、
// 並進運動も止める。follower側の「2体キャリブレーション」（自機IR OFF+回転して
// 相方ロボットの発光を読むモード）の基準発光源として、leaderを一時的に使うためのもの。
// ダッシュボードの「選択した2機で相互キャリブレーション」から自動制御される内部状態。
bool     ir_force_on = false;

// ビーコンのON/OFFに合わせてWS2812Bを赤/青に固定表示するための上書きフラグ（followerと同一設計）
enum LedOverrideMode : uint8_t { LED_OV_NONE = 0, LED_OV_RED = 1, LED_OV_BLUE = 2 };
LedOverrideMode led_override = LED_OV_NONE;

// 位相虹色表示モード（followerと同一設計、既定有効）。'r 0'コマンドで従来のIR同期青パルスに戻せる。
bool led_rainbow_mode = true;

// --- LiDAR / トラック状態（followerと同一） ---
struct Track { bool alive; float bearing; float dist_m; uint8_t hit; uint8_t miss; uint8_t width_deg; };
Track trk[MAX_TRACKS] = {};

// chain測定専用のロック状態（followerと同一設計・同一理由）
struct ZoneLock { int trk_idx = -1; float last_dist = -1.0f; };
ZoneLock chainFwdLock, chainBackLock;

uint16_t bin_dist[360];
uint8_t  bin_conf[360];
uint32_t bin_tick[360];

float    lidar_hz_target = 10.0f;
float    lidar_duty      = 0.50f;
uint16_t lidar_speed_dps = 0;

// ================================================================
//  WS2812B (FastLED)
// ================================================================
CRGB leds[NUM_LEDS];
static const CRGB COLOR_OFF   = CRGB::Black;
static const CRGB COLOR_OTA   = CRGB(0, 80, 255);
static const CRGB COLOR_CAL_A = CRGB(255, 0, 0);   // ビーコンON: 赤（followerのPhase Aと同色）
static const CRGB COLOR_CAL_B = CRGB(0, 0, 255);   // ビーコンOFF後の待機: 青（followerのPhase Bと同色）

// 位相虹色マッピング用アンカー色（followerと同一：赤→青→緑→赤、120°ずつ）。
static const CRGB RAINBOW_ANCHOR[3] = { CRGB(255, 0, 0), CRGB(0, 0, 255), CRGB(0, 255, 0) };

// ================================================================
//  1kHz tick ISR
// ================================================================
void IRAM_ATTR onTick1kHz() {
    g_tick++;
}

// ================================================================
//  Motor low-level（IN/IN slow-decay方式。followerと同一実装）
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
// followerと同一のスティクション補償（スナップ方式）に統一。
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

void stopAllMotors() {
    for (int i = 0; i < 3; i++) {
        pid_target[i]      = 0.0f;
        pid_target_filt[i] = 0.0f;   // 明示的な停止は即座に反映（フィルタの残留を持ち越さない）
        pid_prev_err[i]    = 0.0f;
        // pid_integral[i] はリセットしない（followerと同一）
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
    c.channel        = PCNT_CHANNEL_0;
    c.pulse_gpio_num = gpioA;
    c.ctrl_gpio_num  = gpioB;
    c.pos_mode       = PCNT_COUNT_DEC;
    c.neg_mode       = PCNT_COUNT_INC;
    c.lctrl_mode     = PCNT_MODE_REVERSE;
    c.hctrl_mode     = PCNT_MODE_KEEP;
    pcnt_unit_config(&c);
    c.channel        = PCNT_CHANNEL_1;
    c.pulse_gpio_num = gpioB;
    c.ctrl_gpio_num  = gpioA;
    c.pos_mode       = PCNT_COUNT_INC;
    c.neg_mode       = PCNT_COUNT_DEC;
    pcnt_unit_config(&c);

    pcnt_set_filter_value(unit, 1023);
    pcnt_filter_enable(unit);
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

void setupEncoders() {
    pinMode(ENC1_A, INPUT); pinMode(ENC1_B, INPUT);
    pinMode(ENC2_A, INPUT); pinMode(ENC2_B, INPUT);
    pinMode(ENC3_A, INPUT); pinMode(ENC3_B, INPUT);
    setupEncoderPCNT(ENC_UNIT[0], ENC1_A, ENC1_B);
    setupEncoderPCNT(ENC_UNIT[1], ENC2_A, ENC2_B);
    setupEncoderPCNT(ENC_UNIT[2], ENC3_A, ENC3_B);
    Serial.printf("[ENC] PCNT x4 decode  CPR(out)=%.0f  filter=12.8us  PID_dt=%dms\n",
                  COUNTS_PER_OUT_REV, PID_TICKS);
}

// ================================================================
//  PID update（位相とは無関係の並進運動制御）
// ================================================================
void updatePID() {
    // pid_target(生の指令)→pid_target_filt(一次遅れ後)の離散化係数（followerと同一）。
    // τ·du/dt+u=pid_target の厳密離散化: alpha = 1 - exp(-PID_DT/τ_filt)。
    float alpha_filt = (tau_filt > 1e-4f) ? (1.0f - expf(-PID_DT / tau_filt)) : 1.0f;

    for (int i = 0; i < 3; i++) {
        int16_t cnt;
        pcnt_get_counter_value(ENC_UNIT[i], &cnt);
        int16_t d16 = (int16_t)(cnt - enc_prev[i]);
        enc_prev[i] = cnt;

        // スプリアスなグリッチはこのtickの分だけ0として捨てる（enc_prevは
        // 正しいcntに更新済みなので次tick以降の計測には影響しない）
        if (abs((int)d16) > ENC_GLITCH_MAX_COUNT) d16 = 0;
        // followerと完全一致: 単発計測+EMAのみ（移動窓は使わない）
        float rpm_raw = (float)(d16 * ENC_POL[i]) / COUNTS_PER_OUT_REV / PID_DT * 60.0f;
        actual_rpm[i] += RPM_EMA_A * (rpm_raw - actual_rpm[i]);

        // 速度指令に一次遅れフィルタを適用（followerと同一。M ẍ+D ẋ=u相当）。
        // 誤差・フィードフォワードは以降すべてフィルタ後の値pid_target_filt[i]を参照する。
        pid_target_filt[i] += alpha_filt * (pid_target[i] - pid_target_filt[i]);

        // followerと完全一致: フィルタ後の値が微小しきい値未満なら停止（積分は凍結保持）。
        // [注意] leaderはyaw補正が常時pid_targetに乗るため、M2(yaw補正のみの輪)が
        // 0.5rpm近傍を跨いで停止⇄駆動を繰り返す可能性がある（過去に問題化した動作）。
        // followerと完全一致させる方針のため、あえてrunningフラグ直接ゲートから変更する。
        // 手動操縦中(manual_mode)はrunning=falseでも駆動を許可する（位置決め用途のため）。
        if (!running && !manual_mode) {
            setMotorPWM(i, 0);
            pid_prev_err[i]    = 0.0f;
            pid_target_filt[i] = 0.0f;
            continue;
        }
        if (fabsf(pid_target_filt[i]) < 0.5f) {
            setMotorPWM(i, 0);
            pid_prev_err[i] = 0.0f;
            continue;
        }
        float err = pid_target_filt[i] - actual_rpm[i];
        float ilim = (Ki > 1e-6f) ? (200.0f / Ki) : 0.0f;   // 積分寄与を±200PWMに制限
        pid_integral[i] = constrain(pid_integral[i] + err * PID_DT, -ilim, ilim);
        float derr = (err - pid_prev_err[i]) / PID_DT;
        pid_prev_err[i] = err;

        float ff = pid_target_filt[i] / FF_MAX_RPM * 255.0f;
        float out = ff + Kp * err + Ki * pid_integral[i] + Kd * derr;
        setMotorPWM(i, constrain((int)out, -255, 255));
    }

    // ヨー角推定（デッドレコニング。followerと同一の導出）:
    // 120°等間隔配置では3輪の速度の単純合算で並進成分が打ち消し合い、回転成分だけが残る。
    float wheel_sum_mps = (actual_rpm[0] + actual_rpm[1] + actual_rpm[2]) * RPM2MPS;
    float yaw_rate = wheel_sum_mps / (3.0f * CHASSIS_R_M);
    yaw_est += yaw_rate * PID_DT;
    yaw_est = fmodf(yaw_est + PI, TWO_PI);
    if (yaw_est < 0.0f) yaw_est += TWO_PI;
    yaw_est -= PI;   // [-π, π]に正規化
}

// ================================================================
//  等速直線運動の目標RPM設定 + 直進性補正（問題②対策）
//    基本パターン [+rpm, 0, -rpm] に対し、
//    (1) ヨー保持: 目標ヨー角0との誤差をP制御し、3輪に同じ量を上乗せ（=純回転補正）
//    (2) ホイールトリム: M3のみSTRAIGHT_TRIMを乗算（タイヤ半径差など、エンコーダに
//        映らない系統誤差の較正用。詳細はSTRAIGHT_TRIM定義部コメント参照）
// ================================================================
void computeDriveTargets() {
    // 手動操縦（PC矢印キー）: Winfreeのrunning/ir_force_on状態より優先する。
    // タイムアウトで自動解除し、以降は通常の判定（running/ir_force_on）にフォールバックする。
    if (manual_mode) {
        if (millis() - manual_last_cmd_ms > MANUAL_TIMEOUT_MS) {
            manual_mode      = false;
            manual_fwd_rpm   = 0.0f;
            manual_side_rpm  = 0.0f;
            manual_rot_rpm   = 0.0f;
            Serial.println("[MANUAL] timeout -> released");
            // フォールスルーせず、この周期は安全側に即停止しておく
            pid_target[0] = 0.0f;
            pid_target[1] = 0.0f;
            pid_target[2] = 0.0f;
            return;
        }
        float rpm  = constrain(manual_fwd_rpm,  -CMD_RPM_LIMIT, CMD_RPM_LIMIT);
        // 横移動の指令値は「前後と同じ機体速度になるrpm」として扱う。
        // 前後パターン[+1,0,-1]は機体速度 v = rpm/cos30° = 1.155·rpm、
        // 横パターン[0.5,-1,0.5]は機体速度 v = side であり、同じ入力値では前後の方が約15%速い。
        // 横にだけ 2/√3 を掛けて両者の機体速度を揃える（M2は 1.155·side になる）。
        const float SIDE_BODY_SCALE = 1.1547005f;
        float side = constrain(manual_side_rpm * SIDE_BODY_SCALE, -CMD_RPM_LIMIT, CMD_RPM_LIMIT);
        float rot  = constrain(manual_rot_rpm,  -CMD_RPM_LIMIT, CMD_RPM_LIMIT);
        // 3輪オムニホイールの合成（並進＋その場回転、rot=0なら従来通り並進のみ）。
        // 前後(rpm)は既存のWinfree並進と同じ基本パターン[+1,0,-1]（M2は前後移動に寄与しない
        // ＝M2の転がり軸が前後軸と一致する配置）。この配置から幾何学的に、M2に直交する
        // 横方向の並進はM2が主動輪(±1倍)、M1/M3がその半分(±0.5倍・M2と逆符号)を
        // 分担する組み合わせになる（3輪120°等配置・M2=前後軸上のオムニ駆動の標準運動学）。
        // 回転(rot)は3輪すべてに同じ値を加算する（120°等配置の対称性により並進成分を
        // 持たない純回転になる。以前の実装が「旋回」として使っていたのと同じ項だが、
        // 今回は独立したQ/Eコマンドとしてfwd/sideと合成できるようにした）。
        // 各項は個別にCMD_RPM_LIMIT以内に収まるが、fwd+side+rotを同時に大きく入力すると
        // 合成値が単軸最大値を超える場合がある点は既存のfwd+side合成と同様（PWM側で±255に
        // クランプされるため暴走はしないが、極端な同時入力では狙った速度比にならないことがある）。
        // ヨー保持。回転入力が無く、並進しているときだけ基準角へ戻す補正を3輪に共通加算する。
        // それ以外では基準角を現在角に追従させ、旋回後の向きや停止中の向きを新しい基準にする。
        float yaw_corr_rpm = 0.0f;
        if (fabsf(rot) < 0.5f && (fabsf(rpm) >= 0.5f || fabsf(side) >= 0.5f)) {
            float yaw_err   = atan2f(sinf(yaw_ref - yaw_est), cosf(yaw_ref - yaw_est));   // [-π, π]
            float omega_cmd = YAW_HOLD_KP * yaw_err;                                      // [rad/s]
            yaw_corr_rpm    = (CHASSIS_R_M * omega_cmd) / RPM2MPS;
        } else {
            yaw_ref = yaw_est;
        }
        pid_target[0] =  rpm                 + 0.5f * side + rot + yaw_corr_rpm;
        pid_target[1] =  0.0f                -        side + rot + yaw_corr_rpm;
        pid_target[2] = -rpm * STRAIGHT_TRIM + 0.5f * side + rot + yaw_corr_rpm;
        return;
    }

    if (!running || ir_force_on) {
        pid_target[0] = 0.0f;
        pid_target[1] = 0.0f;
        pid_target[2] = 0.0f;
        return;
    }
    float rpm = constrain(CONST_DRIVE_RPM, -CMD_RPM_LIMIT, CMD_RPM_LIMIT);

    // ヨー保持補正: 推定も補正も同じ「3輪合算」座標で閉じた負帰還（符号は構成上正しい）
    float yaw_err      = 0.0f - yaw_est;                         // yaw_estは[-π,π]
    float omega_cmd    = YAW_HOLD_KP * yaw_err;                  // [rad/s]
    float yaw_corr_rpm = (CHASSIS_R_M * omega_cmd) / RPM2MPS;    // 3輪共通の上乗せ分[RPM]

    pid_target[0] =  rpm                 + yaw_corr_rpm;
    pid_target[1] =  0.0f                + yaw_corr_rpm;
    pid_target[2] = -rpm * STRAIGHT_TRIM + yaw_corr_rpm;
}

// ================================================================
//  Sensor（1ch/1ms パイプライン）
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

void processFrame();

void sensorTick() {
    static uint8_t cur_ch = 0;
    raw_frame[cur_ch] = readAdcAveraged();
    uint8_t done = cur_ch;
    cur_ch = (cur_ch + 1) % N_SENSORS;
    setMuxChannel(cur_ch);
    if (done == N_SENSORS - 1) processFrame();
}

void updateLED();

// フレーム完了（12msごと）: V_j算出 → 位相Euler積分 → IR発光更新
void processFrame() {
    static uint32_t last_frame_us = 0;
    uint32_t now_frame_us = micros();
    if (last_frame_us != 0) {
        actual_frame_dt_ms = (float)(now_frame_us - last_frame_us) / 1000.0f;
    }
    last_frame_us = now_frame_us;

    bool blanked = ir_on || ((int32_t)(g_tick - ir_off_guard_end) < 0);

    if (!blanked) {
        float rs = 0.0f;
        for (int j = 0; j < N_SENSORS; j++) {
            float x = (float)raw_frame[j] / 4095.0f;
            V_base[j] = (1.0f - BASELINE_ALPHA) * V_base[j] + BASELINE_ALPHA * x;
            float v = x - V_base[j];
            if (v < 0.0f) v = 0.0f;
            V[j] = v;
            rs += v;
        }
        raw_sumV = rs;
        sumV = rs;
    }

    // ビーコンモード: Winfree位相・RUN/STOP状態を無視してIRを常時点灯し続ける
    // （follower側「2体キャリブレーション」の基準発光源として使うためのモード）。
    // 並進運動もさせない。running状態やphiは変化させず、モード解除後にそのまま復帰できる。
    if (ir_force_on) {
        if (!ir_on) {
            ir_on = true;
            digitalWrite(IR_CTRL, HIGH);
        }
        led_override = LED_OV_RED;
        updateLED();
        stopAllMotors();
        return;
    }

    if (!running) {
        if (ir_on) {
            ir_on = false;
            digitalWrite(IR_CTRL, LOW);
        }
        // 虹色モードはir_onに依存せず点灯し続けるため、STOP直後にir_onがすでに
        // falseだった場合でも消灯を反映できるよう毎フレーム呼ぶ（内部で変化検出するので軽量）。
        updateLED();
        return;
    }

    phi += (omega - kappa * sumV) * FRAME_DT;
    phi = fmodf(phi, TWO_PI);
    if (phi < 0.0f) phi += TWO_PI;

    bool na = (cosf(phi) > tau);
    if (na != ir_on) {
        ir_on = na;
        digitalWrite(IR_CTRL, ir_on ? HIGH : LOW);
        if (!ir_on) ir_off_guard_end = g_tick + IR_OFF_GUARD_TICKS;
        updateLED();
    }
}

// ================================================================
//  LiDAR (LD19) — UART1受信 + 360ビン + クラスタ/トラック。followerと同一実装
// ================================================================
static uint8_t crc_tbl[256];
void initLidarCrc() {
    for (int i = 0; i < 256; i++) {
        uint8_t c = i;
        for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x4D) : (uint8_t)(c << 1);
        crc_tbl[i] = c;
    }
}

static inline int lidarDegToBodyBin(float lidar_deg) {
    float b = LIDAR_MOUNT_OFFSET_DEG - lidar_deg;
    b = fmodf(b, 360.0f); if (b < 0) b += 360.0f;
    return ((int)(b + 0.5f)) % 360;
}

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
        da /= 11.0f;
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
    return half * 2.0f + 3.0f;   // +3°はLiDAR角度分解能・ブレのマージン
}

void lidarUpdateTracks() {
    uint32_t now = g_tick;
    float   c_bear[MAX_TRACKS * 2]; float c_dist[MAX_TRACKS * 2]; uint8_t c_width[MAX_TRACKS * 2];
    int     nc = 0;
    int     run_start = -1, gap = 0;
    float   dmin = 1e9f; double sx = 0, sy = 0; int npts = 0;
    float   dmin0 = 1e9f; double sx0 = 0, sy0 = 0;
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
    for (int c = 0; c < nc; c++) {
        if (used[c]) continue;
        for (int k = 0; k < MAX_TRACKS; k++) {
            if (trk[k].alive) continue;
            trk[k] = { true, c_bear[c], c_dist[c], 1, 0, c_width[c] };
            break;
        }
    }
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

// 生bin_confの平均値を返す（followerと同一実装。詳細はfollower側コメント参照）
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

void lidarSpeedLoop() {
    float target_dps = lidar_hz_target * 360.0f;
    if (lidar_speed_dps > 0) {
        lidar_duty += 0.00002f * (target_dps - (float)lidar_speed_dps);
        lidar_duty = constrain(lidar_duty, 0.35f, 0.60f);
    }
    ledcWrite(LIDAR_PWM_CH, (uint32_t)(lidar_duty * 255.0f));
}

// ================================================================
//  起動時ベースラインキャリブレーション（IR消灯状態での環境光レベル）
// ================================================================
#define CAL_ITERATIONS 50

void calibrateBaseline() {
    stopAllMotors();
    digitalWrite(IR_CTRL, LOW);
    long sum[N_SENSORS] = {0};
    for (int i = 0; i < CAL_ITERATIONS; i++) {
        for (int ch = 0; ch < N_SENSORS; ch++) {
            setMuxChannel(ch);
            delayMicroseconds(1000);
            sum[ch] += readAdcAveraged();
        }
        if (wifi_ok) ArduinoOTA.handle();
    }
    for (int ch = 0; ch < N_SENSORS; ch++)
        V_base[ch] = (float)(sum[ch] / CAL_ITERATIONS) / 4095.0f;
    Serial.println("[CAL] baseline done.");
}

// ================================================================
//  Serial/UDP共通コマンド処理
// ================================================================
void printStatus() {
    Serial.printf("phi=%.4f  omega=%.4f(%.2fHz)  kappa=%.4f  tau=%.3f  running=%d  ir_force_on=%d\n",
                  phi, omega, omega / TWO_PI, kappa, tau, (int)running, (int)ir_force_on);
    Serial.printf("CONST_DRIVE_RPM=%.1f  Kp=%.3f Ki=%.3f Kd=%.3f  tau_filt=%.4f\n",
                  CONST_DRIVE_RPM, Kp, Ki, Kd, tau_filt);
    Serial.printf("YAW_HOLD_KP=%.3f  yaw_est=%.2fdeg  STRAIGHT_TRIM=%.4f\n",
                  YAW_HOLD_KP, yaw_est * RAD_TO_DEG, STRAIGHT_TRIM);
    Serial.printf("manual_mode=%d  manual_fwd_rpm=%.1f  manual_side_rpm=%.1f  manual_rot_rpm=%.1f\n",
                  (int)manual_mode, manual_fwd_rpm, manual_side_rpm, manual_rot_rpm);
    Serial.printf("rpm[%.1f %.1f %.1f]  tgt[%.1f %.1f %.1f]  pwm[%d %d %d]\n",
                  actual_rpm[0], actual_rpm[1], actual_rpm[2],
                  pid_target[0], pid_target[1], pid_target[2],
                  mot_pwm[0], mot_pwm[1], mot_pwm[2]);
    Serial.printf("OTA: host=%s  IP=%s\n", ota_hostname, WiFi.localIP().toString().c_str());
}

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
        case 'g': CONST_DRIVE_RPM = val;
                  Serial.printf("CONST_DRIVE_RPM = %.1f\n", CONST_DRIVE_RPM); break;
        case 'f': CONF_MIN = val;
                  Serial.printf("CONF_MIN = %.1f\n", CONF_MIN); break;
        case 'q': lidar_hz_override = val;
                  lidar_hz_target   = val;
                  Serial.printf("lidar_hz_target = %.1f Hz (override)\n", lidar_hz_target); break;
        case 'P': Kp = val;
                  Serial.printf("Kp = %.4f\n", Kp); break;
        case 'I': Ki = val;
                  for (int i = 0; i < 3; i++) pid_integral[i] = 0;
                  Serial.printf("Ki = %.4f (integral reset)\n", Ki); break;
        case 'D': Kd = val;
                  Serial.printf("Kd = %.4f\n", Kd); break;
        case 'L': tau_filt = fmaxf(val, 0.0f);
                  Serial.printf("tau_filt = %.4f s (%s)\n", tau_filt,
                                (tau_filt > 1e-4f) ? "有効" : "無効・素通し"); break;
        case 'y': YAW_HOLD_KP = val;   // ヨー保持Pゲイン（0=無効。符号は構成上正、要調整は大きさのみ）
                  Serial.printf("YAW_HOLD_KP = %.3f\n", YAW_HOLD_KP); break;
        case 'w': STRAIGHT_TRIM = constrain(val, 0.9f, 1.1f);   // M3トリム（タイヤ半径差の較正。±10%まで）
                  Serial.printf("STRAIGHT_TRIM = %.4f\n", STRAIGHT_TRIM); break;
        case 'r': led_rainbow_mode = (val != 0.0f);   // WS2812B: 位相虹色表示(既定1) / 0でIR同期青パルスに戻す
                  updateLED();
                  Serial.printf("led_rainbow_mode = %d\n", (int)led_rainbow_mode); break;
        case 'c': calibrateBaseline(); break;
        case 'e': ir_force_on = (val != 0.0f);   // ビーコンモード: followerの2体キャリブレーション基準用にIR常時点灯
                  if (!ir_force_on) {
                      ir_on = false;
                      digitalWrite(IR_CTRL, LOW);
                      led_override = LED_OV_BLUE;
                  } else {
                      led_override = LED_OV_RED;
                  }
                  updateLED();
                  Serial.printf("ir_force_on = %d (beacon mode)\n", (int)ir_force_on); break;
        case '0': running = false;
                  manual_mode = false;   // 矢印キー手動操縦も併せて解除する
                  stopAllMotors();
                  Serial.println("[MOT] all stop (manual)"); break;
        default:  printStatus(); break;
    }
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

    // 手動操縦（PC矢印キー+QE回転）: "M <fwd_rpm> <side_rpm> <rot_rpm>" を毎回上書き受信する
    // （side_rpmは横方向＝平行移動、rot_rpmはその場回転の指令。rot_rpmは省略可＝省略時0で
    // 従来の2引数コマンドとの後方互換を保つ）。
    // ダッシュボード側はキー押下中150ms間隔で再送し続ける（keepalive兼用）。
    // RUN/STOP/一斉停止では下のRUN/STOP分岐およびprocessCommand内'0'で必ず解除する。
    if (line.startsWith("M ")) {
        int   sp1 = line.indexOf(' ', 2);
        int   sp2 = (sp1 > 0) ? line.indexOf(' ', sp1 + 1) : -1;
        float f1 = (sp1 > 0) ? line.substring(2, sp1).toFloat() : line.substring(2).toFloat();
        float f2 = (sp1 > 0) ? ((sp2 > 0) ? line.substring(sp1 + 1, sp2).toFloat() : line.substring(sp1 + 1).toFloat()) : 0.0f;
        float f3 = (sp2 > 0) ? line.substring(sp2 + 1).toFloat() : 0.0f;
        manual_fwd_rpm      = constrain(f1, -CMD_RPM_LIMIT, CMD_RPM_LIMIT);
        manual_side_rpm     = constrain(f2, -CMD_RPM_LIMIT, CMD_RPM_LIMIT);
        manual_rot_rpm      = constrain(f3, -CMD_RPM_LIMIT, CMD_RPM_LIMIT);
        if (!manual_mode) yaw_ref = yaw_est;   // 手動操縦に入った時点の向きを基準にする
        manual_mode          = true;
        manual_last_cmd_ms   = millis();
        return;
    }

    if (line == "RUN" || line == "START") {
        running = true;
        manual_mode = false;   // 実験開始時は手動操縦を必ず解除する
        // ヨー角推定はデッドレコニング（積分）なので長時間で誤差が蓄積する。
        // RUN=走行開始時点の向きを「まっすぐ」の基準(0)とみなしてリセットする。
        yaw_est = 0.0f;
        // [バグ修正] ビーコンモード('e 1')がONのまま残っていると、processFrame()内で
        // ir_force_onのチェックがrunningの判定より先に来るため、running=trueにしても
        // 動かない事故が起きる。RUN受信時は必ず解除する。
        if (ir_force_on) {
            ir_force_on = false;
            ir_on = false;
            digitalWrite(IR_CTRL, LOW);
        }
        led_override = LED_OV_NONE;
        Serial.println("[RUN] started");
        return;
    }
    if (line == "STOP") {
        running = false;
        manual_mode = false;   // 手動操縦も併せて解除する
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
//  WS2812B
// ================================================================
// 位相φ(rad)を虹色にマッピングする（followerと同一実装）。
// φ=0で赤、2π/3で青、4π/3で緑、2πで赤に戻る3色の直線RGB補間。
CRGB phaseToRainbow(float p) {
    p = fmodf(p, TWO_PI);
    if (p < 0.0f) p += TWO_PI;
    const float SEG = TWO_PI / 3.0f;
    int   idx  = (int)(p / SEG);
    if (idx > 2) idx = 2;
    float frac = (p - idx * SEG) / SEG;
    const CRGB& c0 = RAINBOW_ANCHOR[idx];
    const CRGB& c1 = RAINBOW_ANCHOR[(idx + 1) % 3];
    CRGB out;
    out.r = (uint8_t)(c0.r + (c1.r - c0.r) * frac);
    out.g = (uint8_t)(c0.g + (c1.g - c0.g) * frac);
    out.b = (uint8_t)(c0.b + (c1.b - c0.b) * frac);
    return out;
}

void updateLED() {
    static bool            last_ir_on     = false;
    static LedOverrideMode last_override  = LED_OV_NONE;
    static float           last_phi_shown = -999.0f;
    static bool            last_running   = false;

    bool rainbow_now = led_rainbow_mode && (led_override == LED_OV_NONE);
    bool phi_moved   = rainbow_now && (fabsf(phi - last_phi_shown) > 0.01f);

    if (!phi_moved && ir_on == last_ir_on && led_override == last_override && running == last_running) return;
    last_ir_on    = ir_on;
    last_override = led_override;
    last_running  = running;
    if (rainbow_now) last_phi_shown = phi;

    CRGB c;
    if (led_override == LED_OV_RED)       c = COLOR_CAL_A;
    else if (led_override == LED_OV_BLUE) c = COLOR_CAL_B;
    else if (led_rainbow_mode)            c = (!running) ? COLOR_OFF : phaseToRainbow(phi);   // 停止中(STOP)は虹色モードでも消灯
    else                                   c = ir_on ? COLOR_OTA : COLOR_OFF;   // 従来: IR同期青パルス
    fill_solid(leds, NUM_LEDS, c);
    FastLED.show();
}

// ================================================================
//  テレメトリ（UDP 41フィールド、bridge_server.py/dashboard.htmlのFIELDS定義と一致）
//    n_tracks_alive / nearest_* / chain_fwd / chain_back / yaw_est_degは実測値。
//    peer_bearing_deg / peer_dist_mはIRゲート未実装のためプレースホルダのまま。
// ================================================================
void sendTelemetry() {
    int   nearest = -1; float nearest_d = 1e9f; int n_alive = 0;
    for (int k = 0; k < MAX_TRACKS; k++) {
        if (!trk[k].alive || trk[k].hit < 2) continue;
        n_alive++;
        if (trk[k].dist_m < nearest_d) { nearest_d = trk[k].dist_m; nearest = k; }
    }
    float near_bear_deg = (nearest >= 0) ? trk[nearest].bearing * RAD_TO_DEG : 0.0f;
    float near_dist_m   = (nearest >= 0) ? nearest_d : 99.9f;

    float chain_fwd_dist_m  = findNearestTrackInZone(CHAIN_FRONT_DEG, CHAIN_ZONE_HALF_DEG, chainFwdLock);
    float chain_back_dist_m = findNearestTrackInZone(CHAIN_BACK_DEG,  CHAIN_ZONE_HALF_DEG, chainBackLock);
    float chain_fwd_conf_avg  = zoneConfAvg(CHAIN_FRONT_DEG, CHAIN_ZONE_HALF_DEG);
    float chain_back_conf_avg = zoneConfAvg(CHAIN_BACK_DEG,  CHAIN_ZONE_HALF_DEG);

    Serial.printf("phi=%5.3f ir=%d run=%d sumV=%5.3f rpm[%5.1f %5.1f %5.1f] "
                  "tgt[%5.1f %5.1f %5.1f] pwm[%4d %4d %4d] "
                  "lidar[trk=%d near=%.2fm] chain[fwd=%.3fm back=%.3fm] frame_dt=%.2fms\n",
                  phi, (int)ir_on, (int)running, sumV,
                  actual_rpm[0], actual_rpm[1], actual_rpm[2],
                  pid_target[0], pid_target[1], pid_target[2],
                  mot_pwm[0], mot_pwm[1], mot_pwm[2],
                  n_alive, near_dist_m, chain_fwd_dist_m, chain_back_dist_m, actual_frame_dt_ms);
    if (!wifi_ok) return;

    char pkt[460];
    int len = snprintf(pkt, sizeof(pkt),
        "%s,%.3f,%.4f,%d,%.4f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.4f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,"
        "%d,%.1f,%.3f,%d,%d,%.1f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.2f\n",
        mac_id, (float)millis() / 1000.0f, phi, (int)ir_on, sumV,
        V[0], V[1], V[2], V[3], V[4], V[5],
        V[6], V[7], V[8], V[9], V[10], V[11],
        raw_sumV,
        actual_rpm[0], actual_rpm[1], actual_rpm[2],
        pid_target[0], pid_target[1], pid_target[2],
        mot_pwm[0], mot_pwm[1], mot_pwm[2],
        n_alive, near_bear_deg, near_dist_m, 0, 0,   // cbf_active/gate_reject_cntは常に0（リーダーは未実装）
        0.0f, 99.9f,            // peer_bearing_deg/peer_dist_m: IRゲート未実装のためプレースホルダ
        actual_frame_dt_ms,
        chain_fwd_dist_m, chain_back_dist_m,
        chain_fwd_conf_avg, chain_back_conf_avg, CONF_MIN,
        yaw_est * RAD_TO_DEG);   // yaw_est_deg（41番目）。[!] 従来のこのファイルは40フィールドしか
                                 //   送っておらず、bridge_server.py(len==41要求)にサイレントdropされる
                                 //   状態だった。yaw追加でFIELDS定義と一致する。
    udp.beginPacket(broadcastIP, UDP_PORT);
    udp.write((uint8_t*)pkt, len);
    udp.endPacket();
}

// ================================================================
//  OTA
// ================================================================
void setupOTA() {
    ArduinoOTA.setHostname(ota_hostname);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() {
        ota_active = true;
        digitalWrite(IR_CTRL, LOW);
        stopAllMotors();
        fill_solid(leds, NUM_LEDS, COLOR_OTA); FastLED.show();
        Serial.println("\n[OTA] Start");
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
        delay(2000);
        fill_solid(leds, NUM_LEDS, COLOR_OFF); FastLED.show();
    });
    ArduinoOTA.begin();
    Serial.printf("[OTA] Ready  host=%s  IP=%s\n", ota_hostname, WiFi.localIP().toString().c_str());
}

// ================================================================
//  setup
// ================================================================
void setup() {
    Serial.begin(115200);

    pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT); pinMode(MUX_S3, OUTPUT);
    setMuxChannel(0);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
    pinMode(IR_CTRL, OUTPUT); digitalWrite(IR_CTRL, LOW);

    FastLED.addLeds<WS2812B, LED_DIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(80);
    fill_solid(leds, NUM_LEDS, COLOR_OFF); FastLED.show();

    // --- LiDAR (LD19) ---
    initLidarCrc();
    Serial1.setRxBufferSize(16384);
    Serial1.begin(230400, SERIAL_8N1, LIDAR_RX_PIN, -1);
    ledcSetup(LIDAR_PWM_CH, LIDAR_PWM_FREQ, MOT_BITS);
    ledcAttachPin(LIDAR_PWM_PIN, LIDAR_PWM_CH);
    ledcWrite(LIDAR_PWM_CH, 128);

    setupMotors();
    setupEncoders();
    calibrateBaseline();

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
    if (lidar_hz_override >= 0.0f) lidar_hz_target = lidar_hz_override;      // 'q'コマンドで上書き済みなら優先
    Serial.printf("[LIDAR] UART1 RX=%d  target=%.0fHz (desync)\n", LIDAR_RX_PIN, lidar_hz_target);

    if (WiFi.status() == WL_CONNECTED) {
        wifi_ok = true;
        WiFi.setSleep(false);
        udp.begin(UDP_PORT);
        cmdUdp.begin(CMD_PORT);
        uint32_t ip = (uint32_t)WiFi.localIP(), mask = (uint32_t)WiFi.subnetMask();
        broadcastIP = IPAddress(ip | ~mask);
        Serial.printf("\n[WiFi] %s  ID:%s  bcast:%s\n",
                      WiFi.localIP().toString().c_str(), mac_id, broadcastIP.toString().c_str());
        setupOTA();
    } else {
        Serial.println("\n[WiFi] timeout — UDP/OTA disabled");
    }

    phi = 0.0f;

    tick_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(tick_timer, &onTick1kHz, true);
    timerAlarmWrite(tick_timer, 1000, true);
    timerAlarmEnable(tick_timer);

    Serial.println("\n=== Winfree LEADER（直列実験用, HW-timer scheduled, LiDAR搭載） ===");
    Serial.printf("phi0=%.4f  omega=%.4f  kappa=%.4f  tau=%.3f  CONST_DRIVE_RPM=%.1f\n",
                  phi, omega, kappa, tau, CONST_DRIVE_RPM);
    Serial.println("[!] 起動直後はrunning=false（スタンバイ）。ダッシュボードの「一斉起動」で開始すること。");
}

// ================================================================
//  loop（周期処理はすべて g_tick 起点でディスパッチ。フォロワーと同じ設計）
// ================================================================
void loop() {
    if (wifi_ok) ArduinoOTA.handle();
    if (ota_active) return;

    handleCmdPort();              // ダッシュボードからのパラメータ変更・一斉RUN/STOPを一括処理（非ブロッキング）
    lidarPoll();                 // 非ブロッキング・バイト単位パース（毎ループ）

    static uint32_t sens_tick = 0, pid_tick = 0, led_tick = 0, tel_tick = 0, lidar_tick = 0;
    static bool first = true;
    uint32_t tick = g_tick;
    if (first) { sens_tick = pid_tick = led_tick = tel_tick = lidar_tick = tick; first = false; }

    // --- センサ 1ch/1ms ---
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

    // --- PID 5ms ---
    tick = g_tick;
    if ((int32_t)(tick - pid_tick) >= PID_TICKS) {
        pid_tick += PID_TICKS;
        computeDriveTargets();
        updatePID();
        if ((int32_t)(tick - pid_tick) >= 4 * PID_TICKS) pid_tick = tick;
    }

    // --- LiDAR 10Hz: クラスタ+トラック更新 + 速度デシンクループ ---
    if ((int32_t)(tick - lidar_tick) >= LIDAR_TICKS) {
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