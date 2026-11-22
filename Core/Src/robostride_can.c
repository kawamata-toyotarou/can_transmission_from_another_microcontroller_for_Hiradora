#include "robostride_can.h"
#include "swerve_config.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan3;

#define ROBOSTRIDE_HOST_ID 0xFDu
#define RS_TYPE_ENABLE 0x03u
#define RS_TYPE_STOP 0x04u
#define RS_TYPE_READ_PARAM 0x11u
#define RS_TYPE_WRITE_PARAM 0x12u
#define CAN3_DIAG_CAPACITY 16U

typedef struct {
    SteeringFeedback feedback;
    uint32_t request_ms;
    bool pending;
    bool homed;
} SteeringAxis;

static SteeringAxis steering[SWERVE_WHEEL_COUNT];

static struct {
    uint16_t index;
    uint8_t id, waiting, received, ok;
    uint8_t data[4];
} home_reply;

/* CAN送信キューへの投入結果。デバッガーで確認できる。 */
volatile uint32_t tx_success_count;
volatile uint32_t tx_fail_count;
volatile HAL_StatusTypeDef rs_last_tx_status;
volatile uint32_t rs_last_tx_id;
volatile uint16_t rs_last_param_index;

/* CAN3 startup error diagnostics: no CAN transmission or automatic recovery. */
typedef struct {
    uint32_t tick;                       /* エラーが記録された時刻 */
    uint32_t psr;                        /* CAN通信の状態を示すレジスタの値 */
    uint32_t ecr;                        /* CAN通信のエラーカウンタの値 */
    uint32_t hal_error;                  /* STM32のHALが管理するエラー情報 */
} CAN3_DiagEvent;

/* CAN3のエラー記録 */
static volatile CAN3_DiagEvent can3_diag_events[CAN3_DIAG_CAPACITY];  // 16件分のエラー情報を保存
static volatile uint32_t can3_diag_count;
static volatile uint32_t can3_diag_busoff_seen;
static volatile uint32_t can3_diag_busoff_psr;
static volatile uint32_t can3_diag_busoff_ecr;

static HAL_StatusTypeDef robostride_send(uint8_t type, uint16_t data_area,
                                        uint8_t motor_id, uint8_t data[8]);

bool robostride_tx_ready(void)
{
    return (hfdcan3.Instance->PSR & FDCAN_PSR_BO) == 0U &&
           HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) != 0U;
}

bool robostride_wait_idle(void)
{
    uint32_t started = HAL_GetTick();                    // 開始時刻を取得

    while (hfdcan3.Instance->TXBRP != 0U) {              // CANの送信要求が残っているか確認 TXBRPはFDCANの送信要求保留レジスタで送信要求が保留されていると、対応するビットが1
        if ((hfdcan3.Instance->PSR & FDCAN_PSR_BO) ||    // FDCAN_PSR_BO は、Bus-Off状態を表すビットのマスク
            HAL_GetTick() - started >= 10U) {            // 10ms以上経過したか
            return false;
        }

        HAL_Delay(1);
    }

    return (hfdcan3.Instance->PSR & FDCAN_PSR_BO) == 0U; // Bus-Offを確認して結果を返す
}

/*モーターを動かしてよい状態にする関数*/
HAL_StatusTypeDef robostride_enable(uint8_t motor_id)
{
    uint8_t data[8] = {0};
    return robostride_send(RS_TYPE_ENABLE, ROBOSTRIDE_HOST_ID, motor_id, data); // type = 0x03 は、RobStrideにモーターを有効にしてと送るEnable指令
}

/*モーターを停止させる関数 */
HAL_StatusTypeDef robostride_stop(uint8_t motor_id, bool clear_fault)
{
    uint8_t data[8] = {0};
    data[0] = clear_fault ? 1U : 0U;
    return robostride_send(RS_TYPE_STOP, ROBOSTRIDE_HOST_ID, motor_id, data);
}

/*送信関数 引数typeはRoboStrideの通信タイプ data_areaはCAN拡張IDの中央部分に入れる値*/
static HAL_StatusTypeDef robostride_send(uint8_t type, uint16_t data_area, uint8_t motor_id, uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef tx = {0};
    /* typeは通信の種類 data_areaは追加情報や送信元など motor_idは通信相手のモーターid */
    tx.Identifier = ((uint32_t)type << 24) | ((uint32_t)data_area << 8) | motor_id; // RoboStride用の29bit拡張CANIDを作成
    tx.IdType = FDCAN_EXTENDED_ID;                                                  // ロボマスではstandardidだったがロボストライドでは拡張id
    tx.TxFrameType = FDCAN_DATA_FRAME;                                              // 	データを持つフレームとして送る
    tx.DataLength = FDCAN_DLC_BYTES_8;                                              // RoboStrideの命令は基本8バイト
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;                                      // CAN FD用のエラー状態欄を設定
    tx.BitRateSwitch = FDCAN_BRS_OFF;                                               // データ部分で通信速度を切り替えない
    tx.FDFormat = FDCAN_CLASSIC_CAN;                                                // 	RobStrideとの通信にClassic CANを使う
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;                                     // 送信イベントを専用のキューに記録しない
    tx.MessageMarker = 0;                                                           // 送信イベントに付ける目印。今回はイベントを記録しないので 0

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx, data); // HAL_FDCAN_AddMessageToTxFifoQ() で送信を予約(送信)し、その予約の結果を保存
    rs_last_tx_status = st;
    rs_last_tx_id = tx.Identifier;
    if (st == HAL_OK) {
        tx_success_count++;
    } else {
        tx_fail_count++;
    }
    return st;
}

/*16bitのデータを2つに分解する関数*/
static void put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
}

/*floatの値をRoboStrideに送る形に詰めるための関数 pにvの4バイトがコピーされている*/
static void put_float_le(uint8_t *p, float v)
{
    memcpy(p, &v, 4);
}

/*floatデータから送信データを作成し、送信する関数*/
HAL_StatusTypeDef robostride_write_float(uint8_t motor_id, uint16_t index, float value)
{
    uint8_t data[8] = {0};

    put_u16_le(&data[0], index);
    put_float_le(&data[4], value);

    rs_last_param_index = index;

    return robostride_send(RS_TYPE_WRITE_PARAM, ROBOSTRIDE_HOST_ID, motor_id, data);
}

/*intデータから送信データを作成し、送信する関数*/
HAL_StatusTypeDef robostride_write_u8(uint8_t motor_id, uint16_t index, uint8_t value)
{
    uint8_t data[8] = {0};

    put_u16_le(&data[0], index);
    data[4] = value;
    rs_last_param_index = index;

    return robostride_send(RS_TYPE_WRITE_PARAM, ROBOSTRIDE_HOST_ID, motor_id, data);
}

static HAL_StatusTypeDef request_parameter(uint8_t id, uint16_t index)
{
    uint8_t request[8] = {0};
    put_u16_le(request, index);  // 16ビットの index を2バイトに分け、下位バイトから request[0] と request[1] に書き込み
    return robostride_send(RS_TYPE_READ_PARAM, ROBOSTRIDE_HOST_ID, id, request); // RS_TYPE_READ_PARAM はパラメータを読み出してという命令の種類
}

/* 原点復帰と通常走行で共用するFIFO0の受信処理。FDCAN3に届いたRobStrideの返信を取り出し、どの車輪の何の返信かを判定する */
void robostride_receive(void)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[64];
    /* RX FIFO0 にフレームがある間、`HAL_FDCAN_GetRxMessage()` で1件ずつ取り出す。n < 8U の8はフレーム数。1回の呼び出しで最大8件だけ処理し、残りは次回に回すことで、ほかの処理にも時間を使える */
    for (uint8_t n = 0; n < 8U && HAL_FDCAN_GetRxFifoFillLevel(&hfdcan3, FDCAN_RX_FIFO0); ++n) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) {
            return;
        }
        /* 拡張ID・Classic CAN・データフレーム・8バイト・宛先が 0xFD のすべてを確認 */
        if (rx.IdType != FDCAN_EXTENDED_ID || rx.FDFormat != FDCAN_CLASSIC_CAN || rx.RxFrameType != FDCAN_DATA_FRAME || rx.DataLength != FDCAN_DLC_BYTES_8 || (rx.Identifier & 0xFFU) != ROBOSTRIDE_HOST_ID) {
            continue;
        }

        uint8_t type = (rx.Identifier >> 24) & 0x1FU;
        uint8_t id = (rx.Identifier >> 8) & 0xFFU;
        for (uint8_t i = 0; i < SWERVE_WHEEL_COUNT; ++i) {
            /* 受信したidが正しくない時はスキップ*/
            if (id != swerve_modules[i].steer_id) {
                continue;
            }

            SteeringAxis *axis = &steering[i];              // axisはその車輪の管理情報
            SteeringFeedback *feedback = &axis->feedback;   // feedbackはその中の角度や故障の情報
            if (type == 2U || type == 24U || type == 21U) { // type が 2、24、21 のとき、このコードは故障情報を確認する
                uint32_t fault = type == 21U ? (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24) : (rx.Identifier >> 16) & 0x3FU;
                if (fault || (axis->homed && type != 21U && ((rx.Identifier >> 22) & 3U) != 2U)) { // fault が0以外なら、故障情報がある 原点復帰済みで、運転状態を持つタイプ2・24の返信を見たとき、CAN IDのbit 23〜22が 2（運転状態）以外なら、操舵に使えないと判断
                    feedback->fault = 1;
                    feedback->valid = 0;
                }
                break;
            }
            if (type != RS_TYPE_READ_PARAM) {  // RS_TYPE_READ_PARAM（0x11）なら、パラメータ読み出しの返信として処理
                break;
            }

            uint16_t index = (uint16_t)data[0] | ((uint16_t)data[1] << 8); // index はどの項目の返信か 例えば data[0] = 0x19、`data[1] = 0x70` なら、`0x19 | (0x70 << 8)` で 0x7019（現在角度）になる
            uint8_t ok = ((rx.Identifier >> 16) & 0xFFU) == 0U;            // ok は読み出しの結果 CAN IDのbit 23〜16を取り出し、この値が 0 なら ok = 1
            /* 待機中で、まだ返信を保存しておらず、モーターIDと項目番号が両方一致する場合だけ、data[4]〜data[7]の4バイトを home_reply.data にコピー */
            if (home_reply.waiting && !home_reply.received && home_reply.id == id && home_reply.index == index) {
                memcpy(home_reply.data, &data[4], sizeof(home_reply.data));
                home_reply.ok = ok;
                home_reply.received = 1;
            }
            /* 通常走行中なら、最新の操舵角を更新する */
            if (index == RS_PARAM_MECH_POS && axis->pending) {
                float angle;
                uint32_t now = HAL_GetTick();
                memcpy(&angle, &data[4], sizeof(angle));
                feedback->valid = ok && !feedback->fault && isfinite(angle) &&
                    now - axis->request_ms < STEER_FEEDBACK_TIMEOUT_MS;
                if (feedback->valid) {
                    feedback->angle_rad = angle;
                    feedback->rx_ms = now;
                }
                axis->pending = 0;
            }
            break;
        }
    }
}

/* 同期待機は原点復帰だけ。受信の解析は共通処理に委ねる。 */
bool robostride_read_parameter(uint8_t id, uint16_t index, void *value, uint8_t size, uint32_t timeout_ms)
{
    if (size > sizeof(home_reply.data)) {  // home_reply.data は4バイトの一時保存欄 制御モードなら1バイト、角度なら4バイト
        return false;
    }

    robostride_receive();  // 受信キューにすでにあるフレームを先に処理
    /* ID id のモーターから、項目番号 index の返信を待つと登録します。waiting = 1は待機中、received = 0はまだ返信を保存していない、という意味 */
    home_reply.id = id;
    home_reply.index = index;
    home_reply.waiting = 1;
    home_reply.received = 0;
    /* request_parameter() はRobStrideへパラメータ読み出し要求を送信キューに登録 */
    if (request_parameter(id, index) == HAL_OK) {
        uint32_t started = HAL_GetTick();
        /* 制限時間内で、FDCAN3がBus-Offになっていない間、受信キューを繰り返し確認 */
        while (HAL_GetTick() - started < timeout_ms && !(hfdcan3.Instance->PSR & FDCAN_PSR_BO)) {
            robostride_receive();
            if (home_reply.received) {
                break;
            }
            HAL_Delay(1);
        }
    }
    home_reply.waiting = 0;
    /* 返信の結果がエラーだった場合 */
    if (!home_reply.received || !home_reply.ok) {
        return false;
    }
    /* memcpy(コピー先, コピー元, バイト数); */
    memcpy(value, home_reply.data, size);
    return true;
}

/* 車輪に保存してある角度、受信時刻、有効フラグ、故障フラグ */
const SteeringFeedback *robostride_feedback(uint8_t wheel)
{
    return &steering[wheel].feedback;
}

void robostride_set_homed(uint8_t wheel)
{
    steering[wheel].homed = true;
}

HAL_StatusTypeDef robostride_request_angle(uint8_t wheel, uint32_t now)
{
    SteeringAxis *axis = &steering[wheel];
    if (axis->pending) {
        if (now - axis->request_ms < STEER_FEEDBACK_TIMEOUT_MS) {
            return HAL_OK;
        }

        axis->pending = false;
        axis->feedback.valid = false;
    }

    HAL_StatusTypeDef status = request_parameter(swerve_modules[wheel].steer_id, RS_PARAM_MECH_POS);
    if (status == HAL_OK) {
        axis->request_ms = now;
        axis->pending = true;
    }
    return status;
}

static void CAN3_DiagCapture(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance != FDCAN3) {
        return;
    }

    /* Reading PSR acknowledges the last error code. Preserve this first read. */
    uint32_t psr = hfdcan->Instance->PSR;  // 通信状態や最後のエラーを保持
    uint32_t ecr = hfdcan->Instance->ECR;  // 送受信エラーカウンタを保持
    uint32_t lec = psr & FDCAN_PSR_LEC;

    /* 最初に検出したBus-Offの情報を残しておく */
    if ((psr & FDCAN_PSR_BO) != 0U && can3_diag_busoff_seen == 0U) {
        can3_diag_busoff_psr = psr;
        can3_diag_busoff_ecr = ecr;
        can3_diag_busoff_seen = 1U;
    }

    /* エラーがなければ終了 */
    if (lec == 0U || lec == 7U) {
        return;
    }

    /* エラー情報を配列に保存 */
    uint32_t index = can3_diag_count;
    if (index < CAN3_DIAG_CAPACITY) {
        can3_diag_events[index].tick = HAL_GetTick();
        can3_diag_events[index].psr = psr;
        can3_diag_events[index].ecr = ecr;
        can3_diag_events[index].hal_error = hfdcan->ErrorCode;
        can3_diag_count = index + 1U;
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    CAN3_DiagCapture(hfdcan);
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    (void)ErrorStatusITs;
    CAN3_DiagCapture(hfdcan);
}

/* FDCAN3を開始した後、最初の送信より前にエラー通知を有効にする。 */
static HAL_StatusTypeDef CAN3_DiagEnable(void)
{
    return HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0U);
}

/* 起動時に記録したエラーをシリアルへ出す。 */
void robostride_print_startup_errors(void)
{
    static uint32_t printed;
    static uint32_t busoff_printed;
    uint32_t count = can3_diag_count;

    while (printed < count) {
        uint32_t i = printed;
        uint32_t psr = can3_diag_events[i].psr;
        uint32_t ecr = can3_diag_events[i].ecr;
        printf("CAN3_FIRST[%lu] t=%lu LEC=%lu TEC=%lu REC=%lu "
            "PSR=0x%08lX ECR=0x%08lX HAL=0x%08lX\r\n",
               (unsigned long)i,
               (unsigned long)can3_diag_events[i].tick,
               (unsigned long)(psr & FDCAN_PSR_LEC),
               (unsigned long)((ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos),
               (unsigned long)((ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos),
               (unsigned long)psr, (unsigned long)ecr,
               (unsigned long)can3_diag_events[i].hal_error);
        printed++;
    }

    if (can3_diag_busoff_seen != 0U && busoff_printed == 0U) {
        printf("CAN3_BUSOFF PSR=0x%08lX ECR=0x%08lX\r\n",
               (unsigned long)can3_diag_busoff_psr,
               (unsigned long)can3_diag_busoff_ecr);
        busoff_printed = 1U;
    }
}

void robostride_can_start(void)
{
    FDCAN_FilterTypeDef filter = {0};              // フィルタの設定を入れる構造体を、すべてゼロで初期化

    filter.IdType       = FDCAN_EXTENDED_ID;       // RobStrideが使う29ビットの拡張IDを対象に
    filter.FilterIndex  = 0;                       // 拡張ID用フィルタの0番に登録
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  // RX FIFO0
    filter.FilterID1    = 0;
    filter.FilterID2    = 0;  // マスク0：すべての拡張IDを通す

    /* この設定を実際にFDCAN3へ登録 */
    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &filter) != HAL_OK) {
        Error_Handler();
    }

    /* フィルタに一致しない標準ID・拡張IDを捨てる設定 */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT,FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        Error_Handler();
    }

    /* HAL_FDCAN_Start() でFDCAN3が送受信 CAN3_DiagEnable() でBus-Offなどのエラー通知を有効に*/
    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK || CAN3_DiagEnable() != HAL_OK) {  // CAN3_DiagEnableでHAL_FDCAN_RxFifo0Callback
        Error_Handler();
    }
}

void robostride_scan_ids(void)
{
    uint8_t seen[256] = {0};  // CAN ID 0〜255 に対応するチェック表　最初は全部 0　例えばID 127 が見つかったら seen[127] = 1
    uint16_t found_count = 0; // 見つかった異なるIDの数

    printf("\r\n=== RobStride ID scan start ===\r\n");

    /* 古い受信データを捨てる */
    while (HAL_FDCAN_GetRxFifoFillLevel( &hfdcan3, FDCAN_RX_FIFO0) > 0U) {  // FDCAN3の受信キュー RX FIFO0 に何件たまっているかを返す
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[64];

        if (HAL_FDCAN_GetRxMessage( &hfdcan3, FDCAN_RX_FIFO0, &header, data) != HAL_OK) { // キューの先頭からフレームを1件ずつ取り出し 残っている個数分捨てる
            break;
        }
    }

    /* 8bitのID候補を順番に問い合わせる */
    for (uint16_t candidate = 0; candidate < 256; candidate++) {
        uint8_t request[8] = {0}; // 送信データ8バイトをすべてゼロで埋める

        /* 通信タイプ0：ID取得。ID変更やEnableは送らない RobStrideへの命令で0x00はID取得 */
        HAL_StatusTypeDef st = robostride_send(0x00, ROBOSTRIDE_HOST_ID, (uint8_t)candidate, request);

        /* もし正常に読み取れなかったらrobostride_print_startup_errors関数でエラー表示 */
        if (st != HAL_OK) {
            robostride_print_startup_errors();
            printf("Scan TX failed: candidate=%u status=%d\r\n", (unsigned int)candidate, (int)st);
            return;
        }

        uint32_t started = HAL_GetTick();

        /* 各IDについて約30ms応答を待つ */
        while ((uint32_t)(HAL_GetTick() - started) < 30U) {  // 返信を約30 ms待つ
            while (HAL_FDCAN_GetRxFifoFillLevel( &hfdcan3, FDCAN_RX_FIFO0) > 0U) {  // while は、FDCAN3の受信キュー RX FIFO0 にたまっているフレームを1件ずつ処理
                FDCAN_RxHeaderTypeDef header;
                uint8_t data[64];

                if (HAL_FDCAN_GetRxMessage( &hfdcan3, FDCAN_RX_FIFO0, &header, data) != HAL_OK) { // FDCAN3の受信キュー RX FIFO0 にたまっているフレームを1件ずつ処理
                    break;
                }

                /* can設定とあっているか確認 */
                if (header.IdType != FDCAN_EXTENDED_ID || header.FDFormat != FDCAN_CLASSIC_CAN || header.RxFrameType != FDCAN_DATA_FRAME || header.DataLength != FDCAN_DLC_BYTES_8) {
                    continue;
                }

                // 式	                                取り出す部分 確かめること
                // (header.Identifier >> 24) & 0x1FU	bit 28〜24	通信タイプがID取得の 0x00 か
                uint8_t type = (uint8_t)((header.Identifier >> 24) & 0x1FU);
                // header.Identifier & 0xFFU	        bit 7〜0	ID返信を表す 0xFE か
                if (type != 0x00U || (header.Identifier & 0xFFU) != 0xFEU) {
                    continue;
                }
                // (header.Identifier >> 8) & 0xFFFFU	bit 23〜8	返信したモーターのID 、欄は16ビットなのだがrobostride05が有効なIDとして扱うのは 0〜255
                uint16_t motor_id = (uint16_t)((header.Identifier >> 8) & 0xFFFFU);

                /* motor_id > 255U を確認し、範囲外なら || の右側を評価せずに飛ばす。範囲内でも、seen[motor_id]がすでに 1 なら表示を飛ばす。初めてのIDだけ記録し、件数を増やす */
                if (motor_id > 255U || seen[motor_id]) {
                    continue;
                }

                seen[motor_id] = 1;
                found_count++;

                printf("FOUND: motor_id=%u (0x%02X)"
                       " CAN_ID=0x%08lX UID=",
                       (unsigned int)motor_id,
                       (unsigned int)motor_id,
                       (unsigned long)header.Identifier);

                for (uint8_t i = 0; i < 8; i++) {
                    printf("%02X", (unsigned int)data[i]);
                }

                printf("\r\n");
            }

            HAL_Delay(1);
        }
    }

    printf("=== Scan finished: %u distinct ID(s) ===\r\n",
           (unsigned int)found_count);
}
