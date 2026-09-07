// ============================================================
// RS775 Smart Motor Driver
// Power Board Direct Drive (ESP32 / UDP)
//
// Logic基板を使わず、ESP32からPower基板のコネクタを
// 直接叩いてRS-775を2台駆動する暫定ファームウェア。
//
// - 制御はPercent output (duty)のみ
// - エンコーダが無いため速度・位置制御は非対応
// - 急な指令変化はスルーレート制限で鈍らせる
// - 方向反転はduty=0を経由し、デッドタイムを挟む
// - 指令が途切れたら自動で出力を落とす
//
// Target : ESP32-WROOM-32E (AE-ESP32-WROOM-32E-MINI)
// Driver : VNH5019A-E (DocID15701 Rev 11)
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "secrets.h"


// ============================================================
// Wi-Fi
// ============================================================

constexpr uint16_t UDP_PORT = 5000;

WiFiUDP udp;


// ============================================================
// Motor Hardware Map
//
// ADC2 (GPIO 0/2/4/12/13/14/15/25/26/27) はWi-Fi動作中に
// 使えないため、CSはADC1 (GPIO 32-36/39) から取る。
// ストラッピングピン (GPIO 0/2/5/12/15) は
// 起動時対策のプルダウンと衝突するため使わない。
// ============================================================

struct MotorPins
{
    uint8_t inA;
    uint8_t inB;
    uint8_t pwm;
    uint8_t en;      // ENA/DIAGAとENB/DIAGBを共通で接続
    uint8_t cs;      // ADC1のみ
    uint8_t channel; // LEDCチャンネル（Core 3.xでは直接使用しない）
};


constexpr MotorPins MOTOR_PINS[] =
{
    // inA inB pwm en  cs  ch
    {  25, 26, 27, 13, 34, 0 },  // motorId = 1
    {  18, 19, 23, 14, 35, 1 }   // motorId = 2
};

constexpr uint8_t MOTOR_COUNT =
    sizeof(MOTOR_PINS) / sizeof(MOTOR_PINS[0]);


// ============================================================
// Control Parameters
// ============================================================

// PWM周波数の上限は20kHz。マージンを取って16kHz
constexpr uint32_t PWM_FREQ_HZ  = 16000;
constexpr uint8_t  PWM_RES_BITS = 11;
constexpr uint16_t PWM_MAX      = (1u << PWM_RES_BITS) - 1;

constexpr uint32_t CONTROL_PERIOD_MS = 1;
constexpr float    CONTROL_DT        = 0.001f;

// 指令が途切れてから出力を落とし始めるまでの時間
constexpr uint32_t COMMAND_TIMEOUT_MS = 150;

// スルーレート [duty/s]
// 2.0 = 0から100%まで0.5秒
constexpr float SLEW_UP       = 2.0f;
constexpr float SLEW_DOWN     = 4.0f;
constexpr float SLEW_FAILSAFE = 4.0f;

// duty=0に到達してから方向ビットを変えるまでの待機時間。
constexpr uint32_t REVERSE_DEADTIME_MS = 20;

// これ以下のdutyは0として扱う
constexpr float DUTY_EPSILON = 0.001f;

// 故障検出時にENを引き下げておく時間
constexpr uint32_t FAULT_HOLD_MS = 250;


// ============================================================
// Current Sense
//
// VNH5019A-E: ISENSE = IOUT / K
//   K typ = 7000
//
// Power基板: R9 = 1k
//   VSENSE = IOUT / 7000 * 1000 = 0.1429 V/A
//
// ESP32側: R8 = 10k とGNDへの8.2kで分圧
//   分圧比 = 8200 / 18200 = 0.4505
//   ADC端の感度 = 64.4 mV/A
//
// この係数は校正前の計算値。
// ============================================================

constexpr float MV_PER_AMP = 64.4f;

// PWMのLo期間はCSがハイインピーダンスになるため、
// 低duty域の読み値は電流値として使えない
constexpr float CS_VALID_MIN_DUTY = 0.30f;

// 校正前の暫定閾値
constexpr float CURRENT_WARN_A = 20.0f;

// CSを読む間隔
constexpr uint8_t CS_SAMPLE_DIVIDER = 10;


// ============================================================
// Protocol
// Pi -> ESP32
// ============================================================

constexpr uint8_t HEADER_1 = 0xAA;
constexpr uint8_t HEADER_2 = 0x55;


enum ControlMode : uint8_t
{
    DUTY_CYCLE = 0,
    VELOCITY   = 1,
    POSITION   = 2,
    DISABLE    = 3
};


// Python:
//
// struct.pack(
//     "<BBBBfHH",
//     0xAA, 0x55,
//     motor_id, control_mode,
//     target,
//     sequence,
//     crc,
// )
//
// Total = 12 bytes

struct __attribute__((packed)) MotorCommand
{
    uint8_t header1;
    uint8_t header2;

    uint8_t motorId;
    uint8_t controlMode;

    float target;

    uint16_t sequence;
    uint16_t crc;
};


static_assert(
    sizeof(MotorCommand) == 12,
    "MotorCommand must be 12 bytes"
);


// ============================================================
// Motor State
// ============================================================

enum FaultState : uint8_t
{
    FAULT_NONE = 0,
    FAULT_HOLD = 1
};


struct MotorState
{
    // --- 受信側が書き、制御側が読む ---
    volatile float    requestDuty;
    volatile bool     requestStop;
    volatile uint32_t lastCommandMs;
    volatile uint16_t lastSequence;
    volatile bool     sequenceValid;

    // --- 制御側のみが触る ---
    float    duty;
    int8_t   direction;
    uint32_t directionAllowedMs;
    uint8_t  csDivider;

    FaultState fault;
    uint32_t   faultUntilMs;

    // --- 統計 ---
    volatile uint32_t packets;
    volatile uint32_t staleDrops;
    volatile uint32_t faultCount;
    volatile uint32_t overCurrentCount;
    volatile float    currentA;
    volatile bool     currentValid;
    volatile bool     timedOut;
};


MotorState motors[MOTOR_COUNT];

portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;


// ============================================================
// RX Statistics
// ============================================================

volatile uint32_t totalPackets       = 0;
volatile uint32_t validPackets       = 0;
volatile uint32_t invalidPackets     = 0;
volatile uint32_t invalidSizePackets = 0;
volatile uint32_t headerErrors       = 0;
volatile uint32_t crcErrors          = 0;
volatile uint32_t unknownMotorId     = 0;
volatile uint32_t unsupportedMode    = 0;

uint32_t lastStatsTime = 0;


// ============================================================
// CRC16
//
// 多項式 0xA001 (MODBUS)、初期値 0xFFFF
// ============================================================

uint16_t calculateCRC(
    const uint8_t* data,
    size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}


// ============================================================
// Low Level Output
// ============================================================

void writeOutput(
    uint8_t index)
{
    const MotorPins& pins = MOTOR_PINS[index];
    MotorState& m = motors[index];


    // ----------------------------------------------------
    // VNH5019A-E 真理値表
    //
    // INA=1 INB=0 : 正転
    // INA=0 INB=1 : 逆転
    // INA=0 INB=0 : GNDブレーキ
    //
    // PWM=Loで両ローサイドがオフになり、
    // 出力はハイインピーダンス (コースト) になる
    // ----------------------------------------------------

    if (m.direction > 0)
    {
        digitalWrite(pins.inA, HIGH);
        digitalWrite(pins.inB, LOW);
    }
    else if (m.direction < 0)
    {
        digitalWrite(pins.inA, LOW);
        digitalWrite(pins.inB, HIGH);
    }
    else
    {
        digitalWrite(pins.inA, LOW);
        digitalWrite(pins.inB, LOW);
    }


    uint16_t compare = 0;

    if (m.direction != 0)
    {
        compare =
            (uint16_t)(m.duty * (float)PWM_MAX + 0.5f);
    }

    // ESP32 Arduino Core 3.x:
    // ledcWrite() はチャンネルではなくGPIOピンを指定
    ledcWrite(pins.pwm, compare);
}


// ============================================================
// Enable Control
//
// 今回の実験では、動作確認できているテストコードと
// 同じく ESP32 GPIO から HIGH を出してENを有効化する。
//
// enable = true
//     EN = HIGH
//
// enable = false
//     EN = LOW
// ============================================================

void setEnable(
    uint8_t index,
    bool enable)
{
    const MotorPins& pins = MOTOR_PINS[index];

    pinMode(pins.en, OUTPUT);

    if (enable)
    {
        digitalWrite(pins.en, HIGH);
    }
    else
    {
        digitalWrite(pins.en, LOW);
    }
}


// ============================================================
// Fault
// ============================================================

bool readFault(
    uint8_t index)
{
    // EN/DIAGピンを読み取る。
    //
    // 今回はENをOUTPUT HIGHにしているため、
    // 通常時はHIGHになる。
    //
    // ドライバ側がLOWに引いた場合はFAULTと判定する。

    return digitalRead(MOTOR_PINS[index].en) == LOW;
}


// ============================================================
// Current Sense
// ============================================================

float readCurrent(
    uint8_t index)
{
    uint32_t mv =
        analogReadMilliVolts(MOTOR_PINS[index].cs);

    return (float)mv / MV_PER_AMP;
}


// ============================================================
// Force Safe Outputs
// ============================================================

void forceSafeOutputs()
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        const MotorPins& pins = MOTOR_PINS[i];

        pinMode(pins.inA, OUTPUT);
        pinMode(pins.inB, OUTPUT);

        digitalWrite(pins.inA, LOW);
        digitalWrite(pins.inB, LOW);

        ledcWrite(pins.pwm, 0);

        setEnable(i, false);
    }
}


// ============================================================
// Control Step
// ============================================================

void controlStep(
    uint8_t index,
    uint32_t now)
{
    MotorState& m = motors[index];


    // ----------------------------------------------------
    // 指令の取り込み
    // ----------------------------------------------------

    float    request;
    bool     stop;
    uint32_t lastCommand;

    portENTER_CRITICAL(&stateMux);

    request     = m.requestDuty;
    stop        = m.requestStop;
    lastCommand = m.lastCommandMs;

    portEXIT_CRITICAL(&stateMux);


    bool timeout =
        (uint32_t)(now - lastCommand) >
        COMMAND_TIMEOUT_MS;

    m.timedOut = timeout;


    // ----------------------------------------------------
    // 故障処理
    // ----------------------------------------------------

    if (m.fault == FAULT_HOLD)
    {
        if ((int32_t)(now - m.faultUntilMs) >= 0)
        {
            m.fault = FAULT_NONE;

            setEnable(index, true);
        }
    }
    else if (m.direction != 0 && readFault(index))
    {
        m.fault        = FAULT_HOLD;
        m.faultUntilMs = now + FAULT_HOLD_MS;
        m.faultCount++;

        setEnable(index, false);

        m.duty      = 0.0f;
        m.direction = 0;

        m.directionAllowedMs =
            now + REVERSE_DEADTIME_MS;

        m.currentValid = false;

        writeOutput(index);

        return;
    }


    // ----------------------------------------------------
    // 目標値の決定
    // ----------------------------------------------------

    bool failsafe =
        timeout ||
        stop ||
        m.fault != FAULT_NONE;

    float goal =
        failsafe ? 0.0f : request;

    if (goal >  1.0f) goal =  1.0f;
    if (goal < -1.0f) goal = -1.0f;


    int8_t goalDirection = 0;

    if (goal >  DUTY_EPSILON)
    {
        goalDirection = 1;
    }

    if (goal < -DUTY_EPSILON)
    {
        goalDirection = -1;
    }


    float goalMagnitude = fabsf(goal);


    // ----------------------------------------------------
    // 方向の遷移
    // ----------------------------------------------------

    float commandMagnitude;

    if (m.direction == 0)
    {
        commandMagnitude = 0.0f;

        if (
            goalDirection != 0 &&
            (int32_t)(now - m.directionAllowedMs) >= 0
        )
        {
            m.direction = goalDirection;
        }
    }
    else if (
        goalDirection != 0 &&
        goalDirection != m.direction
    )
    {
        commandMagnitude = 0.0f;
    }
    else
    {
        commandMagnitude = goalMagnitude;
    }


    // ----------------------------------------------------
    // スルーレート制限
    // ----------------------------------------------------

    float rate;

    if (failsafe)
    {
        rate = SLEW_FAILSAFE;
    }
    else if (commandMagnitude > m.duty)
    {
        rate = SLEW_UP;
    }
    else
    {
        rate = SLEW_DOWN;
    }


    float step =
        rate * CONTROL_DT;

    float delta =
        commandMagnitude - m.duty;


    if (delta >  step) delta =  step;
    if (delta < -step) delta = -step;


    m.duty += delta;


    if (m.duty < DUTY_EPSILON)
    {
        m.duty = 0.0f;
    }

    if (m.duty > 1.0f)
    {
        m.duty = 1.0f;
    }


    // ----------------------------------------------------
    // duty=0になったら方向解除
    // ----------------------------------------------------

    if (
        m.duty == 0.0f &&
        m.direction != 0
    )
    {
        bool reversing =
            goalDirection != 0 &&
            goalDirection != m.direction;

        if (
            reversing ||
            goalDirection == 0
        )
        {
            m.direction = 0;

            m.directionAllowedMs =
                now + REVERSE_DEADTIME_MS;
        }
    }


    writeOutput(index);


    // ----------------------------------------------------
    // 電流センス
    // ----------------------------------------------------

    m.csDivider++;

    if (m.csDivider >= CS_SAMPLE_DIVIDER)
    {
        m.csDivider = 0;

        if (
            m.direction != 0 &&
            m.duty >= CS_VALID_MIN_DUTY
        )
        {
            float amps =
                readCurrent(index);

            m.currentA =
                amps;

            m.currentValid =
                true;

            if (amps > CURRENT_WARN_A)
            {
                m.overCurrentCount++;
            }
        }
        else
        {
            m.currentValid =
                false;
        }
    }
}


// ============================================================
// Control Task
//
// Wi-Fiスタックはcore 0で回るため、
// 制御はcore 1に固定して1kHzで走らせる
// ============================================================

void controlTask(
    void* arg)
{
    (void)arg;

    TickType_t last =
        xTaskGetTickCount();

    for (;;)
    {
        uint32_t now =
            millis();

        for (
            uint8_t i = 0;
            i < MOTOR_COUNT;
            i++
        )
        {
            controlStep(i, now);
        }

        vTaskDelayUntil(
            &last,
            pdMS_TO_TICKS(CONTROL_PERIOD_MS)
        );
    }
}


// ============================================================
// UDP Packet Receive
// ============================================================

void receivePackets()
{
    while (true)
    {
        int packetSize =
            udp.parsePacket();

        if (packetSize <= 0)
        {
            return;
        }


        // ----------------------------------------------------
        // Packet Size Check
        // ----------------------------------------------------

        if (
            packetSize !=
            (int)sizeof(MotorCommand)
        )
        {
            invalidPackets++;
            invalidSizePackets++;

            while (udp.available())
            {
                udp.read();
            }

            continue;
        }


        // ----------------------------------------------------
        // Read Packet
        // ----------------------------------------------------

        MotorCommand command;

        int received =
            udp.read(
                reinterpret_cast<uint8_t*>(&command),
                sizeof(command)
            );

        if (
            received !=
            (int)sizeof(command)
        )
        {
            invalidPackets++;
            continue;
        }

        totalPackets++;


        // ----------------------------------------------------
        // Header / CRC Check
        // ----------------------------------------------------

        bool headerOK =
            command.header1 == HEADER_1 &&
            command.header2 == HEADER_2;


        uint16_t calculatedCRC =
            calculateCRC(
                reinterpret_cast<uint8_t*>(
                    &command.motorId
                ),
                sizeof(command.motorId)
                + sizeof(command.controlMode)
                + sizeof(command.target)
                + sizeof(command.sequence)
            );


        bool crcOK =
            command.crc == calculatedCRC;


        if (!headerOK)
        {
            headerErrors++;
        }

        if (!crcOK)
        {
            crcErrors++;
        }


        if (!headerOK || !crcOK)
        {
            invalidPackets++;
            continue;
        }


        // ----------------------------------------------------
        // Motor ID Check
        // ----------------------------------------------------

        if (
            command.motorId < 1 ||
            command.motorId > MOTOR_COUNT
        )
        {
            unknownMotorId++;
            invalidPackets++;
            continue;
        }


        uint8_t index =
            command.motorId - 1;

        MotorState& m =
            motors[index];


        // ----------------------------------------------------
        // Sequence Check
        // ----------------------------------------------------

        uint32_t now =
            millis();


        bool resync =
            !m.sequenceValid ||
            (uint32_t)(
                now - m.lastCommandMs
            ) > COMMAND_TIMEOUT_MS;


        if (!resync)
        {
            int16_t diff =
                (int16_t)(
                    command.sequence -
                    m.lastSequence
                );

            if (diff <= 0)
            {
                m.staleDrops++;
                continue;
            }
        }


        // ----------------------------------------------------
        // Apply
        // ----------------------------------------------------

        switch (command.controlMode)
        {
            case DUTY_CYCLE:
            {
                float target =
                    command.target;

                if (target >  1.0f)
                {
                    target = 1.0f;
                }

                if (target < -1.0f)
                {
                    target = -1.0f;
                }


                portENTER_CRITICAL(&stateMux);

                m.requestDuty   = target;
                m.requestStop   = false;
                m.lastCommandMs = now;
                m.lastSequence  = command.sequence;
                m.sequenceValid = true;

                portEXIT_CRITICAL(&stateMux);


                validPackets++;
                m.packets++;

                break;
            }


            case DISABLE:
            {
                portENTER_CRITICAL(&stateMux);

                m.requestDuty   = 0.0f;
                m.requestStop   = true;
                m.lastCommandMs = now;
                m.lastSequence  = command.sequence;
                m.sequenceValid = true;

                portEXIT_CRITICAL(&stateMux);


                validPackets++;
                m.packets++;

                break;
            }


            case VELOCITY:
            case POSITION:
            default:
            {
                // Logic基板が無いためエンコーダを持たない。
                // 指令を無視する。
                unsupportedMode++;

                break;
            }
        }
    }
}


// ============================================================
// Statistics
// ============================================================

void printStatistics()
{
    uint32_t now =
        millis();


    if (
        now - lastStatsTime <
        1000
    )
    {
        return;
    }


    lastStatsTime =
        now;


    Serial.println();
    Serial.println(
        "========== STATUS =========="
    );


    Serial.printf(
        "rx  total=%lu valid=%lu invalid=%lu\n",
        (unsigned long)totalPackets,
        (unsigned long)validPackets,
        (unsigned long)invalidPackets
    );


    Serial.printf(
        "err size=%lu header=%lu crc=%lu id=%lu mode=%lu\n",
        (unsigned long)invalidSizePackets,
        (unsigned long)headerErrors,
        (unsigned long)crcErrors,
        (unsigned long)unknownMotorId,
        (unsigned long)unsupportedMode
    );


    for (
        uint8_t i = 0;
        i < MOTOR_COUNT;
        i++
    )
    {
        MotorState& m =
            motors[i];


        Serial.printf(
            "M%u  pkt=%lu stale=%lu dir=%+d duty=%.3f "
            "req=%+.3f fault=%lu I=%s%.1fA oc=%lu%s\n",

            i + 1,

            (unsigned long)m.packets,

            (unsigned long)m.staleDrops,

            m.direction,

            m.duty,

            m.requestDuty,

            (unsigned long)m.faultCount,

            m.currentValid ? "" : "~",

            m.currentValid
                ? m.currentA
                : 0.0f,

            (unsigned long)m.overCurrentCount,

            m.timedOut
                ? " TIMEOUT"
                : ""
        );
    }


    Serial.printf(
        "wifi status=%d rssi=%d\n",
        (int)WiFi.status(),
        WiFi.RSSI()
    );


    Serial.println(
        "============================"
    );
}


// ============================================================
// setup
// ============================================================

void setup()
{
    Serial.begin(115200);


    // --------------------------------------------------------
    // 何よりも先に出力を安全側へ固定する
    // --------------------------------------------------------

    for (
        uint8_t i = 0;
        i < MOTOR_COUNT;
        i++
    )
    {
        const MotorPins& pins =
            MOTOR_PINS[i];


        pinMode(
            pins.inA,
            OUTPUT
        );

        pinMode(
            pins.inB,
            OUTPUT
        );


        digitalWrite(
            pins.inA,
            LOW
        );

        digitalWrite(
            pins.inB,
            LOW
        );


        setEnable(
            i,
            false
        );


        // ----------------------------------------------------
        // ESP32 Arduino Core 3.x
        //
        // 旧:
        // ledcSetup(channel, freq, resolution);
        // ledcAttachPin(pin, channel);
        //
        // 新:
        // ledcAttach(pin, freq, resolution);
        // ----------------------------------------------------

        if (
            !ledcAttach(
                pins.pwm,
                PWM_FREQ_HZ,
                PWM_RES_BITS
            )
        )
        {
            Serial.printf(
                "ERROR: PWM attach failed for motor %u\n",
                i + 1
            );
        }


        ledcWrite(
            pins.pwm,
            0
        );


        MotorState& m =
            motors[i];


        m.requestDuty        = 0.0f;
        m.requestStop        = true;
        m.lastCommandMs      = 0;
        m.lastSequence       = 0;
        m.sequenceValid      = false;

        m.duty               = 0.0f;
        m.direction          = 0;
        m.directionAllowedMs = 0;
        m.csDivider          = 0;

        m.fault              = FAULT_NONE;
        m.faultUntilMs       = 0;

        m.packets            = 0;
        m.staleDrops         = 0;
        m.faultCount         = 0;
        m.overCurrentCount   = 0;
        m.currentA           = 0.0f;
        m.currentValid       = false;
        m.timedOut           = true;
    }


    forceSafeOutputs();


    // --------------------------------------------------------
    // ADC
    // --------------------------------------------------------

    analogReadResolution(12);


    for (
        uint8_t i = 0;
        i < MOTOR_COUNT;
        i++
    )
    {
        analogSetPinAttenuation(
            MOTOR_PINS[i].cs,
            ADC_11db
        );
    }


    delay(500);


    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        "  RS775 POWER BOARD / ESP32 UDP DRIVE"
    );

    Serial.println(
        "========================================"
    );


    Serial.printf(
        "motors            : %u\n",
        MOTOR_COUNT
    );


    Serial.printf(
        "MotorCommand size : %u\n",
        (unsigned)sizeof(MotorCommand)
    );


    Serial.printf(
        "pwm               : %lu Hz / %u bit\n",
        (unsigned long)PWM_FREQ_HZ,
        PWM_RES_BITS
    );


    // --------------------------------------------------------
    // Wi-Fi
    // --------------------------------------------------------

    WiFi.mode(
        WIFI_STA
    );

    WiFi.setSleep(
        false
    );

    WiFi.setAutoReconnect(
        true
    );


#ifdef USE_STATIC_IP

    // secrets.hで
    // WIFI_LOCAL_IP / WIFI_GATEWAY / WIFI_SUBNET
    // を定義している場合は固定IPを使用

    WiFi.config(
        WIFI_LOCAL_IP,
        WIFI_GATEWAY,
        WIFI_SUBNET
    );

#endif


    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
    );


    Serial.print(
        "Connecting to Wi-Fi"
    );


    while (
        WiFi.status() !=
        WL_CONNECTED
    )
    {
        delay(500);

        Serial.print(
            "."
        );
    }


    Serial.println();


    Serial.print(
        "ESP32 IP          : "
    );

    Serial.println(
        WiFi.localIP()
    );


    // --------------------------------------------------------
    // UDP
    // --------------------------------------------------------

    udp.begin(
        UDP_PORT
    );


    Serial.print(
        "UDP Port          : "
    );

    Serial.println(
        UDP_PORT
    );


    // --------------------------------------------------------
    // 制御タスク
    // --------------------------------------------------------

    xTaskCreatePinnedToCore(
        controlTask,
        "control",
        4096,
        nullptr,
        5,
        nullptr,
        1
    );


    // --------------------------------------------------------
    // ここでENを解放
    //
    // 今回はテストコードと同じく
    // EN = HIGH
    // --------------------------------------------------------

    for (
        uint8_t i = 0;
        i < MOTOR_COUNT;
        i++
    )
    {
        setEnable(
            i,
            true
        );
    }


    lastStatsTime =
        millis();


    Serial.println(
        "Ready"
    );
}


// ============================================================
// loop
//
// UDPの取り込みと統計表示のみ。
// Wi-Fi切断はタイムアウトが拾うので、
// ここでは扱わない。
// ============================================================

void loop()
{
    receivePackets();

    printStatistics();
}