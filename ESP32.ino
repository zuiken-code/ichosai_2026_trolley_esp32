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
// Protocol
// Pi → ESP32
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
// <BBBBfHH
//
// uint8_t  header1
// uint8_t  header2
// uint8_t  motorId
// uint8_t  controlMode
// float    target
// uint16_t sequence
// uint16_t crc
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


// 構造体サイズが12バイトでなければコンパイルエラー
static_assert(
    sizeof(MotorCommand) == 12,
    "MotorCommand must be 12 bytes"
);


// ============================================================
// RX Statistics
// ============================================================

uint32_t totalPackets      = 0;
uint32_t motor1Packets     = 0;
uint32_t motor2Packets     = 0;
uint32_t validPackets      = 0;
uint32_t invalidPackets    = 0;
uint32_t invalidSizePackets = 0;

uint32_t lastStatsTime = 0;


// ============================================================
// CRC16
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
// ControlMode → 文字列
// ============================================================

const char* controlModeToString(
    uint8_t mode)
{
    switch (mode)
    {
        case DUTY_CYCLE:
            return "DUTY_CYCLE";

        case VELOCITY:
            return "VELOCITY";

        case POSITION:
            return "POSITION";

        case DISABLE:
            return "DISABLE";

        default:
            return "UNKNOWN";
    }
}


// ============================================================
// UDP Packet Receive
// ============================================================

void receivePackets()
{
    // UDP受信バッファに残っているパケットを
    // 可能な限り全部処理する
    while (true)
    {
        int packetSize = udp.parsePacket();

        if (packetSize <= 0)
        {
            return;
        }


        // ----------------------------------------------------
        // Packet Size Check
        // ----------------------------------------------------

        if (packetSize != sizeof(MotorCommand))
        {
            invalidPackets++;
            invalidSizePackets++;

            // パケットを捨てる
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

        int received = udp.read(
            reinterpret_cast<uint8_t*>(&command),
            sizeof(command)
        );


        if (received != sizeof(command))
        {
            invalidPackets++;
            continue;
        }


        // ----------------------------------------------------
        // Total Packet Count
        // ----------------------------------------------------

        totalPackets++;


        // ----------------------------------------------------
        // Motor ID Count
        // ----------------------------------------------------

        if (command.motorId == 1)
        {
            motor1Packets++;
        }
        else if (command.motorId == 2)
        {
            motor2Packets++;
        }


        // ----------------------------------------------------
        // Header Check
        // ----------------------------------------------------

        bool headerOK =
            command.header1 == HEADER_1 &&
            command.header2 == HEADER_2;


        // ----------------------------------------------------
        // CRC Check
        //
        // Python側:
        //
        // payload = struct.pack(
        //     "<BBfB",
        //     motor_id,
        //     control_type,
        //     reference,
        //     sequence,
        // )
        //
        // CRC対象:
        //
        // motorId
        // controlMode
        // target
        // sequence
        //
        // 合計8バイト
        // ----------------------------------------------------

        uint16_t calculatedCRC =
            calculateCRC(
                reinterpret_cast<uint8_t*>(&command.motorId),
                sizeof(command.motorId)
                + sizeof(command.controlMode)
                + sizeof(command.target)
                + sizeof(command.sequence)
            );


        bool crcOK =
            command.crc == calculatedCRC;


        // ----------------------------------------------------
        // Final Validation
        // ----------------------------------------------------

        bool packetOK =
            headerOK &&
            crcOK;


        if (packetOK)
        {
            validPackets++;

            // 必要最低限のログだけ表示
            Serial.printf(
                "RECV motor=%u seq=%u target=%.2f\n",
                command.motorId,
                command.sequence,
                command.target
            );
        }
        else
        {
            invalidPackets++;

            // 異常パケットだけ表示
            Serial.printf(
                "INVALID motor=%u seq=%u header=%s crc=%s\n",
                command.motorId,
                command.sequence,
                headerOK ? "OK" : "NG",
                crcOK ? "OK" : "NG"
            );
        }


        // ----------------------------------------------------
        // ここから先で実際のモーター制御に繋げる
        //
        // 例:
        //
        // if (packetOK)
        // {
        //     if (command.motorId == 1)
        //     {
        //         // Motor 1へ送信
        //     }
        //     else if (command.motorId == 2)
        //     {
        //         // Motor 2へ送信
        //     }
        // }
        // ----------------------------------------------------
    }
}


// ============================================================
// Statistics
// ============================================================

void printStatistics()
{
    uint32_t now = millis();

    if (now - lastStatsTime < 1000)
    {
        return;
    }

    lastStatsTime = now;


    Serial.println();
    Serial.println("========== RX STATISTICS ==========");

    Serial.printf(
        "Total packets : %lu\n",
        totalPackets
    );

    Serial.printf(
        "Motor 1       : %lu\n",
        motor1Packets
    );

    Serial.printf(
        "Motor 2       : %lu\n",
        motor2Packets
    );

    Serial.printf(
        "Valid         : %lu\n",
        validPackets
    );

    Serial.printf(
        "Invalid       : %lu\n",
        invalidPackets
    );

    Serial.printf(
        "Invalid size  : %lu\n",
        invalidSizePackets
    );

    Serial.println("===================================");
}


// ============================================================
// setup
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);


    // --------------------------------------------------------
    // Startup Message
    // --------------------------------------------------------

    Serial.println();
    Serial.println("========================================");
    Serial.println("        ESP32 UDP TEST RECEIVER");
    Serial.println("========================================");


    // --------------------------------------------------------
    // Struct Size Check
    // --------------------------------------------------------

    Serial.print("MotorCommand size: ");
    Serial.println(sizeof(MotorCommand));


    // --------------------------------------------------------
    // Wi-Fi
    // --------------------------------------------------------

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
    );

    Serial.print("Connecting to Wi-Fi");

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);

        Serial.print(".");
    }

    Serial.println();

    Serial.println("Wi-Fi connected");


    // --------------------------------------------------------
    // IP Address
    // --------------------------------------------------------

    Serial.print("ESP32 IP : ");
    Serial.println(WiFi.localIP());


    // --------------------------------------------------------
    // UDP
    // --------------------------------------------------------

    udp.begin(UDP_PORT);

    Serial.print("UDP Port : ");
    Serial.println(UDP_PORT);


    Serial.println();
    Serial.println("Waiting for packets...");


    lastStatsTime = millis();
}


// ============================================================
// loop
// ============================================================

void loop()
{
    // UDPを最優先で処理
    receivePackets();

    // 1秒ごとに統計表示
    printStatistics();
}