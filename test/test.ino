#include <Arduino.h>

// ============================================================
// Motor 1
// ============================================================

constexpr uint8_t M1_INA = 25;
constexpr uint8_t M1_INB = 26;
constexpr uint8_t M1_PWM = 27;
constexpr uint8_t M1_EN  = 13;


// ============================================================
// Motor 2
// ============================================================

constexpr uint8_t M2_INA = 18;
constexpr uint8_t M2_INB = 19;
constexpr uint8_t M2_PWM = 23;
constexpr uint8_t M2_EN  = 14;


// ============================================================
// PWM
// ============================================================

constexpr uint32_t PWM_FREQ_HZ  = 16000;
constexpr uint8_t  PWM_RES_BITS = 11;

constexpr uint16_t PWM_MAX =
    (1u << PWM_RES_BITS) - 1;


// ============================================================
// テスト設定
// ============================================================

// 20%
constexpr float TEST_DUTY = 0.20f;

// 回転時間
constexpr uint32_t RUN_TIME_MS = 1000;

// 停止時間
constexpr uint32_t STOP_TIME_MS = 1000;

// 逆転前のデッドタイム
constexpr uint32_t REVERSE_DEADTIME_MS = 20;


// ============================================================
// Motor 1
// ============================================================

void motor1Stop()
{
    digitalWrite(M1_INA, LOW);
    digitalWrite(M1_INB, LOW);
    ledcWrite(M1_PWM, 0);
}

void motor1Forward(float duty)
{
    duty = constrain(duty, 0.0f, 1.0f);

    digitalWrite(M1_INA, HIGH);
    digitalWrite(M1_INB, LOW);

    uint16_t pwm = (uint16_t)(duty * PWM_MAX);

    ledcWrite(M1_PWM, pwm);
}

void motor1Reverse(float duty)
{
    duty = constrain(duty, 0.0f, 1.0f);

    digitalWrite(M1_INA, LOW);
    digitalWrite(M1_INB, HIGH);

    uint16_t pwm = (uint16_t)(duty * PWM_MAX);

    ledcWrite(M1_PWM, pwm);
}


// ============================================================
// Motor 2
// ============================================================

void motor2Stop()
{
    digitalWrite(M2_INA, LOW);
    digitalWrite(M2_INB, LOW);
    ledcWrite(M2_PWM, 0);
}

void motor2Forward(float duty)
{
    duty = constrain(duty, 0.0f, 1.0f);

    digitalWrite(M2_INA, HIGH);
    digitalWrite(M2_INB, LOW);

    uint16_t pwm = (uint16_t)(duty * PWM_MAX);

    ledcWrite(M2_PWM, pwm);
}

void motor2Reverse(float duty)
{
    duty = constrain(duty, 0.0f, 1.0f);

    digitalWrite(M2_INA, LOW);
    digitalWrite(M2_INB, HIGH);

    uint16_t pwm = (uint16_t)(duty * PWM_MAX);

    ledcWrite(M2_PWM, pwm);
}


// ============================================================
// 両方停止
// ============================================================

void motorsStop()
{
    motor1Stop();
    motor2Stop();
}


// ============================================================
// setup
// ============================================================

void setup()
{
    Serial.begin(115200);

    // --------------------------------------------------------
    // GPIO
    // --------------------------------------------------------

    pinMode(M1_INA, OUTPUT);
    pinMode(M1_INB, OUTPUT);
    pinMode(M1_EN, OUTPUT);

    pinMode(M2_INA, OUTPUT);
    pinMode(M2_INB, OUTPUT);
    pinMode(M2_EN, OUTPUT);

    // 最初は停止
    motorsStop();

    // EN有効
    digitalWrite(M1_EN, HIGH);
    digitalWrite(M2_EN, HIGH);

    // --------------------------------------------------------
    // PWM
    // ESP32 Arduino Core 3.x
    // --------------------------------------------------------

    if (!ledcAttach(
            M1_PWM,
            PWM_FREQ_HZ,
            PWM_RES_BITS))
    {
        Serial.println("ERROR: Motor 1 PWM attach failed!");
        while (true)
        {
            delay(1000);
        }
    }

    if (!ledcAttach(
            M2_PWM,
            PWM_FREQ_HZ,
            PWM_RES_BITS))
    {
        Serial.println("ERROR: Motor 2 PWM attach failed!");
        while (true)
        {
            delay(1000);
        }
    }

    ledcWrite(M1_PWM, 0);
    ledcWrite(M2_PWM, 0);

    delay(1000);

    Serial.println();
    Serial.println("============================");
    Serial.println(" Dual Motor Test Start");
    Serial.println("============================");
    Serial.printf(
        "Duty = %.0f%%\n",
        TEST_DUTY * 100.0f
    );

    motorsStop();
}


// ============================================================
// loop
// ============================================================

void loop()
{
    // ========================================================
    // Motor 1 正転
    // ========================================================

    Serial.println("MOTOR 1 : FORWARD");

    motor1Forward(TEST_DUTY);

    delay(RUN_TIME_MS);

    motor1Stop();

    Serial.println("MOTOR 1 : STOP");

    delay(STOP_TIME_MS);


    // ========================================================
    // Motor 1 逆転
    // ========================================================

    Serial.println("MOTOR 1 : REVERSE");

    motor1Stop();
    delay(REVERSE_DEADTIME_MS);

    motor1Reverse(TEST_DUTY);

    delay(RUN_TIME_MS);

    motor1Stop();

    Serial.println("MOTOR 1 : STOP");

    delay(STOP_TIME_MS);


    // ========================================================
    // Motor 2 正転
    // ========================================================

    Serial.println("MOTOR 2 : FORWARD");

    motor2Forward(TEST_DUTY);

    delay(RUN_TIME_MS);

    motor2Stop();

    Serial.println("MOTOR 2 : STOP");

    delay(STOP_TIME_MS);


    // ========================================================
    // Motor 2 逆転
    // ========================================================

    Serial.println("MOTOR 2 : REVERSE");

    motor2Stop();
    delay(REVERSE_DEADTIME_MS);

    motor2Reverse(TEST_DUTY);

    delay(RUN_TIME_MS);

    motor2Stop();

    Serial.println("MOTOR 2 : STOP");

    delay(STOP_TIME_MS);


    // ========================================================
    // 両方 正転
    // ========================================================

    Serial.println("BOTH : FORWARD");

    motor1Forward(TEST_DUTY);
    motor2Forward(TEST_DUTY);

    delay(RUN_TIME_MS);

    motorsStop();

    Serial.println("BOTH : STOP");

    delay(STOP_TIME_MS);


    // ========================================================
    // 両方 逆転
    // ========================================================

    Serial.println("BOTH : REVERSE");

    motorsStop();
    delay(REVERSE_DEADTIME_MS);

    motor1Reverse(TEST_DUTY);
    motor2Reverse(TEST_DUTY);

    delay(RUN_TIME_MS);

    motorsStop();

    Serial.println("BOTH : STOP");

    delay(STOP_TIME_MS);
}