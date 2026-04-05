#include <Wire.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>

// =================================================================
//  JOINT CONTROL SYSTEM + ROS UART INTEGRATION
//
//  Joints:
//    0 = M3        (2:1 gear, encoder on motor, multi-turn)
//    1 = Pitch     (wrist differential)
//    2 = Roll      (wrist differential)
//    3 = Z-axis    (3:1 gear, encoder on motor, multi-turn, base rotation)
// =================================================================

// ===================== ROS COMMUNICATION STRUCTS =================
struct __attribute__((packed)) CommandPacketB {
    char    header[2];
    int32_t motor_cmd[4];   // [M3, pitch, roll, Z-axis]
    char    footer;
};  // 19 bytes

struct __attribute__((packed)) FeedbackPacketB {
    char    header[2]      = {'F', 'B'};
    int32_t motor_pos[4]   = {0, 0, 0, 0};
    char    footer         = '\n';
};  // 19 bytes

bool tryReadPacket(CommandPacketB& cmd);

// ===================== ROS UART CONFIG ===========================
const int ROS_RX_PIN = 8;
const int ROS_TX_PIN = 9;
HardwareSerial ROSSerial(1);

// ===================== STATUS LED ================================
#define LED_PIN   21
#define LED_COUNT 1
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
unsigned long lastLedUpdate = 0;
const unsigned long LED_UPDATE_MS = 100;

void setLed(uint32_t colour) {
    led.setPixelColor(0, colour);
    led.show();
}

// ===================== WATCHDOG ==================================
const unsigned long COMMS_TIMEOUT_MS = 1000;
unsigned long last_valid_cmd_time    = 0;
bool comms_ok                        = false;

// ================= USER CALIBRATION SECTION ======================
const bool JOINT_ALLOWED[4] = {true, true, true, true};
int HOME_OFFSETS[4] = {430, 418, 1676, 3813};
const int POS_MIN[4] = {0, 0, 0, 0};
const int POS_MAX[4] = {4096, 1837, 4096, 4096};
const int PITCH_LIMIT_MIN = 0;
const int PITCH_LIMIT_MAX = 1837;

// ================= M3 CONFIG =====================================
const float M3_GEAR_RATIO   = 2.0f;
const float M3_MOTOR_LIMIT  = 360.0f;
const float M3_OUTPUT_LIMIT = 180.0f;

// ================= Z-AXIS CONFIG =================================
const float ZA_GEAR_RATIO      = 2.7272f;
const float ZA_MOTOR_LIMIT     = 360.0f;
const float ZA_OUTPUT_LIMIT    = 120.0f;
const float ZA_PID_OUTPUT_MAX  = 127.0f;

// ================= ANGLE CONVERSION ==============================
const float PITCH_RANGE_DEG = (1837.0f / 4096.0f) * 360.0f;
const float ROLL_RANGE_DEG  = 360.0f;

float motorDegToSteps(float motor_deg) { return (motor_deg / 360.0f) * 4096.0f; }
float motorStepsToDeg(float steps)     { return (steps / 4096.0f) * 360.0f; }
float outputToMotorDeg(float output_deg)   { return output_deg * M3_GEAR_RATIO; }
float motorToOutputDeg(float motor_deg)    { return motor_deg / M3_GEAR_RATIO; }
float zaOutputToMotorDeg(float output_deg) { return output_deg * ZA_GEAR_RATIO; }
float zaMotorToOutputDeg(float motor_deg)  { return motor_deg / ZA_GEAR_RATIO; }
float pitchStepsToDeg(float steps) { return (steps / (float)PITCH_LIMIT_MAX) * PITCH_RANGE_DEG; }
float pitchDegToSteps(float deg)   { return (deg / PITCH_RANGE_DEG) * (float)PITCH_LIMIT_MAX; }
float rollStepsToDeg(float steps)  { return (steps / 4096.0f) * ROLL_RANGE_DEG; }
float rollDegToSteps(float deg)    { return (deg / ROLL_RANGE_DEG) * 4096.0f; }

// ================= HARDWARE CONFIGURATION ========================
#define I2C_MUX_ADDR   0x70
#define AS5600_ADDR    0x36
#define ENCODER_COUNTS 4096
#define ENCODER_HALF   2048

const int SDA_PIN = 2;
const int SCL_PIN = 1;
const int PWM_PINS[4]     = {13, 4, 6, 10};
const int DIR_PINS[4]     = {12, 5, 7, 11};
const int MUX_CHANNELS[4] = {5, 0, 1, 2};
const int LEDC_FREQ       = 10000;
const int LEDC_RES        = 8;
const uint32_t I2C_FREQ   = 400000;
bool USE_ROLL_ENCODER = true;

float Kp[4] = {1.0f,  2.0f,  2.0f,  1.0f};
float Ki[4] = {0.05f, 0.02f, 0.02f, 0.05f};
float Kd[4] = {0.01f, 0.05f, 0.05f, 0.01f};

const float PID_OUTPUT_MAX = 255.0f;
const float INTEGRAL_MAX   = 100.0f;
const float ERROR_DEADZONE = 1.0f;
const int   MAX_SENSOR_FAILS = 10;
const float MAX_SLEW_RATE  = 40.0f;

// ================= DIRECTION SIGNS ===============================
int       M3_PID_SIGN         = 1;
int       ZA_PID_SIGN         = -1;
const int MOTOR_DIR_SIGN[4]   = {1, 1, 1, 1};
int       MOTOR_A_ROLL_SIGN   = 1;
int       ROLL_PID_SIGN       = 1;

// ================= STATE VARIABLES ===============================
bool  motor_enabled[4]     = {false};
float target_pos[4]        = {0};
float commanded_pos[4]     = {0};
float current_pos[4]       = {0};
float prev_pos[4]          = {0};
float integral[4]          = {0};
float prev_error[4]        = {0};
float pid_output[4]        = {0};
int   sensor_fail_count[4] = {0};
bool  sensor_read_ok[4]    = {false};

// ===== M3 MULTI-TURN TRACKING ===================================
int   m3_wrap_count = 0;  int m3_prev_raw = 0;
float m3_continuous_pos = 0, m3_continuous_target = 0;
float m3_continuous_cmd = 0, m3_continuous_prev   = 0;
const float M3_CONT_LIMIT = 4096.0f, M3_CONT_SOFT_ZONE = 400.0f;

// ===== Z-AXIS MULTI-TURN TRACKING ===============================
int   za_wrap_count = 0;  int za_prev_raw = 0;
float za_continuous_pos = 0, za_continuous_target = 0;
float za_continuous_cmd = 0, za_continuous_prev   = 0;
const float ZA_CONT_LIMIT = 4096.0f, ZA_CONT_SOFT_ZONE = 400.0f;

unsigned long last_pid_time   = 0;
unsigned long last_debug_time = 0;
bool manual_test_mode         = false;
bool debug_show[4] = {true, true, true, true};

// =================================================================
//                       CORE FUNCTIONS
// =================================================================

void tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(I2C_MUX_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
    delayMicroseconds(5);
}

float wrapValue(float val) {
    while (val < 0) val += ENCODER_COUNTS;
    while (val >= ENCODER_COUNTS) val -= ENCODER_COUNTS;
    return val;
}

float wrapError(float error) {
    if (error > ENCODER_HALF) error -= ENCODER_COUNTS;
    if (error < -ENCODER_HALF) error += ENCODER_COUNTS;
    return error;
}

float applyDeadzone(float error, float deadzone) {
    if (fabsf(error) < deadzone) return 0.0f;
    return (error > 0) ? (error - deadzone) : (error + deadzone);
}

int readEncoderRaw(int joint) {
    tcaSelect(MUX_CHANNELS[joint]);
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0C);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, 2);
    if (Wire.available() >= 2) {
        uint8_t highByte = Wire.read();
        uint8_t lowByte  = Wire.read();
        return (highByte << 8) | lowByte;
    }
    return -1;
}

int readEncoder(int joint) {
    int raw = readEncoderRaw(joint);
    if (raw == -1) return -1;
    int corrected = raw - HOME_OFFSETS[joint];
    while (corrected < 0)    corrected += 4096;
    while (corrected >= 4096) corrected -= 4096;
    return corrected;
}

bool m3UpdatePosition() {
    int raw = readEncoder(0);
    if (raw == -1) return false;
    int delta = raw - m3_prev_raw;
    if (delta >  ENCODER_HALF) m3_wrap_count--;
    else if (delta < -ENCODER_HALF) m3_wrap_count++;
    m3_prev_raw = raw;
    m3_continuous_pos = (float)(m3_wrap_count * ENCODER_COUNTS) + (float)raw;
    current_pos[0] = (float)raw;
    return true;
}

bool zaUpdatePosition() {
    int raw = readEncoder(3);
    if (raw == -1) return false;
    int delta = raw - za_prev_raw;
    if (delta >  ENCODER_HALF) za_wrap_count--;
    else if (delta < -ENCODER_HALF) za_wrap_count++;
    za_prev_raw = raw;
    za_continuous_pos = (float)(za_wrap_count * ENCODER_COUNTS) + (float)raw;
    current_pos[3] = (float)raw;
    return true;
}

float m3OutputDegToContSteps(float output_deg) {
    output_deg = constrain(output_deg, -M3_OUTPUT_LIMIT, M3_OUTPUT_LIMIT);
    return motorDegToSteps(outputToMotorDeg(output_deg));
}
float m3ContStepsToOutputDeg(float cont_steps) {
    return motorToOutputDeg(motorStepsToDeg(cont_steps));
}
float zaOutputDegToContSteps(float output_deg) {
    output_deg = constrain(output_deg, -ZA_OUTPUT_LIMIT, ZA_OUTPUT_LIMIT);
    return motorDegToSteps(zaOutputToMotorDeg(output_deg));
}
float zaContStepsToOutputDeg(float cont_steps) {
    return zaMotorToOutputDeg(motorStepsToDeg(cont_steps));
}

float jointStepsToDeg(int joint, float steps) {
    if (joint == 0) return m3ContStepsToOutputDeg(m3_continuous_pos);
    if (joint == 1) return pitchStepsToDeg(steps);
    if (joint == 2) return rollStepsToDeg(steps);
    if (joint == 3) return zaContStepsToOutputDeg(za_continuous_pos);
    return 0.0f;
}

const char* jointName(int joint) {
    if (joint == 0) return "M3";
    if (joint == 1) return "Pitch";
    if (joint == 2) return "Roll";
    if (joint == 3) return "Z-axis";
    return "Unknown";
}

void i2cReset() {
    Wire.end();
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, OUTPUT);
    for (int i = 0; i < 9; i++) {
        digitalWrite(SCL_PIN, LOW);  delayMicroseconds(5);
        digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
    }
    pinMode(SDA_PIN, OUTPUT);
    digitalWrite(SDA_PIN, LOW);  delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(SDA_PIN, HIGH); delayMicroseconds(5);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQ);
    Serial.println("I2C RESET");
}

bool i2cHealthCheck() {
    Wire.beginTransmission(I2C_MUX_ADDR);
    return (Wire.endTransmission() == 0);
}

void setMotorM3(int pwm_val) {
    if (!motor_enabled[0]) { ledcWrite(PWM_PINS[0], 0); return; }
    pwm_val *= M3_PID_SIGN * MOTOR_DIR_SIGN[0];
    int speed = constrain(abs(pwm_val), 0, 255);
    if (speed < 15) speed = 0;
    digitalWrite(DIR_PINS[0], (pwm_val >= 0) ? HIGH : LOW);
    ledcWrite(PWM_PINS[0], speed);
}

void setMotorZA(int pwm_val) {
    if (!motor_enabled[3]) { ledcWrite(PWM_PINS[3], 0); return; }
    pwm_val *= ZA_PID_SIGN * MOTOR_DIR_SIGN[3];
    int speed = constrain(abs(pwm_val), 0, 255);
    if (speed < 15) speed = 0;
    digitalWrite(DIR_PINS[3], (pwm_val >= 0) ? HIGH : LOW);
    ledcWrite(PWM_PINS[3], speed);
}

void driveMotorRaw(int pin_index, int pwm_signed) {
    int speed = constrain(abs(pwm_signed), 0, 255);
    digitalWrite(DIR_PINS[pin_index], (pwm_signed >= 0) ? HIGH : LOW);
    ledcWrite(PWM_PINS[pin_index], speed);
}

void stopAllMotorsRaw() {
    for (int i = 0; i < 4; i++) ledcWrite(PWM_PINS[i], 0);
}

void emergencyStopAll() {
    manual_test_mode = false;
    for (int i = 0; i < 4; i++) {
        motor_enabled[i] = false;
        ledcWrite(PWM_PINS[i], 0);
        integral[i] = 0;
    }
    Serial.println("!!! E-STOP !!!");
}
// =================================================================
//                      SENSOR READING
// =================================================================

void readAllSensors() {
    static int i2c_fail_count = 0;
    if (!i2cHealthCheck()) {
        if (++i2c_fail_count > 3) { i2cReset(); i2c_fail_count = 0; }
        return;
    }
    i2c_fail_count = 0;

    sensor_read_ok[0] = false;
    if (motor_enabled[0]) {
        if (m3UpdatePosition()) { sensor_fail_count[0] = 0; sensor_read_ok[0] = true; }
        else { if (++sensor_fail_count[0] > MAX_SENSOR_FAILS) emergencyStopAll(); }
    }

    for (int i = 1; i < 3; i++) {
        sensor_read_ok[i] = false;
        if (!motor_enabled[i]) continue;
        if (i == 2 && !USE_ROLL_ENCODER) { sensor_read_ok[i] = true; continue; }
        int raw = readEncoder(i);
        if (raw != -1) { current_pos[i] = raw; sensor_fail_count[i] = 0; sensor_read_ok[i] = true; }
        else { if (++sensor_fail_count[i] > MAX_SENSOR_FAILS) emergencyStopAll(); }
    }

    sensor_read_ok[3] = false;
    if (motor_enabled[3]) {
        if (zaUpdatePosition()) { sensor_fail_count[3] = 0; sensor_read_ok[3] = true; }
        else { if (++sensor_fail_count[3] > MAX_SENSOR_FAILS) emergencyStopAll(); }
    }
}

// =================================================================
//                    TRAJECTORY
// =================================================================

void updateTrajectory() {
    if (motor_enabled[0]) {
        float e = m3_continuous_target - m3_continuous_cmd;
        if (fabsf(e) > MAX_SLEW_RATE) m3_continuous_cmd += (e > 0) ? MAX_SLEW_RATE : -MAX_SLEW_RATE;
        else m3_continuous_cmd = m3_continuous_target;
        m3_continuous_cmd = constrain(m3_continuous_cmd, -M3_CONT_LIMIT, M3_CONT_LIMIT);
    } else { m3_continuous_cmd = m3_continuous_pos; }

    for (int i = 1; i < 3; i++) {
        if (!motor_enabled[i]) { commanded_pos[i] = current_pos[i]; continue; }
        float e = wrapError(target_pos[i] - commanded_pos[i]);
        if (fabsf(e) > MAX_SLEW_RATE) {
            commanded_pos[i] += (e > 0) ? MAX_SLEW_RATE : -MAX_SLEW_RATE;
            commanded_pos[i] = wrapValue(commanded_pos[i]);
        } else { commanded_pos[i] = target_pos[i]; }
    }

    if (motor_enabled[3]) {
        float e = za_continuous_target - za_continuous_cmd;
        if (fabsf(e) > MAX_SLEW_RATE) za_continuous_cmd += (e > 0) ? MAX_SLEW_RATE : -MAX_SLEW_RATE;
        else za_continuous_cmd = za_continuous_target;
        za_continuous_cmd = constrain(za_continuous_cmd, -ZA_CONT_LIMIT, ZA_CONT_LIMIT);
    } else { za_continuous_cmd = za_continuous_pos; }
}

// =================================================================
//                         PID
// =================================================================

void computePID(float dt) {
    // === M3 ===
    if (motor_enabled[0]) {
        float error = m3_continuous_cmd - m3_continuous_pos;
        float error_dz = applyDeadzone(error, ERROR_DEADZONE);
        float p_term = Kp[0] * error_dz;
        if (fabsf(pid_output[0]) < PID_OUTPUT_MAX) {
            integral[0] += error_dz * dt;
            integral[0] = constrain(integral[0], -INTEGRAL_MAX, INTEGRAL_MAX);
        }
        if (fabsf(error_dz) < 0.1f) integral[0] = 0;
        float i_term = Ki[0] * integral[0];
        float d_term = 0.0f;
        if (sensor_read_ok[0]) {
            d_term = Kd[0] * ((m3_continuous_pos - m3_continuous_prev) / dt);
            m3_continuous_prev = m3_continuous_pos;
        }
        pid_output[0] = constrain(p_term + i_term - d_term, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);

        float pos = m3_continuous_pos;
        if (pos >= M3_CONT_LIMIT && pid_output[0] > 0) { pid_output[0] = 0; integral[0] = 0; }
        if (pos <= -M3_CONT_LIMIT && pid_output[0] < 0) { pid_output[0] = 0; integral[0] = 0; }
        if (pos > (M3_CONT_LIMIT - M3_CONT_SOFT_ZONE) && pos < M3_CONT_LIMIT && pid_output[0] > 0)
            pid_output[0] *= constrain((M3_CONT_LIMIT - pos) / M3_CONT_SOFT_ZONE, 0.0f, 1.0f);
        if (pos < (-M3_CONT_LIMIT + M3_CONT_SOFT_ZONE) && pos > -M3_CONT_LIMIT && pid_output[0] < 0)
            pid_output[0] *= constrain((pos + M3_CONT_LIMIT) / M3_CONT_SOFT_ZONE, 0.0f, 1.0f);
    } else { pid_output[0] = 0; integral[0] = 0; }

    // === Pitch & Roll ===
    for (int i = 1; i < 3; i++) {
        if (!motor_enabled[i]) { pid_output[i] = 0; integral[i] = 0; continue; }
        float error = wrapError(commanded_pos[i] - current_pos[i]);
        float error_dz = applyDeadzone(error, ERROR_DEADZONE);
        float p_term = Kp[i] * error_dz;
        if (fabsf(pid_output[i]) < PID_OUTPUT_MAX) {
            integral[i] += error_dz * dt;
            integral[i] = constrain(integral[i], -INTEGRAL_MAX, INTEGRAL_MAX);
        }
        if (fabsf(error_dz) < 0.1f) integral[i] = 0;
        float i_term = Ki[i] * integral[i];
        float d_term = 0.0f;
        if (sensor_read_ok[i]) {
            d_term = Kd[i] * (wrapError(current_pos[i] - prev_pos[i]) / dt);
            prev_pos[i] = current_pos[i];
        }
        pid_output[i] = constrain(p_term + i_term - d_term, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
    }

    // === Pitch Safety ===
    if (motor_enabled[1]) {
        float pos = current_pos[1];
        const float WRAP_BUFFER = 200.0f;
        bool in_valid = (pos <= (float)PITCH_LIMIT_MAX);
        bool in_wrap  = (pos >= (4096.0f - WRAP_BUFFER));
        bool forbidden = (!in_valid && !in_wrap);
        if (forbidden || (pos >= (float)PITCH_LIMIT_MAX && pos <= (float)ENCODER_HALF)) {
            if (pid_output[1] > 0) { pid_output[1] = 0; integral[1] = 0; }
        }
        if (pos <= (float)PITCH_LIMIT_MIN && pos < WRAP_BUFFER) {
            if (pid_output[1] < 0) { pid_output[1] = 0; integral[1] = 0; }
        }
    }

    // === Z-axis (50% speed cap) ===
    if (motor_enabled[3]) {
        float error = za_continuous_cmd - za_continuous_pos;
        float error_dz = applyDeadzone(error, ERROR_DEADZONE);
        float p_term = Kp[3] * error_dz;
        if (fabsf(pid_output[3]) < ZA_PID_OUTPUT_MAX) {
            integral[3] += error_dz * dt;
            integral[3] = constrain(integral[3], -INTEGRAL_MAX, INTEGRAL_MAX);
        }
        if (fabsf(error_dz) < 0.1f) integral[3] = 0;
        float i_term = Ki[3] * integral[3];
        float d_term = 0.0f;
        if (sensor_read_ok[3]) {
            d_term = Kd[3] * ((za_continuous_pos - za_continuous_prev) / dt);
            za_continuous_prev = za_continuous_pos;
        }
        pid_output[3] = constrain(p_term + i_term - d_term, -ZA_PID_OUTPUT_MAX, ZA_PID_OUTPUT_MAX);

        float pos = za_continuous_pos;
        if (pos >= ZA_CONT_LIMIT && pid_output[3] > 0) { pid_output[3] = 0; integral[3] = 0; }
        if (pos <= -ZA_CONT_LIMIT && pid_output[3] < 0) { pid_output[3] = 0; integral[3] = 0; }
        if (pos > (ZA_CONT_LIMIT - ZA_CONT_SOFT_ZONE) && pos < ZA_CONT_LIMIT && pid_output[3] > 0)
            pid_output[3] *= constrain((ZA_CONT_LIMIT - pos) / ZA_CONT_SOFT_ZONE, 0.0f, 1.0f);
        if (pos < (-ZA_CONT_LIMIT + ZA_CONT_SOFT_ZONE) && pos > -ZA_CONT_LIMIT && pid_output[3] < 0)
            pid_output[3] *= constrain((pos + ZA_CONT_LIMIT) / ZA_CONT_SOFT_ZONE, 0.0f, 1.0f);
    } else { pid_output[3] = 0; integral[3] = 0; }
}

// =================================================================
//                    MOTOR OUTPUT
// =================================================================

void applyMotorOutputs() {
    if (manual_test_mode) return;

    if (motor_enabled[0]) setMotorM3((int)pid_output[0]);
    else ledcWrite(PWM_PINS[0], 0);

    bool pitch_active = motor_enabled[1] && sensor_read_ok[1];
    bool roll_active  = motor_enabled[2] && sensor_read_ok[2] && USE_ROLL_ENCODER;
    if (pitch_active || roll_active) {
        float pitch_cmd = pitch_active ? pid_output[1] : 0.0f;
        float roll_cmd  = roll_active  ? (pid_output[2] * ROLL_PID_SIGN) : 0.0f;
        float mA = pitch_cmd + roll_cmd;
        float mB = -pitch_cmd + roll_cmd;
        float max_val = max(fabsf(mA), fabsf(mB));
        if (max_val > 255.0f) { float s = 255.0f / max_val; mA *= s; mB *= s; }

        int pwmA = (int)mA * MOTOR_DIR_SIGN[1];
        int speedA = constrain(abs(pwmA), 0, 255);
        digitalWrite(DIR_PINS[1], (pwmA >= 0) ? HIGH : LOW);
        ledcWrite(PWM_PINS[1], (speedA < 15) ? 0 : speedA);

        int pwmB = (int)mB * MOTOR_DIR_SIGN[2];
        int speedB = constrain(abs(pwmB), 0, 255);
        digitalWrite(DIR_PINS[2], (pwmB >= 0) ? HIGH : LOW);
        ledcWrite(PWM_PINS[2], (speedB < 15) ? 0 : speedB);
    } else { ledcWrite(PWM_PINS[1], 0); ledcWrite(PWM_PINS[2], 0); }

    if (motor_enabled[3]) setMotorZA((int)pid_output[3]);
    else ledcWrite(PWM_PINS[3], 0);
}

// =================================================================
//                        INIT HELPERS
// =================================================================

void m3Init(int encoder_val) {
    m3_prev_raw = encoder_val; m3_wrap_count = 0;
    m3_continuous_pos = (float)encoder_val;
    m3_continuous_target = m3_continuous_pos;
    m3_continuous_cmd = m3_continuous_pos;
    m3_continuous_prev = m3_continuous_pos;
}

void zaInit(int encoder_val) {
    za_prev_raw = encoder_val; za_wrap_count = 0;
    za_continuous_pos = (float)encoder_val;
    za_continuous_target = za_continuous_pos;
    za_continuous_cmd = za_continuous_pos;
    za_continuous_prev = za_continuous_pos;
}

// =================================================================
//            ROS UART — PACKET READER & COMMAND PROCESSING
// =================================================================

bool tryReadPacket(CommandPacketB& cmd) {
    while (ROSSerial.available() >= (int)sizeof(CommandPacketB)) {
        if (ROSSerial.peek() != 'S') { ROSSerial.read(); continue; }
        uint8_t buf[sizeof(CommandPacketB)];
        ROSSerial.readBytes((char*)buf, sizeof(CommandPacketB));
        memcpy(&cmd, buf, sizeof(CommandPacketB));
        if (cmd.header[0] == 'S' && cmd.header[1] == 'T' && cmd.footer == '\n') return true;
        Serial.println("ROS: bad packet — re-syncing (no flush).");
    }
    return false;
}

void processROSCommand() {
    CommandPacketB cmd;
    bool got_any = false;
    while (tryReadPacket(cmd)) got_any = true;
    if (!got_any) return;

    last_valid_cmd_time = millis();

    if (!comms_ok) {
        comms_ok = true;
        Serial.println("ROS: connected — motors re-enabled.");
        for (int i = 0; i < 4; i++) {
            if (!JOINT_ALLOWED[i]) continue;
            int val = readEncoder(i);
            if (val != -1) {
                motor_enabled[i] = true; current_pos[i] = val;
                commanded_pos[i] = val; prev_pos[i] = val; integral[i] = 0;
                if (i == 0) { m3UpdatePosition(); m3_continuous_cmd = m3_continuous_pos; m3_continuous_prev = m3_continuous_pos; }
                if (i == 3) { zaUpdatePosition(); za_continuous_cmd = za_continuous_pos; za_continuous_prev = za_continuous_pos; }
            }
        }
    }

    if (JOINT_ALLOWED[0]) m3_continuous_target = constrain((float)cmd.motor_cmd[0], -M3_CONT_LIMIT, M3_CONT_LIMIT);
    if (JOINT_ALLOWED[1]) target_pos[1] = constrain((float)cmd.motor_cmd[1], (float)POS_MIN[1], (float)POS_MAX[1]);
    if (JOINT_ALLOWED[2]) target_pos[2] = wrapValue((float)cmd.motor_cmd[2]);
    if (JOINT_ALLOWED[3]) za_continuous_target = constrain((float)cmd.motor_cmd[3], -ZA_CONT_LIMIT, ZA_CONT_LIMIT);

    FeedbackPacketB fb;
    fb.motor_pos[0] = (int32_t)m3_continuous_pos;
    fb.motor_pos[1] = (int32_t)current_pos[1];
    fb.motor_pos[2] = (int32_t)current_pos[2];
    fb.motor_pos[3] = (int32_t)za_continuous_pos;
    ROSSerial.write((uint8_t*)&fb, sizeof(FeedbackPacketB));
}

void checkCommsWatchdog() {
    if (!comms_ok) return;
    if ((millis() - last_valid_cmd_time) > COMMS_TIMEOUT_MS) {
        comms_ok = false;
        for (int i = 0; i < 4; i++) { motor_enabled[i] = false; ledcWrite(PWM_PINS[i], 0); integral[i] = 0; }
        Serial.println("ROS WATCHDOG: comms lost — motors stopped. USB serial still active.");
    }
}

void updateLed() {
    if (millis() - lastLedUpdate < LED_UPDATE_MS) return;
    lastLedUpdate = millis();
    if (!comms_ok) { setLed(led.Color(180, 0, 180)); return; }
    bool moving = false;
    for (int i = 0; i < 4; i++) { if (motor_enabled[i] && fabsf(pid_output[i]) > 15.0f) { moving = true; break; } }
    setLed(moving ? led.Color(0, 0, 255) : led.Color(0, 180, 0));
}
// =================================================================
//          USB SERIAL COMMAND INTERFACE
// =================================================================

void processSerialCommand() {
    if (!Serial.available()) return;
    char type = Serial.peek();

    if (type >= '0' && type <= '9') {
        int id = Serial.parseInt();
        float deg = Serial.parseFloat();
        if (id >= 0 && id < 4 && JOINT_ALLOWED[id]) {
            if (id == 0) {
                deg = constrain(deg, -M3_OUTPUT_LIMIT, M3_OUTPUT_LIMIT);
                m3_continuous_target = m3OutputDegToContSteps(deg);
                m3_continuous_target = constrain(m3_continuous_target, -M3_CONT_LIMIT, M3_CONT_LIMIT);
                Serial.printf("CMD M3: -> %.1f shaft deg (cont step %.0f)\n", deg, m3_continuous_target);
            } else if (id == 1) {
                deg = constrain(deg, 0.0f, PITCH_RANGE_DEG);
                target_pos[1] = pitchDegToSteps(deg);
                Serial.printf("CMD Pitch: -> %.1f deg\n", deg);
            } else if (id == 2) {
                target_pos[2] = wrapValue(rollDegToSteps(deg));
                Serial.printf("CMD Roll: -> %.1f deg\n", deg);
            } else if (id == 3) {
                deg = constrain(deg, -ZA_OUTPUT_LIMIT, ZA_OUTPUT_LIMIT);
                za_continuous_target = zaOutputDegToContSteps(deg);
                za_continuous_target = constrain(za_continuous_target, -ZA_CONT_LIMIT, ZA_CONT_LIMIT);
                Serial.printf("CMD Z-axis: -> %.1f shaft deg (cont step %.0f)\n", deg, za_continuous_target);
            }
        } else { Serial.printf("J%d not allowed\n", id); }
    } else {
        char cmd = Serial.read();
        switch (cmd) {
            case 'A': case 'a': {
                float m3_deg = Serial.parseFloat();
                float p_deg  = Serial.parseFloat();
                float r_deg  = Serial.parseFloat();
                float z_deg  = Serial.parseFloat();
                if (JOINT_ALLOWED[0]) {
                    m3_deg = constrain(m3_deg, -M3_OUTPUT_LIMIT, M3_OUTPUT_LIMIT);
                    m3_continuous_target = constrain(m3OutputDegToContSteps(m3_deg), -M3_CONT_LIMIT, M3_CONT_LIMIT);
                }
                if (JOINT_ALLOWED[1]) { p_deg = constrain(p_deg, 0.0f, PITCH_RANGE_DEG); target_pos[1] = pitchDegToSteps(p_deg); }
                if (JOINT_ALLOWED[2]) { target_pos[2] = wrapValue(rollDegToSteps(r_deg)); }
                if (JOINT_ALLOWED[3]) {
                    z_deg = constrain(z_deg, -ZA_OUTPUT_LIMIT, ZA_OUTPUT_LIMIT);
                    za_continuous_target = constrain(zaOutputDegToContSteps(z_deg), -ZA_CONT_LIMIT, ZA_CONT_LIMIT);
                }
                Serial.printf("CMD ALL -> M3:%.1f Pitch:%.1f Roll:%.1f Z:%.1f\n", m3_deg, p_deg, r_deg, z_deg);
                break;
            }
            case 'M': case 'm': {
                float shaft_deg = Serial.parseFloat();
                if (JOINT_ALLOWED[0]) {
                    shaft_deg = constrain(shaft_deg, -M3_OUTPUT_LIMIT, M3_OUTPUT_LIMIT);
                    m3_continuous_target = constrain(m3OutputDegToContSteps(shaft_deg), -M3_CONT_LIMIT, M3_CONT_LIMIT);
                    Serial.printf("M3 -> %.1f shaft deg (cont step %.0f, wraps=%d)\n", shaft_deg, m3_continuous_target, m3_wrap_count);
                }
                break;
            }
            case 'Z': case 'z': {
                float shaft_deg = Serial.parseFloat();
                if (JOINT_ALLOWED[3]) {
                    shaft_deg = constrain(shaft_deg, -ZA_OUTPUT_LIMIT, ZA_OUTPUT_LIMIT);
                    za_continuous_target = constrain(zaOutputDegToContSteps(shaft_deg), -ZA_CONT_LIMIT, ZA_CONT_LIMIT);
                    Serial.printf("Z-axis -> %.1f shaft deg (cont step %.0f, wraps=%d)\n", shaft_deg, za_continuous_target, za_wrap_count);
                }
                break;
            }
            case 'W': case 'w': {
                float p_deg = Serial.parseFloat();
                float r_deg = Serial.parseFloat();
                if (JOINT_ALLOWED[1]) { p_deg = constrain(p_deg, 0.0f, PITCH_RANGE_DEG); target_pos[1] = pitchDegToSteps(p_deg); }
                if (JOINT_ALLOWED[2]) { target_pos[2] = wrapValue(rollDegToSteps(r_deg)); }
                Serial.printf("Wrist -> P:%.1f R:%.1f deg\n", p_deg, r_deg);
                break;
            }
            case 'P': case 'p': { int id = Serial.parseInt(); float val = Serial.parseFloat(); if (id >= 0 && id < 4) { Kp[id] = val; Serial.printf("Kp[%s] = %.2f\n", jointName(id), val); } break; }
            case 'I': case 'i': { int id = Serial.parseInt(); float val = Serial.parseFloat(); if (id >= 0 && id < 4) { Ki[id] = val; Serial.printf("Ki[%s] = %.3f\n", jointName(id), val); } break; }
            case 'D': case 'd': { int id = Serial.parseInt(); float val = Serial.parseFloat(); if (id >= 0 && id < 4) { Kd[id] = val; Serial.printf("Kd[%s] = %.3f\n", jointName(id), val); } break; }

            case 'E': case 'e': {
                Serial.println("--- ENCODERS ---");
                for (int i = 0; i < 4; i++) {
                    int val = readEncoder(i);
                    if (val != -1) {
                        if (i == 0) Serial.printf("  M3: enc=%d cont=%.0f wraps=%d shaft=%.1f deg\n", val, m3_continuous_pos, m3_wrap_count, m3ContStepsToOutputDeg(m3_continuous_pos));
                        else if (i == 3) Serial.printf("  Z-axis: enc=%d cont=%.0f wraps=%d shaft=%.1f deg\n", val, za_continuous_pos, za_wrap_count, zaContStepsToOutputDeg(za_continuous_pos));
                        else { float deg = (i == 1) ? pitchStepsToDeg((float)val) : rollStepsToDeg((float)val); Serial.printf("  %s: step=%d deg=%.1f %s\n", jointName(i), val, deg, JOINT_ALLOWED[i] ? "ON" : "off"); }
                    } else { Serial.printf("  %s: READ FAIL\n", jointName(i)); }
                }
                break;
            }

            case 'T': case 't': {
                char sub = Serial.read();
                while (sub == ' ' || sub == '\n' || sub == '\r') sub = Serial.read();

                if (sub == 'A' || sub == 'a' || sub == 'B' || sub == 'b' || sub == 'C' || sub == 'c' || sub == 'Z' || sub == 'z') {
                    int pwm = Serial.parseInt();
                    int pin_idx = (sub == 'C' || sub == 'c') ? 0 : (sub == 'A' || sub == 'a') ? 1 : (sub == 'B' || sub == 'b') ? 2 : 3;
                    const char* name = (pin_idx == 0) ? "M3" : (pin_idx == 1) ? "Wrist A" : (pin_idx == 2) ? "Wrist B" : "Z-axis";
                    manual_test_mode = true;
                    stopAllMotorsRaw();
                    for (int j = 0; j < 4; j++) { motor_enabled[j] = false; integral[j] = 0; }

                    int before[4];
                    float m3_cont_before = m3_continuous_pos;
                    float za_cont_before = za_continuous_pos;
                    Serial.printf("TEST: %s (pin %d) PWM=%d for 1s\n", name, PWM_PINS[pin_idx], pwm);
                    Serial.println("BEFORE:");
                    for (int j = 0; j < 4; j++) {
                        before[j] = readEncoder(j);
                        if (before[j] != -1) {
                            if (j == 0) Serial.printf("  M3: enc=%d cont=%.0f shaft=%.1f deg\n", before[j], m3_continuous_pos, m3ContStepsToOutputDeg(m3_continuous_pos));
                            else if (j == 3) Serial.printf("  Z-axis: enc=%d cont=%.0f shaft=%.1f deg\n", before[j], za_continuous_pos, zaContStepsToOutputDeg(za_continuous_pos));
                            else Serial.printf("  %s: %.1f deg (step %d)\n", jointName(j), jointStepsToDeg(j, (float)before[j]), before[j]);
                        }
                    }

                    driveMotorRaw(pin_idx, pwm);
                    unsigned long test_start = millis();
                    while (millis() - test_start < 1000) {
                        if (pin_idx == 0) m3UpdatePosition();
                        if (pin_idx == 3) zaUpdatePosition();
                        delay(10);
                    }
                    stopAllMotorsRaw();

                    Serial.println("AFTER:");
                    for (int j = 0; j < 4; j++) {
                        int after = readEncoder(j);
                        if (after != -1) {
                            if (j == 0) { m3UpdatePosition(); Serial.printf("  M3: enc=%d cont=%.0f shaft=%.1f deg (delta=%+.0f)\n", after, m3_continuous_pos, m3ContStepsToOutputDeg(m3_continuous_pos), m3_continuous_pos - m3_cont_before); }
                            else if (j == 3) { zaUpdatePosition(); Serial.printf("  Z-axis: enc=%d cont=%.0f shaft=%.1f deg (delta=%+.0f)\n", after, za_continuous_pos, zaContStepsToOutputDeg(za_continuous_pos), za_continuous_pos - za_cont_before); }
                            else Serial.printf("  %s: %.1f deg (step %d) delta=%+.0f\n", jointName(j), jointStepsToDeg(j, (float)after), after, wrapError((float)after - (float)before[j]));
                        }
                    }
                    if (pin_idx == 0) {
                        float cd = m3_continuous_pos - m3_cont_before;
                        Serial.println("--- M3 DIRECTION CHECK ---");
                        if (pwm > 0 && cd > 0) Serial.println("  +PWM = +cont: M3_PID_SIGN should be +1");
                        if (pwm > 0 && cd < 0) Serial.println("  +PWM = -cont: M3_PID_SIGN should be -1 >>> F 3");
                        if (pwm < 0 && cd < 0) Serial.println("  -PWM = -cont: M3_PID_SIGN should be +1");
                        if (pwm < 0 && cd > 0) Serial.println("  -PWM = +cont: M3_PID_SIGN should be -1 >>> F 3");
                        Serial.printf("  Current M3_PID_SIGN = %d  wraps = %d\n", M3_PID_SIGN, m3_wrap_count);
                    }
                    if (pin_idx == 3) {
                        float cd = za_continuous_pos - za_cont_before;
                        Serial.println("--- Z-AXIS DIRECTION CHECK ---");
                        if (pwm > 0 && cd > 0) Serial.println("  +PWM = +cont: ZA_PID_SIGN should be +1");
                        if (pwm > 0 && cd < 0) Serial.println("  +PWM = -cont: ZA_PID_SIGN should be -1 >>> F Z");
                        if (pwm < 0 && cd < 0) Serial.println("  -PWM = -cont: ZA_PID_SIGN should be +1");
                        if (pwm < 0 && cd > 0) Serial.println("  -PWM = +cont: ZA_PID_SIGN should be -1 >>> F Z");
                        Serial.printf("  Current ZA_PID_SIGN = %d  wraps = %d\n", ZA_PID_SIGN, za_wrap_count);
                    }
                }
                else if (sub == '0') {
                    stopAllMotorsRaw();
                    Serial.println("Motors stopped");
                    for (int j = 0; j < 4; j++) {
                        int val = readEncoder(j);
                        if (val != -1) {
                            if (j == 0) Serial.printf("  M3: %.1f deg\n", m3ContStepsToOutputDeg(m3_continuous_pos));
                            else if (j == 3) Serial.printf("  Z-axis: %.1f deg\n", zaContStepsToOutputDeg(za_continuous_pos));
                            else Serial.printf("  %s: %.1f deg\n", jointName(j), jointStepsToDeg(j, (float)val));
                        }
                    }
                }
                else if (sub == 'X' || sub == 'x') {
                    manual_test_mode = false; stopAllMotorsRaw();
                    Serial.println("TEST MODE OFF — send R to re-enable PID");
                }
                else {
                    manual_test_mode = true; stopAllMotorsRaw();
                    for (int j = 0; j < 4; j++) { motor_enabled[j] = false; integral[j] = 0; }
                    Serial.println("=== MANUAL TEST MODE ===");
                    Serial.println("  T C [pwm] - Motor 3 for 1s (tracks wraps!)");
                    Serial.println("  T A [pwm] - Wrist motor A for 1s");
                    Serial.println("  T B [pwm] - Wrist motor B for 1s");
                    Serial.println("  T Z [pwm] - Z-axis motor for 1s (tracks wraps!)");
                    Serial.println("  T 0       - Stop all, read encoders");
                    Serial.println("  T X       - Exit test mode");
                    Serial.printf("  M3: cont=%.0f wraps=%d shaft=%.1f deg\n", m3_continuous_pos, m3_wrap_count, m3ContStepsToOutputDeg(m3_continuous_pos));
                    Serial.printf("  Z-axis: cont=%.0f wraps=%d shaft=%.1f deg\n", za_continuous_pos, za_wrap_count, zaContStepsToOutputDeg(za_continuous_pos));
                }
                break;
            }

            case 'F': case 'f': {
                char which = Serial.read();
                while (which == ' ') which = Serial.read();
                if (which == '3') { M3_PID_SIGN *= -1; Serial.printf("M3_PID_SIGN -> %d\n", M3_PID_SIGN); }
                else if (which == 'Z' || which == 'z') { ZA_PID_SIGN *= -1; Serial.printf("ZA_PID_SIGN -> %d\n", ZA_PID_SIGN); }
                else if (which == 'R' || which == 'r') { ROLL_PID_SIGN *= -1; Serial.printf("ROLL_PID_SIGN -> %d\n", ROLL_PID_SIGN); }
                else if (which == 'M' || which == 'm') { MOTOR_A_ROLL_SIGN *= -1; Serial.printf("MOTOR_A_ROLL_SIGN -> %d\n", MOTOR_A_ROLL_SIGN); }
                else { Serial.printf("Signs: M3_PID=%d ZA_PID=%d ROLL_PID=%d ROLL_MIX=%d\n", M3_PID_SIGN, ZA_PID_SIGN, ROLL_PID_SIGN, MOTOR_A_ROLL_SIGN); }
                break;
            }

            case 'V': case 'v': {
                int id = Serial.parseInt();
                if (id >= 0 && id < 4) { debug_show[id] = !debug_show[id]; Serial.printf("%s debug: %s\n", jointName(id), debug_show[id] ? "ON" : "OFF"); }
                else { Serial.printf("Visible: M3=%d Pitch=%d Roll=%d Z=%d\n", debug_show[0], debug_show[1], debug_show[2], debug_show[3]); }
                break;
            }

            case 'S': case 's': emergencyStopAll(); break;

            case 'R': case 'r':
                manual_test_mode = false;
                Serial.println("Re-enabling all joints...");
                for (int j = 0; j < 4; j++) {
                    if (!JOINT_ALLOWED[j]) { motor_enabled[j] = false; Serial.printf("  %s: DISABLED\n", jointName(j)); continue; }
                    int val = readEncoder(j);
                    if (val != -1) {
                        motor_enabled[j] = true; current_pos[j] = val; target_pos[j] = val;
                        commanded_pos[j] = val; prev_pos[j] = val; integral[j] = 0;
                        if (j == 0) {
                            m3UpdatePosition();
                            m3_continuous_target = m3_continuous_pos; m3_continuous_cmd = m3_continuous_pos; m3_continuous_prev = m3_continuous_pos;
                            Serial.printf("  M3: ON at shaft %.1f deg (cont=%.0f wraps=%d)\n", m3ContStepsToOutputDeg(m3_continuous_pos), m3_continuous_pos, m3_wrap_count);
                        } else if (j == 3) {
                            zaUpdatePosition();
                            za_continuous_target = za_continuous_pos; za_continuous_cmd = za_continuous_pos; za_continuous_prev = za_continuous_pos;
                            Serial.printf("  Z-axis: ON at shaft %.1f deg (cont=%.0f wraps=%d)\n", zaContStepsToOutputDeg(za_continuous_pos), za_continuous_pos, za_wrap_count);
                        } else {
                            float deg = (j == 1) ? pitchStepsToDeg((float)val) : rollStepsToDeg((float)val);
                            Serial.printf("  %s: ON at %.1f deg (step %d)\n", jointName(j), deg, val);
                        }
                    } else { Serial.printf("  %s: ENCODER FAIL\n", jointName(j)); }
                }
                break;

            case '?': {
                Serial.println("========== STATUS ==========");
                float m3_out = m3ContStepsToOutputDeg(m3_continuous_pos);
                float m3_tgt = m3ContStepsToOutputDeg(m3_continuous_target);
                float m3_err = m3_continuous_cmd - m3_continuous_pos;
                Serial.printf("  M3:      cur=%+.1f tgt=%+.1f pid=%.0f en=%d\n", m3_out, m3_tgt, pid_output[0], motor_enabled[0]);
                Serial.printf("           cont: pos=%.0f tgt=%.0f cmd=%.0f err=%.0f\n", m3_continuous_pos, m3_continuous_target, m3_continuous_cmd, m3_err);
                Serial.printf("           enc=%d wraps=%d limit=±%.0f\n", m3_prev_raw, m3_wrap_count, M3_CONT_LIMIT);
                Serial.printf("           Kp=%.2f Ki=%.3f Kd=%.3f\n", Kp[0], Ki[0], Kd[0]);
                for (int j = 1; j < 3; j++) {
                    float cur = jointStepsToDeg(j, current_pos[j]);
                    float tgt = jointStepsToDeg(j, target_pos[j]);
                    float err = wrapError(commanded_pos[j] - current_pos[j]);
                    Serial.printf("  %s: cur=%+.1f tgt=%+.1f pid=%.0f en=%d err=%.0f\n", jointName(j), cur, tgt, pid_output[j], motor_enabled[j], err);
                    Serial.printf("           Kp=%.2f Ki=%.3f Kd=%.3f\n", Kp[j], Ki[j], Kd[j]);
                }
                float za_out = zaContStepsToOutputDeg(za_continuous_pos);
                float za_tgt = zaContStepsToOutputDeg(za_continuous_target);
                float za_err = za_continuous_cmd - za_continuous_pos;
                Serial.printf("  Z-axis:  cur=%+.1f tgt=%+.1f pid=%.0f en=%d\n", za_out, za_tgt, pid_output[3], motor_enabled[3]);
                Serial.printf("           cont: pos=%.0f tgt=%.0f cmd=%.0f err=%.0f\n", za_continuous_pos, za_continuous_target, za_continuous_cmd, za_err);
                Serial.printf("           enc=%d wraps=%d limit=±%.0f  maxPWM=%.0f\n", za_prev_raw, za_wrap_count, ZA_CONT_LIMIT, ZA_PID_OUTPUT_MAX);
                Serial.printf("           Kp=%.2f Ki=%.3f Kd=%.3f\n", Kp[3], Ki[3], Kd[3]);
                Serial.printf("  Signs: M3_PID=%d ZA_PID=%d ROLL_PID=%d ROLL_MIX=%d DIR=[%d,%d,%d,%d]\n",
                    M3_PID_SIGN, ZA_PID_SIGN, ROLL_PID_SIGN, MOTOR_A_ROLL_SIGN,
                    MOTOR_DIR_SIGN[0], MOTOR_DIR_SIGN[1], MOTOR_DIR_SIGN[2], MOTOR_DIR_SIGN[3]);
                Serial.printf("  Mode: %s | ROS: %s\n", manual_test_mode ? "MANUAL" : "PID", comms_ok ? "CONNECTED" : "DISCONNECTED");
                Serial.println("============================");
                break;
            }

            case 'H': case 'h':
                Serial.println("========= COMMANDS =========");
                Serial.println("  A [m3] [pitch] [roll] [z] - Move ALL");
                Serial.println("  0 [deg]          - Move M3 (-180 to +180 shaft)");
                Serial.println("  1 [deg]          - Move Pitch");
                Serial.println("  2 [deg]          - Move Roll");
                Serial.println("  3 [deg]          - Move Z-axis (-120 to +120 shaft)");
                Serial.println("  M [deg]          - Move M3 shaft");
                Serial.println("  Z [deg]          - Move Z-axis shaft");
                Serial.println("  W [pitch] [roll] - Move wrist");
                Serial.println("  P/I/D [id] [val] - Tune PID (0=M3 1=Pitch 2=Roll 3=Z)");
                Serial.println("  T                - Raw motor test mode");
                Serial.println("  T C/A/B/Z [pwm]  - Raw drive M3/WristA/WristB/Z-axis");
                Serial.println("  F 3 / F Z / F R / F M - Flip M3/Z/roll/mixer signs");
                Serial.println("  V [id]           - Toggle debug for joint");
                Serial.println("  E - Encoders | S - Stop | R - Reset");
                Serial.println("  ? - Status   | H - This help");
                Serial.println("  --- ROS UART on pins 8(RX)/9(TX) ---");
                Serial.println("  ROS packets auto-enable motors;");
                Serial.println("  watchdog stops them if ROS goes silent.");
                Serial.println("============================");
                break;
        }
    }
    while (Serial.available()) Serial.read();
}
// =================================================================
//                           SETUP
// =================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    led.begin();
    led.setBrightness(40);
    setLed(led.Color(180, 0, 180));

    ROSSerial.begin(115200, SERIAL_8N1, ROS_RX_PIN, ROS_TX_PIN);

    Serial.println("===== 4-JOINT CONTROL SYSTEM + ROS UART =====");
    Serial.println("M3: wrap-tracking (encoder on motor, 2:1 gearbox)");
    Serial.printf("M3: shaft -180 to +180 deg (motor +/-360, PID_SIGN=%d)\n", M3_PID_SIGN);
    Serial.println("Z-axis: wrap-tracking (encoder on motor, 3:1 gearbox)");
    Serial.printf("Z-axis: shaft -120 to +120 deg (motor +/-360, PID_SIGN=%d, maxPWM=%.0f)\n", ZA_PID_SIGN, ZA_PID_OUTPUT_MAX);
    Serial.printf("Pitch: 0-%.1f deg | Roll: 0-360 deg\n", PITCH_RANGE_DEG);
    Serial.printf("PID Kp=[%.2f,%.2f,%.2f,%.2f]\n", Kp[0], Kp[1], Kp[2], Kp[3]);
    Serial.printf("PID Ki=[%.3f,%.3f,%.3f,%.3f]\n", Ki[0], Ki[1], Ki[2], Ki[3]);
    Serial.printf("PID Kd=[%.3f,%.3f,%.3f,%.3f]\n", Kd[0], Kd[1], Kd[2], Kd[3]);
    Serial.printf("ROS UART: RX=%d TX=%d @ 115200  Watchdog=%lums\n", ROS_RX_PIN, ROS_TX_PIN, COMMS_TIMEOUT_MS);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQ);

    for (int i = 0; i < 4; i++) {
        pinMode(DIR_PINS[i], OUTPUT);
        ledcAttach(PWM_PINS[i], LEDC_FREQ, LEDC_RES);
        ledcWrite(PWM_PINS[i], 0);
    }

    if (!i2cHealthCheck()) i2cReset();

    for (int i = 0; i < 4; i++) {
        if (!JOINT_ALLOWED[i]) {
            Serial.printf("  %s: DISABLED\n", jointName(i));
            continue;
        }
        int val = readEncoder(i);
        if (val != -1) {
            motor_enabled[i]  = false;
            current_pos[i]    = val;
            target_pos[i]     = val;
            commanded_pos[i]  = val;
            prev_pos[i]       = val;

            if (i == 0) {
                m3Init(val);
                Serial.printf("  M3: OK at shaft %.1f deg (enc=%d, cont=%.0f)\n",
                    m3ContStepsToOutputDeg(m3_continuous_pos), val, m3_continuous_pos);
                Serial.println("  >>> M3 assumes startup = home half (-90 to +90 shaft)");
                Serial.println("  >>> If M3 starts beyond +/-90, position will be WRONG");
            } else if (i == 3) {
                zaInit(val);
                Serial.printf("  Z-axis: OK at shaft %.1f deg (enc=%d, cont=%.0f)\n",
                    zaContStepsToOutputDeg(za_continuous_pos), val, za_continuous_pos);
                Serial.println("  >>> Z-axis assumes startup = home third (-60 to +60 shaft)");
                Serial.println("  >>> If Z-axis starts beyond +/-60, position will be WRONG");
            } else {
                float deg = (i == 1) ? pitchStepsToDeg((float)val) : rollStepsToDeg((float)val);
                Serial.printf("  %s: OK at %.1f deg (step %d)\n", jointName(i), deg, val);
            }
        } else {
            Serial.printf("  %s: ENCODER FAIL\n", jointName(i));
        }
    }

    last_valid_cmd_time = millis();

    Serial.println("");
    Serial.println(">>> Motors OFF until ROS connects or you send 'R' via USB.");
    Serial.println(">>> Run T C 100 to check M3 direction, F 3 to flip");
    Serial.println(">>> Run T Z 100 to check Z-axis direction, F Z to flip");
    Serial.println("");
    Serial.println("Send H for help, S for stop");
    Serial.println("==============================================");
}

// =================================================================
//                           LOOP
// =================================================================

void loop() {
    unsigned long now = millis();

    // 1. Watchdog
    checkCommsWatchdog();

    // 2. ROS UART
    processROSCommand();

    // 3. USB serial
    processSerialCommand();

    // 4. PID control loop at 50 Hz
    if (!manual_test_mode && (now - last_pid_time >= 20)) {
        float dt = (now - last_pid_time) / 1000.0f;
        if (dt > 0.1f) dt = 0.02f;
        last_pid_time = now;

        readAllSensors();
        updateTrajectory();
        computePID(dt);
        applyMotorOutputs();
    }

    // 5. Status LED
    updateLed();

    // 6. Debug output every 200ms
    if (now - last_debug_time > 200) {
        last_debug_time = now;
        if (!manual_test_mode) {
            if (motor_enabled[0] && debug_show[0]) {
                float cur = m3ContStepsToOutputDeg(m3_continuous_pos);
                float tgt = m3ContStepsToOutputDeg(m3_continuous_target);
                float err = m3_continuous_cmd - m3_continuous_pos;
                float err_dz = applyDeadzone(err, ERROR_DEADZONE);
                Serial.printf("M3 | tgt:%+.1f cur:%+.1f | pid:%.0f | P:%.1f I:%.1f | cont:%.0f w:%d\n",
                    tgt, cur, pid_output[0], Kp[0] * err_dz, Ki[0] * integral[0],
                    m3_continuous_pos, m3_wrap_count);
            }
            for (int i = 1; i < 3; i++) {
                if (!motor_enabled[i] || !debug_show[i]) continue;
                float cur = jointStepsToDeg(i, current_pos[i]);
                float tgt = jointStepsToDeg(i, target_pos[i]);
                float err = wrapError(commanded_pos[i] - current_pos[i]);
                float err_dz = applyDeadzone(err, ERROR_DEADZONE);
                Serial.printf("%s | tgt:%+.1f cur:%+.1f | pid:%.0f | P:%.1f I:%.1f\n",
                    jointName(i), tgt, cur, pid_output[i], Kp[i] * err_dz, Ki[i] * integral[i]);
            }
            if (motor_enabled[3] && debug_show[3]) {
                float cur = zaContStepsToOutputDeg(za_continuous_pos);
                float tgt = zaContStepsToOutputDeg(za_continuous_target);
                float err = za_continuous_cmd - za_continuous_pos;
                float err_dz = applyDeadzone(err, ERROR_DEADZONE);
                Serial.printf("Z | tgt:%+.1f cur:%+.1f | pid:%.0f | P:%.1f I:%.1f | cont:%.0f w:%d\n",
                    tgt, cur, pid_output[3], Kp[3] * err_dz, Ki[3] * integral[3],
                    za_continuous_pos, za_wrap_count);
            }
        }
    }
}