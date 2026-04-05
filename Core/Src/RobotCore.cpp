#include "RobotCore.h"
#include "cmsis_os2.h"
#include "main.h"
#include <cstring>
#include <cstdlib>

// ── XOR checksum (shared with protocol.hpp on the ROS2 side) ──────────────────
uint8_t compute_checksum(const uint8_t* data, size_t len) {
    uint8_t x = 0;
    for (size_t i = 0; i < len; i++) x ^= data[i];
    return x;
}

extern UART_HandleTypeDef huart4;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

// =============================================================================
//  ADC DMA OVERSAMPLING BUFFER
//  DMA fills this circularly: [CH8, CH9, CH8, CH9, ...] interleaved.
//  ADC_OVS_SIZE pairs = ADC_OVS_SIZE samples per channel.
// =============================================================================
static const uint32_t ADC_OVS_SIZE = 1024;   // samples per channel
static volatile uint16_t adc_dma_buf[ADC_OVS_SIZE * 2];  // 2 channels, interleaved

// Read oversampled ADC for one channel (0=M1/CH8, 1=M2/CH9).
// Sums ADC_OVS_SIZE samples and divides: effectively a moving average
// that adds ~5 bits of virtual resolution via oversampling.
static inline uint16_t adc_read_oversampled(uint8_t ch) {
    uint32_t acc = 0;
    for (uint32_t i = ch; i < ADC_OVS_SIZE * 2; i += 2)
        acc += adc_dma_buf[i];
    return (uint16_t)(acc / ADC_OVS_SIZE);
}

// =============================================================================
//  UART DMA CACHE & PACKETS
// =============================================================================
static const int RX_BUF_SIZE = 128;
uint8_t rx_buffer[RX_BUF_SIZE];
static const int USB_RX_BUF_SIZE = 512;
uint8_t usb_rx_buffer[USB_RX_BUF_SIZE];
volatile uint32_t usb_rx_head = 0;
uint32_t usb_rx_tail = 0;

extern "C" uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

extern "C" void RobotCore_USB_Receive_Callback(uint8_t* buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        usb_rx_buffer[usb_rx_head] = buf[i];
        usb_rx_head = (usb_rx_head + 1) % USB_RX_BUF_SIZE;
    }
}
uint32_t rx_tail = 0;
uint32_t ms_ticks = 0;

float current_Kp_enc[4];
float current_Ki_enc[4];
float current_Kd_enc[4];

CommandPacketUnified  current_cmd;
FeedbackPacketUnified current_fb;

uint32_t last_valid_cmd_time = 0;
bool comms_ok = false;
const uint32_t COMMS_TIMEOUT_MS = 2000;

// =============================================================================
//  ESP-A: M1 / M2 CONFIG (POTENTIOMETERS)
// =============================================================================
#include "RobotConfig.h"
// PotMotors Instance
PotMotor motor1(&htim1, TIM_CHANNEL_1, GPIOE, GPIO_PIN_7, angles_M1, adcs_M1, PTS_M1, false, 0, 15.0f, 1.0f, 0.8f, 1);
PotMotor motor2(&htim1, TIM_CHANNEL_2, GPIOE, GPIO_PIN_8, angles_M2, adcs_M2, PTS_M2, false, 1, 15.0f, 1.0f, 0.8f, -1);
PotMotor* pots[2] = {&motor1, &motor2};

// =============================================================================
//  ESP-B: I2C ENCODERS CONFIG & STATE
// =============================================================================

struct I2CJoint {
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    GPIO_TypeDef* dirPort;
    uint16_t dirPin;
};

// [0]=M3, [1]=Pitch, [2]=Roll, [3]=Z-Axis
I2CJoint enc_joints[4] = {
    {&htim1, TIM_CHANNEL_3, GPIOE, GPIO_PIN_10}, // M3
    {&htim1, TIM_CHANNEL_4, GPIOE, GPIO_PIN_12}, // Pitch
    {&htim2, TIM_CHANNEL_2, GPIOE, GPIO_PIN_15}, // Roll
    {&htim2, TIM_CHANNEL_3, GPIOD, GPIO_PIN_0}   // Z-ax
};



bool  enc_enabled[4]     = {false};
float target_pos[4]      = {0};
float commanded_pos[4]   = {0};
float current_pos[4]     = {0};
float prev_pos[4]        = {0};
float integral[4]        = {0};
float prev_error[4]      = {0};
float pid_output[4]      = {0};
int   sensor_fail_count[4] = {0};
bool  sensor_read_ok[4]  = {false};

// M3 Multi-turn
int   m3_wrap_count = 0;  int m3_prev_raw = 0;
double m3_continuous_pos = 0, m3_continuous_target = 0;
double m3_continuous_cmd = 0, m3_continuous_prev   = 0;

// Z-Axis Multi-turn
int   za_wrap_count = 0;  int za_prev_raw = 0;
double za_continuous_pos = 0, za_continuous_target = 0;
double za_continuous_cmd = 0, za_continuous_prev   = 0;
float za_profile_vel    = 0; // Trapezoidal profile: current velocity (steps/tick)
int   za_stall_counter = 0; // Tracks consecutive overload ticks
bool  z_collision_fault = false;

// ── Velocity feedback state (Step 4 from protocol patch) ─────────────────────
// Dedicated prev-state so PID D-term consumption doesn't zero the deltas.
float fb_vel_m3    = 0.0f, fb_prev_m3    = 0.0f;
float fb_vel_pitch = 0.0f, fb_prev_pitch = 0.0f;
float fb_vel_roll  = 0.0f, fb_prev_roll  = 0.0f;
float fb_vel_z     = 0.0f, fb_prev_z     = 0.0f;

// M3 profile velocity
float m3_profile_vel    = 0; // Trapezoidal profile: current velocity (steps/tick)

bool systemActive = false;
float pot_targets[2] = {0};

bool is_open_loop_mode = false;
int open_loop_pwm[6] = {0};

int debug_raw_encoders[4] = {0};
int debug_raw_adcs[2] = {0};
bool pitch_frozen = false;  // Set when pitch exceeds limits by >5°

// =============================================================================
//  HELPER MATH
// =============================================================================
float wrapValue(float val) {
    while (val < 0) val += 4096.0f;
    while (val >= 4096.0f) val -= 4096.0f;
    return val;
}
float wrapError(float error) {
    if (error > 2048.0f) error -= 4096.0f;
    if (error < -2048.0f) error += 4096.0f;
    return error;
}
float applyDeadzone(float error, float deadzone) {
    if (fabsf(error) < deadzone) return 0.0f;
    return (error > 0) ? (error - deadzone) : (error + deadzone);
}

// Conversions
float motorDegToSteps(float motor_deg) { return (motor_deg / 360.0f) * 4096.0f; }
float outputToMotorDeg(float output_deg)   { return output_deg * M3_GEAR_RATIO; }
float zaOutputToMotorDeg(float output_deg) { return output_deg * ZA_GEAR_RATIO; }
float pitchDegToSteps(float deg) {
    // Inverted: positive degrees DECREASE corrected value (travel crosses zero)
    float steps = (float)PITCH_HOME_STEPS - (deg / PITCH_RANGE_DEG) * (float)(PITCH_LIMIT_MAX - PITCH_LIMIT_MIN);
    if (steps < PITCH_LIMIT_MIN) steps = PITCH_LIMIT_MIN;
    if (steps > PITCH_LIMIT_MAX) steps = PITCH_LIMIT_MAX;
    return steps;
}
float rollDegToSteps(float deg)    { return (deg / ROLL_RANGE_DEG) * 4096.0f; }

float m3OutputDegToContSteps(float output_deg) {
    output_deg = (output_deg > M3_OUTPUT_LIMIT) ? M3_OUTPUT_LIMIT : (output_deg < -M3_OUTPUT_LIMIT) ? -M3_OUTPUT_LIMIT : output_deg;
    return motorDegToSteps(outputToMotorDeg(output_deg));
}
float zaOutputDegToContSteps(float output_deg) {
    output_deg = (output_deg > ZA_OUTPUT_LIMIT) ? ZA_OUTPUT_LIMIT : (output_deg < -ZA_OUTPUT_LIMIT) ? -ZA_OUTPUT_LIMIT : output_deg;
    return motorDegToSteps(zaOutputToMotorDeg(output_deg));
}

// =============================================================================
//  HARDWARE I/O WRAPPERS
// =============================================================================

void DriveMotor(TIM_HandleTypeDef* htim, uint32_t channel, GPIO_TypeDef* dirPort, uint16_t dirPin, int pwm_signed) {
    int speed = abs(pwm_signed);
    if (speed > 255) speed = 255;
    if (speed < 15) speed = 0; // Motor cutoff
    
    HAL_GPIO_WritePin(dirPort, dirPin, (pwm_signed >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(htim, channel, speed);
}

void tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    uint8_t data = 1 << channel;
    HAL_I2C_Master_Transmit(&hi2c1, 0x70 << 1, &data, 1, 10);
}

int readEncoderRaw(int joint) {
    tcaSelect(MUX_CHANNELS[joint]);
    uint8_t buf[2] = {0};
    if (HAL_I2C_Mem_Read(&hi2c1, 0x36 << 1, 0x0C, I2C_MEMADD_SIZE_8BIT, buf, 2, 10) == HAL_OK) {
        return (buf[0] << 8) | buf[1];
    }
    return -1;
}

int readEncoder(int joint) {
    int raw = readEncoderRaw(joint);
    debug_raw_encoders[joint] = raw;
    if (raw == -1) return -1;
    // ENCODER_DIRECTION: +1 = encoder counts UP with movement (normal)
    //                    -1 = encoder counts DOWN with movement (invert, e.g. Pitch)
    int corrected;
    if (ENCODER_DIRECTION[joint] == -1) {
        corrected = HOME_OFFSETS[joint] - raw;  // inverted: home=0, extends to positive
    } else {
        corrected = raw - HOME_OFFSETS[joint];  // normal
    }
    while (corrected < 0)    corrected += 4096;
    while (corrected >= 4096) corrected -= 4096;
    return corrected;
}

void setLed(bool connected) {
    // Green = LD4 (PD12), Red = LD5 (PD14)
    if (connected) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    }
}

// =============================================================================
//  POT MOTOR LOGIC (ESP-A)
// =============================================================================
void PotMotor::init(int initialADC) {
    currentAngle   = adcToAngle(initialADC);
    lastAngle      = currentAngle;
    commandedAngle = currentAngle;
    prevStallAngle = currentAngle;
    lastValidAngle = currentAngle;
    ema.setEstimate((float)initialADC);
}

float PotMotor::adcToAngle(int adc) {
    bool increasing = adcTable[numPoints - 1] > adcTable[0];
    if (increasing) {
        if (adc <= adcTable[0])             return angleTable[0];
        if (adc >= adcTable[numPoints - 1]) return angleTable[numPoints - 1];
    } else {
        if (adc >= adcTable[0])             return angleTable[0];
        if (adc <= adcTable[numPoints - 1]) return angleTable[numPoints - 1];
    }
    for (int i = 0; i < numPoints - 1; i++) {
        int a0 = adcTable[i], a1 = adcTable[i + 1];
        if ((adc >= a0 && adc <= a1) || (adc <= a0 && adc >= a1)) {
            if (a1 == a0) return angleTable[i]; // Prevent division by zero
            float t = (float)(adc - a0) / (float)(a1 - a0);
            return angleTable[i] + t * (angleTable[i + 1] - angleTable[i]);
        }
    }
    return angleTable[numPoints - 1];
}

void PotMotor::resetStall() {
    stalled        = false;
    stallCounter   = 0;
    prevStallAngle = currentAngle;
}

void PotMotor::updateTrajectory(float target) {
    const float MAX_SLEW_DEG = 2.0f;
    float err = target - commandedAngle;
    if (fabsf(err) > MAX_SLEW_DEG)
        commandedAngle += (err > 0.0f) ? MAX_SLEW_DEG : -MAX_SLEW_DEG;
    else
        commandedAngle = target;
}

void PotMotor::update(float target, float dt, int rawADC) {
    if (rawADC < 10 || rawADC > 4085) { DriveMotor(htim, channel, pinDirPort, pinDirPin, 0); return; }

    float filteredADC = ema.updateEstimate((float)rawADC);
    float prevAngle   = currentAngle;
    float newAngle    = adcToAngle((int)filteredADC);

    // Angle deadband: only update currentAngle if the joint actually moved
    // more than angleDead. Prevents integrator wind-up against ADC jitter.
    if (fabsf(newAngle - lastValidAngle) >= angleDead) {
        lastValidAngle = newAngle;
    }
    currentAngle  = lastValidAngle;
    velocityCDS   = ((currentAngle - prevAngle) / dt) * 100.0f;

    if (!enabled) { DriveMotor(htim, channel, pinDirPort, pinDirPin, 0); return; }

    updateTrajectory(target);
    float error = commandedAngle - currentAngle;
    const float DEADZONE = 1.0f;

    if (stalled) {
        resetStall(); // Fallback reset in case it was somehow set
    }

    // Removed stall detection completely as requested
    if (fabsf(error) > DEADZONE) {
        float P         = kp * error;
        integral        = integral + (error * ki) * dt;
        if(integral > 100.0f) integral = 100.0f;
        if(integral < -100.0f) integral = -100.0f;
        
        float D         = -kd * ((currentAngle - lastAngle) / dt);
        float rawOutput = (P + integral + D) * pid_sign;

        // HARD LIMIT: prevent driving past calibration bounds
        float minAngle = angleTable[0];
        float maxAngle = angleTable[numPoints - 1];
        if (minAngle > maxAngle) { float t = minAngle; minAngle = maxAngle; maxAngle = t; }
        // If at/past min angle and trying to drive lower (negative error -> negative output for pot)
        if (currentAngle <= minAngle && rawOutput < 0) { rawOutput = 0; integral = 0; }
        // If at/past max angle and trying to drive higher
        if (currentAngle >= maxAngle && rawOutput > 0) { rawOutput = 0; integral = 0; }
        
        if(rawOutput > 255.0f) rawOutput = 255.0f;
        if(rawOutput < -255.0f) rawOutput = -255.0f;

        // Map minimum power
        const int MIN_POWER = 120;
        int pwm = 0;
        if (fabsf(rawOutput) > 0) {
            pwm = MIN_POWER + (int)(fabsf(rawOutput) * (255 - MIN_POWER) / 255.0f);
        }

        bool dir = (rawOutput > 0);
        if (reverseDir) dir = !dir;
        
        int signed_pwm = dir ? pwm : -pwm;
        DriveMotor(htim, channel, pinDirPort, pinDirPin, signed_pwm);

        currentPWM = pwm;
        lastAngle  = currentAngle;
    } else {
        DriveMotor(htim, channel, pinDirPort, pinDirPin, 0);
        integral   = 0;
        currentPWM = 0;
        resetStall();
    }
}

// =============================================================================
//  SYSTEM INITIALIZATION
// =============================================================================
extern "C" {

void RobotCore_Init(void) {
    // Start PWM Timers
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    // Start ADC DMA — runs continuously in background, circular
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, ADC_OVS_SIZE * 2);

    // Read initial oversampled ADC values for motor init
    // Wait one full buffer fill (~4ms at 84-cycle sample time, 168MHz APB2/2)
    HAL_Delay(10);
    uint16_t adc_init[2];
    adc_init[0] = adc_read_oversampled(0);
    adc_init[1] = adc_read_oversampled(1);

    pots[0]->init(adc_init[0]);
    pots[1]->init(adc_init[1]);
    pot_targets[0] = pots[0]->currentAngle;
    pot_targets[1] = pots[1]->currentAngle;

    // Initialize dynamic PID constants for encoders
    for (int i = 0; i < 4; i++) {
        current_Kp_enc[i] = Kp_enc[i];
        current_Ki_enc[i] = Ki_enc[i];
        current_Kd_enc[i] = Kd_enc[i];
    }

    // Start UART DMA
    HAL_UART_Receive_DMA(&huart4, rx_buffer, RX_BUF_SIZE);
    
    current_fb.header[0] = 'F'; current_fb.header[1] = 'B';
    memset(current_fb.motor_vel, 0, sizeof(current_fb.motor_vel));
    current_fb.flags = 0;
    current_fb.checksum = 0;
}

void emergencyStopAll() {
    systemActive   = false;
    comms_ok       = false;
    for (int i = 0; i < 2; i++) {
        pots[i]->enabled = false;
        DriveMotor(pots[i]->htim, pots[i]->channel, pots[i]->pinDirPort, pots[i]->pinDirPin, 0);
    }
    for (int i = 0; i < 4; i++) {
        enc_enabled[i] = false;
        DriveMotor(enc_joints[i].htim, enc_joints[i].channel, enc_joints[i].dirPort, enc_joints[i].dirPin, 0);
        integral[i] = 0;
    }
}

// =============================================================================
// =============================================================================
bool parseCommandPacket(uint8_t* buf, uint32_t buf_size, uint32_t* tail, uint32_t head) {
    bool found = false;
    while (*tail != head) {
        if (buf[*tail] == 'S' || buf[*tail] == 'D' || buf[*tail] == 'K' || buf[*tail] == 'R') {
            uint32_t bytes_available = (head >= *tail) ? (head - *tail) : (buf_size - *tail + head);
            if (bytes_available >= sizeof(CommandPacketUnified)) {
                uint8_t pkt[sizeof(CommandPacketUnified)];
                for (size_t i = 0; i < sizeof(CommandPacketUnified); i++) {
                    pkt[i] = buf[(*tail + i) % buf_size];
                }
                CommandPacketUnified* cmd = (CommandPacketUnified*)pkt;
                if (cmd->header[1] == 'T' && cmd->footer == '\n') {
                    // Validate XOR checksum: covers bytes [0..26]
                    uint8_t expected = compute_checksum(pkt, offsetof(CommandPacketUnified, checksum));
                    if (expected != cmd->checksum) {
                        // Bad checksum — skip this byte and keep scanning
                        *tail = (*tail + 1) % buf_size;
                        continue;
                    }
                    memcpy(&current_cmd, cmd, sizeof(CommandPacketUnified));
                    last_valid_cmd_time = osKernelGetTickCount();
                    found = true;
                    *tail = (*tail + sizeof(CommandPacketUnified)) % buf_size;
                    continue;
                }
            } else {
                break;
            }
        }
        *tail = (*tail + 1) % buf_size;
    }
    return found;
}

void RobotCore_ROSTask(void) {
    uint32_t uart_head = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx);
    bool uart_pkt = parseCommandPacket(rx_buffer, RX_BUF_SIZE, &rx_tail, uart_head);
    
    uint32_t cur_usb_head = usb_rx_head; // Safe snapshot
    bool usb_pkt = parseCommandPacket(usb_rx_buffer, USB_RX_BUF_SIZE, &usb_rx_tail, cur_usb_head);

    if (uart_pkt || usb_pkt) {
        if (!comms_ok) {
            comms_ok = true;
            setLed(true);
            
            // Clear latching safety faults on reconnect
            pitch_frozen = false;
            z_collision_fault = false;
            za_stall_counter = 0;
            za_profile_vel = 0;
            m3_profile_vel = 0;
            
            // Re-enable ESP-A (DMA Oversampling)
            uint16_t adcs[2];
            adcs[0] = adc_read_oversampled(0);
            adcs[1] = adc_read_oversampled(1);
            
            for (int i=0; i<2; i++) {
                pots[i]->currentAngle = pots[i]->adcToAngle(adcs[i]);
                pots[i]->commandedAngle = pots[i]->currentAngle;
                
                // Ensure targets can't exceed calibration bounds,
                // otherwise a corrupt re-enable target can cause runaway.
                if (pot_targets[0] < 47.4f) pot_targets[0] = 47.4f;
                if (pot_targets[0] > 74.0f) pot_targets[0] = 74.0f;
                if (pot_targets[1] < 1.0f)  pot_targets[1] = 1.0f;
                if (pot_targets[1] > 61.2f) pot_targets[1] = 61.2f;

                pots[i]->integral = 0;
                pots[i]->enabled = true;
                pots[i]->resetStall();
            }
            
            // Re-enable ESP-B
            for (int i=0; i<4; i++) {
                if (!JOINT_ALLOWED[i]) continue;  // Skip disabled joints
                int val = readEncoder(i);
                if (val != -1) {
                    enc_enabled[i] = true; current_pos[i] = val;
                    commanded_pos[i] = val; prev_pos[i] = val; target_pos[i] = val; integral[i] = 0;
                    if (i == 0) {
                        int val = readEncoder(0);
                        if(val != -1) {
                            m3_prev_raw = val;
                            // Home sector: if corrected val>2048 the arm is in the negative half.
                            // Set wrap_count=-1 so the running formula (wrap_count*4096)+raw gives
                            // the correct signed value automatically every tick.
                            if (val > 2048) {
                                m3_wrap_count = -1;
                            } else {
                                m3_wrap_count = 0;
                            }
                            m3_continuous_pos = (double)m3_wrap_count * 4096.0 + (double)val;
                        }
                        m3_continuous_cmd = m3_continuous_pos; m3_continuous_prev = m3_continuous_pos; m3_continuous_target = m3_continuous_pos;
                    }
                    if (i == 3) {
                        int val = readEncoder(3);
                        if(val != -1) {
                            za_prev_raw = val;
                            // Home sector: if corrected val>2048 the arm is in the negative half.
                            // Set wrap_count=-1 so the running formula (wrap_count*4096)+raw gives
                            // the correct signed value automatically every tick.
                            if (val > 2048) {
                                za_wrap_count = -1;
                            } else {
                                za_wrap_count = 0;
                            }
                            za_continuous_pos = (double)za_wrap_count * 4096.0 + (double)val;
                        }
                        za_continuous_cmd = za_continuous_pos; za_continuous_prev = za_continuous_pos; za_continuous_target = za_continuous_pos;
                    }
                }
            }
        }
        
        // Target Update
        if (current_cmd.header[0] == 'S') {
            is_open_loop_mode = false;
            pot_targets[0] = (float)current_cmd.motor_cmd[0] / 1000.0f;
            pot_targets[1] = (float)current_cmd.motor_cmd[1] / 1000.0f;
            if (pot_targets[0] < 47.4f) pot_targets[0] = 47.4f; else if (pot_targets[0] > 74.0f)  pot_targets[0] = 74.0f;
            if (pot_targets[1] <  1.0f) pot_targets[1] =  1.0f; else if (pot_targets[1] > 61.2f)  pot_targets[1] = 61.2f;
            
            m3_continuous_target = (double)m3OutputDegToContSteps((float)current_cmd.motor_cmd[2]/1000.0f);
            target_pos[1] = pitchDegToSteps((float)current_cmd.motor_cmd[3]/1000.0f);
            if (target_pos[1] < PITCH_LIMIT_MIN) target_pos[1] = PITCH_LIMIT_MIN;
            if (target_pos[1] > PITCH_LIMIT_MAX) target_pos[1] = PITCH_LIMIT_MAX;
            target_pos[2] = wrapValue(rollDegToSteps((float)current_cmd.motor_cmd[4]/1000.0f));
            za_continuous_target = (double)zaOutputDegToContSteps((float)current_cmd.motor_cmd[5]/1000.0f);
        } else if (current_cmd.header[0] == 'D') {
            // Passive debugging mode. Do not change physical setpoints, just loop.
            is_open_loop_mode = false;
        } else if (current_cmd.header[0] == 'R') {
            // Passive Request mode. Just reply with angles, do not move.
            is_open_loop_mode = false;
        } else if (current_cmd.header[0] == 'K') {
            int target_var = current_cmd.motor_cmd[0];
            float p = (float)current_cmd.motor_cmd[1] / 1000.0f;
            float i = (float)current_cmd.motor_cmd[2] / 1000.0f;
            float d = (float)current_cmd.motor_cmd[3] / 1000.0f;
            if (target_var == 0)      { pots[0]->kp = p; pots[0]->ki = i; pots[0]->kd = d; }
            else if (target_var == 1) { pots[1]->kp = p; pots[1]->ki = i; pots[1]->kd = d; }
            else if (target_var == 2) { current_Kp_enc[0] = p; current_Ki_enc[0] = i; current_Kd_enc[0] = d; }
            else if (target_var == 3) { current_Kp_enc[1] = p; current_Ki_enc[1] = i; current_Kd_enc[1] = d; }
            else if (target_var == 4) { current_Kp_enc[2] = p; current_Ki_enc[2] = i; current_Kd_enc[2] = d; }
            else if (target_var == 5) { current_Kp_enc[3] = p; current_Ki_enc[3] = i; current_Kd_enc[3] = d; }
            is_open_loop_mode = false;
        }
    }
    
    // Watchdog
    if (comms_ok && (osKernelGetTickCount() - last_valid_cmd_time > COMMS_TIMEOUT_MS)) {
        emergencyStopAll();
        setLed(false);
    }
}

// =============================================================================
//  50Hz PID CONTROL LOOP (HIGH PRIORITY TASK)
// =============================================================================
void RobotCore_PIDTask(void) {
    const float dt = 0.02f; // 50Hz
    
    // 1. Read ADCs for ESP-A (DMA Oversampling — non-blocking)
    uint16_t adcs[2];
    adcs[0] = adc_read_oversampled(0);
    adcs[1] = adc_read_oversampled(1);
    
    debug_raw_adcs[0] = adcs[0];
    debug_raw_adcs[1] = adcs[1];
    
    // 2. Read Encoders for ESP-B
    if (enc_enabled[0]) {
        int raw = readEncoder(0);
        if (raw != -1) {
            int delta = raw - m3_prev_raw;
            if (delta > 2048) m3_wrap_count--; else if (delta < -2048) m3_wrap_count++;
            m3_prev_raw = raw;
            m3_continuous_pos = (double)m3_wrap_count * 4096.0 + (double)raw;
            current_pos[0] = (float)raw;
            sensor_read_ok[0] = true;
            sensor_fail_count[0] = 0;
        } else {
            if (++sensor_fail_count[0] > MAX_SENSOR_FAILS) emergencyStopAll();
        }
    }
    for (int i=1; i<3; i++) {
        if (!enc_enabled[i]) continue;
        int raw = readEncoder(i);
        if (raw != -1) { current_pos[i] = raw; sensor_fail_count[i] = 0; sensor_read_ok[i] = true; }
        else if (++sensor_fail_count[i] > MAX_SENSOR_FAILS) emergencyStopAll();
    }
    if (enc_enabled[3]) {
        int raw = readEncoder(3);
        if (raw != -1) {
            int delta = raw - za_prev_raw;
            if (delta > 2048) za_wrap_count--; else if (delta < -2048) za_wrap_count++;
            za_prev_raw = raw;
            za_continuous_pos = (double)za_wrap_count * 4096.0 + (double)raw;
            current_pos[3] = (float)raw;
            sensor_read_ok[3] = true;
            sensor_fail_count[3] = 0;
        } else {
            if (++sensor_fail_count[3] > MAX_SENSOR_FAILS) emergencyStopAll();
        }
    }
    
    // 3. Update ESP-A Motors
    pots[0]->update(pot_targets[0], dt, adcs[0]);
    pots[1]->update(pot_targets[1], dt, adcs[1]);
    
    // 4. Update ESP-B Trajectories — Trapezoidal Motion Profiles
    // ── M3 ────────────────────────────────────────────────────────────────────
    if (enc_enabled[0]) {
        float err = (float)(m3_continuous_target - m3_continuous_cmd);
        // Acceleration step toward desired velocity
        float desired_vel = (err > 0) ? M3_PROF_MAX_VEL : (err < 0) ? -M3_PROF_MAX_VEL : 0.0f;
        // Deceleration: if we're close, reduce desired vel proportionally
        float brake_dist = (m3_profile_vel * m3_profile_vel) / (2.0f * M3_PROF_MAX_ACCEL);
        if (fabsf(err) < brake_dist + 1.0f) {
            desired_vel = (err > 0) ? M3_PROF_MAX_VEL * (fabsf(err) / (brake_dist + 1.0f))
                                    : -M3_PROF_MAX_VEL * (fabsf(err) / (brake_dist + 1.0f));
        }
        // Clamp acceleration
        float dv = desired_vel - m3_profile_vel;
        if (fabsf(dv) > M3_PROF_MAX_ACCEL) dv = (dv > 0) ? M3_PROF_MAX_ACCEL : -M3_PROF_MAX_ACCEL;
        m3_profile_vel += dv;
        // At target — stop
        if (fabsf(err) < 0.5f) { m3_profile_vel = 0; m3_continuous_cmd = m3_continuous_target; }
        else { m3_continuous_cmd += (double)m3_profile_vel; }
        if (m3_continuous_cmd > (double)M3_CONT_LIMIT)  { m3_continuous_cmd = (double)M3_CONT_LIMIT;  m3_profile_vel = 0; }
        if (m3_continuous_cmd < -(double)M3_CONT_LIMIT) { m3_continuous_cmd = -(double)M3_CONT_LIMIT; m3_profile_vel = 0; }
    } else { m3_continuous_cmd = m3_continuous_pos; m3_profile_vel = 0; }

    // Pitch trajectory (LINEAR — no wrapping, corrected values are continuous)
    if (enc_enabled[1]) {
        float e = target_pos[1] - commanded_pos[1];
        if (fabsf(e) > MAX_SLEW_RATE) {
            commanded_pos[1] += (e > 0) ? MAX_SLEW_RATE : -MAX_SLEW_RATE;
        } else commanded_pos[1] = target_pos[1];
        if (commanded_pos[1] < PITCH_LIMIT_MIN) commanded_pos[1] = PITCH_LIMIT_MIN;
        if (commanded_pos[1] > PITCH_LIMIT_MAX) commanded_pos[1] = PITCH_LIMIT_MAX;
    } else { commanded_pos[1] = current_pos[1]; }

    // Roll trajectory (CIRCULAR — uses wrap)
    if (enc_enabled[2]) {
        float e = wrapError(target_pos[2] - commanded_pos[2]);
        if (fabsf(e) > MAX_SLEW_RATE) {
            commanded_pos[2] += (e > 0) ? MAX_SLEW_RATE : -MAX_SLEW_RATE;
            commanded_pos[2] = wrapValue(commanded_pos[2]);
        } else commanded_pos[2] = target_pos[2];
    } else { commanded_pos[2] = current_pos[2]; }

    // ── Z-Axis ────────────────────────────────────────────────────────────────
    if (enc_enabled[3]) {
        float err = (float)(za_continuous_target - za_continuous_cmd);
        // Desired velocity based on direction
        float desired_vel = (err > 0) ? ZA_PROF_MAX_VEL : (err < 0) ? -ZA_PROF_MAX_VEL : 0.0f;
        // Deceleration: scale velocity to zero as we approach target
        float brake_dist = (za_profile_vel * za_profile_vel) / (2.0f * ZA_PROF_MAX_ACCEL);
        if (fabsf(err) < brake_dist + 1.0f) {
            desired_vel = (err > 0) ? ZA_PROF_MAX_VEL * (fabsf(err) / (brake_dist + 1.0f))
                                    : -ZA_PROF_MAX_VEL * (fabsf(err) / (brake_dist + 1.0f));
        }
        // Clamp acceleration
        float dv = desired_vel - za_profile_vel;
        if (fabsf(dv) > ZA_PROF_MAX_ACCEL) dv = (dv > 0) ? ZA_PROF_MAX_ACCEL : -ZA_PROF_MAX_ACCEL;
        za_profile_vel += dv;
        // At target — stop
        if (fabsf(err) < 0.5f) { za_profile_vel = 0; za_continuous_cmd = za_continuous_target; }
        else { za_continuous_cmd += (double)za_profile_vel; }
        if (za_continuous_cmd > (double)ZA_CONT_LIMIT)  { za_continuous_cmd = (double)ZA_CONT_LIMIT;  za_profile_vel = 0; }
        if (za_continuous_cmd < -(double)ZA_CONT_LIMIT) { za_continuous_cmd = -(double)ZA_CONT_LIMIT; za_profile_vel = 0; }
    } else { za_continuous_cmd = za_continuous_pos; za_profile_vel = 0; }
    
    // 5. Compute ESP-B PIDs
    // M3
    if (enc_enabled[0]) {
        float error_dz = applyDeadzone((float)(m3_continuous_cmd - m3_continuous_pos), ERROR_DEADZONE);
        float p_term = current_Kp_enc[0] * error_dz;
        if (fabsf(pid_output[0]) < PID_OUTPUT_MAX) {
            integral[0] += error_dz * dt;
            if(integral[0] > INTEGRAL_MAX) integral[0] = INTEGRAL_MAX;
            if(integral[0] < -INTEGRAL_MAX) integral[0] = -INTEGRAL_MAX;
        }
        if (fabsf(error_dz) < 0.1f) integral[0] = 0;
        float d_term = 0.0f;
        if (sensor_read_ok[0]) {
            d_term = current_Kd_enc[0] * (float)((m3_continuous_pos - m3_continuous_prev) / (double)dt);
            m3_continuous_prev = m3_continuous_pos;
        }
        pid_output[0] = p_term + current_Ki_enc[0] * integral[0] - d_term;
        if(pid_output[0] > PID_OUTPUT_MAX) pid_output[0] = PID_OUTPUT_MAX;
        if(pid_output[0] < -PID_OUTPUT_MAX) pid_output[0] = -PID_OUTPUT_MAX;

        // --- M3 Physical Hard Stop & Soft limits ---
        float m3_pos = (float)m3_continuous_pos;
        if (m3_pos >= M3_CONT_LIMIT && pid_output[0] > 0) { pid_output[0] = 0; integral[0] = 0; }
        if (m3_pos <= -M3_CONT_LIMIT && pid_output[0] < 0) { pid_output[0] = 0; integral[0] = 0; }
        
        if (m3_pos > (M3_CONT_LIMIT - M3_CONT_SOFT_ZONE) && m3_pos < M3_CONT_LIMIT && pid_output[0] > 0)
            pid_output[0] *= (M3_CONT_LIMIT - m3_pos) / M3_CONT_SOFT_ZONE;
        if (m3_pos < (-M3_CONT_LIMIT + M3_CONT_SOFT_ZONE) && m3_pos > -M3_CONT_LIMIT && pid_output[0] < 0)
            pid_output[0] *= (m3_pos + M3_CONT_LIMIT) / M3_CONT_SOFT_ZONE;
    } else { pid_output[0] = 0; integral[0] = 0; }
    
    // Pitch PID (LINEAR — no wrapError, corrected values are continuous)
    if (enc_enabled[1]) {
        float error_dz = applyDeadzone(commanded_pos[1] - current_pos[1], ERROR_DEADZONE);
        float p_term = current_Kp_enc[1] * error_dz;
        if (fabsf(pid_output[1]) < PID_OUTPUT_MAX) {
            integral[1] += error_dz * dt;
            if(integral[1] > INTEGRAL_MAX) integral[1] = INTEGRAL_MAX;
            if(integral[1] < -INTEGRAL_MAX) integral[1] = -INTEGRAL_MAX;
        }
        if (fabsf(error_dz) < 0.1f) integral[1] = 0;
        float d_term = 0.0f;
        if (sensor_read_ok[1]) {
            d_term = current_Kd_enc[1] * ((current_pos[1] - prev_pos[1]) / dt);
            prev_pos[1] = current_pos[1];
        }
        pid_output[1] = p_term + (current_Ki_enc[1] * integral[1]) - d_term;
        if(pid_output[1] > PID_OUTPUT_MAX) pid_output[1] = PID_OUTPUT_MAX;
        if(pid_output[1] < -PID_OUTPUT_MAX) pid_output[1] = -PID_OUTPUT_MAX;
    } else { pid_output[1] = 0; integral[1] = 0; }

    // Roll PID (CIRCULAR — uses wrapError)
    if (enc_enabled[2]) {
        float error_dz = applyDeadzone(wrapError(commanded_pos[2] - current_pos[2]), ERROR_DEADZONE);
        float p_term = current_Kp_enc[2] * error_dz;
        if (fabsf(pid_output[2]) < PID_OUTPUT_MAX) {
            integral[2] += error_dz * dt;
            if(integral[2] > INTEGRAL_MAX) integral[2] = INTEGRAL_MAX;
            if(integral[2] < -INTEGRAL_MAX) integral[2] = -INTEGRAL_MAX;
        }
        if (fabsf(error_dz) < 0.1f) integral[2] = 0;
        float d_term = 0.0f;
        if (sensor_read_ok[2]) {
            d_term = current_Kd_enc[2] * (wrapError(current_pos[2] - prev_pos[2]) / dt);
            prev_pos[2] = current_pos[2];
        }
        pid_output[2] = p_term + (current_Ki_enc[2] * integral[2]) - d_term;
        if(pid_output[2] > PID_OUTPUT_MAX) pid_output[2] = PID_OUTPUT_MAX;
        if(pid_output[2] < -PID_OUTPUT_MAX) pid_output[2] = -PID_OUTPUT_MAX;
    } else { pid_output[2] = 0; integral[2] = 0; }
    
    // Z-Axis
    if (enc_enabled[3]) {
        float error_dz = applyDeadzone((float)(za_continuous_cmd - za_continuous_pos), ERROR_DEADZONE);
        float p_term = current_Kp_enc[3] * error_dz;
        if (fabsf(pid_output[3]) < ZA_PID_OUTPUT_MAX) {
            integral[3] += error_dz * dt;
            if(integral[3] > INTEGRAL_MAX) integral[3] = INTEGRAL_MAX;
            if(integral[3] < -INTEGRAL_MAX) integral[3] = -INTEGRAL_MAX;
        }
        if (fabsf(error_dz) < 0.1f) integral[3] = 0;
        float d_term = 0.0f;
        float actual_velocity = 0.0f;
        if (sensor_read_ok[3]) {
            actual_velocity = (float)((za_continuous_pos - za_continuous_prev) / (double)dt);
            d_term = current_Kd_enc[3] * actual_velocity;
            za_continuous_prev = za_continuous_pos;
        }
        pid_output[3] = p_term + (current_Ki_enc[3] * integral[3]) - d_term;
        if(pid_output[3] > ZA_PID_OUTPUT_MAX) pid_output[3] = ZA_PID_OUTPUT_MAX;
        if(pid_output[3] < -ZA_PID_OUTPUT_MAX) pid_output[3] = -ZA_PID_OUTPUT_MAX;

        // --- Z-axis Physical Hard Stop & Soft limits ---
        float za_pos = za_continuous_pos;
        if (za_pos >= ZA_CONT_LIMIT && pid_output[3] > 0) { pid_output[3] = 0; integral[3] = 0; }
        if (za_pos <= -ZA_CONT_LIMIT && pid_output[3] < 0) { pid_output[3] = 0; integral[3] = 0; }
        
        // Continuous Proportional Soft Stop (Smooth braking at physical extremes)
        if (za_pos > (ZA_CONT_LIMIT - ZA_CONT_SOFT_ZONE) && za_pos < ZA_CONT_LIMIT && pid_output[3] > 0)
            pid_output[3] *= (ZA_CONT_LIMIT - za_pos) / ZA_CONT_SOFT_ZONE;
        if (za_pos < (-ZA_CONT_LIMIT + ZA_CONT_SOFT_ZONE) && za_pos > -ZA_CONT_LIMIT && pid_output[3] < 0)
            pid_output[3] *= (za_pos + ZA_CONT_LIMIT) / ZA_CONT_SOFT_ZONE;
        
        // --- User Docking Zone Limits ---
        // Converts the chosen physical normal limit to continuous raw encoder steps
        float normal_limit_steps = (ZA_NORMAL_LIMIT_DEG * ZA_GEAR_RATIO / 360.0f) * 4096.0f;
        if (fabsf(za_pos) > normal_limit_steps) {
            pid_output[3] *= ZA_DOCKING_SCALE;
        }

        // --- Virtual Current Sensing (Collision Detection) ---
        if (ZA_COLLISION_ENABLED) {
            // If the integral windup is high BUT the arm is barely moving, we've hit an obstacle.
            if (fabsf(integral[3]) > ZA_COLLISION_INTEGRAL && fabsf(actual_velocity) < ZA_COLLISION_VELOCITY) {
                za_stall_counter++;
                if (za_stall_counter > ZA_COLLISION_TIMEOUT) {
                    z_collision_fault = true;
                    za_continuous_cmd = za_continuous_pos; // Zero the effort immediately
                    za_continuous_target = za_continuous_pos;
                    integral[3] = 0;
                }
            } else {
                za_stall_counter = 0; // Reset immediately if we start moving or pressure releases
            }
            
            // If user pulls the slider back in the opposite direction of the error, unlatch fault.
            if (z_collision_fault && fabsf(za_continuous_target - za_continuous_pos) > 10.0f) {
                z_collision_fault = false;
            }
        } else {
            za_stall_counter = 0;
            z_collision_fault = false;
        }

    } else { pid_output[3] = 0; integral[3] = 0; za_stall_counter = 0; }
    
    // 6. Apply ESP-B PWM Outputs
    if (is_open_loop_mode) {
        DriveMotor(pots[0]->htim, pots[0]->channel, pots[0]->pinDirPort, pots[0]->pinDirPin, open_loop_pwm[0]);
        DriveMotor(pots[1]->htim, pots[1]->channel, pots[1]->pinDirPort, pots[1]->pinDirPin, open_loop_pwm[1]);
        DriveMotor(enc_joints[0].htim, enc_joints[0].channel, enc_joints[0].dirPort, enc_joints[0].dirPin, open_loop_pwm[2]);
        DriveMotor(enc_joints[1].htim, enc_joints[1].channel, enc_joints[1].dirPort, enc_joints[1].dirPin, open_loop_pwm[3]);
        DriveMotor(enc_joints[2].htim, enc_joints[2].channel, enc_joints[2].dirPort, enc_joints[2].dirPin, open_loop_pwm[4]);
        DriveMotor(enc_joints[3].htim, enc_joints[3].channel, enc_joints[3].dirPort, enc_joints[3].dirPin, open_loop_pwm[5]);
    } else {
        if (enc_enabled[0]) {
            DriveMotor(enc_joints[0].htim, enc_joints[0].channel, enc_joints[0].dirPort, enc_joints[0].dirPin, pid_output[0] * M3_PID_SIGN);
        } else {
            DriveMotor(enc_joints[0].htim, enc_joints[0].channel, enc_joints[0].dirPort, enc_joints[0].dirPin, 0);
        }
        
        bool pitch_active = enc_enabled[1] && sensor_read_ok[1];
        bool roll_active  = enc_enabled[2] && sensor_read_ok[2];
        if (pitch_active || roll_active) {
            float pitch_cmd = pitch_active ? (pid_output[1] * PITCH_PID_SIGN) : 0.0f;
            
            // RAW ENCODER DEAD-ZONE SAFETY for pitch:
            // If the raw encoder is in the dead zone (between endpoints), kill pitch.
            if (pitch_active) {
                int pitch_raw = debug_raw_encoders[1];
                if (pitch_raw > PITCH_RAW_MIN && pitch_raw < PITCH_RAW_MAX) {
                    pitch_cmd = 0;  // In dead zone = beyond physical limits
                }
            }
            
            // PITCH FREEZE: if corrected position exceeds limits by >5°, freeze completely
            if (pitch_active && !pitch_frozen) {
                if (current_pos[1] < (PITCH_LIMIT_MIN - PITCH_FREEZE_TOLERANCE) ||
                    current_pos[1] > (PITCH_LIMIT_MAX + PITCH_FREEZE_TOLERANCE)) {
                    pitch_frozen = true;
                    pitch_cmd = 0;
                }
            }
            if (pitch_frozen) pitch_cmd = 0;
            float roll_cmd  = roll_active  ? (pid_output[2] * ROLL_PID_SIGN) : 0.0f;
            float mA = pitch_cmd + roll_cmd;
            float mB = -pitch_cmd + roll_cmd;
            float max_val = fmaxf(fabsf(mA), fabsf(mB));
            if (max_val > 255.0f) { float s = 255.0f / max_val; mA *= s; mB *= s; }
            DriveMotor(enc_joints[1].htim, enc_joints[1].channel, enc_joints[1].dirPort, enc_joints[1].dirPin, mA * WRIST_MOTOR_A_SIGN);
            DriveMotor(enc_joints[2].htim, enc_joints[2].channel, enc_joints[2].dirPort, enc_joints[2].dirPin, mB * WRIST_MOTOR_B_SIGN);
        } else {
            DriveMotor(enc_joints[1].htim, enc_joints[1].channel, enc_joints[1].dirPort, enc_joints[1].dirPin, 0);
            DriveMotor(enc_joints[2].htim, enc_joints[2].channel, enc_joints[2].dirPort, enc_joints[2].dirPin, 0);
        }
        
        if (enc_enabled[3]) {
            DriveMotor(enc_joints[3].htim, enc_joints[3].channel, enc_joints[3].dirPort, enc_joints[3].dirPin, pid_output[3] * ZA_PID_SIGN);
        } else {
            DriveMotor(enc_joints[3].htim, enc_joints[3].channel, enc_joints[3].dirPort, enc_joints[3].dirPin, 0);
        }
    }

    // 7. Transmit Feedback
    if (current_cmd.header[0] == 'D' || current_cmd.header[0] == 'P') {
        current_fb.motor_pos[0] = (int32_t)debug_raw_adcs[0];
        current_fb.motor_pos[1] = (int32_t)debug_raw_adcs[1];
        current_fb.motor_pos[2] = (int32_t)debug_raw_encoders[0];
        current_fb.motor_pos[3] = (int32_t)debug_raw_encoders[1];
        current_fb.motor_pos[4] = (int32_t)debug_raw_encoders[2];
        current_fb.motor_pos[5] = (int32_t)debug_raw_encoders[3];
        // In debug mode, velocities are zero
        memset(current_fb.motor_vel, 0, sizeof(current_fb.motor_vel));
    } else {
        current_fb.motor_pos[0] = (int32_t)(pots[0]->currentAngle * 1000.0f);
        current_fb.motor_pos[1] = (int32_t)(pots[1]->currentAngle * 1000.0f);
        
        // Convert continuous hardware steps back to Output Degrees (* 1000)
        current_fb.motor_pos[2] = (int32_t)(((m3_continuous_pos / 4096.0f) * 360.0f / M3_GEAR_RATIO) * 1000.0f); 
        // Pitch feedback: inverted formula (positive degrees = decreasing corrected)
        if (pitch_frozen) {
            current_fb.motor_pos[3] = SENTINEL_VAL;
        } else {
            current_fb.motor_pos[3] = (int32_t)(((float)PITCH_HOME_STEPS - current_pos[1]) / (float)(PITCH_LIMIT_MAX - PITCH_LIMIT_MIN) * PITCH_RANGE_DEG * 1000.0f);
        }
        current_fb.motor_pos[4] = (int32_t)(((current_pos[2] / 4096.0f) * ROLL_RANGE_DEG) * 1000.0f);
        if (z_collision_fault) {
            current_fb.motor_pos[5] = SENTINEL_VAL;
        } else {
            current_fb.motor_pos[5] = (int32_t)(((za_continuous_pos / 4096.0f) * 360.0f / ZA_GEAR_RATIO) * 1000.0f);
        }

        // ── Velocities (milli-deg/s) — uses dedicated fb_prev_* state ─────────
        const float DT = 0.02f;  // 50 Hz tick

        // Pots: velocityCDS already in deg/s from PotMotor::update()
        current_fb.motor_vel[0] = (int32_t)(pots[0]->velocityCDS * 1000.0f);
        current_fb.motor_vel[1] = (int32_t)(pots[1]->velocityCDS * 1000.0f);

        // M3: continuous position → output deg/s
        fb_vel_m3 = ((m3_continuous_pos - fb_prev_m3) / 4096.0f * 360.0f / M3_GEAR_RATIO) / DT;
        fb_prev_m3 = m3_continuous_pos;
        current_fb.motor_vel[2] = (int32_t)(fb_vel_m3 * 1000.0f);

        // Pitch: inverted axis, range-scaled (matches position formula)
        fb_vel_pitch = (-(current_pos[1] - fb_prev_pitch)
                        / (float)(PITCH_LIMIT_MAX - PITCH_LIMIT_MIN)
                        * PITCH_RANGE_DEG) / DT;
        fb_prev_pitch = current_pos[1];
        current_fb.motor_vel[3] = (int32_t)(fb_vel_pitch * 1000.0f);

        // Roll: wrap-aware delta → ROLL_RANGE_DEG
        fb_vel_roll = ((wrapError(current_pos[2] - fb_prev_roll) / 4096.0f) * ROLL_RANGE_DEG) / DT;
        fb_prev_roll = current_pos[2];
        current_fb.motor_vel[4] = (int32_t)(fb_vel_roll * 1000.0f);

        // Z: continuous position → output deg/s
        fb_vel_z = ((za_continuous_pos - fb_prev_z) / 4096.0f * 360.0f / ZA_GEAR_RATIO) / DT;
        fb_prev_z = za_continuous_pos;
        current_fb.motor_vel[5] = (int32_t)(fb_vel_z * 1000.0f);
    }

    // ── Flags byte ────────────────────────────────────────────────────────────
    current_fb.flags = 0;
    if (pitch_frozen)      current_fb.flags |= FB_FLAG_PITCH_FROZEN;
    if (z_collision_fault) current_fb.flags |= FB_FLAG_Z_COLLISION;

    // ── Checksum: XOR of bytes [0..50] ────────────────────────────────────────
    current_fb.checksum = compute_checksum(
        (const uint8_t*)&current_fb,
        offsetof(FeedbackPacketUnified, checksum));

    HAL_UART_Transmit(&huart4, (uint8_t*)&current_fb, sizeof(FeedbackPacketUnified), 10);
    CDC_Transmit_FS((uint8_t*)&current_fb, sizeof(FeedbackPacketUnified));
}

} // extern "C"