#ifndef ROBOTCORE_H
#define ROBOTCORE_H

#ifdef __cplusplus
#include <cstdint>
#include <cmath>
#include "main.h"

// =============================================================================
//  ROS UART — UNIFIED PACKET STRUCTS
//  Wire layout MUST match protocol.hpp on the ROS2 side exactly.
// =============================================================================
struct __attribute__((packed)) CommandPacketUnified {
    char    header[2];       // 'S','T' (position) | 'P','T' (estop) | 'D','T' | 'K','T' | 'R','T'
    int32_t motor_cmd[6];    // [0]M1, [1]M2, [2]M3, [3]Pitch, [4]Roll, [5]Z  (milli-degrees)
    uint8_t flags;           // bit0=ESTOP
    uint8_t checksum;        // XOR of bytes [0..26]
    char    footer;          // '\n'
};
_Static_assert(sizeof(CommandPacketUnified) == 29, "CommandPacketUnified size mismatch");

struct __attribute__((packed)) FeedbackPacketUnified {
    char    header[2];       // 'F', 'B'
    int32_t motor_pos[6];    // milli-degrees (or 999999 on fault)
    int32_t motor_vel[6];    // milli-deg/s
    uint8_t flags;           // bit0=pitch_frozen, bit1=z_collision
    uint8_t checksum;        // XOR of bytes [0..50]
};
_Static_assert(sizeof(FeedbackPacketUnified) == 52, "FeedbackPacketUnified size mismatch");

// Flag bits (match protocol.hpp on ROS2 side)
#define CMD_FLAG_ESTOP          (1u << 0)
#define FB_FLAG_PITCH_FROZEN    (1u << 0)
#define FB_FLAG_Z_COLLISION     (1u << 1)
#define SENTINEL_VAL            999999

// =============================================================================
//  EMA FILTER (Exponential Moving Average)
//  alpha: 0.0 = maximum smoothing (infinite lag), 1.0 = no smoothing
//  At 50Hz, alpha=0.2 gives ~90ms settling time — fast enough for motor
//  control, slow enough to crush ADC noise by ~5x.
// =============================================================================
class EMAFilter {
private:
    float alpha;
    float estimate;
    bool  initialized;
public:
    EMAFilter(float a) : alpha(a), estimate(0), initialized(false) {}

    float updateEstimate(float mea) {
        if (!initialized) { estimate = mea; initialized = true; }
        estimate = alpha * mea + (1.0f - alpha) * estimate;
        return estimate;
    }

    void setEstimate(float est) {
        estimate = est;
        initialized = true;
    }

    float getEstimate() const { return estimate; }
};

// =============================================================================
//  POTENTIOMETER MOTOR CLASS
// =============================================================================
class PotMotor {
public:
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    GPIO_TypeDef* pinDirPort;
    uint16_t pinDirPin;
    
    const float* angleTable;
    const int* adcTable;
    int          numPoints;
    bool         reverseDir;
    int          motorIdx;

    EMAFilter ema;
    float currentAngle, lastAngle, integral;
    float lastValidAngle;   // Deadband: last angle that passed the threshold
    float angleDead;        // Deadband threshold in degrees (default 0.5°)
    int   currentPWM;
    uint32_t lastStallCheck;
    float         prevStallAngle;
    int           stallCounter;
    bool          stalled;
    bool          enabled;

    // Trajectory State
    float commandedAngle;
    float velocityCDS;
    
    float kp, ki, kd;
    int pid_sign;

    PotMotor(TIM_HandleTypeDef* t, uint32_t ch, GPIO_TypeDef* dirPort, uint16_t dirPin,
             const float* angTab, const int* adcTab, int nPoints,
             bool rev, int idx, float P, float I, float D, int sign)
      : htim(t), channel(ch), pinDirPort(dirPort), pinDirPin(dirPin),
        angleTable(angTab), adcTable(adcTab), numPoints(nPoints),
        reverseDir(rev), motorIdx(idx),
        ema(0.2f),
        currentAngle(0), lastAngle(0), integral(0),
        lastValidAngle(0), angleDead(0.5f),
        currentPWM(0),
        lastStallCheck(0), prevStallAngle(0), stallCounter(0), stalled(false),
        enabled(false), commandedAngle(0), velocityCDS(0),
        kp(P), ki(I), kd(D), pid_sign(sign) {}

    void init(int initialADC);
    float adcToAngle(int adc);
    void resetStall();
    void updateTrajectory(float target);
    void update(float target, float dt, int rawADC);
};

#endif

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
//  C-BRIDGES
// =============================================================================
void RobotCore_Init(void);
void RobotCore_ROSTask(void);
void RobotCore_PIDTask(void);
uint8_t compute_checksum(const uint8_t* data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // ROBOTCORE_H