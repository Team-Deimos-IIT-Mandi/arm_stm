#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

/* =============================================================================
 *  ROBOT ARM CONFIGURATION
 *  Pure C header (no constexpr). Use this file to tweak calibration,
 *  PID gains, and safety limits.
 * ============================================================================= */

/* =============================================================================
 *  ESP-A (Potentiometer Motors M1 & M2)
 * ============================================================================= */
/* M1 (Shoulder / Link-1): 47.4° to 74.0°  — 20 calibration points */
#define PTS_M1 20
static const float angles_M1[PTS_M1] = { 47.4f, 52.5f, 53.1f, 56.2f, 57.3f, 58.0f, 59.5f, 60.0f, 61.0f, 61.0f, 62.8f, 63.0f, 65.5f, 65.5f, 65.5f, 66.0f, 71.0f, 71.8f, 72.0f, 74.0f };
static const int   adcs_M1[PTS_M1]   = { 2689, 2629, 2616, 2581, 2567, 2553, 2532, 2528, 2503, 2510, 2491, 2492, 2439, 2440, 2454, 2438, 2363, 2347, 2346, 2328 };

/* M2 (Elbow / Link-2): 1.0° to 61.2°  — 26 calibration points */
#define PTS_M2 26
static const float angles_M2[PTS_M2] = { 1.0f, 3.4f, 4.5f, 6.5f, 8.3f, 10.0f, 11.7f, 13.0f, 15.0f, 17.0f, 18.8f, 20.5f, 22.7f, 24.8f, 27.4f, 29.0f, 34.2f, 38.0f, 41.5f, 43.0f, 45.5f, 47.8f, 50.4f, 54.0f, 57.4f, 61.2f };
static const int   adcs_M2[PTS_M2]   = { 2997, 2957, 2958, 2932, 2903, 2877, 2846, 2826, 2791, 2754, 2729, 2698, 2664, 2630, 2576, 2553, 2398, 2389, 2335, 2299, 2256, 2212, 2166, 2100, 2032, 1957 };

/* =============================================================================
 *  ESP-B (I2C Encoder Motors)
 *  [0] = M3 | [1] = Pitch | [2] = Roll | [3] = Z-Axis
 * ============================================================================= */

/* Enable/Disable individual joints */
static const int JOINT_ALLOWED[4] = {1, 1, 1, 1};

/* Encoder Home Positions (0 to 4095) */
static const int HOME_OFFSETS[4] = {3857, 2574, 733, 236};

/* Hard Encoder Limits (corrected encoder steps)
 * NOTE: For pitch, LIMIT_MIN is the max-extension end, LIMIT_MAX is home/min end
 *       because travel crosses zero → corrected values go 1123 → 2972 */
static const int POS_MIN[4] = {0,    1123, 0,    0};
static const int POS_MAX[4] = {4096, 2915, 4096, 4096};
#define PITCH_LIMIT_MIN  1123   /* Corrected value at raw 3697 (max extension) */
#define PITCH_LIMIT_MAX  2915   /* Corrected value ~5° before raw 1450 (home buffer) */

/* Raw encoder endpoint limits for pitch
 * Dead zone = raw 1451 to raw 3696 (arm cannot physically reach here) */
#define PITCH_RAW_MIN    1450   /* Home endpoint */
#define PITCH_RAW_MAX    3697   /* Max extension endpoint */

/* Degrees-to-encoder range mappings */
#define PITCH_RANGE_DEG  ((1849.0f / 4096.0f) * 360.0f)  /* = 162.5 degrees */
#define ROLL_RANGE_DEG   360.0f

/* Pitch home (0° on slider) in corrected encoder steps
 * 0° = home (corrected 2972), positive degrees = arm extends toward corrected 1123 */
#define PITCH_HOME_STEPS 2972

/* Pitch freeze: if current position exceeds limits by this many steps (~5°), freeze motor */
#define PITCH_FREEZE_TOLERANCE 57

/* Encoder direction: +1 = counts UP with joint movement, -1 = counts DOWN */
static const int ENCODER_DIRECTION[4] = {1, 1, 1, 1};

/* Gear Ratios and Shaft Output Limits */
#define M3_GEAR_RATIO    2.0f
#define M3_OUTPUT_LIMIT  180.0f    /* +/- 180 deg output shaft */

#define ZA_GEAR_RATIO       2.7272f
#define ZA_OUTPUT_LIMIT     120.0f  /* +/- 120 deg output shaft */
#define ZA_NORMAL_LIMIT_DEG  60.0f  /* +/- 60 deg normal speed zone */
#define ZA_DOCKING_SCALE      0.25f /* 25% speed outside normal zone */
#define ZA_PID_OUTPUT_MAX   127.0f  /* Cap Z-axis motor speed (0-255) */

/* Virtual Current Sensing / Collision Detection for Z-Axis */
#define ZA_COLLISION_ENABLED   1     /* Master switch — set 1 to enable */
#define ZA_COLLISION_INTEGRAL  80.0f /* Tune 1: Max integral before tripping (0-100) */
#define ZA_COLLISION_VELOCITY  0.3f  /* Tune 2: Speed below this is considered "stuck" */
#define ZA_COLLISION_TIMEOUT   20    /* Tune 3: 25 ticks @ 50Hz = 500ms delay */

/* I2C Multiplexer Wiring (TCA9548A Channels) */
static const int MUX_CHANNELS[4] = {5, 0, 1, 2};

/* =============================================================================
 *  PID TUNING (ESP-B Motors)
 * ============================================================================= */
static const float Kp_enc[4] = {1.0f,  2.0f,  2.0f,  1.0f};
static const float Ki_enc[4] = {0.05f, 0.02f, 0.02f, 0.05f};
static const float Kd_enc[4] = {0.05f, 0.01f, 0.01f, 0.1f};

/* PID Constraints / Physical Protection */
#define PID_OUTPUT_MAX   255.0f
#define INTEGRAL_MAX     100.0f
#define ERROR_DEADZONE   1.0f    /* Stop seeking when within 1.0 step */
#define MAX_SENSOR_FAILS 10
#define MAX_SLEW_RATE    40.0f   /* Control signal smoothing */

/* Direction Signs (+1 or -1) */
#define M3_PID_SIGN           (-1)
#define ZA_PID_SIGN           1
#define WRIST_MOTOR_A_SIGN    1
#define WRIST_MOTOR_B_SIGN    1
#define PITCH_PID_SIGN        1
#define ROLL_PID_SIGN         1

/* Multi-turn wrap tracking constraints (Steps) */
#define M3_CONT_LIMIT     4096.0f
#define M3_CONT_SOFT_ZONE  400.0f

#define ZA_CONT_LIMIT     4096.0f
#define ZA_CONT_SOFT_ZONE  400.0f

/* =============================================================================
 *  TRAPEZOIDAL MOTION PROFILE (Z-Axis and M3)
 *  Units: encoder steps per PID tick (50Hz → 1 tick = 20ms)
 * ============================================================================= */
#define ZA_PROF_MAX_VEL    110.0f  /* steps/tick  ≈ 30°/s output shaft */
#define ZA_PROF_MAX_ACCEL    6.0f  /* steps/tick² ≈ 0.5s to full speed */

#define M3_PROF_MAX_VEL    150.0f  /* steps/tick  ≈ 41°/s output shaft */
#define M3_PROF_MAX_ACCEL    8.0f  /* steps/tick² */

#endif /* ROBOTCONFIG_H */
