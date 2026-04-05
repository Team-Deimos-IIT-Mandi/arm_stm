// =============================================================================
//  JOINT CONTROL SYSTEM  —  ESP-A  (2 potentiometer motors)
//
//  Status: FIXED & SWAPPED
//  - Link 1 (ROS) now controls Motor 2 (Physical)
//  - Link 2 (ROS) now controls Motor 1 (Physical)
//  - Math Units Fixed: Millidegrees <-> Degrees
// =============================================================================

#include <Arduino.h>
#include <SimpleKalmanFilter.h>
#include <HardwareSerial.h>

// =============================================================================
//  ROS UART — PACKET STRUCTS
// =============================================================================

struct __attribute__((packed)) CommandPacketA {
    char    header[2];       // 'S', 'T'
    int32_t motor_cmd[2];    // millidegrees (int)
    char    footer;          // '\n'
};                           // 11 bytes total

struct __attribute__((packed)) FeedbackPacketA {
    char    header[2]    = {'F', 'B'};
    int32_t motor_pos[2];   // millidegrees (angle * 1000)
    int32_t motor_vel[2];   // millidegrees/s
    char    footer       = '\n';
};                           // 19 bytes total

// =============================================================================
//  ROS UART CONFIG
// =============================================================================
const int ROS_RX_PIN = 12;        
const int ROS_TX_PIN = 13;        
HardwareSerial ROSSerial(1);

// =============================================================================
//  WATCHDOG
// =============================================================================
const unsigned long COMMS_TIMEOUT_MS = 1000;
unsigned long last_valid_cmd_time    = 0;
bool comms_ok                        = false;

// =============================================================================
//  CALIBRATION DATA
// =============================================================================

// --- MOTOR 1  (-80 to 0 degrees) ---
const int PTS_M1 = 28;
const float angles_M1[PTS_M1] = {
  -80, -77, -74, -71, -68, -65, -62, -59, -56, -53,
  -50, -47, -44, -41, -38, -35, -32, -29, -26, -23,
  -20, -17, -14, -11, -8, -5, -2, 1
};
const int adcs_M1[PTS_M1] = {
  1065, 1135, 1230, 1310, 1370, 1440, 1485, 1565, 1620, 1675,
  1735, 1785, 1844, 1900, 1940, 2000, 2050, 2100, 2152, 2200,
  2262, 2310, 2360,2410 ,2460 ,2502 , 2540, 2600
};

// --- MOTOR 2  (18 to 78 degrees) ---
const int PTS_M2 = 18;
const float angles_M2[PTS_M2] = {
  78, 75, 72, 69, 66, 63, 60, 57, 54,
  51, 48, 45, 42, 39, 36, 33, 30, 27
};
const int adcs_M2[PTS_M2] = {
  1100, 1130, 1175, 1220, 1265, 1300, 1360, 1390, 1430,
  1465, 1500, 1540, 1575, 1630, 1675, 1750, 1790, 1830
};

// =============================================================================
//  PID TUNING & PHYSICS
// =============================================================================
float Kp[2] = {15.0f, 15.0f};
float Ki[2] = { 1.0f,  1.0f};
float Kd[2] = { 0.8f,  0.8f};

const int   MIN_POWER = 120;
const int   MAX_POWER = 255;
const float DEADZONE  = 1.0f;
const float MAX_SLEW_DEG = 2.0f; // Anti-jerk smoothing

const bool JOINT_ALLOWED[2] = {true, true};
int PID_SIGN[2] = {1, 1};

// =============================================================================
//  MOTOR CLASS
// =============================================================================

class Motor {
public:
    int pinPot, pinPWM, pinDir;
    const float* angleTable;
    const int* adcTable;
    int          numPoints;
    bool         reverseDir;
    int          motorIdx;

    SimpleKalmanFilter kf;
    float currentAngle, lastAngle, integral;
    int   currentPWM;
    unsigned long lastStallCheck;
    float         prevStallAngle;
    int           stallCounter;
    bool          stalled;
    bool          enabled;

    // Trajectory State
    float commandedAngle;
    float velocityCDS;

    Motor(int pot, int pwm, int dir,
          const float* angTab, const int* adcTab, int nPoints,
          bool rev, int idx)
      : pinPot(pot), pinPWM(pwm), pinDir(dir),
        angleTable(angTab), adcTable(adcTab), numPoints(nPoints),
        reverseDir(rev), motorIdx(idx),
        kf(2.0f, 2.0f, 0.01f),
        currentAngle(0), lastAngle(0), integral(0), currentPWM(0),
        lastStallCheck(0), prevStallAngle(0), stallCounter(0), stalled(false),
        enabled(false), commandedAngle(0), velocityCDS(0) {}

    void init() {
        pinMode(pinDir, OUTPUT);
        ledcAttach(pinPWM, 20000, 8);
        currentAngle   = adcToAngle(analogRead(pinPot));
        lastAngle      = currentAngle;
        commandedAngle = currentAngle;
        prevStallAngle = currentAngle;
    }

    float adcToAngle(int adc) {
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
                float t = (float)(adc - a0) / (float)(a1 - a0);
                return angleTable[i] + t * (angleTable[i + 1] - angleTable[i]);
            }
        }
        return angleTable[numPoints - 1];
    }

    void resetStall() {
        stalled        = false;
        stallCounter   = 0;
        prevStallAngle = currentAngle;
    }

    // --- SMOOTHING LOGIC ---
    void updateTrajectory(float target) {
        float err = target - commandedAngle;
        if (fabsf(err) > MAX_SLEW_DEG)
            commandedAngle += (err > 0.0f) ? MAX_SLEW_DEG : -MAX_SLEW_DEG;
        else
            commandedAngle = target;
    }

    void update(float target, float dt) {
        int rawADC = analogRead(pinPot);
        if (rawADC < 10 || rawADC > 4085) { ledcWrite(pinPWM, 0); return; }

        float filteredADC = kf.updateEstimate((float)rawADC);
        float prevAngle   = currentAngle;
        currentAngle      = adcToAngle((int)filteredADC);
        velocityCDS = ((currentAngle - prevAngle) / dt) * 100.0f;

        if (stalled || !enabled) { ledcWrite(pinPWM, 0); return; }

        // 1. Move commanded angle smoothly towards target
        updateTrajectory(target);

        // 2. PID chases the SMOOTH commanded angle
        float error = commandedAngle - currentAngle;

        if (fabsf(error) > DEADZONE) {
            // Stall Check
            if (millis() - lastStallCheck > 100) {
                if (fabsf(currentAngle - prevStallAngle) < 0.1f) stallCounter++;
                else stallCounter = 0;
                prevStallAngle = currentAngle;
                lastStallCheck = millis();
                if (stallCounter > 20) {
                    stalled = true;
                    ledcWrite(pinPWM, 0);
                    return;
                }
            }
            // PID Calculation
            float P         = Kp[motorIdx] * error;
            integral        = constrain(integral + (error * Ki[motorIdx]) * dt, -100.0f, 100.0f);
            float D         = -Kd[motorIdx] * ((currentAngle - lastAngle) / dt);
            float rawOutput = (P + integral + D) * PID_SIGN[motorIdx];

            float clampedOutput = constrain(rawOutput, -255.0f, 255.0f);
            int   pwm           = map((long)fabsf(clampedOutput), 0, 255, MIN_POWER, MAX_POWER);

            bool dir = (clampedOutput > 0);
            if (reverseDir) dir = !dir;
            digitalWrite(pinDir, dir ? LOW : HIGH);
            ledcWrite(pinPWM, pwm);

            currentPWM = pwm;
            lastAngle  = currentAngle;
        } else {
            ledcWrite(pinPWM, 0);
            integral   = 0;
            currentPWM = 0;
            resetStall();
        }
    }
};

Motor motor1(4, 6, 7, angles_M1, adcs_M1, PTS_M1, false, 0);
Motor motor2(5, 8, 9, angles_M2, adcs_M2, PTS_M2, false, 1);
Motor* motors[2] = {&motor1, &motor2};

float target1        = 0.0f;
float target2        = 0.0f;
bool  systemActive   = false;
bool  manualTestMode = false;
unsigned long lastLoopTime  = 0;
unsigned long lastDebugTime = 0;
String        inputBuffer   = "";
bool debugShow[2] = {true, true};

const char* motorName(int i) { return (i == 0) ? "M1" : "M2"; }

// =============================================================================
//  ROS UART LOGIC (WITH SWAP)
// =============================================================================

bool tryReadPacketA(CommandPacketA& cmd) {
    while (ROSSerial.available() >= (int)sizeof(CommandPacketA)) {
        if (ROSSerial.peek() != 'S') {
            ROSSerial.read(); 
            continue;
        }
        uint8_t buf[sizeof(CommandPacketA)];
        ROSSerial.readBytes((char*)buf, sizeof(CommandPacketA));
        memcpy(&cmd, buf, sizeof(CommandPacketA));

        if (cmd.header[0] == 'S' && cmd.header[1] == 'T' && cmd.footer == '\n') {
            return true;
        }
    }
    return false;
}

void processROSCommand() {
    CommandPacketA cmd;
    bool got_any = false;

    // Drain buffer to get the freshest command
    while (tryReadPacketA(cmd)) {
        got_any = true;
    }
    if (!got_any) return;

    last_valid_cmd_time = millis();

    // 1. AUTO-RESET STALL ON NEW COMMAND
    // Every time ROS sends a new packet, we "forgive" any previous stalls
    for (int i = 0; i < 2; i++) {
        if (motors[i]->stalled) {
            motors[i]->resetStall();
            // Optional: Serial.printf("ROS Reset: Motor %d unblocked\n", i+1);
        }
    }

    // 2. Re-enable safely if comms were previously lost
    if (!comms_ok) {
        comms_ok = true;
        Serial.println("ROS: connected — motors re-enabled.");
        for (int i = 0; i < 2; i++) {
            if (!JOINT_ALLOWED[i]) continue;
            int rawADC = analogRead(motors[i]->pinPot);
            motors[i]->currentAngle   = motors[i]->adcToAngle(rawADC);
            motors[i]->commandedAngle = motors[i]->currentAngle; 
            motors[i]->integral       = 0;
            motors[i]->enabled        = true;
            // resetStall already called above, but good for safety
            motors[i]->resetStall();
        }
        target1 = motor1.currentAngle;
        target2 = motor2.currentAngle;
        systemActive = true;
    }

    // 3. Convert incoming Millidegrees to Degrees
    float raw_link1 = (float)cmd.motor_cmd[0] / 1000.0f;
    float raw_link2 = (float)cmd.motor_cmd[1] / 1000.0f;

    // 4. SWAP MAPPING & Constraints
    target2 = constrain(raw_link1, 27.0f, 78.0f);
    target1 = constrain(raw_link2, -80.0f, -15.0f);

    // 5. SEND FEEDBACK
    FeedbackPacketA fb;
    fb.motor_pos[0] = (int32_t)(motor2.currentAngle * 1000.0f);
    fb.motor_vel[0] = (int32_t)(motor2.velocityCDS * 10.0f);
    fb.motor_pos[1] = (int32_t)(motor1.currentAngle * 1000.0f);
    fb.motor_vel[1] = (int32_t)(motor1.velocityCDS * 10.0f);
    
    ROSSerial.write((uint8_t*)&fb, sizeof(FeedbackPacketA));
}



void checkCommsWatchdog() {
    if (!comms_ok) return;
    if ((millis() - last_valid_cmd_time) > COMMS_TIMEOUT_MS) {
        comms_ok     = false;
        systemActive = false;
        for (int i = 0; i < 2; i++) {
            motors[i]->enabled = false;
            ledcWrite(motors[i]->pinPWM, 0);
            motors[i]->integral = 0;
        }
        Serial.println("ROS WATCHDOG: comms lost — motors stopped.");
    }
}

void emergencyStopAll() {
    manualTestMode = false;
    systemActive   = false;
    comms_ok       = false;
    for (int i = 0; i < 2; i++) {
        motors[i]->enabled = false;
        ledcWrite(motors[i]->pinPWM, 0);
    }
    Serial.println("!!! E-STOP !!!");
}

// =============================================================================
//  USB SERIAL COMMANDS
// =============================================================================

void processSerialCommand() {
    if (!Serial.available()) return;
    char type = Serial.peek();
    
    char cmd = Serial.read(); 
    switch (cmd) {
        case 'A': case 'a': {
            float a1 = Serial.parseFloat(); float a2 = Serial.parseFloat();
            if (JOINT_ALLOWED[0]) target1 = constrain(a1, -80.0f, -15.0f);
            if (JOINT_ALLOWED[1]) target2 = constrain(a2, 27.0f, 78.0f);
            Serial.printf("CMD ALL -> M1:%.2f M2:%.2f\n", target1, target2);
            break;
        }
        case '1': {
            float deg = Serial.parseFloat();
            if (JOINT_ALLOWED[0]) target1 = constrain(deg, -80.0f, -15.0f);
            if (motor1.enabled) systemActive = true;
            Serial.printf("CMD M1: -> %.2f deg\n", target1);
            break;
        }
        case '2': {
            float deg = Serial.parseFloat();
            if (JOINT_ALLOWED[1]) target2 = constrain(deg, 27.0f, 78.0f);
            if (motor2.enabled) systemActive = true;
            Serial.printf("CMD M2: -> %.2f deg\n", target2);
            break;
        }
        case 'P': case 'p': {
            int id = Serial.parseInt(); float val = Serial.parseFloat();
            if (id>=1 && id<=2) { Kp[id-1] = val; Serial.printf("Kp[M%d]=%.3f\n", id, val); }
            break;
        }
        case 'I': case 'i': {
            int id = Serial.parseInt(); float val = Serial.parseFloat();
            if (id>=1 && id<=2) { Ki[id-1] = val; Serial.printf("Ki[M%d]=%.3f\n", id, val); }
            break;
        }
        case 'D': case 'd': {
            int id = Serial.parseInt(); float val = Serial.parseFloat();
            if (id>=1 && id<=2) { Kd[id-1] = val; Serial.printf("Kd[M%d]=%.3f\n", id, val); }
            break;
        }
        case 'T': case 't': {
            char sub = Serial.read();
            while(sub==' ') sub=Serial.read();
            if(sub=='1' || sub=='2') {
                int mIdx = sub - '1'; int pwm = Serial.parseInt();
                manualTestMode=true; systemActive=false;
                for(int j=0;j<2;j++) { motors[j]->enabled=false; ledcWrite(motors[j]->pinPWM, 0); }
                Serial.printf("TEST M%d PWM=%d\n", mIdx+1, pwm);
                int speed = constrain(abs(pwm), 0, 255);
                digitalWrite(motors[mIdx]->pinDir, (pwm>=0)?LOW:HIGH);
                ledcWrite(motors[mIdx]->pinPWM, speed);
                unsigned long ts = millis(); while(millis()-ts<1000) delay(10);
                ledcWrite(motors[mIdx]->pinPWM, 0);
            } else if(sub=='X'||sub=='x') {
                manualTestMode=false; Serial.println("Test mode OFF.");
            }
            break;
        }
        case 'F': case 'f': {
            int id = Serial.parseInt();
            if(id>=1 && id<=2) { PID_SIGN[id-1]*=-1; Serial.printf("Sign[M%d]=%d\n", id, PID_SIGN[id-1]); }
            break;
        }
        case 'E': case 'e': {
            Serial.printf("M1: %.2f (cmd %.2f)  M2: %.2f (cmd %.2f)\n", 
                motor1.currentAngle, motor1.commandedAngle, motor2.currentAngle, motor2.commandedAngle);
            break;
        }
        case 'S': case 's': emergencyStopAll(); break;
        case 'R': case 'r': 
            manualTestMode=false;
            for(int j=0; j<2; j++) {
                if(!JOINT_ALLOWED[j]) continue;
                int raw = analogRead(motors[j]->pinPot);
                motors[j]->currentAngle = motors[j]->adcToAngle(raw);
                motors[j]->commandedAngle = motors[j]->currentAngle;
                motors[j]->enabled = true;
                motors[j]->integral = 0;
            }
            target1 = motor1.currentAngle; target2 = motor2.currentAngle;
            Serial.println("Re-enabled.");
            break;
    }
}

// =============================================================================
//  SETUP & LOOP
// =============================================================================

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    ROSSerial.begin(115200, SERIAL_8N1, ROS_RX_PIN, ROS_TX_PIN);

    motor1.init();
    motor2.init();
    target1 = motor1.currentAngle;
    target2 = motor2.currentAngle;
    last_valid_cmd_time = millis();

    Serial.println("ESP-A (Potentiometer Motors) ONLINE");
    Serial.println("Waiting for ROS or USB command...");
}

void loop() {
    // 1. Safety & Comms
    checkCommsWatchdog();
    processROSCommand();

    // 2. USB Routing
    while (Serial.available()) {
        char p = Serial.peek();
        if (p == '-' || (p >= '0' && p <= '9') || p == ',' || p == '.' || p == '\n' || p == '\r') {
             char c = Serial.read();
             if (c == '\n') {
                 if (inputBuffer.length() > 0) {
                     int comma = inputBuffer.indexOf(',');
                     if (comma > 0) {
                         target1 = constrain(inputBuffer.substring(0, comma).toFloat(), -80, -15);
                         target2 = constrain(inputBuffer.substring(comma+1).toFloat(), 27, 78);
                         if(!systemActive) systemActive = true;
                         Serial.printf("CSV: %.2f, %.2f\n", target1, target2);
                     }
                 }
                 inputBuffer = "";
             } else if (c != '\r') {
                 inputBuffer += c;
             }
        } 
        else {
             processSerialCommand(); 
        }
    }

    // 3. Control Loop (50Hz)
    unsigned long now = millis();
    if (!manualTestMode && (now - lastLoopTime >= 20)) {
        float dt = (now - lastLoopTime) / 1000.0f;
        if (dt > 0.1f) dt = 0.02f;
        lastLoopTime = now;

        motor1.update(target1, dt);
        motor2.update(target2, dt);
    }
    
    // 4. Debug Prints (200ms)
    if (now - lastDebugTime > 200) {
        lastDebugTime = now;
        if (!manualTestMode && debugShow[0]) {
             Serial.printf("%.2f,%.2f,%d,%.2f,%.2f,%d\n", 
                target1, motor1.currentAngle, (int)motor1.stalled,
                target2, motor2.currentAngle, (int)motor2.stalled);
        }
    }
}