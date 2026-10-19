// ============================================================
//  Agatha Robot — Arduino Nano Firmware
//  Serial protocol (115200 baud, GPIO TX/RX on Pi):
//    Pi  → Arduino : "V<left_rad_s> <right_rad_s>\n"
//    Arduino → Pi  : "E<left_ticks> <right_ticks>\n"  (50 Hz)
// ============================================================

// ── Robot constants ─────────────────────────────────────────
static constexpr float WHEEL_RADIUS   = 0.068f;  // metres
static constexpr float WHEEL_SEP      = 0.198f;  // metres (L)
static constexpr long  TICKS_PER_REV  = 1980L;   // encoder ticks/revolution

// ── Motor driver pins (L298N or equivalent) ─────────────────
static constexpr uint8_t LEFT_IN1  = 5;   // PWM
static constexpr uint8_t LEFT_IN2  = 6;   // PWM
static constexpr uint8_t RIGHT_IN1 = 9;   // PWM
static constexpr uint8_t RIGHT_IN2 = 10;  // PWM

// ── Encoder pins ────────────────────────────────────────────
static constexpr uint8_t LEFT_ENC_A  = 2;  // interrupt
static constexpr uint8_t LEFT_ENC_B  = 4;
static constexpr uint8_t RIGHT_ENC_A = 3;  // interrupt
static constexpr uint8_t RIGHT_ENC_B = 7;

// ── Ultrasonic pins ─────────────────────────────────────────
static constexpr uint8_t NUM_US        = 4;
static constexpr uint8_t TRIG_PINS[4]  = {8, 11, 12, 13};
static constexpr uint8_t ECHO_PINS[4]  = {A0, A1, A2, A3};

// ── Timing ──────────────────────────────────────────────────
static constexpr unsigned long CONTROL_PERIOD_MS  = 20;   // 50 Hz control loop
static constexpr unsigned long ODOM_PERIOD_MS     = 20;   // 50 Hz odometry publish
static constexpr unsigned long US_PERIOD_MS       = 50;   // 20 Hz ultrasonic scan
static constexpr unsigned long CMD_TIMEOUT_MS     = 500;  // stop if no cmd received

// ── PID gains (tune these) ───────────────────────────────────
static constexpr float KP = 25.0f;
static constexpr float KI = 40.0f;
static constexpr float KD = 0.3f;

// ── Encoder state (volatile — written in ISR) ────────────────
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

// ── Control state ────────────────────────────────────────────
struct WheelController {
  float targetRadS  = 0.0f;
  float measuredRadS= 0.0f;
  float integral    = 0.0f;
  float prevError   = 0.0f;
  long  lastTicks   = 0;
  int   pwm         = 0;

  void reset() {
    targetRadS = measuredRadS = integral = prevError = 0.0f;
    pwm = 0;
  }
};

WheelController leftWheel;
WheelController rightWheel;

// ── Timing state ─────────────────────────────────────────────
unsigned long lastControlMs  = 0;
unsigned long lastOdomMs     = 0;
unsigned long lastUsMs       = 0;
unsigned long lastCmdMs      = 0;

// ── Ultrasonic state ─────────────────────────────────────────
float usDistances[NUM_US] = {0};
uint8_t usCurrent = 0;

enum UsState { US_IDLE, US_TRIG, US_WAIT_HIGH, US_WAIT_LOW };
UsState usState = US_IDLE;
unsigned long usTrigTime = 0;
unsigned long usEchoStart = 0;

// ── Serial parse buffer ──────────────────────────────────────
static constexpr uint8_t SERIAL_BUF_LEN = 40;
char serialBuf[SERIAL_BUF_LEN];
uint8_t serialIdx = 0;

// ============================================================
//  ISRs
// ============================================================
void onLeftEnc() {
  if (digitalRead(LEFT_ENC_A) != digitalRead(LEFT_ENC_B))
    leftTicks++;
  else
    leftTicks--;
}

void onRightEnc() {
  if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B))
    rightTicks++;
  else
    rightTicks--;
}

// ============================================================
//  Motor drive helpers
// ============================================================
void driveMotor(uint8_t pin1, uint8_t pin2, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWrite(pin1, pwm);
    analogWrite(pin2, 0);
  } else if (pwm < 0) {
    analogWrite(pin1, 0);
    analogWrite(pin2, -pwm);
  } else {
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
  }
}

void stopMotors() {
  driveMotor(LEFT_IN1,  LEFT_IN2,  0);
  driveMotor(RIGHT_IN1, RIGHT_IN2, 0);
}

// ============================================================
//  PID — called every CONTROL_PERIOD_MS
// ============================================================
void runPID(WheelController &wc, long currentTicks, float dtSec) {
  if (fabsf(wc.targetRadS) < 0.01f) {
      wc.lastTicks = currentTicks;
      wc.pwm = 0;
      wc.integral = 0.0f;
      return;
  }

  long deltaTicks = currentTicks - wc.lastTicks;
  wc.lastTicks    = currentTicks;
  wc.measuredRadS = (float(deltaTicks) / float(TICKS_PER_REV))
                    * (2.0f * PI) / dtSec;

  float error     = wc.targetRadS - wc.measuredRadS;
  wc.integral    += error * dtSec;
  wc.integral     = constrain(wc.integral, -10.0f, 10.0f);
  float derivative= (error - wc.prevError) / dtSec;
  wc.prevError    = error;

  float ff = wc.targetRadS * 25.5f;
  wc.pwm   = int(ff + KP * error + KI * wc.integral + KD * derivative);
  wc.pwm   = constrain(wc.pwm, -255, 255);

  // DEBUG — print every 500ms so we don't flood serial
  static unsigned long lastDebugMs = 0;
  if (millis() - lastDebugMs > 500) {
    lastDebugMs = millis();
    Serial.print("DBG ff="); Serial.print(ff);
    Serial.print(" err=");   Serial.print(error);
    Serial.print(" pwm=");   Serial.println(wc.pwm);
  }
}
// ============================================================
//  Odometry publish  (sent to Pi at 50 Hz)
// ============================================================
void publishOdom() {
  // Snapshot ticks atomically
  Serial.println("ALIVE");
  noInterrupts();
  long lt = leftTicks;
  long rt = rightTicks;
  interrupts();

  // Format: "E<left_ticks> <right_ticks>\n"
  Serial.print('E');
  Serial.print(lt);
  Serial.print(' ');
  Serial.println(rt);
}

// ============================================================
//  Serial command parser
//  Expects: "V<left_rad_s> <right_rad_s>\n"
// ============================================================
void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // 1. Tell the Pi EXACTLY what was received
    Serial.print("RAW_CHAR:[");
    Serial.print(c);
    Serial.println("]");

    if (c == 'V') {
      serialIdx = 0;
      serialBuf[serialIdx++] = c;
    } 
    else if (c == '\n' || c == '\r') {
      if (serialIdx > 0) {
        serialBuf[serialIdx] = '\0';
        
        // 2. Confirm the full string before parsing
        Serial.print("BUFFER_READY:[");
        Serial.print(serialBuf);
        Serial.println("]");
        
        parseCommand(serialBuf);
        serialIdx = 0;
      }
    } 
    else if (serialIdx < SERIAL_BUF_LEN - 1) {
      serialBuf[serialIdx++] = c;
    }
  }
}

void parseCommand(const char *line) {
  // Check if it starts with V
  if (line[0] != 'V') {
    Serial.println("PARSE_FAIL: Not a V command");
    return;
  }

  char* firstSpace = strchr(line, ' ');
  if (firstSpace == nullptr) {
    Serial.println("PARSE_FAIL: No space found");
    return;
  }

  leftWheel.targetRadS  = atof(line + 1);
  rightWheel.targetRadS = atof(firstSpace + 1);
  lastCmdMs = millis();

  Serial.print("SUCCESS: L="); Serial.print(leftWheel.targetRadS);
  Serial.print(" R="); Serial.println(rightWheel.targetRadS);
}

// ============================================================
//  Non-blocking ultrasonic scanner (cycles through sensors)
// ============================================================
void tickUltrasonic() {
  unsigned long now = millis();

  switch (usState) {

    case US_IDLE:
      // Trigger the current sensor
      digitalWrite(TRIG_PINS[usCurrent], LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG_PINS[usCurrent], HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PINS[usCurrent], LOW);
      usTrigTime = micros();
      usState = US_WAIT_HIGH;
      break;

    case US_WAIT_HIGH:
      if (digitalRead(ECHO_PINS[usCurrent]) == HIGH) {
        usEchoStart = micros();
        usState = US_WAIT_LOW;
      } else if (micros() - usTrigTime > 30000UL) {
        // Timeout — no echo
        usDistances[usCurrent] = -1.0f;
        usState = US_IDLE;
        usCurrent = (usCurrent + 1) % NUM_US;
      }
      break;

    case US_WAIT_LOW:
      if (digitalRead(ECHO_PINS[usCurrent]) == LOW) {
        float duration = float(micros() - usEchoStart);
        usDistances[usCurrent] = duration * 0.0343f / 2.0f;  // cm
        usState = US_IDLE;
        usCurrent = (usCurrent + 1) % NUM_US;
      } else if (micros() - usEchoStart > 30000UL) {
        usDistances[usCurrent] = -1.0f;
        usState = US_IDLE;
        usCurrent = (usCurrent + 1) % NUM_US;
      }
      break;

    default:
      usState = US_IDLE;
      break;
  }
}

// ============================================================
//  setup
// ============================================================
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  stopMotors();

  // Encoder pins
  pinMode(LEFT_ENC_A,  INPUT_PULLUP);
  pinMode(LEFT_ENC_B,  INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  onLeftEnc,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), onRightEnc, CHANGE);

  // Ultrasonic pins
  for (uint8_t i = 0; i < NUM_US; i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    pinMode(ECHO_PINS[i], INPUT);
    digitalWrite(TRIG_PINS[i], LOW);
  }

  lastCmdMs = millis();
}

// ============================================================
//  loop
// ============================================================
void loop() {
  unsigned long now = millis();

  // 1. Read incoming serial commands
  readSerial();

  // 2. Command timeout — stop if Pi hasn't sent anything recently
  if (now - lastCmdMs > CMD_TIMEOUT_MS) {
    leftWheel.targetRadS  = 0.0f;
    rightWheel.targetRadS = 0.0f;
  }

  // 3. Control loop — 50 Hz
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    float dt = float(now - lastControlMs) / 1000.0f;
    lastControlMs = now;

    noInterrupts();
    long lt = leftTicks;
    long rt = rightTicks;
    interrupts();

    runPID(leftWheel,  lt, dt);
    runPID(rightWheel, rt, dt);

    driveMotor(LEFT_IN1,  LEFT_IN2,  leftWheel.pwm);
    driveMotor(RIGHT_IN1, RIGHT_IN2, rightWheel.pwm);
  }

  // 4. Odometry publish — 50 Hz
  if (now - lastOdomMs >= ODOM_PERIOD_MS) {
    lastOdomMs = now;
    publishOdom();
  }

  // 5. Ultrasonic scanning — 20 Hz per sensor cycle
  if (now - lastUsMs >= US_PERIOD_MS) {
    lastUsMs = now;
    tickUltrasonic();
  }
}


