/*
 * Athena rover motor controller — Raspberry Pi Pico W
 * 6 wheels, 3 per side, each a DC motor on a DIR + PWM driver channel.
 *
 * ── Serial protocol, 115200 baud, newline-terminated ────────────────
 *   V <left> <right>   variable drive. Signed PWM -255..255 per SIDE,
 *                      positive = that side drives the rover FORWARD.
 *                      e.g. "V 120 -80"   (left fwd 120, right rev 80)
 *   M <n> <pwm>        drive ONE motor only, for wiring checks.
 *                      n: 1=FL 2=ML 3=RL 4=FR 5=MR 6=RR
 *                      e.g. "M 1 120"     (front-left forward)
 *                      Auto-stops after MOTOR_TEST_MS so a mistake
 *                      cannot run away.
 *   F B L R S          legacy discrete moves at speedVal / stop
 *   P <0-255>          set the speed F/B/L/R use
 *   ?                  status line
 *   H                  this help
 *
 * ── Why it is built this way ────────────────────────────────────────
 *  20 kHz PWM.  The RP2040 core defaults to ~1 kHz, which sits right in
 *  the audible band (loud whine) and gives poor low-speed torque because
 *  each pulse is long relative to the motor's electrical time constant.
 *  20 kHz is above hearing and lets the winding current stay continuous,
 *  so slow speeds are smooth and controllable instead of steppy.
 *
 *  Slew limiting.  Commanded PWM ramps toward the target rather than
 *  stepping.  Protects the gearboxes, stops the wheels breaking traction
 *  on every command change, and keeps the motion smooth enough that the
 *  visual odometry upstream can actually track it.
 *
 *  Watchdog.  If no valid command arrives for WATCHDOG_MS the motors stop.
 *  The host streams at 20 Hz, so a crashed node, unplugged cable or dead
 *  Jetson halts the rover in under half a second.
 *
 * Pin map and motor polarity are unchanged from the original sketch:
 * left side forward = DIR LOW, right side forward = DIR HIGH.
 */

// ───────────────────────── pins (unchanged) ─────────────────────────
#define FRONT_M1_PWM  8    // M1 = LEFT side
#define FRONT_M1_DIR  9
#define FRONT_M2_PWM  6    // M2 = RIGHT side
#define FRONT_M2_DIR  7

#define MID_M1_PWM   18
#define MID_M1_DIR   19
#define MID_M2_PWM   20
#define MID_M2_DIR   21

#define REAR_M1_PWM  10
#define REAR_M1_DIR  11
#define REAR_M2_PWM  12
#define REAR_M2_DIR  13

// polarity taken from the original, proven sketch
#define LEFT_FWD_LEVEL   LOW
#define RIGHT_FWD_LEVEL  HIGH

// ───────────────────────── tuning ─────────────────────────
#define PWM_FREQ_HZ      20000   // above audible; smooth at low duty
#define WATCHDOG_MS      500     // stop if the host goes quiet
#define RAMP_INTERVAL_MS 10      // slew update period
#define RAMP_STEP        8       // PWM counts per interval
                                 //   0->255 in ~0.32 s, full reverse ~0.64 s
#define MOTOR_TEST_MS    1500    // "M" command auto-stop

int speedVal = 200;              // used by legacy F/B/L/R

// signed PWM, -255..255, positive = forward
int targetL = 0, targetR = 0;    // what the host asked for
int currentL = 0, currentR = 0;  // what the motors are actually doing

unsigned long lastCmdMs   = 0;
unsigned long lastRampMs  = 0;
unsigned long motorTestMs = 0;   // non-zero while an "M" test is running
bool wdtTripped = false;

char lineBuf[40];
uint8_t lineLen = 0;

struct Motor { uint8_t pwm; uint8_t dir; bool leftSide; };
const Motor MOTORS[6] = {
  {FRONT_M1_PWM, FRONT_M1_DIR, true },   // 1 FL
  {MID_M1_PWM,   MID_M1_DIR,   true },   // 2 ML
  {REAR_M1_PWM,  REAR_M1_DIR,  true },   // 3 RL
  {FRONT_M2_PWM, FRONT_M2_DIR, false},   // 4 FR
  {MID_M2_PWM,   MID_M2_DIR,   false},   // 5 MR
  {REAR_M2_PWM,  REAR_M2_DIR,  false},   // 6 RR
};
const char *MOTOR_NAME[6] = {"front-left", "mid-left", "rear-left",
                             "front-right", "mid-right", "rear-right"};

// ───────────────────────── motor output ─────────────────────────
void driveMotor(const Motor &m, int spwm) {
  int mag = abs(spwm);
  if (mag > 255) mag = 255;
  bool fwd = (spwm >= 0);
  int level = m.leftSide ? (fwd ? LEFT_FWD_LEVEL  : !LEFT_FWD_LEVEL)
                         : (fwd ? RIGHT_FWD_LEVEL : !RIGHT_FWD_LEVEL);
  digitalWrite(m.dir, level);
  analogWrite(m.pwm, mag);
}

void applySide(bool leftSide, int spwm) {
  for (int i = 0; i < 6; i++)
    if (MOTORS[i].leftSide == leftSide) driveMotor(MOTORS[i], spwm);
}

void applyMotors() {
  applySide(true,  currentL);
  applySide(false, currentR);
}

void hardStop() {
  targetL = targetR = 0;
  currentL = currentR = 0;
  motorTestMs = 0;
  applyMotors();
}

// ───────────────────────── ramping ─────────────────────────
int stepToward(int current, int target) {
  if (current < target) {
    current += RAMP_STEP;
    if (current > target) current = target;
  } else if (current > target) {
    current -= RAMP_STEP;
    if (current < target) current = target;
  }
  return current;
}

// ───────────────────────── commands ─────────────────────────
void feedWatchdog() {
  lastCmdMs = millis();
  if (wdtTripped) {
    wdtTripped = false;
    Serial.println("WDT cleared");
  }
}

void printHelp() {
  Serial.println(F("athena-drive v2  (6 motors, variable speed)"));
  Serial.println(F("  V <l> <r>    drive, signed PWM -255..255 per side"));
  Serial.println(F("  M <n> <pwm>  ONE motor for wiring checks, n=1..6"));
  Serial.println(F("               1=FL 2=ML 3=RL 4=FR 5=MR 6=RR"));
  Serial.println(F("  F B L R S    legacy moves / stop"));
  Serial.println(F("  P <0-255>    speed used by F/B/L/R"));
  Serial.println(F("  ? H          status / this help"));
}

void printStatus() {
  Serial.print(F("L=")); Serial.print(currentL);
  Serial.print('/');     Serial.print(targetL);
  Serial.print(F(" R=")); Serial.print(currentR);
  Serial.print('/');      Serial.print(targetR);
  Serial.print(F(" spd=")); Serial.print(speedVal);
  Serial.print(F(" pwmHz=")); Serial.print(PWM_FREQ_HZ);
  Serial.print(F(" wdt=")); Serial.println(wdtTripped ? "TRIPPED" : "ok");
}

void handleLine(char *s) {
  while (*s == ' ') s++;
  if (*s == '\0') return;
  char cmd = toupper(*s);

  switch (cmd) {
    case 'V': {
      int l, r;
      if (sscanf(s + 1, "%d %d", &l, &r) == 2) {
        targetL = constrain(l, -255, 255);
        targetR = constrain(r, -255, 255);
        motorTestMs = 0;
        feedWatchdog();
      } else {
        Serial.println(F("ERR usage: V <left> <right>"));
      }
      break;
    }
    case 'M': {
      int n, pwm;
      if (sscanf(s + 1, "%d %d", &n, &pwm) == 2 && n >= 1 && n <= 6) {
        hardStop();
        pwm = constrain(pwm, -255, 255);
        driveMotor(MOTORS[n - 1], pwm);
        motorTestMs = millis();
        feedWatchdog();
        Serial.print(F("motor ")); Serial.print(n);
        Serial.print(F(" (")); Serial.print(MOTOR_NAME[n - 1]);
        Serial.print(F(") pwm=")); Serial.print(pwm);
        Serial.print(F("  auto-stop in ")); Serial.print(MOTOR_TEST_MS);
        Serial.println(F(" ms"));
      } else {
        Serial.println(F("ERR usage: M <1-6> <pwm>"));
      }
      break;
    }
    case 'F': targetL =  speedVal; targetR =  speedVal; motorTestMs = 0; feedWatchdog(); break;
    case 'B': targetL = -speedVal; targetR = -speedVal; motorTestMs = 0; feedWatchdog(); break;
    case 'L': targetL = -speedVal; targetR =  speedVal; motorTestMs = 0; feedWatchdog(); break;
    case 'R': targetL =  speedVal; targetR = -speedVal; motorTestMs = 0; feedWatchdog(); break;
    case 'S': hardStop(); feedWatchdog(); Serial.println(F("Stopped")); break;
    case 'P': {
      int p;
      if (sscanf(s + 1, "%d", &p) == 1) {
        speedVal = constrain(p, 0, 255);
        Serial.print(F("speedVal=")); Serial.println(speedVal);
        feedWatchdog();
      }
      break;
    }
    case '?': printStatus(); break;
    case 'H': printHelp(); break;
    default:
      Serial.println(F("ERR use V <l> <r> | M <n> <pwm> | F B L R S | P <n> | ? H"));
      break;
  }
}

// ───────────────────────── setup / loop ─────────────────────────
void setup() {
  for (int i = 0; i < 6; i++) {
    pinMode(MOTORS[i].pwm, OUTPUT);
    pinMode(MOTORS[i].dir, OUTPUT);
  }

  // 20 kHz, 8-bit. Must be set before the first analogWrite to take effect.
  analogWriteFreq(PWM_FREQ_HZ);
  analogWriteRange(255);

  hardStop();

  Serial.begin(115200);
  Serial.println();
  printHelp();
  lastCmdMs = millis();
}

void loop() {
  // -- accumulate a line, act on newline --
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;             // overflow: drop the garbage line
    }
  }

  unsigned long now = millis();

  // -- single-motor test auto-stop --
  if (motorTestMs && (now - motorTestMs) > MOTOR_TEST_MS) {
    hardStop();
    Serial.println(F("motor test ended"));
  }

  // -- watchdog: host went quiet --
  // A running "M" test is exempt: it is a deliberate one-shot with its own
  // shorter-intent timeout, and the operator is not streaming commands
  // during it.  Without this the 500 ms watchdog would cut every wiring
  // check short at 500 ms instead of MOTOR_TEST_MS.
  if (!wdtTripped && !motorTestMs && (now - lastCmdMs) > WATCHDOG_MS) {
    wdtTripped = true;
    hardStop();
    Serial.println(F("WDT stop (no command)"));
  }

  // -- slew-limited ramp toward target --
  if (!wdtTripped && !motorTestMs && (now - lastRampMs) >= RAMP_INTERVAL_MS) {
    lastRampMs = now;
    int newL = stepToward(currentL, targetL);
    int newR = stepToward(currentR, targetR);
    if (newL != currentL || newR != currentR) {
      currentL = newL;
      currentR = newR;
      applyMotors();
    }
  }
}
