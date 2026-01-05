#include <Arduino.h>
#include <Servo.h>

// ------------------ Motor & Encoder ------------------
#define ENA 11
#define IN1 8
#define IN2 12
#define ENB 5
#define IN3 6
#define IN4 7

#define ENCODER_LEFT_A 2
#define ENCODER_LEFT_B 4
#define ENCODER_RIGHT_A 3
#define ENCODER_RIGHT_B A2

volatile long left_ticks = 0;
volatile long right_ticks = 0;

float wheel_radius = 0.03;      // meters
float wheel_separation = 0.30;  // meters
float ticks_per_rev = 3600;


// ------------------ Quadrature Encoder ISRs ------------------
void leftEncoderISR_A() {
  int a = digitalRead(ENCODER_LEFT_A);
  int b = digitalRead(ENCODER_LEFT_B);
  if (a == b) left_ticks--;
  else left_ticks++;
}

void leftEncoderISR_B() {
  int a = digitalRead(ENCODER_LEFT_A);
  int b = digitalRead(ENCODER_LEFT_B);
  if (a != b) left_ticks--;
  else left_ticks++;
}

void rightEncoderISR_A() {
  int a = digitalRead(ENCODER_RIGHT_A);
  int b = digitalRead(ENCODER_RIGHT_B);
  if (a == b) right_ticks++;
  else right_ticks--;
}

void rightEncoderISR_B() {
  int a = digitalRead(ENCODER_RIGHT_A);
  int b = digitalRead(ENCODER_RIGHT_B);
  if (a != b) right_ticks++;
  else right_ticks--;
}


// ------------------ Ultrasonic Lidar ------------------
#define TRIG_PIN A0
#define ECHO_PIN A1
#define SERVO_PIN 13

Servo servo;
int angle = 100; 
int direction = 1;
int step = 2;
int min_angle = 10;
int max_angle = 170;

unsigned long last_lidar_time = 0;
unsigned long lidar_interval = 150; // ms between servo steps

// ------------------ Setup ------------------
void setup() {
  Serial.begin(115200);

  // Motors
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

 // Encoders
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), leftEncoderISR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), rightEncoderISR_B, CHANGE);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  servo.attach(SERVO_PIN);
  servo.write(angle);
}

// ------------------ Helper Functions ------------------
float get_distance_cm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 20000); // 20 ms timeout
  if (duration == 0) return 6000; // no echo (60 m)
  return duration * 0.034 / 2.0; // in cm
}

void setMotorSpeeds(float v_left, float v_right) {
  const int PWM_MIN = 70;
  const int PWM_MAX = 200;
  const float MAX_VELOCITY = 0.8; // m/s

  // ------------------ Calibration factors ------------------
  // Adjust these to balance straight-line motion
  const float LEFT_CALIB = 1.00;   // 1.0 = no change
  const float RIGHT_CALIB = 0.75;  // reduce right motor power by 10%

  int pwm_left = 0;
  int pwm_right = 0;

  if (fabs(v_left) > 0.05)
    pwm_left = constrain((fabs(v_left) / MAX_VELOCITY) * (PWM_MAX - PWM_MIN) + PWM_MIN, PWM_MIN, PWM_MAX);
  if (fabs(v_right) > 0.05)
    pwm_right = constrain((fabs(v_right) / MAX_VELOCITY) * (PWM_MAX - PWM_MIN) + PWM_MIN, PWM_MIN, PWM_MAX);

  // ------------------ Apply calibration ------------------
  pwm_left = pwm_left * LEFT_CALIB;
  pwm_right = pwm_right * RIGHT_CALIB;

  // ------------------ Motor direction control ------------------
  digitalWrite(IN1, v_left > 0 ? HIGH : LOW);
  digitalWrite(IN2, v_left > 0 ? LOW : HIGH);
  analogWrite(ENA, pwm_left * 0.6);  // 0.6 is your global scaling factor

  digitalWrite(IN3, v_right > 0 ? HIGH : LOW);
  digitalWrite(IN4, v_right > 0 ? LOW : HIGH);
  analogWrite(ENB, pwm_right);
}


// ------------------ Main Loop ------------------
void loop() {
  // ----- 1. Read velocity commands -----
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("VEL")) {
      float v_left, v_right;
      int firstSpace = cmd.indexOf(' ');
      int secondSpace = cmd.indexOf(' ', firstSpace + 1);
      v_left = cmd.substring(firstSpace + 1, secondSpace).toFloat();
      v_right = cmd.substring(secondSpace + 1).toFloat();
      setMotorSpeeds(v_left, v_right);
    }
  }

  // ----- 2. Send encoder ticks every 100 ms -----
  static unsigned long last_enc_time = 0;
  if (millis() - last_enc_time > 100) {
    // ------------------ Encoder calibration ------------------
    const float LEFT_TICK_SCALE = 1.50;   // adjust slightly
    const float RIGHT_TICK_SCALE = 1.00;  // e.g. increase right ticks 5%

    long adj_left = left_ticks * LEFT_TICK_SCALE;
    long adj_right = right_ticks * RIGHT_TICK_SCALE;

    Serial.print("ENC ");
    Serial.print(adj_left);
    Serial.print(" ");
    Serial.println(adj_right);

    last_enc_time = millis();
  }


  // ----- 3. Servo lidar sweep -----
  if (millis() - last_lidar_time >= lidar_interval) {
    servo.write(angle);
    delay(20); // small stabilization delay

    float distance = get_distance_cm();
    Serial.print("LID ");
    Serial.print(angle);
    Serial.print(" ");
    Serial.println(distance * 10.0); // mm

    angle += direction * step;
    if (angle >= max_angle) direction = -1;
    else if (angle <= min_angle) direction = 1;

    last_lidar_time = millis();
  }
}
