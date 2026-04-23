#include <Servo.h>

// ----------- PINS ----------- //
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

// ----------- OBJECTS ----------- //
Servo sensor_servo;
Servo left_front_motor;
Servo left_rear_motor;
Servo right_rear_motor;
Servo right_front_motor;

// ----------- SERIAL ----------- //
HardwareSerial* SerialCom;

// ----------- GLOBALS ----------- //
float speed_val = 100;

// ----------- SETUP ----------- //
void setup() {

  SerialCom = &Serial1;   // match your main code
  SerialCom->begin(115200);

  sensor_servo.attach(10);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  left_front_motor.attach(left_front);
  left_rear_motor.attach(left_rear);
  right_rear_motor.attach(right_rear);
  right_front_motor.attach(right_front);

  stop();

  SerialCom->println("Align-to-wall test starting...");
  delay(2000);
}

// ----------- LOOP ----------- //
void loop() {
  align_to_wall_sonar();
  SerialCom->println("Aligned. Waiting...");
  delay(4000);
}

// ----------- ULTRASONIC ----------- //
float HC_SR04_range() {
  unsigned long t1;

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    if (micros() - t1 > 30000) return 300;
  }

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    if (micros() - t1 > 30000) return 300;
  }

  return (micros() - t1) / 58.0;
}

// ----------- MOTOR CONTROL ----------- //
void stop() {
  left_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);
}

void cw() {
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void ccw() {
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

// ----------- ALIGN FUNCTION ----------- //
void align_to_wall_sonar() {

  float dist_left, dist_right;
  float error, prev_error = 0;

  const float ALIGNED_THRESHOLD = 1.0;
  const int MAX_ITERATIONS = 50;

  int centre_angle = 90;
  int sweep_angle = 15;
  int iterations = 0;

  sensor_servo.write(centre_angle);
  delay(300);

  do {
    sensor_servo.write(centre_angle + sweep_angle);
    delay(300);
    dist_left = HC_SR04_range();

    sensor_servo.write(centre_angle - sweep_angle);
    delay(300);
    dist_right = HC_SR04_range();

    error = dist_left - dist_right;

    SerialCom->print("L: ");
    SerialCom->print(dist_left);
    SerialCom->print(" R: ");
    SerialCom->print(dist_right);
    SerialCom->print(" Error: ");
    SerialCom->println(error);

    float kp = 5;
    speed_val = constrain((kp * abs(error)), 80, 130);

    if (abs(error) <= ALIGNED_THRESHOLD) break;

    (error > 0) ? cw() : ccw();

    delay(80);
    stop();
    delay(100);

    prev_error = error;
    iterations++;

  } while (iterations < MAX_ITERATIONS);

  stop();
  sensor_servo.write(90);
  delay(300);

  SerialCom->println("Alignment complete");
}
