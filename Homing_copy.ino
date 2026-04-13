#include <Servo.h>  //Need for Servo pulse output

//#include <Adafruit_BNO08x.h>  //Need for Gyroscope

//Gyroscope initialisation
//Adafruit_BNO08x bno08x(-1);
//sh2_SensorValue_t sensorValue;
float rad = 0.0;

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;


//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_font_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;

//Servo Setup
Servo sensor_servo;  

int speed_val = 100;
int speed_change;

// variables for IR and sonar sensors

int frontleftsensor = A6; //frontleftsensor is attached on pinA0
int backleftsensor = A7; //frontleftsensor is attached on pinA1
int frontrightsensor = A4; //frontleftsensor is attached on pinA2
int backrightsensor = A5; //frontleftsensor is attached on pinA3

byte serialRead = 0; //for control serial communication

int signalADC0 = 0; // the read out signal in 0-1023 corresponding to 0-5v
int signalADC1 = 0; // the read out signal in 0-1023 corresponding to 0-5v
int signalADC2 = 0; // the read out signal in 0-1023 corresponding to 0-5v
int signalADC3 = 0; // the read out signal in 0-1023 corresponding to 0-5v

bool wall = false;

float frontleftsensor_cm = 0; // the calculated distance in cm from the front left sensor
float backleftsensor_cm = 0; // the calculated distance in cm from the back left sensor
float frontrightsensor_cm = 0; // the calculated distance in cm from the front right sensor
float backrightsensor_cm = 0; // the calculated distance in cm from the back right sensor   
float sonarsensor_cm = 0; // the calculated distance in cm from the sonar sensor
float left_sonarsensor_cm = 0; 
float right_sonarsensor_cm = 0; 
float horizontal_distance_cm = 0; // the calculated horizontal distance from the front of the robot to the wall
float turn_angle = 0; // the calculated angle to turn to be parallel to the wall

float robot_width = 20.5; // the width of the robot in cm
float robot_length = 22.5; // the length of the robot in cm
float map_width = 121.5; // the width of the map in cm
float map_length = 199; // the length of the map in cm


//Serial Pointer
HardwareSerial *SerialCom;

// variable for home states
int home_side = 0;
int home_face = 0;
#define LEFT 0
#define RIGHT 1
#define FRONT 0
#define BACK 1

int pos = 0;
void setup(void) {
  Serial.begin(115200);
  turret_motor.attach(11);
  enable_motors();
  stop();
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  //Servo Setup for ultrasonic sensor
  sensor_servo.attach(10);
  sensor_servo.write(90);

  // Use USB Serial for debug output and reserve Serial1 for command input only.
  SerialCom = &Serial1;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code");
  delay(1000);
  SerialCom->println("Setup....");

  delay(1000);  //settling time but no really needed
}

/*
void loop () {
  if (Serial.available()) // Check for input from terminal
  {
  serialRead = Serial.read(); // Read input
  if (serialRead==49) // Check for flag to execute, 49 is ascii for 1, stop serial printing
  {
  Serial.end(); // end the serial communication to get the sensor data
  }
  }



}
*/

void loop() {
  
  if (Serial.available()) // Check for input from terminal
  {
  serialRead = Serial.read(); // Read input
  if (serialRead==49) // Check for flag to execute, 49 is ascii for 1, stop serial printing
  {
  Serial.end(); // end the serial communication to get the sensor data
  }
  }

  // initial scan
    // turn servo forward
    sensor_servo.write(90);
    delay(1000);
    sonarsensor_cm = read_sonarsensor();

    // turn servo 90deg to left and read sonar
    sensor_servo.write(180);
    delay(1000);
    left_sonarsensor_cm = read_sonarsensor();
    Serial.print("left sonar");
    Serial.println(left_sonarsensor_cm );
    delay(50); 

    // turn servo 90deg to right and read sonar
    sensor_servo.write(0);
    delay(1000);
    right_sonarsensor_cm = read_sonarsensor();
    Serial.print("right sonar");
    Serial.println( right_sonarsensor_cm);
    delay(100); 


  if (right_sonarsensor_cm >left_sonarsensor_cm) {
    // If right is further than left, align to left wall first
  
    // safety check 
    if (sonarsensor_cm < 20 && left_sonarsensor_cm > 15) {
      speed_val = 150;
      reverse();
      speed_val = 100;
      delay(1000);
      stop();
    } 

    // turn servo 90 deg left
    sensor_servo.write(180);
    delay(1000);
    left_sonarsensor_cm = read_sonarsensor();

    // home towards left wall
    home(LEFT);
    delay(50);

    // check if aligned to correct wall
    // rotate servo 90 deg to right and read sonar
    sensor_servo.write(0);
    delay(1000);
    right_sonarsensor_cm = read_sonarsensor();

    if (right_sonarsensor_cm > 120) {
      // Aligned to short wall on left side
      Serial.print("Aligned to short wall");

      // rotate robot 90 deg 
      cw();
      delay(2500);
      stop();

      // align to closest wall 
      if (right_sonarsensor_cm < 50) {
        // align to the right 
        home_side = RIGHT;
        home(RIGHT);
      } else {
        // align to the left
        // turn servo 90 deg left 
        sensor_servo.write(180);
        left_sonarsensor_cm = read_sonarsensor();
        // home towards left wall
        home_side = LEFT;
        home(LEFT);
      }
    }
  } else {
    // If left is further than right, align to right wall first

    // safety check 
    if (sonarsensor_cm < 20 && right_sonarsensor_cm > 15) {
      speed_val = 150;
      reverse();
      speed_val = 100;
      delay(500);
      stop();
    } 

    // turn servo 90 deg right
    sensor_servo.write(0);
    delay(1000);
    right_sonarsensor_cm = read_sonarsensor();
    // home towards right wall
    home_side = RIGHT;
    home(RIGHT);
    delay(50);

    // check if aligned to correct wall
    // rotate servo 90 deg to left and read sonar
    sensor_servo.write(180);
    delay(1000);
    left_sonarsensor_cm = read_sonarsensor();
    if (left_sonarsensor_cm > 120) {
     // if true aligned on short wall
      Serial.print("Aligned to short wall");
      // rotate robot ~ 90 deg 
      cw();
      delay(2500);
      stop();
      // check which way is closer wall 
      left_sonarsensor_cm = read_sonarsensor();
      if (left_sonarsensor_cm < 50) {
        // home towards left wall
        home_side = LEFT;
        home(LEFT);
      } else {
        // turn servo 90 deg right
        sensor_servo.write(0);
        delay(1000);
        right_sonarsensor_cm = read_sonarsensor();
        // home towards right wall
        home_side = RIGHT;
        home(RIGHT);
    }   
  }
  }

  // align front/back direction 

  // turn servo forward
  sensor_servo.write(90);
  delay(1000);
  sonarsensor_cm = read_sonarsensor();
  Serial.print(" distance from far wall: ");
  Serial.println(sonarsensor_cm);
  
  speed_val = 150;
  if (sonarsensor_cm > 110) {
    home_face = BACK;
    while (sonarsensor_cm < 170) {
      //Serial.print(" distance from far wall: ");
      //Serial.println(sonarsensor_cm);
      sonarsensor_cm = read_sonarsensor();
      reverse();
      delay(50);
    }
  } else {
    home_face = FRONT;
    while (sonarsensor_cm > 6) {
    forward();
    sonarsensor_cm = read_sonarsensor(); 
    delay(50);   
    }
  } 
  speed_val = 100;
  stop();
  // final align on side 
  if (home_side == LEFT) {
    // turn servo 90 deg left
    sensor_servo.write(180);
    delay(1000);
    left_sonarsensor_cm = read_sonarsensor();
    align(LEFT);
  } else {
    // turn servo 90 deg right
    sensor_servo.write(0);
    delay(1000);
    right_sonarsensor_cm = read_sonarsensor();
    align(RIGHT);
  }
  // should be aligned to corner and homing complete
  Serial.println("Homing complete!");
}

void home(int dir) {
  speed_val = 150;
  if (dir == LEFT) {
    read_IR_sensors(LEFT);
    home_side = LEFT;
      // go toward left 
    while (left_sonarsensor_cm > 15) {
      //if (left_sonarsensor_cm < 25 && backleftsensor_cm < 7) {
      //break;
      //}
      strafe_left();
      read_IR_sensors(LEFT);  ////????
      left_sonarsensor_cm = read_sonarsensor();
      delay(50);
    }
    stop();
    align(LEFT);
  } else {
    read_IR_sensors(RIGHT);
    home_side = RIGHT;
    while (right_sonarsensor_cm > 15) {
      //if (right_sonarsensor_cm <25 && backrightsensor_cm < 12) {
        // break;
      //}
      strafe_right();
      read_IR_sensors(RIGHT);
      right_sonarsensor_cm = read_sonarsensor();
      delay(50);
    }
    stop();
    align(RIGHT);
  }
  speed_val = 100;
}

void align(int dir) {
  Serial.print("left sonar");
  Serial.println(left_sonarsensor_cm);
  read_IR_sensors(dir);
  float error = 0;
  float error_sum = 0;
  if (dir == LEFT) {
    error = frontleftsensor_cm - backleftsensor_cm;
  } else {
  error = frontrightsensor_cm - backrightsensor_cm;
  }
  speed_val = 60 + 25.0*abs(error);

  // control loop 
  while (abs(error) > 0.2) {
    if (abs(error) < 2) {
      error_sum += abs(error);
    }
    speed_val = 60 + 27.5*abs(error) + 0*abs(error_sum);

    // dir 1 = left, 2 = right
    if (dir == 1) {
      // left 
      if (error > 0) { 
        // front is further than back, turn cw
        ccw();
        delay(100);
        stop();
        delay(50);
      } else {
        // back is further than front, turn ccw
        cw();
        delay(100);
        stop();
        delay(50);
      } 
    } else {
      // right
      if (error > 0) { 
        // front is further than back, turn ccw
        Serial.println("cw");
        cw();
        delay(100);
        stop();
        delay(50);
      } else {
        Serial.println("ccw");
        // back is further than front, turn cw
        ccw();
        delay(100);
        stop();
        delay(50);
      } 
    }
    
    read_IR_sensors(dir);
    delay(50);
    if (dir == 1) {
      error = frontleftsensor_cm - backleftsensor_cm;
    } else {
      error = frontrightsensor_cm - backrightsensor_cm;
    }
    Serial.print("Error: ");
    Serial.println(error);
    Serial.print("right front: ");
    Serial.println(frontrightsensor_cm);    
    Serial.print("right back: ");
    Serial.println(backrightsensor_cm);
    delay(50);
  
  }
  stop();
  speed_val = 100; // reset speed value after alignment
  
}


void read_IR_sensors(int dir){
  // dir = 1
  long sumfl = 0, sumbl =0, sumfr = 0, sumbr = 0;
  if (dir == LEFT) { 
    // left
  for (int i = 0; i < 4; i++) {
    sumfl += analogRead(frontleftsensor);
    sumbl += analogRead(backleftsensor);
    delay(5);
  }
  signalADC0 = sumfl/4;
  signalADC1 = sumbl/4;
  frontleftsensor_cm = 17948*pow(signalADC0,-1.22);
  backleftsensor_cm = 17948*pow(signalADC1,-1.22);
  backleftsensor_cm = backleftsensor_cm / (0.024 * frontleftsensor + 0.86857);
  SerialCom->print("frontleftsensor_cm = ");
  SerialCom->println(frontleftsensor_cm);
  SerialCom->print("backleftsensor_cm = ");
  SerialCom->println(backleftsensor_cm);
  Serial.print("left front: ");
  Serial.println(frontleftsensor_cm);    
  Serial.print("left back: ");
  Serial.println(backleftsensor_cm);
  } else {
    // right
    for (int i = 0; i < 4; i++) {
    sumfr += analogRead(frontrightsensor);
    sumbr += analogRead(backrightsensor);
    delay(5);
    }
    signalADC2 = sumfr/4;
    signalADC3 = sumbr/4;
    frontrightsensor_cm = 17948*pow(signalADC2,-1.22);
    backrightsensor_cm = 0.5*17948*pow(signalADC3,-1.22);
    backrightsensor_cm = backrightsensor_cm * (-0.0167* frontrightsensor_cm + 1.0076);
    SerialCom->print("frontrightsensor_cm = ");
    SerialCom->println(frontrightsensor_cm);
    SerialCom->print("backrightsensor_cm = ");
    SerialCom->println(backrightsensor_cm);
    Serial.print("right front: ");
    Serial.println(frontrightsensor_cm);    
    Serial.print("right back: ");
    Serial.println(backrightsensor_cm);
  }

}
  

float read_sonarsensor() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

   // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;

  // Print out results
  if (pulse_width > MAX_DIST) {
    SerialCom->println("HC-SR04: Out of range");
  } else {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
  }
  return cm; 
}

void HC_SR04_range() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      // SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      // SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (pulse_width > MAX_DIST) {
    // SerialCom->println("HC-SR04: Out of range");
  } else {
    // SerialCom->print("HC-SR04:");
    // SerialCom->print(cm);
    // SerialCom->println("cm");
  }
}


//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors() {
  left_font_motor.detach();   // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();   // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors() {
  left_font_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);     // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);   // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop()  //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void reverse() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}
