#include <Adafruit_BNO08x.h>  //Need for Gyroscope
#include <Servo.h>            //Need for Servo pulse output


//----------Hardware Initialisation----------//

//Gyroscope
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;
float rad = 0.0;
float gyroZ = 0;

// Servo
Servo sensor_servo;

// Motors
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

Servo left_front_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29

// Ultrasonic Sensor
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Anything over 400 cm (23200 us pulse) is "out of range". Hint:If you decrease
// to this the ranging sensor but the timeout is short, you may not need to read
// up to 4meters.
const unsigned int MAX_DIST = 23200;

// IR Sensors
int frontleftsensor = A6; //frontleftsensor is attached on pinA0
int backleftsensor = A7; //frontleftsensor is attached on pinA1
int frontrightsensor = A4; //frontleftsensor is attached on pinA2
int backrightsensor = A5; //frontleftsensor is attached on pinA3

int signalADC0 = 0; // the read out signal in 0-1023 corresponding to 0-5v
int signalADC1 = 0; // the read out signal in 0-1023 corresponding to 0-5v
int signalADC2 = 0; // the read out signal in 0-1023 corresponding to 0-5v
int signalADC3 = 0; // the read out signal in 0-1023 corresponding to 0-5v

//-----------------------------------------//

float speed_val = 100;

//----------Homing-----------//

float frontleftsensor_cm = 0; // the calculated distance in cm from the front left sensor
float backleftsensor_cm = 0; // the calculated distance in cm from the back left sensor
float frontrightsensor_cm = 0; // the calculated distance in cm from the front right sensor
float backrightsensor_cm = 0; // the calculated distance in cm from the back right sensor   
float sonarsensor_cm = 0; // the calculated distance in cm from the sonar sensor
float left_sonarsensor_cm = 0; 
float right_sonarsensor_cm = 0; 

//---------------------------//

//----------Path Following----------//

int speed_change;
int direction = 1;
int y_dir = 1;
float y_distance = 10.0;
float x_distance = 14.0;
int x_dir = 1;
float past_error_y = 0.00;
float past_error_x = 0.00;

//----------------------------------//

//----------Finite State Machine----------//

enum STATE { INITIALISING, HOMING, PATH_FOLLOWING, STOPPED };
STATE machine_state = INITIALISING;

//Homing
int home_side = 0;
int home_face = 0;
#define LEFT 1
#define RIGHT -1
#define FRONT 1
#define BACK -1


//----------------------------------------//


//----------Miscellaneous----------//

Servo turret_motor;
int pos = 0;

// #define NO_READ_GYRO  //Uncomment if GYRO is not attached.
// #define NO_HC -SR04  //Uncomment if HC-SR04 ultrasonic ranging sensor is not attached.
// #define NO_BATTERY_V_OK //Uncomment if BATTERY_V_OK if you do not care about battery damage.

// Serial Pointer
HardwareSerial* SerialCom;
byte serialRead = 0; //for control serial communication

//---------------------------------//


void setup(void) {

  turret_motor.attach(11); //???
  pinMode(LED_BUILTIN, OUTPUT); //???
  
  sensor_servo.attach(10);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Use USB Serial for debug output and reserve Serial1 for command input only.
  SerialCom = &Serial1;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code");
  delay(1000);
}

void loop(void) {
  delay(50);
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case HOMING:
      machine_state = homing();
      break;
    case PATH_FOLLOWING:
      machine_state = path_following();
      break;
  }

}

  
//--------------------------------------------------------------------------//

STATE initialising() {
  enable_motors();

  if (!bno08x.begin_I2C() ||
      !bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 10000)) {
    while (1) {
      SerialCom->println("IMU failed");
      delay(100);
    }
  }

  sensor_servo.write(78);
  delay(500);
  return HOMING;
}

STATE homing() {

  // initial scan
    // turn servo forward
    sensor_servo.write(78);
    delay(1000);
    sonarsensor_cm = HC_SR04_range();

    // turn servo 90deg to left and read sonar
    sensor_servo.write(175);
    delay(1000);
    left_sonarsensor_cm = HC_SR04_range();
    Serial.print("left sonar");
    Serial.println(left_sonarsensor_cm );
    delay(50); 

    // turn servo 90deg to right and read sonar
    sensor_servo.write(0);
    delay(1000);
    right_sonarsensor_cm = HC_SR04_range();
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
    sensor_servo.write(175);
    delay(1000);
    left_sonarsensor_cm = HC_SR04_range();

    // home towards left wall
    home(LEFT);
    delay(50);

    // check if aligned to correct wall
    // rotate servo 90 deg to right and read sonar
    sensor_servo.write(0);
    delay(1000);
    right_sonarsensor_cm = HC_SR04_range();

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
        sensor_servo.write(175);
        left_sonarsensor_cm = HC_SR04_range();
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
    right_sonarsensor_cm = HC_SR04_range();
    // home towards right wall
    home_side = RIGHT;
    home(RIGHT);
    delay(50);

    // check if aligned to correct wall
    // rotate servo 90 deg to left and read sonar
    sensor_servo.write(175);
    delay(1000);
    left_sonarsensor_cm = HC_SR04_range();
    if (left_sonarsensor_cm > 120) {
     // if true aligned on short wall
      Serial.print("Aligned to short wall");
      // rotate robot ~ 90 deg 
      cw();
      delay(2500);
      stop();
      // check which way is closer wall 
      left_sonarsensor_cm = HC_SR04_range();
      if (left_sonarsensor_cm < 50) {
        // home towards left wall
        home_side = LEFT;
        home(LEFT);
      } else {
        // turn servo 90 deg right
        sensor_servo.write(0);
        delay(1000);
        right_sonarsensor_cm = HC_SR04_range();
        // home towards right wall
        home_side = RIGHT;
        home(RIGHT);
    }   
  }
  }

  // align front/back direction 

  // turn servo forward
  sensor_servo.write(78);
  delay(1000);
  sonarsensor_cm = HC_SR04_range();
  Serial.print(" distance from far wall: ");
  Serial.println(sonarsensor_cm);
  
  speed_val = 150;
  if (sonarsensor_cm > 110) {
    home_face = BACK;
    while (sonarsensor_cm < 170) {
      //Serial.print(" distance from far wall: ");
      //Serial.println(sonarsensor_cm);
      sonarsensor_cm = HC_SR04_range();
      //reverse();
      straight_y_homing(-1);
      delay(20);
    }
  } else {
    home_face = FRONT;
    while (sonarsensor_cm > 6) {
    //forward();
    straight_y_homing(1);
    sonarsensor_cm = HC_SR04_range(); 
    delay(20);   
    }
  } 
  speed_val = 100;
  stop();

  // final adjustment 
    // align 
    	if (home_side == LEFT) {
        // turn servo 90 deg left
        sensor_servo.write(175);
        delay(1000);
        left_sonarsensor_cm = HC_SR04_range();
        align(LEFT);
        speed_val = 100;
        while (left_sonarsensor_cm > 12.5) {
          straight_x_homing(LEFT);
          left_sonarsensor_cm = HC_SR04_range();
          delay(10);
        }
        stop();
      } else {
        // turn servo 90 deg right
        sensor_servo.write(0);
        delay(1000);
        right_sonarsensor_cm = HC_SR04_range();
        align(RIGHT);
        while (right_sonarsensor_cm > 13.5) {
          straight_x_homing(RIGHT);
          right_sonarsensor_cm = HC_SR04_range();
          delay(10);
        }
        stop();
      }
    // turn servo forward
    sensor_servo.write(78);
    // check front and back distance
    if (home_face == BACK) {
        while (sonarsensor_cm < 170) {
          sonarsensor_cm = HC_SR04_range();
          straight_y_homing(-1);
          delay(10);
        }
    } else {
      while (sonarsensor_cm > 6) {
      straight_y_homing(1);
      sonarsensor_cm = HC_SR04_range(); 
      delay(20);   
      }
    }

  // add in final straight x and y move to make sure in corner
  SerialCom->println("Homing complete!");
  delay(500);
  stop();

  // reset gyros

  if (home_face == FRONT) {
    y_dir = -1;
  } else if (home_face == BACK) {
    y_dir = 1;
  }

  if (home_side == LEFT) {
    x_dir = -1;
  } else if (home_side == RIGHT) {
    x_dir = 1;
  }

  past_error_x = 0;
  past_error_y = 0;

  return PATH_FOLLOWING;
}

STATE path_following(){

  if ((direction == 1) && (y_dir == 1)) {
    if (HC_SR04_range() <= 7) {
      stop();
      sensor_servo.write(175);
      delay(300);
      direction = 2;
      x_distance = x_distance + 10;
      past_error_x = past_error_y;
      if (x_distance >= 106) {
        x_distance = 106;
      }
      y_dir = -1;
      speed_val = 120;
    } else {
      straight_y(y_dir);
    }
  } else if ((direction == 1) && (y_dir == -1)) {
    if (HC_SR04_range() >= 165) {
      stop();
      sensor_servo.write(175);
      delay(300);
      direction = 2;
      x_distance = x_distance + 10;
      past_error_x = past_error_y;
      if (x_distance >= 106) {
        x_distance = 106;
      }
      y_dir = 1;
      speed_val = 120;
    } else {
      straight_y(y_dir);
    }
  } else if (direction == 2) {

    if (HC_SR04_range() >= x_distance) {
      stop();
      sensor_servo.write(78);
      delay(300);
      if (x_distance == 106) {
        direction = 0;
      } else {
        direction = 1;
      }
      past_error_y = past_error_x;
      speed_val = 200;
    } else {
      straight_x(x_dir); // -1 is right, 1 is left
    }
  } else {
    stop();
  }
  
  return PATH_FOLLOWING;
}

void home(int dir) {
  speed_val = 150;
  if (dir == LEFT) {
    read_IR_sensors(LEFT);
    home_side = LEFT;
      // go toward left 
    while (left_sonarsensor_cm > 15 && backleftsensor_cm > 5) {
      //if (left_sonarsensor_cm < 25 && backleftsensor_cm < 7) {
      //break;
      //}
      //strafe_left();
      straight_x_homing(LEFT);
      read_IR_sensors(LEFT);  
      left_sonarsensor_cm = HC_SR04_range();
     
      delay(50);
    }
    stop();
    align(LEFT);
  } else {
    read_IR_sensors(RIGHT);
    home_side = RIGHT;
    while (right_sonarsensor_cm > 15 && backrightsensor_cm > 5) {
      //if (right_sonarsensor_cm <25 && backrightsensor_cm < 12) {
        // break;
      //}
      //strafe_right();
      straight_x_homing(RIGHT);
      read_IR_sensors(RIGHT);
      right_sonarsensor_cm = HC_SR04_range();
      delay(50);
    }
    stop();
    align(RIGHT);
  }
  speed_val = 100;
  return PATH_FOLLOWING;
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
    speed_val = 50 + 27.5*abs(error) + 0*abs(error_sum);

    // dir 1 = left, 2 = right
    if (dir == LEFT) {
      // left 
      if (error > 0) { 
        // front is further than back, turn cw
         Serial.println("ccw");
        ccw();
        delay(100);
        stop();
        delay(50);
      } else {
        // back is further than front, turn ccw
         Serial.println("cw");
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
    if (dir == LEFT) {
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

float HC_SR04_range() {
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
  // of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;

  // Print out results
  // if (pulse_width > MAX_DIST) {
  //   SerialCom->println("HC-SR04: Out of range");
  // } else {
  //   SerialCom->print("HC-SR04:");
  //   SerialCom->print(cm);
  //   SerialCom->println("cm");
  // }
  return cm;
}

float GYRO_reading() {
  float gyroZ;
  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GYROSCOPE_UNCALIBRATED) {
      gyroZ = sensorValue.un.gyroscope
                  .z;  // Current Measured Angular Velocity Around The Z Axis
      // SerialCom->print("Gyroscope I2C: ");
    }
  }
  return gyroZ;
}


//----------------------Battery-----------------------//
// Stop if Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) {  // print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");

  #ifndef NO_BATTERY_V_OK
    // 500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) {  // Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return ;
      }
    } else {
      counter_lipo_voltage_ok = 0;
    }
  #endif
  }
  return STOPPED;
}

void slow_flash_LED_builtin() {
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

boolean is_battery_voltage_OK() {
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  // the voltage of a LiPo cell depends on its chemistry and varies from
  // about 3.5V (discharged) = 717(3.5V Min)
  // https://oscarliang.com/lipo-battery-guide/ to about 4.20-4.25V (fully
  // charged) = 860(4.2V Max) Lipo Cell voltage should never go below 3V,
  // So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println(
          "Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println(
          "Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}

//----------------------Motor moments------------------------
// The Vex Motor Controller 29 use Servo Control signals to determine speed and
// direction, with 0 degrees meaning neutral
// https://en.wikipedia.org/wiki/Servo_control

void disable_motors() {
  left_front_motor.detach();   // detach the servo on pin left_front to turn Vex
                               // Motor Controller 29 Off
  left_rear_motor.detach();    // detach the servo on pin left_rear to turn Vex
                               // Motor Controller 29 Off
  right_rear_motor.detach();   // detach the servo on pin right_rear to turn Vex
                               // Motor Controller 29 Off
  right_front_motor.detach();  // detach the servo on pin right_front to turn
                               // Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors() {
  left_front_motor.attach(left_front);  // attaches the servo on pin left_front
                                        // to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to
                                      // turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear
                                        // to turn Vex Motor Controller 29 On
  right_front_motor.attach(
      right_front);  // attaches the servo on pin right_front to turn Vex Motor
                     // Controller 29 On
}

void stop() {
  left_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);
}

void cw() {
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void reverse() {
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void ccw() {
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

//----------------------STRAIGHT SPEED CONTROL------------------------//
void straight_y_homing(int dir) {
  past_error_y = 0;
  float error = GYRO_reading();
  float dif = error + past_error_y;

  // float left_g = 200;
  // float right_g = 140;
  float gain = 150;

  // float left_correction = abs(dif) * left_g;
  // float right_correction = abs(dif) * right_g;
  float correction = abs(dif) * gain;

  if (dir == 1) { //forward 
    if (dif > 0) {
      // SerialCom->print("positive error");
      left_front_motor.writeMicroseconds(1500 + speed_val + correction);
      left_rear_motor.writeMicroseconds(1500 + speed_val + correction);
      right_rear_motor.writeMicroseconds(1500 - speed_val); 
      right_front_motor.writeMicroseconds(1500 - speed_val);
    } else if (dif < 0) {
      // SerialCom->print("neg error");
      left_front_motor.writeMicroseconds(1500 + speed_val);
      left_rear_motor.writeMicroseconds(1500 + speed_val);
      right_rear_motor.writeMicroseconds(1500 - speed_val - correction);
      right_front_motor.writeMicroseconds(1500 - speed_val - correction); 
    } else {
      left_front_motor.writeMicroseconds(1500 + speed_val);
      left_rear_motor.writeMicroseconds(1500 + speed_val);
      right_rear_motor.writeMicroseconds(1500 - speed_val);
      right_front_motor.writeMicroseconds(1500 - speed_val); 
    }
  } else if (dir == -1) { //reverse
    if (dif > 0) {
      left_front_motor.writeMicroseconds(1500 - speed_val); 
      left_rear_motor.writeMicroseconds(1500 - speed_val);
      right_rear_motor.writeMicroseconds(1500 + speed_val + correction);
      right_front_motor.writeMicroseconds(1500 + speed_val + correction);
    } else if (dif < 0) {
      left_front_motor.writeMicroseconds(1500 - speed_val - correction);
      left_rear_motor.writeMicroseconds(1500 - speed_val - correction); 
      right_rear_motor.writeMicroseconds(1500 + speed_val);
      right_front_motor.writeMicroseconds(1500 + speed_val);
    } else {
      left_front_motor.writeMicroseconds(1500 - speed_val);
      left_rear_motor.writeMicroseconds(1500 - speed_val); 
      right_rear_motor.writeMicroseconds(1500 + speed_val);
      right_front_motor.writeMicroseconds(1500 + speed_val);
    }
  }
  past_error_y = error;

}

void straight_x_homing(int dir) {
  past_error_x = 0;
    float error = GYRO_reading();
    float dif = error + past_error_x;

    float gain = 250;

    float correction = abs(dif) * gain;
    // SerialCom->print("Correction:");
    // SerialCom->println(correction);

  if (dir == RIGHT) { //strafe right
    if (dif > 0) {
      left_front_motor.writeMicroseconds(1500 + ((speed_val)) + correction);
      left_rear_motor.writeMicroseconds(1500 - speed_val);
      right_rear_motor.writeMicroseconds(1500 - (speed_val*1.8)); //
      right_front_motor.writeMicroseconds(1500 + speed_val + correction);
    } else if (dif < 0) {
      left_front_motor.writeMicroseconds(1500 + (speed_val));
      left_rear_motor.writeMicroseconds(1500 - speed_val - correction); 
      right_rear_motor.writeMicroseconds(1500 - ((speed_val)*1.8) - correction);
      right_front_motor.writeMicroseconds(1500 + speed_val);
    } else {
       left_front_motor.writeMicroseconds(1500 + (speed_val));
      left_rear_motor.writeMicroseconds(1500 - (speed_val));
      right_rear_motor.writeMicroseconds(1500 - (speed_val * 1.8));
      right_front_motor.writeMicroseconds(1500 + speed_val);
    }
  } else if (dir == LEFT) { //strafe left
    if (dif > 0) {
      left_front_motor.writeMicroseconds(1500 - (speed_val)); 
      left_rear_motor.writeMicroseconds(1500 + speed_val + correction); 
      right_rear_motor.writeMicroseconds(1500 + ((speed_val) * 1.5) + correction); 
      right_front_motor.writeMicroseconds(1500 - speed_val);
    } else if (dif < 0) {
      left_front_motor.writeMicroseconds(1500 - ((speed_val)) - correction); 
      left_rear_motor.writeMicroseconds(1500 + speed_val);
      right_rear_motor.writeMicroseconds(1500 + (speed_val * 1.5));
      right_front_motor.writeMicroseconds(1500 - speed_val - correction); 
    } else {
      left_front_motor.writeMicroseconds(1500 - (speed_val));
      left_rear_motor.writeMicroseconds(1500 + speed_val);
      right_rear_motor.writeMicroseconds(1500 + (speed_val * 1.5));
      right_front_motor.writeMicroseconds(1500 - speed_val);
    }
  } 
  past_error_x = error;
}

void straight_y(int dir) {
float error = GYRO_reading();
  float dif = error + past_error_y;

  float gain = 150;

  if (HC_SR04_range() <= 45) {
      speed_val = (2*HC_SR04_range()) + 80;
    } else if (HC_SR04_range() >= 138) {
      speed_val = (-2*HC_SR04_range()) + 426;
    } else {
      speed_val = 150;
      gain = 200;
  }

  float correction = abs(dif) * gain;

  if (dir == 1) { //forward 
    if (dif > 0) {
      // SerialCom->print("positive error");
      left_front_motor.writeMicroseconds(1500 + speed_val + correction);
      left_rear_motor.writeMicroseconds(1500 + speed_val + correction);
      right_rear_motor.writeMicroseconds(1500 - speed_val); 
      right_front_motor.writeMicroseconds(1500 - speed_val);
    } else if (dif < 0) {
      // SerialCom->print("neg error");
      left_front_motor.writeMicroseconds(1500 + speed_val);
      left_rear_motor.writeMicroseconds(1500 + speed_val);
      right_rear_motor.writeMicroseconds(1500 - speed_val - correction);
      right_front_motor.writeMicroseconds(1500 - speed_val - correction); 
    } else {
      left_front_motor.writeMicroseconds(1500 + speed_val);
      left_rear_motor.writeMicroseconds(1500 + speed_val);
      right_rear_motor.writeMicroseconds(1500 - speed_val);
      right_front_motor.writeMicroseconds(1500 - speed_val); 
    }
  } else if (dir == -1) { //reverse
    if (dif > 0) {
      left_front_motor.writeMicroseconds(1500 - speed_val); 
      left_rear_motor.writeMicroseconds(1500 - speed_val);
      right_rear_motor.writeMicroseconds(1500 + speed_val + correction);
      right_front_motor.writeMicroseconds(1500 + speed_val + correction);
    } else if (dif < 0) {
      left_front_motor.writeMicroseconds(1500 - speed_val - correction);
      left_rear_motor.writeMicroseconds(1500 - speed_val - correction); 
      right_rear_motor.writeMicroseconds(1500 + speed_val);
      right_front_motor.writeMicroseconds(1500 + speed_val);
    } else {
      left_front_motor.writeMicroseconds(1500 - speed_val);
      left_rear_motor.writeMicroseconds(1500 - speed_val); 
      right_rear_motor.writeMicroseconds(1500 + speed_val);
      right_front_motor.writeMicroseconds(1500 + speed_val);
  }
}

past_error_y = dif; 
}

void straight_x(int dir)      { 
    float error = GYRO_reading();
    float dif = error + past_error_x;

    float gain = 250;

    float correction = abs(dif) * gain;

  if (dir == -1) { //strafe right
    if (dif > 0) {
      left_front_motor.writeMicroseconds(1500 + ((speed_val + correction)));
      left_rear_motor.writeMicroseconds(1500 - (speed_val));
      right_rear_motor.writeMicroseconds(1500 - (speed_val)); //
      right_front_motor.writeMicroseconds(1500 + ((speed_val + correction)));
    } else if (dif < 0) {
      left_front_motor.writeMicroseconds(1500 + (speed_val));
      left_rear_motor.writeMicroseconds(1500 - ((correction + speed_val))); 
      right_rear_motor.writeMicroseconds(1500 - ((correction + speed_val)));
      right_front_motor.writeMicroseconds(1500 + (speed_val));
    } else {
      left_front_motor.writeMicroseconds(1500 + (speed_val));
      left_rear_motor.writeMicroseconds(1500 - (speed_val));
      right_rear_motor.writeMicroseconds(1500 - (speed_val));
      right_front_motor.writeMicroseconds(1500 + (speed_val));
    }
  } else if (dir == 1) { //strafe left
    if (dif > 0) {
      left_front_motor.writeMicroseconds(1500 - (speed_val)); 
      left_rear_motor.writeMicroseconds(1500 + speed_val + correction); 
      right_rear_motor.writeMicroseconds(1500 + ((correction + speed_val))); 
      right_front_motor.writeMicroseconds(1500 - speed_val);
    } else if (dif < 0) {
      left_front_motor.writeMicroseconds(1500 - ((correction + speed_val))); 
      left_rear_motor.writeMicroseconds(1500 + speed_val );
      right_rear_motor.writeMicroseconds(1500 + (speed_val));
      right_front_motor.writeMicroseconds(1500 - speed_val - correction); 
    } else {
      left_front_motor.writeMicroseconds(1500 - (speed_val));
      left_rear_motor.writeMicroseconds(1500 + speed_val);
      right_rear_motor.writeMicroseconds(1500 + (speed_val)); 
      right_front_motor.writeMicroseconds(1500 - speed_val);
    }
  } 
  past_error_x = error;
}