#include <SoftwareSerial.h>
#include <Servo.h>  //Need for Servo pulse output
//#include <Adafruit_BNO08x.h>  //Need for Gyroscope

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

//Servo Setup //
Servo sensor_servo;  

//Serial Pointer //
HardwareSerial *SerialCom;

// Sonar Sensor Set up //
float sonarsensor_cm = 0; // the calculated distance in cm from the sonar sensor
float left_sonarsensor_cm = 0; 
float right_sonarsensor_cm = 0; 

byte serialRead = 0; //for control serial communication

//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

#define INTERNAL_LED 13

// Serial Data input pin
#define BLUETOOTH_RX 19
// Serial Data output pin
#define BLUETOOTH_TX 18

#define STARTUP_DELAY 10 // Seconds
#define LOOP_DELAY 10 // miliseconds
#define SAMPLE_DELAY 10 // miliseconds


// USB Serial Port
#define OUTPUTMONITOR 0
#define OUTPUTPLOTTER 0

// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1

volatile int32_t Counter = 1;

SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

int pos = 0; ///?

void setup() {
  Serial.begin(115200);
  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  //Servo Setup for ultrasonic sensor
  sensor_servo.attach(10);
  //sensor_servo.write(180);

  // Use USB Serial for debug output and reserve Serial1 for command input only.
  SerialCom = &Serial1;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code");
  delay(1000);
  SerialCom->println("Setup....");

  pinMode(INTERNAL_LED, OUTPUT);

  BluetoothSerial.begin(115200);

  Serial.print("Ready, waiting for ");
  Serial.print(STARTUP_DELAY, DEC);
  Serial.println(" seconds");

  delay(1000);  //settling time but no really needed
}

void delaySeconds(int TimedDelaySeconds)
{
  for (int i = 0; i < TimedDelaySeconds; i++)
  {
    delay(1000);
  }
}

void flashLED(int LedNumber, int TimedDelay)
{
  digitalWrite(LedNumber, HIGH);
  delaySeconds(TimedDelay);
  digitalWrite(LedNumber, LOW);
  delaySeconds(TimedDelay);
}

void serialOutputMonitor(int32_t Value1, int32_t Value2, int32_t Value3)
{
  String Delimiter = ", ";
  
  Serial.print(Value1, DEC);
  Serial.print(Delimiter);
  Serial.print(Value2, DEC);
  Serial.print(Delimiter);
  Serial.println(Value3, DEC);
}

void serialOutputPlotter(int32_t Value1, int32_t Value2, int32_t Value3)
{
  String Delimiter = ", ";
  
  Serial.print(Value1, DEC);
  Serial.print(Delimiter);
  Serial.print(Value2, DEC);
  Serial.print(Delimiter);
  Serial.println(Value3, DEC);
}

void bluetoothSerialOutputMonitor(int32_t Value1, int32_t Value2, int32_t Value3)
{
  String Delimiter = ", ";
  
  BluetoothSerial.print(Value1, DEC);
  BluetoothSerial.print(Delimiter);
  BluetoothSerial.print(Value2, DEC);
  BluetoothSerial.print(Delimiter);
  BluetoothSerial.println(Value3, DEC);
}

void serialOutput(int32_t Value1, int32_t Value2, int32_t Value3)
{
  if (OUTPUTMONITOR)
  {
    serialOutputMonitor(Value1, Value2, Value3);
  }

  if (OUTPUTPLOTTER)
  {
    serialOutputPlotter(Value1, Value2, Value3);
  }

  if (OUTPUTBLUETOOTHMONITOR)
  {
    bluetoothSerialOutputMonitor(Value1, Value2, Value3);;
  }
}



void loop(){
// not sure what this bit is for 
  if (Serial.available()) {// Check for input from terminal
    serialRead = Serial.read(); // Read input
    if (serialRead==49){ // Check for flag to execute, 49 is ascii for 1, stop serial printing
      Serial.end(); // end the serial communication to get the sensor data
    }
  }

  bool front_wall_detected = false;
  bool left_wall_detected = false;
  bool right_wall_detected = false;

  // check front, left and right distances

  // add in if too close move away or something 
  sensor_servo.write(90);
  front_sonarsensor_cm = read_sonarsensor();
  if (front_sonarsensor_cm < 30){
    front_wall_detected = true;
  }
  sensor_servo.write(180);
  left_sonarsensor_cm = read_sonarsensor();
  if (left_sonarsensor_cm < 30){
    left_wall_detected = true;
  }
  sensor_servo.write(0);
  right_sonarsensor_cm = read_sonarsensor();
  if (right_sonarsensor_cm < 30){
    right_wall_detected = true;
  }

  // align to a wall 
  if (left_wall_detected){
    align_left();
  }
  else if (right_wall_detected){
    align_right();
  }
  else{
    if (!front_wall_detected){ //get front within range 
      while (front_sonarsenor_cm > 30){
        move_forward();
        front_sonarsensor_cm = read_sonarsensor();
      }
      stop();
    }
    // move left or right to align side to a wall
    if (left_sonarsenor_cm < right_sonarsensor_cm){
      sensor_servo.write(180);
      while (left_sonarsensor_cm > 30){
        strafe_left(); //add in speed change based on error later
        left_sonarsensor_cm = read_sonarsensor();
        // will need to check front sensor as well
      }
      stop();
      align_left();
    }
    else {
      sensor_servo.write(0);
      while (right_sonarsensor_cm > 30){
        strafe_right(); //add in speed change based on error later
        right_sonarsensor_cm = read_sonarsensor();
        // will need to check front sensor as well
      }
      stop();
      align_right();
    }
  }

  // at this point, robot should be aligned (left or right) to any wall 
    // set gyroscope to 0 orientation 
  
  sensor_servo.write(180);
  left_sonarsensor_cm = read_sonarsensor();
  
  sensor_servo.write(0);
  right_sonarsensor_cm = read_sonarsensor();

  if (left_sonarsensor_cm > 100 || right_sonarsensor_cm > 100){ //align to short wall instead of long wall
    ccw(); //90 degrees
    // using gryo and feedback to loop to ensure 90 
    if ( left_sonarsensor_cm > right_sonarsensor_cm){
      while (right_sonarsensor_cm > 30){
        sensor_servo.write(0);
        strafe_right(); //add in speed change based on error later
        right_sonarsensor_cm = read_sonarsensor();
        // will need to check front sensor as well
      }
      stop();
      align_right():
    }
    else{
      sensor_servo.write(180);
      while (left_sonarsensor_cm > 30){
        strafe_left(); //add in speed change based on error later
        left_sonarsensor_cm = read_sonarsensor();
        // will need to check front sensor as well
      }
      stop();
      align_left();
    }
    // reset gyroscope
  }
  sensor_servo.write(90);
  front_sonarsensor_cm = read_sonarsensor();
  if (front_sonarsenor_cm < 30){
    //move forward
  }

  // local adjustments to get within desired distance of the walls, using gyro to maintain align 
  // confirm with IR sensors 
  // reset gyro if need be (this is baseline orientation for the rest of the movement)


  

  delay(2000); 


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

