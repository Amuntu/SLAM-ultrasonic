#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX
/*** RC-MOTORS  Pins ***/
#define R_EN  8  //Right Motor enable Pin.
#define R_in1 28 //Right Motor 1st direction Pin.
#define R_in2 29 //Right Motor 2nd direction Pin.
#define L_EN  9  //Left Motor enable Pin.
#define L_in3 30 //Left Motor 1st direction Pin.
#define L_in4 31 //Left Motor 2nd direction Pin.
/*** ENCODERS Pins ***/
#define L_channel_A 2 //Right Motor Encoder channel(A) Pin.
#define L_channel_B 4 //Right Motor Encoder channel(B) Pin.
#define R_channel_A 3 //Left Motor Encoder channel(A) Pin.
#define R_channel_B 5 //Left Motor Encoder channel(B) Pin.
/*** ULTRASONIC Sensors Pins ***/
#define R_trig 22 //Right UltraSonic Sensor (Trigger) Pin.
#define R_echo 23 //Right UltraSonic Sensor (Echo) Pin.
#define M_trig 24 //Middle UltraSonic Sensor (Trigger) Pin.
#define M_echo 25 //Middle UltraSonic Sensor (Echo) Pin.
#define L_trig 26 //Left UltraSonic Sensor (Trigger) Pin.
#define L_echo 27 //Left UltraSonic Sensor (Echo) Pin.

/*** Encoders Variables & Functions ***/
#define R_COUNT_PER_REVELUTION 610.0 //Right Encoder Pulses per-revolution.
#define L_COUNT_PER_REVELUTION 610.0 //Left Encoder Pulses per-revolution.
#define encoderReadingEadge CHANGE
byte readingFactor = 1;
double L_COUNT = 0, R_COUNT = 0; // Encoder number of counts every [dt] period of Time for Integration & Derivation to calculate Speeds and PID-control.

void R_ENC_interrupt_A() { // Interrupt function of the Right-Encoder on channel(A).
  R_COUNT++;
}
void L_ENC_interrupt_A() { // Interrupt function of the Left-Encoder on channel(A).
  L_COUNT++;
}
void R_ENC_interrupt_B();  // Interrupt function of the Right-Encoder on channel(B).
void L_ENC_interrupt_B();  // Interrupt function of the Left-Encoder on channel(B).
void calculateAcualSpeeds();
void attachEncoders(), detachEncoders();
/*** Motors Speeds Variables & Functions ***/
#define R_w_max 9.5 //Maximum Speed of the Right Motor in [rad/s].
#define L_w_max 9.5 //Maximum Speed of the Left Motor in [rad/s].

float R_w_encoder = 0, L_w_encoder = 0; // Real motors speeds in [rad/s], Comes from encodes.
float R_w_encoder_pid = 0, L_w_encoder_pid = 0; // After -PID- motors speeds in [rad/s], Comes from PID-Control function.
float R_w_encoder_ref = 4, L_w_encoder_ref = 4;
int R_dir = 0, L_dir = 0;  // Variables to deside the direction for each motor (1 -> CW, 0 -> Stop, -1 -> CCW).

/*** Kinematics Variable ***/
#define b 0.1800  //Distance btw 2 wheels [m].
#define r 0.0325  // Radius of each wheel [m].

float x = 0;      // Center position on X-axis [m].
float y = 0;      // Center position on Y-axis [m].
float theta = 0;  // (M_PI / 2.0) // Angle of the robot.
float V_real = 0, W_real = 0; // Acuatl (Linear[m/s] & Angular[rad/s]) Speeds of the robot.
void poistion(); //For updating the position of the robot.

/*** PID Variables & Functions ***/
#define R_Kp 0.40000 // Kp factor for the Right motor.
#define R_Ki 0.00001 // Ki factor for the Right motor.
#define R_Kd 0.00300 // Kd factor for the Right motor.
#define L_Kp 0.40000 // Kp factor for the Left motor.
#define L_Ki 0.00005 // Ki factor for the Left motor.
#define L_Kd 0.00300 // Kd factor for the Left motor.
#define PID_interval 8.0 //In [ms].

float lastPID_millis = 0;
double dt; // In [ms].

float R_P = 0, L_P = 0;
float R_I = 0, L_I = 0;
float R_D = 0, L_D = 0;
float R_errorOld = 0, L_errorOld = 0;

void PID(); // PID-Control function.

/*** UltraSonic Variables & Functions ***/
#define echoInterval 3000 // Interval for waiting the sensor to return value in [us], the naximum distance in this case is about 50cm.
#define minDistFromObs 15.0 // Minimum distance threshold for the ultrasonic to consider it the robot get into the (Danger-Zone).

float ObsDistance = 0;
byte ultraSonic_trigPin[3] = {L_trig, M_trig, R_trig};
byte ultraSonic_echoPin[3] = {L_echo, M_echo, R_echo};
bool readNow = false;
byte index_current_UltraSonicSensor = 0; // 0: Left, 1:Middle, 2:Right.
void triggerSensor(byte);             // Function for setting the command to Trigger the selected UltraSonic Sensor.
float getDistanceFromSensor(byte);       // Function for setting the command to read from the selected UltraSonic Sensor in [cm].
void setupPinsConfigurations();
float ultrasonicSensors_readings[3] = {0, 0, 0};
void setup() {
  setupPinsConfigurations();
  Serial.begin(115200); // Setting Serial Port with a BaudRate of 115200[bit/s].
   mySerial.begin(9600);
   lastPID_millis = millis();
  if (encoderReadingEadge == CHANGE)
    readingFactor = 2;
}
void loop() {
   R_w_encoder_ref = 5;
   L_w_encoder_ref = 5;
  if(ultrasonicSensors_readings[0]>0 && ultrasonicSensors_readings[0] <minDistFromObs){
    R_w_encoder_ref = 5;
    L_w_encoder_ref = -5;
  }
  if(ultrasonicSensors_readings[1]>0 && ultrasonicSensors_readings[1] <minDistFromObs){
    R_w_encoder_ref = 5;
    L_w_encoder_ref = -5;
  }
  if(ultrasonicSensors_readings[2]>0 && ultrasonicSensors_readings[2] <minDistFromObs){
    R_w_encoder_ref = 5;
    L_w_encoder_ref = -5;
  }
  String msg = String(x) + ","  + String(y) +  "," + String(theta*180.0/M_PI) + "," 
                         + String(ultrasonicSensors_readings[0]) + "," 
                         + String(ultrasonicSensors_readings[1]) + "," 
                         + String(ultrasonicSensors_readings[2])
                         + "\n";
  if (millis() - lastPID_millis > PID_interval) {
    dt = (millis() - lastPID_millis) * 1e-3;
    detachEncoders();
    calculateAcualSpeeds();
    PID();
    setMotorsDirection_and_run();
    readNow = true;
    poistion();
    mySerial.print(msg);
    attachEncoders();
    lastPID_millis = millis();
  }
  if (readNow) {
    triggerSensor(index_current_UltraSonicSensor);
    ultrasonicSensors_readings[index_current_UltraSonicSensor] = getDistanceFromSensor(index_current_UltraSonicSensor);
    index_current_UltraSonicSensor++;
    if (index_current_UltraSonicSensor > 2)
      index_current_UltraSonicSensor = 0;
    readNow = false;
  }
}
void setupPinsConfigurations() {
  // Setting Motors Pins:
  pinMode(R_EN, OUTPUT);  pinMode(L_EN, OUTPUT);
  pinMode(R_in1, OUTPUT); pinMode(L_in3, OUTPUT);
  pinMode(R_in2, OUTPUT); pinMode(L_in4, OUTPUT);
  // Setting Encoders Pins:
  pinMode(R_channel_A, INPUT); pinMode(R_channel_B, INPUT);
  pinMode(L_channel_A, INPUT); pinMode(L_channel_B, INPUT);
  attachEncoders();
  // Setting UltraSonic Sensor Pins:
  pinMode(R_trig, OUTPUT); pinMode(R_echo, INPUT);
  pinMode(M_trig, OUTPUT); pinMode(M_echo, INPUT);
  pinMode(L_trig, OUTPUT); pinMode(L_echo, INPUT);
}
void triggerSensor(byte index) {
  digitalWrite(ultraSonic_trigPin[index], HIGH);
  delayMicroseconds(10);
  digitalWrite(ultraSonic_trigPin[index], LOW);
}
float getDistanceFromSensor(byte index) {
  return pulseIn(ultraSonic_echoPin[index], HIGH, echoInterval) * .0343 / 2.0;
}
void calculateAcualSpeeds() {
  L_w_encoder = ((2.0 * M_PI) * float(L_COUNT) / (readingFactor * L_COUNT_PER_REVELUTION)) / dt;
  R_w_encoder = ((2.0 * M_PI) * float(R_COUNT) / (readingFactor * R_COUNT_PER_REVELUTION)) / dt;
  L_COUNT = 0;
  R_COUNT = 0;
}
void PID() {
  float R_error = R_w_encoder_ref - (R_dir * R_w_encoder);   // Get error.
  R_P = R_Kp * R_error;                     // Calcultate P-term.
  R_I = R_I + R_Ki * R_error;               // Calcultate I-term.
  R_D = R_Kd * (R_error - R_errorOld) / dt; // Calcultate D-term.
  R_errorOld = R_error;  // Update last error.

  float L_error = L_w_encoder_ref - (L_dir * L_w_encoder);
  L_P = L_Kp * L_error;
  L_I = L_Ki * (L_I + L_error);
  L_D = L_Kd * (L_error - L_errorOld) / dt;
  L_errorOld = L_error;
  //Appling pid to the motor speed:
  R_w_encoder_pid += (R_P + R_D + R_I);
  L_w_encoder_pid += (L_P + L_D + L_I);

  // Constrain the motor speed within the motors maximum speeds:
  if (R_w_encoder_pid > R_w_max)
    R_w_encoder_pid = R_w_max;
  else if (R_w_encoder_pid < -R_w_max)
    R_w_encoder_pid = -R_w_max;
  if (L_w_encoder_pid > L_w_max)
    L_w_encoder_pid = L_w_max;
  else if (L_w_encoder_pid < -L_w_max)
    L_w_encoder_pid = -L_w_max;
}
void setMotorsDirection_and_run() {
  if (R_w_encoder_pid < 0) R_dir = -1;  // Right motor CCW
  else R_dir = 1;                       // CW
  if (L_w_encoder_pid < 0) L_dir = -1;  // Left motor CCW
  else L_dir = 1;                       // CW

  if (R_dir > 0) {
    digitalWrite(R_in1, HIGH);
    digitalWrite(R_in2, LOW);
  } else if (R_dir < 0) {
    digitalWrite(R_in1, LOW);
    digitalWrite(R_in2, HIGH);
  } else {
    digitalWrite(R_in1, LOW);
    digitalWrite(R_in2, LOW);
  }
  if (L_dir > 0) {
    digitalWrite(L_in3, HIGH);
    digitalWrite(L_in4, LOW);
  }
  else if (L_dir < 0) {
    digitalWrite(L_in3, LOW);
    digitalWrite(L_in4, HIGH);
  }
  else {
    digitalWrite(L_in3, LOW);
    digitalWrite(L_in4, LOW);
  }
  analogWrite(R_EN, fabs(R_w_encoder_pid * 255.0 / R_w_max));
  analogWrite(L_EN, fabs(L_w_encoder_pid * 255.0 / L_w_max));
}
void poistion() {
  // Left & Right real (from encodes) linear-speeds of wheels[m/s]:
  float Vr_real = (R_dir * R_w_encoder) * r;
  float Vl_real = (L_dir * L_w_encoder) * r;

  V_real = (Vr_real + Vl_real) / 2.0;
  W_real = (Vr_real - Vl_real) / b;

  x = x + V_real * cos(theta) * dt;
  y = y + V_real * sin(theta) * dt;
  theta = theta + W_real * dt;
}
void detachEncoders() {
  detachInterrupt(digitalPinToInterrupt(R_channel_A));// detachInterrupt(digitalPinToInterrupt(R_channel_B));
  detachInterrupt(digitalPinToInterrupt(L_channel_A)); //detachInterrupt(digitalPinToInterrupt(L_channel_B));
}
void attachEncoders() {
  attachInterrupt(digitalPinToInterrupt(R_channel_A), R_ENC_interrupt_A, encoderReadingEadge);// attachInterrupt(digitalPinToInterrupt(R_channel_B), R_ENC_interrupt_B, RISING);
  attachInterrupt(digitalPinToInterrupt(L_channel_A), L_ENC_interrupt_A, encoderReadingEadge); //attachInterrupt(digitalPinToInterrupt(L_channel_B), L_ENC_interrupt_B, RISING);
}
