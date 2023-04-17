#include <Servo.h>

Servo Servo_0;
Servo Servo_1;
Servo Servo_2;
Servo Servo_3;

// Print the data
#define DataPrint

// Record data
int SensVal[4] = {0};
int Joint0[50] = {0};
int Joint1[50] = {0};
int Joint2[50] = {0};
int Joint3[50] = {0};
int Dif0[50] = {0};
int Dif1[50] = {0};
int Dif2[50] = {0};
int Dif3[50] = {0};

int KeyValue = 0;
int Time = 0;
int M0 = 0, M1 = 0, M2 = 0, M3 = 0;

void setup() {
  Serial.begin(9600);

  // Attach the servos to the pins
  Servo_0.attach(4);
  Servo_1.attach(5);
  Servo_2.attach(6);
  Servo_3.attach(7);

  // Set input to pin 3
  pinMode(3, INPUT);

  // Read current value of the potentiometer and map it to the angle value
  ReadPot();
  Mapping0();

  // Record current value of potentiometer
  M0 = SensVal[0];
  M1 = SensVal[1];
  M2 = SensVal[2];
  M3 = SensVal[3];

  // Set the initial position of the arm 
  Servo_0.write(90);
  Servo_1.write(90);
  Servo_2.write(90);
  Servo_3.write(90);

}

void loop() {
  #ifdef DataPrint
  while(1){
    ReadPot();
    Serial.print("Sensor Value 0: ");
    Serial.println(SensVal[0]);
    Serial.print("Sensor Value 1: ");
    Serial.println(SensVal[1]);
    Serial.print("Sensor Value 2: ");
    Serial.println(SensVal[2]);
    Serial.print("Sensor Value 3: ");
    Serial.println(SensVal[3]);
    delay(200);
  }
  #endif

  // Read the current value of the potentiometer
  ReadPot();
  Mapping0();

  // First axis
  if (SensVal[0] > M0) {
    KeyValue = SensVal[0] - M0;
    Time = KeyValue * 0.5;
    Servo_0.write(90 + KeyValue);
    delay(Time);
  }
  else if (SensVal[0] < M0) {
    KeyValue = M0 - SensVal[0];
    Time = KeyValue * 0.5;
    Servo_0.write(90 - KeyValue);
    delay(Time);
  }
  // Second axis
  if (SensVal[1] > M1) {
    KeyValue = SensVal[1] - M1;
    Time = KeyValue * 0.5;
    Servo_1.write(90 + KeyValue);
    delay(Time);
  }
  else if (SensVal[1] < M1) {
    KeyValue = M1 - SensVal[1];
    Time = KeyValue * 0.5;
    Servo_1.write(90 - KeyValue);
    delay(Time);
  }
  // Third axis
  if (SensVal[2] > M2) {
    KeyValue = SensVal[2] - M2;
    Time = KeyValue * 0.5;
    Servo_2.write(90 + KeyValue);
    delay(Time);
  }
  else if (SensVal[2] < M2) {
    KeyValue = M2 - SensVal[2];
    Time = KeyValue * 0.5;
    Servo_2.write(90 - KeyValue);
    delay(Time);
  }
  // Fourth axis
  if (SensVal[3] > M3) {
    KeyValue = SensVal[3] - M3;
    Time = KeyValue * 0.5;
    Servo_3.write(90 + KeyValue);
    delay(Time);
  }
  else if (SensVal[3] < M3) {
    KeyValue = M3 - SensVal[3];
    Time = KeyValue * 0.5;
    Servo_3.write(90 - KeyValue);
    delay(Time);
  }

  // Record current value of potentiometer
  M0 = SensVal[0];
  M1 = SensVal[1];
  M2 = SensVal[2];
  M3 = SensVal[3];
  delay(10);
}
