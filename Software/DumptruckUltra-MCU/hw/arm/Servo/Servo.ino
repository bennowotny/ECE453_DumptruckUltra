/* -----------------------------------------------------------------------------
  Author             : Allen
  Check              : Amy
  Version            : V1.0
  Date               : 17/06/2016
  Description        : Initial positions of steering gears
  Company website    : http://www.sunfounder.com
   ---------------------------------------------------------------------------*/

/* Include -------------------------------------------------------------------*/
#include <Servo.h>    // Create servo object to control a servo 

/* Define -------------------------------------------------------------------*/
Servo Servo_0;
Servo Servo_1;
Servo Servo_2;
Servo Servo_3;

/*
  - setup function
  ---------------------------------------------------------------------------*/
void setup() 
{
  //Start the serial.
  Serial.begin(9600);
  
  //Attach the servos on pins to the servo object
  Servo_0.attach(4);
  Servo_1.attach(5);
  Servo_2.attach(6);
  Servo_3.attach(7);

}

void gradualMovement(int M, int new_Servo, Servo Servo){
  // Gradual movement for Servo_0
  if((new_Servo - M) >= 0){
    for(; M <= new_Servo; M++){
      Servo.write(M);
      delay(30);
    }
  }else{
    for(; M > new_Servo; M--){
      Servo.write(M);
      delay(30);
    }
  }
}


void resetArmPosition(int M0, int M1, int M2, int M3){
  // Reset the initial positions of steering gears
  int initialM0 = 90;      
  int initialM1 = 30;
  int initialM2 = 100;
  int initialM3 = 180;
  gradualMovement(M0, initialM0, Servo_0);
  gradualMovement(M1, initialM1, Servo_1);
  gradualMovement(M2, initialM2, Servo_2);
  gradualMovement(M3, initialM3, Servo_3);
  delay(2000);
}

/*
  - loop function
   ---------------------------------------------------------------------------*/
void loop() 
{
  double angle1 = 0;
  double angle2 = 0;
  double angle3 = 0;

  //Set the initial positions of arms
  resetArmPosition(90,30,100,180);
  delay(1000);
  gradualMovement(180, 90, Servo_3);    // open claw

  // M0: Shoulder, lower angles to left w.r.t it facing towards you
  // M1: Elbow, lower angles above w.r.t it facing towards you
  // M2: Wrist, lower angles below w.r.t it facing towards you
  // M3: Claw, lower angles to open w.r.t it facing towards you
  int M0 = Servo_0.read();
  int M1 = Servo_1.read();
  int M2 = Servo_2.read();
  int M3 = Servo_3.read();
  int newM0 = 60;
  int newM1 = 120;
  //int newM1 = 25; 
  int newM2 = 90;
  int openM3 = 80;
  int closeM3 = 170;
  gradualMovement(M0, newM0, Servo_0);
  gradualMovement(M1, newM1, Servo_1);
  gradualMovement(M2, newM2, Servo_2);
  gradualMovement(M3, openM3, Servo_3);
  delay(2000);
  gradualMovement(openM3, closeM3, Servo_3);
  delay(1000);
  //gradualMovement(160, newM3, Servo_3);
  //delay(2000);
  Servo_0.write(newM0);
  Servo_1.write(newM1);
  Servo_2.write(newM2);
  Servo_3.write(closeM3);
  int currentM0 = Servo_0.read();
  int currentM1 = Servo_1.read();
  int currentM2 = Servo_2.read();
  int currentM3 = Servo_3.read();
  resetArmPosition(currentM0, currentM1, currentM2, currentM3);
  int maxCloseM3 = 180;
  gradualMovement(closeM3, maxCloseM3, Servo_3);

  // Wait for 3 seconds
  //delay(3000);

  while(1);
}
