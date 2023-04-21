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

void inverse_kinematics(
    double x,
    double y,
    double z,
    double L1,
    double L2,
    double L3,
    double &theta1,
    double &theta2,
    double &theta3
){ 
    /*
        x: desired x position
        y: desired y position
        z: desired z position
        L1: length of first link
        L2: length of second link
        L3: length of third link
        theta1: angle between base and first link [shoulder]
        theta2: angle between first and second links [elbow]
        theta3: angle between second link and end-effector [wrist]
    */
    // IK for shoulder calculated using inverse tangent in the (x,y) plane
    theta1 = atan2(y, x);

    // IK for wrist orientation calculated using cosine rule where the lengths
    // are L2, L3, and rxy (the distance between origin and the wrist) and where the angle between
    // L2 and L3 is theta3 and is opposite to L1.
    double rxy = sqrt(pow(x,2) + pow(y,2));
    theta3 = acos((pow(rxy,2) + pow((z-L1),2) - pow(L2,2) - pow(L3,2))/(2*L2*L3));
    theta3 = (z-L1)>0 ? theta3:-theta3;

    // IK for elbow orientation calculated by first computing the forward kinematics of the wrist
    // First term in theta2 is the angle between the x-axis and the projection of the wrist position onto the x-y plane
    // Second term in theta2 is the angle between the projection of the wrist position onto the x-y plane and the second link
    double x_wrist = cos(theta1) * ((L2 * cos(theta2)) + (L3 * cos(theta2 + theta3)));
    double y_wrist = sin(theta1) * ((L2 * cos(theta2)) + (L3 * cos(theta2 + theta3)));
    double z_wrist = L1 + (L2 * sin(theta2)) + (L3 * sin(theta2 + theta3));
    theta2 = atan2(y_wrist, x_wrist) - atan2(L3 * sin(theta3), L2 + (L3 * cos(theta3)));
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
  //int initialM1 = 30;
  int initialM1 = 45;
  //int initialM2 = 100;
  int initialM2 = 90;
  int initialM3 = 180;
  gradualMovement(M0, initialM0, Servo_0);
  gradualMovement(M1, initialM1, Servo_1);
  gradualMovement(M2, initialM2, Servo_2);
  gradualMovement(M3, initialM3, Servo_3);
  delay(2000);
}




void moveArmForward(float theta1, float theta2, float theta3){
  int M0 = Servo_0.read();
  int M1 = Servo_1.read();
  int M2 = Servo_2.read();
  int M3 = Servo_3.read();

  gradualMovement(M0, (int)(theta1), Servo_0);
  gradualMovement(M1, (int)(theta2), Servo_1);
  gradualMovement(M2, (int)(theta3), Servo_2);


  
}

/*
  - loop function
   ---------------------------------------------------------------------------*/
void loop() 
{

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


  // DEFINE lengths of the arms
  float L1 = 94.0;   // 9.4 cm (from ground)
  float L2 = 97.0;   // 9.7 cm
  double L3 = 54.0;   // 5.4 cm        

  // DEFINE x,y,z location of the object
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;


  ////////////////// RETURN IK VALUES //////////////////
  float theta1 = 60.0;
  float theta2 = 120.0;
  float theta3 = 90.0;
  int newM0 = (int)(theta1);
  int newM1 = (int)(theta2);
  int newM2 = (int)(theta3);
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
