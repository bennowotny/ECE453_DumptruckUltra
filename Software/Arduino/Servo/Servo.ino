#include <Servo.h>    // Create servo object to control a servo 

Servo Servo_0;
Servo Servo_1;
Servo Servo_2;
Servo Servo_3;

const double L1 = 87.0;
const double L2 = 50.0;
const double L3 = 82.0;
const int openM3 = 80;          // Servo angle for open claw
const int closeM3 = 180;        // Servo angle for closed claw
const int Servo_delay = 20;     // Delay between servo movements


void setup() {
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
    /********************************************
        x: desired x position
        y: desired y position
        z: desired z position
        L1: length of first link
        L2: length of second link
        L3: length of third link
        theta1: angle between base and first link [shoulder]
        theta2: angle between first and second links [elbow]
        theta3: angle between second link and end-effector [wrist]
    *********************************************/
    // IK for shoulder calculated using inverse tangent in the (x,y) plane
    theta1 = atan2(y, x);

    double rxy = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
    theta3 = acos((pow(rxy,2) - pow(L2,2) - pow(L3,2))/(2*L2*L3));
    theta3 = (z-L1)>0 ? theta3:-theta3;

    double x_wrist = cos(theta1) * ((L2 * cos(theta2)) + (L3 * cos(theta2 + theta3)));
    double y_wrist = sin(theta1) * ((L2 * cos(theta2)) + (L3 * cos(theta2 + theta3)));
    theta2 = atan2(y_wrist, x_wrist) - atan2(L3 * sin(theta3), L2 + (L3 * cos(theta3)));

    // Return in degrees and between 0 and 180
    theta1 = theta1 * 180 / M_PI;
    theta2 = theta2 * 180 / M_PI;
    theta3 = theta3 * 180 / M_PI;
    if(theta1 < 0) theta1 += 180;
    if(theta2 < 0) theta2 += 180;
    if(theta3 < 0) theta3 += 180;
    if(theta1 > 180) theta1 -= 180;
    if(theta2 > 180) theta2 -= 180;
    if(theta3 > 180) theta3 -= 180;
}



void gradualMovement(int M, int new_Servo, Servo Servo){
  // Gradual movement for Servo_0
  if((new_Servo - M) >= 0){
    for(; M <= new_Servo; M++){
      Servo.write(M);
      delay(Servo_delay); 
    }
  }else{
    for(; M > new_Servo; M--){
      Servo.write(M);
      delay(Servo_delay);
    }
  }
}




void resetArmPosition(int currentM0, int currentM1, int currentM2, int currentM3) {
  // Reset the initial positions of steering gears
  int initialM0 = 90;      
  int initialM1 = 30;
  //int initialM1 = 45;
  int initialM2 = 100;
  //int initialM2 = 90;
  int initialM3 = closeM3;

  // Get current positions
  gradualMovement(currentM0, initialM0, Servo_0);
  gradualMovement(currentM1, initialM1, Servo_1);
  gradualMovement(currentM2, initialM2, Servo_2);
  //gradualMovement(currentM3, initialM3, Servo_3);
  delay(2000);
}



void dumpArmMotion() {
  // Assumes that the arm is currently in a reset position
  int currentM1 = Servo_1.read();
  int currentM2 = Servo_2.read();
  //int currentM3 = Servo_3.read();
  int reverseM1 = 0;
  int reverseM2 = 180;

  // Flip arm over and open/close claw
  gradualMovement(currentM1, reverseM1, Servo_1);
  gradualMovement(currentM2, reverseM2 , Servo_2);
  gradualMovement(Servo_3.read(), openM3, Servo_3);
  delay(2000);
  gradualMovement(Servo_3.read(), closeM3, Servo_3);  
  delay(1000);

  // Writing once more before reading/resetting helps to avoid jitter
  int currentM0 = Servo_0.read();
  resetArmPosition(currentM0, reverseM1, reverseM2, closeM3);
}




void moveArmForward(float theta1, float theta2, float theta3){
  int M0 = Servo_0.read();
  int M1 = Servo_1.read();
  int M2 = Servo_2.read();
  int M3 = Servo_3.read();
  gradualMovement(M0, (int)(theta1), Servo_0);
  gradualMovement(M1, (int)(theta2), Servo_1);
  gradualMovement(M2, (int)(theta3), Servo_2);
  gradualMovement(M3, openM3, Servo_3);
  delay(1000);
  gradualMovement(openM3, closeM3, Servo_3);
  delay(1000);

  // Reset to starting position
  resetArmPosition((int)(theta1), (int)(theta2), (int)(theta3), closeM3);
}



/*
  - loop function
   ---------------------------------------------------------------------------*/
void loop() 
{
  /*
   * M0: Shoulder, lower angles to left w.r.t it facing towards you
   * M1: Elbow, lower angles above w.r.t it facing towards you
   * M2: Wrist, lower angles below w.r.t it facing towards you
   * M3: Claw, lower angles to open w.r.t it facing towards you
  */
  resetArmPosition(90,30,100,closeM3);
  delay(1000);
  gradualMovement(180, 90, Servo_3);    // open claw

  // DEFINE x,y,z location of the object
  /*
  float x = 55.0;
  float y = 0.0;
  float z = -15;

  // Inverse kinematics  
  double theta1 = 90.0;   // 90-theta1
  double theta2 = 150;    // 180-theta2
  double theta3 = 130;    // 100+theta3

  float x = 50;
  float y = 50;
  float z = -15;
  */

  // Inverse kinematics  
  //double theta1 = 90;                 // theta1
  //double theta2 = 180-(16.68778);     // 180-theta2
  //double theta3 = 180-(37.574513);    // 180-theta3
  double x = 50;
  double y = 50;
  double z = -15;
  double theta1 = 0.0;
  double theta2 = 0.0;
  double theta3 = 0.0;
  inverse_kinematics(x, y, z, L1, L2, L3, theta1, theta2, theta3);
  theta1 = 180-(theta1);
  theta2 = theta2>90 ? theta2 : 180-(theta2);
  theta3 = theta3>90 ? theta3 : 180-(theta3);
  //double theta1 = 180-(45.000000);
  //double theta2 = 127.040029;
  //double theta3 = 180-(60.811599);

  // Move arm accordingly
  moveArmForward(theta1, theta2, theta3);
  delay(2000);


  // Dump object into dumptruck
  //dumpArmMotion();
  delay(1000);

  while(1);
}
