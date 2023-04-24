#include <Servo.h>    // Create servo object to control a servo 

Servo Servo_0;
Servo Servo_1;
Servo Servo_2;
Servo Servo_3;

const float L1 = 94.0;          // 9.4 cm (from ground)
const float L2 = 97.0;          // 9.7 cm
const float L3 = 54.0;          // 5.4 cm
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


void inverseKinematics(double x,double y,double z,double L1,double L2,double L3,double &theta1,double &theta2,double &theta3){ 
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

void inverseKinematicsV2(double x, double y, double z, double L1, double L2, double L3, double &theta1, double &theta2, double &theta3){
    double d = sqrt(x*x + y*y + z*z);
    double cosTheta1 = (d*d + L1*L1 - L2*L2) / (2 * d * L1);
    theta1 = atan2(sqrt(1 - cosTheta1*cosTheta1), cosTheta1);
    double cosTheta2 = (L1*L1 + L2*L2 - d*d) / (2 * L1 * L2);
    theta2 = atan2(sqrt(1 - cosTheta2*cosTheta2), cosTheta2);
    theta3 = atan2(y, x) - (theta1 + theta2);

    // Radians to degrees
    theta1 = theta1 * 180.0 / M_PI;
    theta2 = theta2 * 180.0 / M_PI;
    theta3 = theta3 * 180.0 / M_PI;

    // Ensure that the joint angles are between 0 and 180 degrees
    if (theta1 < 0) 
        theta1 += 360.0;
    if (theta2 < 0)
        theta2 += 360.0;
    if (theta3 < 0)
        theta3 += 360.0;
    if (theta1 > 180.0)
        theta1 -= 180.0;
    if (theta2 > 180.0)
        theta2 -= 180.0;
    if (theta3 > 180.0)
        theta3 -= 180.0;
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
  float x = 50.0;
  float y = 0.0;
  float z = 0.0;

  // Inverse kinematics  
  double theta1 = 0.0;
  double theta2 = 0.0;
  double theta3 = 0.0;
  inverseKinematicsV2(x, y, z, L1, L2, L3, theta1, theta2, theta3);

  // Move arm accordingly
  moveArmForward(theta1, theta2, theta3);
  delay(2000);

  // Dump object into dumptruck
  dumpArmMotion();
  delay(1000);

  while(1);
}
