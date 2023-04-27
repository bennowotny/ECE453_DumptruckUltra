
/*
#include <cy_result.h>
#include <cyhal_pwm.h>
#include <cy_pdl.h>

#define SERVO_0_PIN P0_4
#define SERVO_1_PIN P0_5
#define SERVO_2_PIN P0_6
#define SERVO_3_PIN P0_7
#define PWM_PERIOD  20000
#define PWM_MIN_PWIDTH  500
#define PWM_MAX_PWIDTH  2500

const float L1 = 94.0;          // 9.4 cm (from ground)
const float L2 = 97.0;          // 9.7 cm
const float L3 = 54.0;          // 5.4 cm
const int openM3 = 80;          // Servo angle for open claw
const int closeM3 = 180;        // Servo angle for closed claw
const int Servo_Cy_SysLib_Delay = 20;     // Cy_SysLib_Delay between servo movements

namespace Servo{
    cyhal_pwm_t servo_0;
    cyhal_pwm_t servo_1;
    cyhal_pwm_t servo_2;
    cyhal_pwm_t servo_3;

    ///////////////////////////////////////////////////
    // Initialization of Servos and PWM
    ///////////////////////////////////////////////////
    void initServos(){
        cy_rslt_t result;
        result = cyhal_pwm_init(&servo_0, SERVO_0_PIN, NULL);
        CY_ASSERT(result==CY_RSLT_SUCCESS);
        result = cyhal_pwm_init(&servo_1, SERVO_1_PIN, NULL);
        CY_ASSERT(result==CY_RSLT_SUCCESS);
        result = cyhal_pwm_init(&servo_2, SERVO_2_PIN, NULL);
        CY_ASSERT(result==CY_RSLT_SUCCESS);
        result = cyhal_pwm_init(&servo_3, SERVO_3_PIN, NULL);
        CY_ASSERT(result==CY_RSLT_SUCCESS);

        // Set PWM period and pulse width
        cyhal_pwm_set_period(&servo_0, PWM_PERIOD, PWM_MIN_PWIDTH);
        cyhal_pwm_set_period(&servo_1, PWM_PERIOD, PWM_MIN_PWIDTH);
        cyhal_pwm_set_period(&servo_2, PWM_PERIOD, PWM_MIN_PWIDTH);
        cyhal_pwm_set_period(&servo_3, PWM_PERIOD, PWM_MIN_PWIDTH);

        // Start PWM
        cyhal_pwm_start(&servo_0);
        cyhal_pwm_start(&servo_1);
        cyhal_pwm_start(&servo_2);
        cyhal_pwm_start(&servo_3);
    }

*/
    ///////////////////////////////////////////////////
    // Initial Function for IK
    ///////////////////////////////////////////////////
    //void inverseKinematics(double x,double y,double z,double L1,double L2,double L3,double &theta1,double &theta2,double &theta3){ 
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


    ///////////////////////////////////////////////////
    // Second function for testing IK
    ///////////////////////////////////////////////////
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


    ///////////////////////////////////////////////////
    // Function for incrementally moving the Servo
    // new_Servo: new degree of the Servo
    ///////////////////////////////////////////////////
    void gradualMovement(int M, int new_Servo, cyhal_pwm_t Servo){
        uint16_t pulse_width = (uint16_t)(PWM_MIN_PWIDTH + (PWM_MAX_PWIDTH-PWM_MIN_PWIDTH) * (new_Servo/180.0));
        if((new_Servo - M) >= 0){
            for(; M <= new_Servo; M++){
                Servo.write(M);
                cyhal_pwm_set_duty_cycle(&Servo, pulse_width, PWM_PERIOD);
                Cy_SysLib_Delay(Servo_Cy_SysLib_Delay); 
            }
        }else{
            for(; M > new_Servo; M--){
                cyhal_pwm_set_duty_cycle(&Servo, pulse_width, PWM_PERIOD);
                Cy_SysLib_Delay(Servo_Cy_SysLib_Delay);
            }
        }
    }


    ///////////////////////////////////////////////////
    // Function for resetting the arm position
    ///////////////////////////////////////////////////
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
        Cy_SysLib_Delay(2000);
    }


    ///////////////////////////////////////////////////
    // Function for dumping the arm once its in reset
    ///////////////////////////////////////////////////
    void dumpArmMotion() {
        // Assumes that the arm is currently in a reset position
        int currentM1 = Servo_1.read();
        int currentM2 = Servo_2.read();
        int reverseM1 = 0;
        int reverseM2 = 180;

        // Flip arm over and open/close claw
        gradualMovement(currentM1, reverseM1, Servo_1);
        gradualMovement(currentM2, reverseM2 , Servo_2);
        gradualMovement(Servo_3.read(), openM3, Servo_3);
        Cy_SysLib_Delay(2000);
        gradualMovement(Servo_3.read(), closeM3, Servo_3);  
        Cy_SysLib_Delay(1000);

        // Writing once more before reading/resetting helps to avoid jitter
        int currentM0 = Servo_0.read();
        resetArmPosition(currentM0, reverseM1, reverseM2, closeM3);
    }


    ///////////////////////////////////////////////////
    // Function for grabbing the object given IK results
    // theta1: angle of the first joint
    // theta2: angle of the second joint
    // theta3: angle of the third joint
    ///////////////////////////////////////////////////
    void moveArmForward(float theta1, float theta2, float theta3){
        int M0 = Servo_0.read();
        int M1 = Servo_1.read();
        int M2 = Servo_2.read();
        int M3 = Servo_3.read();
        gradualMovement(M0, (int)(theta1), Servo_0);
        gradualMovement(M1, (int)(theta2), Servo_1);
        gradualMovement(M2, (int)(theta3), Servo_2);
        gradualMovement(M3, openM3, Servo_3);
        Cy_SysLib_Delay(1000);
        gradualMovement(openM3, closeM3, Servo_3);
        Cy_SysLib_Delay(1000);

        // Reset to starting position
        resetArmPosition((int)(theta1), (int)(theta2), (int)(theta3), closeM3);
    }

};



int main(){*/
  /*
   * M0: Shoulder, lower angles to left w.r.t it facing towards you
   * M1: Elbow, lower angles above w.r.t it facing towards you
   * M2: Wrist, lower angles below w.r.t it facing towards you
   * M3: Claw, lower angles to open w.r.t it facing towards you
  resetArmPosition(90,30,100,closeM3);
  Cy_SysLib_Delay(1000);
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
  Cy_SysLib_Delay(2000);

  // Dump object into dumptruck
  dumpArmMotion();
  Cy_SysLib_Delay(1000);

  return 0;
}
*/