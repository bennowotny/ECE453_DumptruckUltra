#include <iostream>
#include <cmath>
using namespace std;



/****** DH parameters ********/
// a: distance between the z-axes of adjacent frames along the x-axis
// d: distance between the x-axes of adjacent frames along the z-axis
const double a1 = 0.1;      // length of first link
const double a2 = 0.2;      // length of second link
const double a3 = 0.1;      // length of third link
const double d1 = 0.15;     // distance between base and first link
const double d2 = 0.1;      // distance between first and second links
const double d3 = 0.05;     // distance between second link and end-effector


//////////////////////////////////////
// My implementation with explanations
//////////////////////////////////////
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
    //double z_wrist = L1 + (L2 * sin(theta2)) + (L3 * sin(theta2 + theta3));
    theta2 = atan2(y_wrist, x_wrist) - atan2(L3 * sin(theta3), L2 + (L3 * cos(theta3)));
}

//////////////////////////////////////
// More efficient implementation
//////////////////////////////////////
void inverse_kinematics_2(
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
    theta1 = atan2(y, x);
    double r2 = sqrt(x*x + y*y) - L1;
    double r3 = sqrt(r2*r2 + z*z);
    theta3 = acos((L2*L2 + L3*L3 - r3*r3) / (2.0*L2*L3));
    theta2 = atan2(z, r2) - atan2(L3*sin(theta3), L2 + L3*cos(theta3));

    //=============================
    //=============================

    double d = sqrt(x*x + y*y + z*z);
    double cosTheta1 = (d*d + L1*L1 - L2*L2) / (2 * d * L1);
    theta1 = atan2(sqrt(1 - cosTheta1*cosTheta1), cosTheta1);
    double cosTheta2 = (L1*L1 + L2*L2 - d*d) / (2 * L1 * L2);
    theta2 = atan2(sqrt(1 - cosTheta2*cosTheta2), cosTheta2);
    theta3 = atan2(y, x) - (theta1 + theta2);
}


void inverseKinematics4(double x, double y, double z, double &theta1, double &theta2, double &theta3, double L1, double L2, double L3){
    double d = sqrt(x*x + y*y + z*z);
    double cosTheta1 = (d*d + L1*L1 - L2*L2) / (2 * d * L1);
    theta1 = atan2(sqrt(1 - cosTheta1*cosTheta1), cosTheta1);
    double cosTheta2 = (L1*L1 + L2*L2 - d*d) / (2 * L1 * L2);
    theta2 = atan2(sqrt(1 - cosTheta2*cosTheta2), cosTheta2);
    theta3 = atan2(y, x) - (theta1 + theta2);
    // Convert the joint angles to degrees
    theta1 = theta1 * 180.0 / M_PI;
    theta2 = theta2 * 180.0 / M_PI;
    theta3 = theta3 * 180.0 / M_PI;

    // Ensure that the joint angles are between 0 and 180 degrees
    if (theta1 < 0) {
        theta1 += 360.0;
    }
    if (theta2 < 0) {
        theta2 += 360.0;
    }
    if (theta3 < 0) {
        theta3 += 360.0;
    }
    if (theta1 > 180.0) {
        theta1 -= 180.0;
    }
    if (theta2 > 180.0) {
        theta2 -= 180.0;
    }
    if (theta3 > 180.0) {
        theta3 -= 180.0;
    }
} 


/*
int main(){
    //double x = 50;      // 40 mm
    //double y = 19.6;    // 1.96 cm
    //double z = 19.6;    // 1.96 cm
    //double z = 0.0;


    double x = 50;
    double y = 50;
    double z = 0;

    // double L1 = 60.0;   // 6.0 cm
    double L1 = 94.0;   // 9.4 cm (from ground)
    double L2 = 97.0;   // 9.7 cm
    double L3 = 54.0;   // 5.4 cm        

    // Forward kinematics
    double theta1 = 0.0;
    double theta2 = 0.0;
    double theta3 = 0.0;
    double theta1_prime = 0.0;
    double theta2_prime = 0.0;
    double theta3_prime = 0.0;
    double theta1_prime3 = 0.0;
    double theta2_prime3 = 0.0;
    double theta3_prime3 = 0.0;

    // Initial arm position
    int initialTheta1 = 90;
    int initialTheta2 = 45;
    int initialTheta3 = 90;



    inverse_kinematics(x, y, z, L1, L2, L3, theta1, theta2, theta3);
    theta1 = theta1 * 180 / M_PI;
    theta2 = theta2 * 180 / M_PI;
    theta3 = theta3 * 180 / M_PI;

    inverse_kinematics_2(x, y, z, L1, L2, L3, theta1_prime, theta2_prime, theta3_prime);
    theta1_prime = theta1_prime * 180 / M_PI;
    theta2_prime = theta2_prime * 180 / M_PI;
    theta3_prime = theta3_prime * 180 / M_PI;

    inverseKinematics4(x, y, z, theta1_prime3, theta2_prime3, theta3_prime3, L1, L2, L3);


    printf("====================================\n");
    printf("Theta 1: %f\n", theta1);
    printf("Theta 2: %f\n", theta2);
    printf("Theta 3: %f\n", theta3);
    printf("====================================\n");

    printf("====================================\n");
    printf("Theta 1: %f\n", theta1_prime);
    printf("Theta 2: %f\n", theta2_prime);
    printf("Theta 3: %f\n", theta3_prime);
    printf("====================================\n");

    printf("====================================\n");
    printf("Theta 1: %f\n", theta1_prime3);
    printf("Theta 2: %f\n", theta2_prime3);
    printf("Theta 3: %f\n", theta3_prime3);
    printf("====================================\n");


    // initial + delta
    int finalTheta1 = initialTheta1 + theta1_prime3;
    int finalTheta2 = initialTheta2 + theta2_prime3;
    int finalTheta3 = initialTheta3 + theta3_prime3;
    printf("====================================\n");
    printf("Final Theta 1: %d\n", finalTheta1);
    printf("Final Theta 2: %d\n", finalTheta2);
    printf("Final Theta 3: %d\n", finalTheta3);
    printf("====================================\n");


    return 0;
}
*/