#include <iostream>
#include <cmath>
using namespace std;


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



//////////////////////////////////////
// Test Implementation
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
    theta1 = std::atan2(y, x);
    double c2 = (x*x + y*y + (z-L1)*(z-L1) - L2*L2 - L3*L3) / (2*L2*L3);
    double s2 = std::sqrt(1 - c2*c2);
    theta2 = std::atan2(z-L1, std::sqrt(x*x + y*y)) + std::acos((L2*L2 + (z-L1)*(z-L1) + x*x + y*y - L3*L3) / (2*L2*std::sqrt((z-L1)*(z-L1) + x*x + y*y)));
    theta3 = std::atan2(L3*s2, L2 + L3*c2) - std::atan2(z-L1, std::sqrt(x*x + y*y)) - M_PI/2 + theta2;

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


/*
int main(){
    double x = 50;      
    double y = 50;
    double z = -15;

    //double L1 = 94.0;   // 9.4 cm (from ground)
    //double L2 = 97.0;   // 9.7 cm
    //double L3 = 54.0;   // 5.4 cm
    double L1 = 87.0;
    double L2 = 50.0;
    double L3 = 82.0;

    // Forward kinematics
    double theta1 = 0.0;
    double theta2 = 0.0;
    double theta3 = 0.0;
    double theta1_prime = 0.0;
    double theta2_prime = 0.0;
    double theta3_prime = 0.0;

    // Initial arm position
    int initialTheta1 = 90;
    int initialTheta2 = 30;
    int initialTheta3 = 100;

    inverse_kinematics(x, y, z, L1, L2, L3, theta1, theta2, theta3);
    inverse_kinematics_2(x, y, z, L1, L2, L3, theta1_prime, theta2_prime, theta3_prime);

    printf("====================================\n");
    printf("Theta 1: %f\n", theta1);
    printf("Theta 2: %f\n", theta2);
    printf("Theta 3: %f\n", theta3);
    printf("====================================\n");

    printf("====================================\n");
    printf("Theta 1 Prime: %f\n", theta1_prime);
    printf("Theta 2 Prime: %f\n", theta2_prime);
    printf("Theta 3 Prime: %f\n", theta3_prime);
    printf("====================================\n");

    return 0;
}
*/