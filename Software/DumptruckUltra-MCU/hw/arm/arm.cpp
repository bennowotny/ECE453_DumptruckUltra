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
    double z_wrist = L1 + (L2 * sin(theta2)) + (L3 * sin(theta2 + theta3));
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
}


void forwardKinematics(double theta1, double theta2, double theta3, double &x, double &y, double &z) {
    // Calculate homogeneous transformation matrices
    double T01[4][4] = {{cos(theta1), -sin(theta1), 0, 0},
                        {sin(theta1), cos(theta1), 0, 0},
                        {0, 0, 1, d1},
                        {0, 0, 0, 1}};

    double T12[4][4] = {{cos(theta2), -sin(theta2), 0, a1},
                        {0, 0, -1, -d2},
                        {sin(theta2), cos(theta2), 0, 0},
                        {0, 0, 0, 1}};

    double T23[4][4] = {{cos(theta3), -sin(theta3), 0, a2},
                        {0, 0, 1, 0},
                        {-sin(theta3), -cos(theta3), 0, 0},
                        {0, 0, 0, 1}};

    double T34[4][4] = {{1, 0, 0, a3},
                        {0, 1, 0, 0},
                        {0, 0, 1, d3},
                        {0, 0, 0, 1}};

    double T04[4][4] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            T04[i][j] = T01[i][0]*T12[0][j] + T01[i][1]*T12[1][j] + T01[i][2]*T12[2][j] + T01[i][3]*T12[3][j];
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            T04[i][j] = T04[i][0]*T23[0][j] + T04[i][1]*T23[1][j] + T04[i][2]*T23[2][j] + T04[i][3]*T23[3][j];
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            T04[i][j] = T04[i][0]*T34[0][j] + T04[i][1]*T34[1][j] + T04[i][2]*T34[2][j] + T04[i][3]*T34[3][j];
        }
    }

    // Extract end-effector position
    x = T04[0][3];
    y = T04[1][3];
    z = T04[0][0];
}



int main(){
    double x = 2.0;
    double y = 1.0;
    double z = 1.5;
    double L1 = 1.0;
    double L2 = 2.0;
    double L3 = 1.5;

    // Forward kinematics
    double theta1 = 0.0;
    double theta2 = 0.0;
    double theta3 = 0.0;
    double theta1_prime = 0.0;
    double theta2_prime = 0.0;
    double theta3_prime = 0.0;
    inverse_kinematics(x, y, z, L1, L2, L3, theta1, theta2, theta3);
    inverse_kinematics_2(x, y, z, L1, L2, L3, theta1_prime, theta2_prime, theta3_prime);

    double px = L1*cos(theta1) + L2*cos(theta1)*cos(theta2) - L2*sin(theta1)*sin(theta2)*cos(theta3) + L3*(cos(theta1)*cos(theta2)*cos(theta3) - sin(theta1)*sin(theta3));
    double py = L1*sin(theta1) + L2*cos(theta2)*sin(theta1) - L2*cos(theta1)*sin(theta2)*cos(theta3) + L3*(cos(theta2)*sin(theta1)*cos(theta3) + cos(theta1)*sin(theta3));
    double pz = L1 + L2*sin(theta2)*sin(theta3) + L2*cos(theta2)*cos(theta3) + L3*sin(theta2)*cos(theta3);
    double px_prime = L1*cos(theta1) + L2*cos(theta1)*cos(theta2) - L2*sin(theta1)*sin(theta2)*cos(theta3) + L3*(cos(theta1)*cos(theta2)*cos(theta3) - sin(theta1)*sin(theta3));
    double py_prime = L1*sin(theta1) + L2*cos(theta2)*sin(theta1) - L2*cos(theta1)*sin(theta2)*cos(theta3) + L3*(cos(theta2)*sin(theta1)*cos(theta3) + cos(theta1)*sin(theta3));
    double pz_prime = L1 + L2*sin(theta2)*sin(theta3) + L2*cos(theta2)*cos(theta3) + L3*sin(theta2)*cos(theta3);
    printf("====================================");
    printf("x0: %f, y0: %f, z0: %f\n", x, y, z);
    printf("x: %f, y: %f, z: %f\n", px, py, pz);
    printf("x': %f, y': %f, z': %f", px_prime, py_prime, pz_prime);
    printf("====================================");

    return 0;
}