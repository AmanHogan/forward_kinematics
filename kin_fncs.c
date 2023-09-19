#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.1415927
#endif

// Author: Aman Hogan
// C Library Reference: Manfred Huber
// Copyright: The University of Texas at Arlington
// 2238-CSE-4360-001 : Robotics


/**
 * Prints Jacobian Matrix
 * @param J[3][4] Jacobian Matrix
 * @returns None
**/
void print_jacobian(double J[3][4]);

/**
 * Finds Jacobian Matrix
 * @param theta[6] Contains five joint angles of the robot arm
 * @returns None   
**/
void jacobian(double theta[6]);

/**
 * Performs Forward Kinematics given joint angles
 * @param theta[6] Contains five joint angles of the robot arm
 * @param x[3] A vector with x,y,z components. This is the position of the tool frame
 * @returns None   
**/
void fwd_kin(double theta[6], double x[3]) 
{
    // Distances between joints: D[1]: -.04 | D[2]: .040 | D[3]: -.040 | D[4]: .040 |
    double D[6] = {0, -0.040, 0.040, -0.040, -0.040, 0 };

   // Lengths between joints: L[0]: -.04 | L[1]: .040 | L[2]: -.040 | L[3]: .040 |
    double L[4] = {0.250, .200, .200, .150};

    // Calculated X Position
    double X = cos(theta[0]) * ((L[1]*cos(theta[1])) + (L[2]*cos(theta[1]+theta[2])) + (D[4]*sin(theta[1]+theta[2]+theta[3])) + (L[3]*cos(theta[1]+theta[2]+theta[3]))) - (D[3]*sin(theta[0]));
    
    // Calculated Y Position
    double Y = sin(theta[0]) * ((L[1]*cos(theta[1])) + (L[2]*cos(theta[1]+theta[2])) + (D[4]*sin(theta[1]+theta[2]+theta[3])) + (L[3]*cos(theta[1]+theta[2]+theta[3]))) + (D[3]*cos(theta[0]));
    
    // Calculated Z Position
    double Z = (L[0]) - (L[1]*sin(theta[1])) - (L[2]*sin(theta[1]+theta[2])) +  (D[4]*cos(theta[1]+theta[2]+theta[3])) - (L[3]*sin(theta[1]+theta[2]+theta[3]));

    // Set Position Vector
    x[0] = X;
    x[1] = Y;
    x[2] = Z;

    // Additionally calculate Jacobian Matrix and print it
    jacobian(theta);
}

/**
 * @param x[3]
 * @param theta[6]  
**/
void inv_kin(double x[3], double theta[6]) 
{
    // Distances between joints: D[1]: -.04 | D[2]: .040 | D[3]: -.040 | D[4]: .040 |
    double D[6] = {0, -0.040, 0.040, -0.040, -0.040, 0 };

    // Lengths between joints: L[0]: -.04 | L[1]: .040 | L[2]: -.040 | L[3]: .040 |
    double L[4] = {0.250, .200, .200, .150};

    // Find offset of theta[0] then find theta[0] using offset
    // offset = arcsin(d3/sqrt(x^2 + y^2))
    // theta[0] = arctan(y/x) - offset
    double alpha = asin(D[3]/sqrt(pow(x[0], 2) + pow(x[1],2)));
    theta[0]  = atan(x[1]/x[0]) - alpha;

    // If theta[0] < 0, make sure it goes in the counterclockwise
    if (x[0] < 0)
        theta[0] = theta[0] + M_PI;

    // Get position of wrist relative to the shoulder frame
    // Now the Wrist is aligned with the Base Frame
    double x_1 = x[0] - (D[4]*cos(theta[0])) + (D[3]*sin(theta[0]));
    double y_1 = x[1] - (D[3]*cos(theta[0])) - (D[4]*sin(theta[0])); 
    double z_1 = x[2] + (L[3]) - (L[0]);

    // Gives us the 3 DOF in the X-Y plane
    double x_2 = sqrt(pow(x_1,2)+pow(y_1,2));
    double y_2 = -z_1;

    // Gives us the hypotenuse of the triangle
    double distance = sqrt(pow(x_2,2) + pow(y_2,2));

    // Intermediate angle offset of theta[1]
    double beta = atan(y_2/x_2); 
    
    // Intermediate calculation for theta[2]
    double theta_2 = (pow(distance,2) - pow(L[1],2) - pow(L[2],2)) / (2*L[1]*L[2]);

    // Intermediate calculation for theta[1]
    double gamma = (pow(distance,2) + pow(L[1],2) - pow(L[2],2)) / (2*distance*L[1]);

    // Refine range of gamma to avoid nan
    while (gamma > 1)
        gamma = gamma - 2;
    gamma = acos(gamma);

    // Refine range of theta[2] to avoid nan
    while (theta_2 > 1)
        theta_2 = theta_2 - 2;
    theta[2] = acos(theta_2);

    // Theta[1] can have two possible values
    // Choose correct value based on theta[2]
    if (theta[2] >= 0)
        theta[1] = beta - gamma;
    else
        theta[1] = beta + gamma;
    
    // Sum of angles property
    theta[3] = (M_PI/2) - (theta[1]) - (theta[2]);
}

void jacobian(double theta[6])
{
    // Distances between joints: D[1]: -.04 | D[2]: .040 | D[3]: -.040 | D[4]: .040 |
    double D[6] = {0, -0.040, 0.040, -0.040, -0.040, 0 };

    // Lengths between joints: L[0]: -.04 | L[1]: .040 | L[2]: -.040 | L[3]: .040 |
    double L[4] = {0.250, .200, .200, .150};

    double X = cos(theta[0]) * ((L[1]*cos(theta[1])) + (L[2]*cos(theta[1]+theta[2])) + (D[4]*sin(theta[1]+theta[2]+theta[3])) + (L[3]*cos(theta[1]+theta[2]+theta[3]))) - (D[3]*sin(theta[0]));
    double Y = sin(theta[0]) * ((L[1]*cos(theta[1])) + (L[2]*cos(theta[1]+theta[2])) + (D[4]*sin(theta[1]+theta[2]+theta[3])) + (L[3]*cos(theta[1]+theta[2]+theta[3]))) + (D[3]*cos(theta[0]));
    double Z = (L[0]) - (L[1]*sin(theta[1])) - (L[2]*sin(theta[1]+theta[2])) +  (D[4]*cos(theta[1]+theta[2]+theta[3])) - (L[3]*sin(theta[1]+theta[2]+theta[3]));

    // Partial Derivatives for x
    double x_0 = -sin(theta[0]) * ((L[1]*cos(theta[1])) + (L[2]*cos(theta[1]+theta[2])) + (D[4]*sin(theta[1]+theta[2]+theta[3])) + (L[3]*cos(theta[1]+theta[2]+theta[3]))) - (D[3]*cos(theta[0]));
    double x_1 = cos(theta[0]) * (-(L[1]*sin(theta[1])) - (L[2]*sin(theta[1]+theta[2])) + (D[4]*cos(theta[1]+theta[2]+theta[3])) - (L[3]*sin(theta[1]+theta[2]+theta[3])));
    double x_2 = cos(theta[0]) * (-(L[2]*sin(theta[1]+theta[2])) + (D[4]*cos(theta[1]+theta[2]+theta[3])) - (L[3]*sin(theta[1]+theta[2]+theta[3])));
    double x_3 = cos(theta[0]) * ((D[4]*cos(theta[1]+theta[2]+theta[3])) - (L[3]*sin(theta[1]+theta[2]+theta[3])));

    // Partial Derivatives for y
    double y_0 = cos(theta[0]) * ((L[1]*cos(theta[1])) + (L[2]*cos(theta[1]+theta[2])) + (D[4]*sin(theta[1]+theta[2]+theta[3])) + (L[3]*cos(theta[1]+theta[2]+theta[3]))) - (D[3]*sin(theta[0]));
    double y_1 = sin(theta[0]) * (-(L[1]*sin(theta[1])) - (L[2]*sin(theta[1]+theta[2])) + (D[4]*cos(theta[1]+theta[2]+theta[3])) - (L[3]*sin(theta[1]+theta[2]+theta[3])));
    double y_2 = sin(theta[0]) * (-(L[2]*sin(theta[1]+theta[2])) + (D[4]*cos(theta[1]+theta[2]+theta[3])) - (L[3]*sin(theta[1]+theta[2]+theta[3])));
    double y_3 = sin(theta[0]) * ((D[4]*cos(theta[1]+theta[2]+theta[3])) - (L[3]*sin(theta[1]+theta[2]+theta[3])));

    // Partial Derivatives for z
    double z_0 = 0;
    double z_1 = - (L[1]*cos(theta[1])) - (L[2]*cos(theta[1]+theta[2])) - (D[4]*sin(theta[1]+theta[2]+theta[3])) - (L[3]*cos(theta[1]+theta[2]+theta[3]));
    double z_2 = - (L[2]*cos(theta[1]+theta[2])) - (D[4]*sin(theta[1]+theta[2]+theta[3])) - (L[3]*cos(theta[1]+theta[2]+theta[3]));
    double z_3 = - (D[4]*sin(theta[1]+theta[2]+theta[3])) - (L[3]*cos(theta[1]+theta[2]+theta[3]));

    double jacobian_matrix[3][4] = {{x_0, x_1, x_2, x_3},
                                    {y_0, y_1, y_2, y_3},
                                    {z_0, z_1, z_2, z_3}};
    print_jacobian(jacobian_matrix);
}

void print_jacobian(double J[3][4]) 
{
    printf("Jacobian Matrix: \n");
    for (int i = 0; i < 3; i++) 
    {
        for (int j = 0; j < 4; j++) 
        {
            printf("%.4lf\t", J[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}