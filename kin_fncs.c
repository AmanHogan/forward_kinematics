#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.1415927
#endif

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

    // Find constant offset of theta[0] | Theta[0] = arcsin((d3)/sqrt(x^2 + y^2))
    double offset = asin((D[3])/(sqrt(pow((x[0]),2) + pow((x[1]), 2))));
    
    // Find theta[0] | Theta[0] = invertan(y/x) - offset
    theta[0] = atan(x[1]/x[0]) - offset;

    // Move base frame relative to wrist
    double x_1[3];
    x_1[0] = x[0] + D[2] * sin(theta[0]) ;
    x_1[1] = x[1] - D[2] * cos(theta[0]); 
    x_1[2] = x[2] - L[0];

    // Move tool frame to wrist
    double x_2[3];
    x_2[0] = x_1[0] - D[4] * cos(theta[0]);
    x_2[1] = x_1[1] - D[4] * sin(theta[0]);
    x_2[2] = x_1[2] + L[3];

    // Derive 3DOF Triangle
    double x_3[3];
    x_3[0] = sqrt((pow(x_2[0], 2)) + (pow(x_2[1],2)));
    x_3[1] = -x_2[2];

    // Derive Theta[2]
    double r = sqrt((pow(x_3[0], 2)) + (pow(x_3[1],2)));
    double beta = atan(x_3[1]/x_3[0]); 
    theta[2] = acos((pow(r,2) - pow(L[1],2) - pow(L[2], 2)) / (2 * L[1] * L[2]));

    // Derive Theta[1]
    double psi = acos((pow(r,2) + pow(L[1],2) - pow(L[2], 2)) / (2 * L[1] * r));
    theta[1] = beta + psi;

    // Derive Theta[3]
    theta[3] = (M_PI/2) - theta[1] - theta[2];
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
