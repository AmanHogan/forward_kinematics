#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.1415927
#endif

// Counter-clockwise Rotation Matricies: 
// RX = {{1, 0, 0}, {0, cos(theta[]), -sin(theta[])}, {0, sin(theta[]), cos(theta[])}}
// RY = {{cos(theta[]), 0, sin(theta[])}, {0, 1, 0}, {-sin(theta[]), 0, cos(theta[])}}
// RZ = {{cos(theta[]), -sin(theta[]), 0}, {sin(theta[]), cos(theta[]), 0}, {0, 0, 1}}

/**
 * @param x*
 * @param theta[6]  
**/
void inv_kin(double *x, double theta[6]) { }

/**
 * Combines R and D to get T
 * @param R[3][3] Rotation Matrix
 * @param D[3] Displacement Matrix
 * @param T[4][4] Homogenous Transform Matrix 
 * @return None
**/
void transform(double R[3][3], double D[3], double T[4][4])
{
    int i = 0;
    int j = 0;
    for (i = 0; i < 3; i++)
    {
        for(j = 0; j < 3; j++)
        {
            T[i][j] = R[i][j];
        }
    }

    // Set Distance Vector in Transform matrix
    T[0][3] = D[0];
    T[1][3] = D[1]; 
    T[2][3] = D[2]; 
    T[3][3] = 1; 

    // Extend Matrix
    T[3][0] = 0; 
    T[3][1] = 0; 
    T[3][2] = 0; 
}

/**
 *  Multiplys two matricies to get resultant matrix
 * @param  A[4][4] Left Transform Matrix
 * @param  B[4][4] Right Transform Matrix
 * @param  C[4][4] Resultant Transform Matrix 
 * @return None
**/
void multiplyMatrices(double A[4][4], double B[4][4], double C[4][4]) 
{
    int i, j, k;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            C[i][j] = 0;
            for (k = 0; k < 4; k++) {
                C[i][j] += (double) A[i][k] * (double) B[k][j];
            }
        }
    }
}

/**
 * Prints Matrix
 * @param T[4][4] Homogenous Transform Matrix
 * @returns None
**/
void printMatrix(double T[4][4]) 
{
    for (int i = 0; i < 4; i++) 
    {
        for (int j = 0; j < 4; j++) 
        {
            printf("%.4lf\t", T[i][j]);
        }
        printf("\n");
    }
}

/**
 * Performs Forward Kinematics given joint angles
 * @param theta[6] Contains five joint angles of the robot arm
 * @param x[3] A vector with x,y,z components. This is the position of the tool frame
 * @returns None   
**/
void fwd_kin(double theta[6], double x[3]) 
{
    // Distances joints 
    double D1 = -0.040;
    double D2 = 0.040;
    double D3 = -0.040;
    double D4 = -0.040;

    // Lengths Between Joints
    double L0 = 0.250;
    double L1 = .200;
    double L2 = .200;
    double L3 = .150;

    ///////////////////////////////////////////////////////////////////
    //  Derivation: Transformation matrix from Tool frame to Base frame
    //  TTB = (T0B) (T10) (T21) (T32) (TT3)
    //  T0B = RZ(0) DZ(LO)
    //  T10 = RY(1) DX(L1) DY(D1)
    //  T21 = RY(2) DX(L2) DY(D2)
    //  T32 = RY(3) DZ(D4) DY(D3)
    //  TT3 = RX(4) DX(L3)
    ///////////////////////////////////////////////////////////////////

    // Frame from 0 to base
    // T0B = RZ(0) DZ(LO)
    double T0B[4][4];
    double R0B[3][3] = {{ cos(theta[0]), -sin(theta[0]), 0 }, {sin(theta[0]), cos(theta[0]), 0 }, {0, 0, (double) 1}};
    double D0B[3] =  {0, 0, L0};
    transform(R0B,D0B,T0B);
    
    // Frame from 1 to 0
    // T10 = RY(1) DX(L1) DY(D1)
    double T10[4][4];
    double R10[3][3] = {{cos(theta[1]), 0, sin(theta[1])}, {0, (double) 1, 0}, {-sin(theta[1]), 0, cos(theta[1])}};
    double D10[3] =  {cos(theta[1]) * L1 , D1, sin(theta[1]) * L1};
    transform(R10,D10,T10);

    // Frame from 2 to 1
    // T21 = RY(2) DX(L2) DY(D2)
    double T21[4][4];
    double R21[3][3] = {{cos(theta[2]), 0, sin(theta[2])}, {0, (double) 1, 0}, {-sin(theta[2]), 0, cos(theta[2])}};
    double D21[3] =  {L2, D2, sin(theta[2]) * L2};
    transform(R21,D21,T21);

    // Frame from 3 to 2
    // T32 = RY(3) DZ(D4) DY(D3)
    double T32[4][4];
    double R32[3][3] = {{cos(theta[3]), 0, sin(theta[3])}, {0, (double) 1, 0}, {-sin(theta[3]), 0, cos(theta[3])}};
    double D32[3] =  {sin(theta[3]) * D3, D3, (cos(theta[3]) * D4)};
    transform(R32,D32,T32);

    // Frame from T to 3
    // TT3 = RX(4) DX(L3)
    double TT3[4][4];
    double RT3[3][3] = {{(double) 1, 0, 0}, {0, cos(theta[4]), -sin(theta[4])}, {0, sin(theta[4]), cos(theta[4])}};
    double DT3[3] =  {L3, 0, 0};
    transform(RT3,DT3,TT3);

    // Outward Inward Derivation
    // TTB = (T0B) (T10) (T21) (T32) (TT3)
    double T1B[4][4];
    double T2B[4][4];
    double T3B[4][4];
    double TTB[4][4];

    // Multiply
    multiplyMatrices(T0B, T10, T1B);
    multiplyMatrices(T1B, T21, T2B);
    multiplyMatrices(T2B, T32, T3B);
    multiplyMatrices(T3B, TT3, TTB);

    // Set position to last column in Transformation Matrix
    x[0] = TTB[0][3];
    x[1] = TTB[1][3];
    x[2] = TTB[2][3];

}