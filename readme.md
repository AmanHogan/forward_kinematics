
### Author: Aman Hogan 
#  Project and Summary

### Assignment description: This Homework assignment aims to derive the forward kinematics and a partial inverse kinematic function for a manipulator and implement them in a robot simulator. To do this you are provided with a C library containing the simulator. All your code should be written in the file kin fncs.c which contains 2 main functions, fwd kin(theta,x) and inv kin(x, theta). These functions are directly called by the simulator

# Set Up Ubuntu (Linux)

To install it under Ubuntu, you need to install a number of extra packages with the development libraries:
`apt-get install build-essential`
`apt-get install xutils-dev`
`apt-get install libxt-dev`
`apt-get install libxaw7-dev`
Put the tar file on your on your computer and uncompress it.
In a terminal:
- Go to the project folder (kin or dyn): `cd ~/<path to where you put the file>/hwk1_Linux64/kin`
- Build the Makefile (you only have to do this once): `xmkmf`
- Compile the project: `make`
- This will produce a few warnings and show you whether there were errors (ignore the can not build Kinematics.man error - this is just that it does not find documentation to build)
- Run the executable: `./Kinematics`
# Code Explanation

1.  **Function Declarations**:
    
    -   The code declares three functions: `print_jacobian`, `jacobian`, `fwd_kin`, and `inv_kin`. These functions are defined later in the code.
    -   These functions are used to calculate and print the Jacobian matrix, perform forward kinematics, and perform inverse kinematics for the robotic arm.
2.  **`fwd_kin` Function**:
    
    -   This function calculates the forward kinematics of the robotic arm based on joint angles (`theta`) and stores the end-effector position in the `x` array.
    -   It uses the arm's geometric parameters and trigonometric calculations to determine the end-effector position.
3.  **`inv_kin` Function**:
    
    -   This function calculates the inverse kinematics of the robotic arm to determine the joint angles (`theta`) needed to reach a specified end-effector position (`x`).
    -   It performs mathematical calculations to find the joint angles based on the given end-effector position and arm geometry.
4.  **`jacobian` Function**:
    
    -   This function calculates the Jacobian matrix for the robotic arm.
    -   The Jacobian matrix relates the rate of change of the end-effector's position to the rates of change of the joint angles.
    -   It computes the Jacobian matrix elements using partial derivatives with respect to joint angles.
5.  **`print_jacobian` Function**:
    
    -   This function is responsible for printing the Jacobian matrix to the console.
    -   It takes the Jacobian matrix as input and formats and prints its elements.

Overall, this code provides a framework for working with a robotic arm, allowing you to perform forward kinematics to calculate the end-effector position given joint angles and inverse kinematics to determine joint angles given an end-effector position. Additionally, it computes and prints the Jacobian matrix, which is essential for tasks like velocity control and trajectory planning in robotics.

# Contributions
- Author:  Aman Hogan
- Professor: Manfred Huber, The University of Texas at Arlington

