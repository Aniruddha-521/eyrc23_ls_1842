1;
Function_File;
pkg load symbolic      # Load the octave symbolic library
syms x1 x2             # Define symbolic variables x1 and x1
x1_dot = -x1 + 2*x1^3 + x2;       # Write the expressions for x1_dot and x2_dot
x2_dot = -x1 - x2;   # YOU CAN MODIFY THESE 2 LINES TO TEST OUT OTHER EQUATIONS
[x_1 x_2 jacobians eigen_values stability] = main_function(x1_dot, x2_dot);
#matrix=[0,1,0;3,-2,1;1,-2,4];
#mat={};
#mat(1)=double(matrix);
#[my_output stability_status]=check_eigen_values(x_1, x_2,mat);
