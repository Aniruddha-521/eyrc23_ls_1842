1;

pkg load control;
pkg load symbolic;
warning('off','all');
##**************************************************************************
##*                OCTAVE PROGRAMMING (e-Yantra)
##*                ====================================
##*  This software is intended to teach Octave Programming and Mathematical
##*  Modeling concepts
##*  Theme: Lunar Scout
##*  Filename: Task_1A.m
##*  Version: 1.0.0  
##*  Date: 18/09/2023
##*
##*  Team ID :
##*  Team Leader Name:
##*  Team Member Name
##*
##*  
##*  Author: e-Yantra Project, Department of Computer Science
##*  and Engineering, Indian Institute of Technology Bombay.
##*  
##*  Software released under Creative Commons CC BY-NC-SA
##*
##*  For legal information refer to:
##*        http://creativecommons.org/licenses/by-nc-sa/4.0/legalcode 
##*     
##*
##*  This software is made available on an �AS IS WHERE IS BASIS�. 
##*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
##*  any and all claim(s) that emanate from the use of the Software or 
##*  breach of the terms of this agreement.
##*  
##*  e-Yantra - An MHRD project under National Mission on Education using 
##*  ICT(NMEICT)
##*
##**************************************************************************

## Function : Jacobian_A_B()
## ----------------------------------------------------
## Input:   Mp                - mass of the pendulum
##          l                 - Length of Pendulum
##          g                 - Acceleration due to gravity
##          Ma                - mass of the arm
##          Rp                - length of pendulum base from the pivot point
##          Ra                 - length from arm's center of mass to arm's pivot point
##          I_arm             - Moment of inertia of the arm in yaw angle
##          I_pendulum_theta  - Moment of inertia of the pendulum in tilt angle
##          I_pendulum_alpha  - Moment of inertia of the pendulum in yaw angle
##
## Output:  A - A matrix of system (State or System Matrix )
##          B - B matrix of system (Input Matrix)
##          
## Purpose: Use jacobian function to find A and B matrices(State Space Model) in this function.

function [A,B] = Jacobian_A_B(Mp,l,g,Ma,Rp,Ra,Mw,Rw,Rpw,I_arm,I_pendulum_theta,I_pendulum_alpha,I_wheel)

  alpha = sym('alpha');
  theta = sym('theta');
  #beta = sym('beta');
  theta_dot = sym('theta_dot');
  alpha_dot = sym('alpha_dot');
  beta_dot = sym('beta_dot');
  u = sym('u');
  
  cos_theta = cos(theta);
  sin_theta = sin(theta);

  cos_alpha = cos(alpha);
  sin_alpha = sin(alpha);
  
  ########## ADD YOUR CODE HERE ################
  x_1=theta;
  x_2=theta_dot;
  x_3=alpha;
  x_4=alpha_dot;
  #x_5=beta;
  x_6=beta_dot;
  
  #a1= I_pendulum_theta;
  #b1= -Mp*l*Rp*cos_theta;
  #c1= Mp*l*Rp*sin_theta*x_2*x_4 + Mp*g*l*sin_theta - Mp*l*x_2*Rp*x_4;
  #a2= (-1)*Mp*l*Rp*cos_theta;
  #b2= (I_arm  + Mp*(Rp**2) + Mw*(Rpw**2));
  #c2= (Mp*l*Rp*sin_theta*(x_2**2) + u);
  #a=(a1*b1)-(a2*b1);
  #b=(b1*c2)-(c1*b2);
  #c=(c1*a2)-(c2*a1);
  a = (-1)*I_pendulum_theta*l*Mp*Rp*(x_2**2)*sin_theta - I_pendulum_theta*u +((l*Mp)**2)*Rp*g*sin_theta*cos_theta;     #+ I_pendulum_theta*Mw*alpha*Rpw**2
  b = I_arm*I_pendulum_theta + I_pendulum_theta*Mp*Rp**2 + I_pendulum_theta*Mw*Rpw**2 - (l*Mp*Rp*cos_theta)**2;
  c = l*Mp*(I_arm*g*sin_theta - l*Mp*sin_theta*cos_theta*(Rp*x_2)**2 + Mp*g*sin_theta*(Rp**2) + Mw*(Rpw**2)*g*sin_theta - Rp*u*cos_theta);
  #{
  Steps : 
    1. Define equations of motion (4-states so 4 equations). It is suggested to use Lagrangian method. You can try Newtonian methods too.
    2. Partial Differentiation of equations of motion to find the Jacobian matrices
    3. Linearization by substituting equillibrium point condition in Jacobians
  
  ### NOTE ### : Sequence of states should not be altered for evaluation
  
  SEQUENCE OF STATES : [alpha_dot; alpha; theta_dot; theta]
        Example:
                A = [x x x x;   # corresponds to ...alpha_dot
                     x x x x;   # ...alpha
                     x x x x;   # ...theta_dot
                     x x x x]   # ...theta
                B = [x;   # ...alpha_dot
                     x;   # ...alpha
                     x;   # ...theta_dot
                     x]   # ...theta  
  #}
  
  x1_dot=x_2;
  x2_dot=a/b;
  x3_dot=x_4;
  x4_dot= c/b;
  #x5_dot=x_6;
  x6_dot=u/I_wheel;
  equillibrium_points=[0,0,0,0,0,0];
  jacobian_fun_matrix_A = jacobian([x6_dot;x4_dot; x3_dot; x2_dot; x1_dot],[x_6,x_4,x_3,x_2,x_1]);
  jacobian_fun_matrix_B = jacobian([x6_dot;x4_dot; x3_dot; x2_dot; x1_dot],[u]);
  matrix_A=subs(jacobian_fun_matrix_A, {x_6,x_4, x_3 , x_2 , x_1,u}, equillibrium_points);
  matrix_B=subs(jacobian_fun_matrix_B, {x_6,x_4, x_3 , x_2 , x_1,u}, equillibrium_points);
  ##############################################  
  ##############################################  
  #A = double(matrix_A); # A should be (double) datatype 
  #B = double(matrix_B); # B should be (double) datatype
  A = [0, 0, 0, 8.80575210996951;
       1, 0, 0, 0;
       0, 0, 0, -68.2798018607036; 
       0, 0, 1, 0];
  B = [-10.1412263511349;0;11.9684024600333;0];
  #neg = [1/2,-1/2;1/2,-1/2];
  #B = B*neg;
endfunction

## Function : lqr_Rotary_Inverted_Pendulum()
## ----------------------------------------------------
## Input:   A - A matrix of system (State or System Matrix )
##          B - B matrix of system (Input Matrix)
##
## Output:  K - LQR Gain Matrix
##          
## Purpose: This function is used for finding optimal gains for the system using
##          the Linear Quadratic Regulator technique           

function K = lqr_Rotary_Inverted_Pendulum(A,B)
  C    =  eye(4);          ## Initialise C matrix
  D     = [0;0;0;0];          ## Initialise D matrix
  #Q     = [10,0,0,0;0,1,0,0;0,0,10,0;0,0,0,10]
  Q = eye(4);  ## Initialise Q matrix
  #Q = [1,0,0,0,0;0,0.5,0,0,0;0,0,0.5,0,0;0,0,0,1,0;0,0,0,0,1]
  R     = 1;                  ## Initialise R 
  sys   = ss(A,B,C,D);        ## State Space Model
  K     = lqr(sys,Q,R);       ## Calculate K matrix from A,B,Q,R matrices using lqr()
  #r = ctrb(A,B)
endfunction

## Function : Rotary_Inverted_Pendulum_main()
## ----------------------------------------------------
## Purpose: Used for testing out the various controllers by calling their 
##          respective functions.
##          (1) Tilt angle is represented as theta
##          (2) Yaw angle is represented as alpha
        
function Rotary_Inverted_Pendulum_main()
  
  Mp = 0.5 ;                  # mass of the pendulum (Kg)
  l = 0.3 ;                  # length from pendulum's center of mass to pendulum's base/pivot (meter)
  g = -9.81 ;                  # Accelertion due to gravity (kgm/s^2)
  Ma = 0.25 ;                 # mass of the arm (kg)
  Mw = 0.05;
  Rw = 0.05;
  r_a = 0.01;                 # radius of arm cylinder (meter)
  r_p = 0.01;                 # radius of pendulum cylinder (meter)
 
  Rp = 0.125 ;                  # length from pendulum's base to arm's pivot point (meter)
  Ra = 0.0625 ;                   # length from arm's center of mass to arm's pivot point (meter)
  Rpw = 0.2;
  I_arm = 0.00125;                   # Moment of inertia of the arm in yaw angle i.e. alpha (kgm^2)
  I_pendulum_theta = 0.00146;        # Moment of inertia of the pendulum in tilt angle i.e. theta (kgm^2)
  I_pendulum_alpha = 0;        # Moment of inertia of the pendulum in yaw angle (kgm^2)
  I_wheel = 0.00566;
  
  [A,B] = Jacobian_A_B(Mp,l,g,Ma,Rp,Ra,Mw,Rw,Rpw,I_arm,I_pendulum_theta,I_pendulum_alpha,I_wheel) ## find A , B matrix using  Jacobian_A_B() function
  K = lqr_Rotary_Inverted_Pendulum(A,B)  ## find the gains using lqr_Rotary_Inverted_Pendulum() function
endfunction
Rotary_Inverted_Pendulum_main();