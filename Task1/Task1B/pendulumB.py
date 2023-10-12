# Once the CoppeliaSim ChildScripts are working, copy the child script for "Pendulum B" and paste it here.

#python
import numpy as np
###### GLOBAL VARIABLES HERE ######
base = None
motor = None
arm = None
pendulum = None
U = None
# You can add variables here 
# as required by your implementation.
###################################

def sysCall_init():
    # do some initialization here
    # This function will be executed once when the simulation starts
    
    ####### ADD YOUR CODE HERE ######
    # Hint: Initialize the scene objects which you will require 
    #       Initialize algorithm related variables here
    
    #################################
    global elbow_motor, elbow_free, pivot
    global target_state, K
    global theta, theta_dot, alpha, alpha_dot
    theta = 0.0
    theta_dot = 0.0
    alpha = 0.0
    alpha_dot = 0.0
    pivot = sim.getObject("/Pivot_B")
    elbow_motor = sim.getObject("/Pivot_B/Arm_B/Elbow_motor_B")
    elbow_free = sim.getObject("/Pivot_B/Arm_B/Elbow_free_B")
    
    target_state = np.array([0.0, 0.0, 0.0 , 0.0], dtype=np.float16)
    K = np.array([1.3766  ,  1.0000  ,  3.6560 ,  27.6569])

def sysCall_actuation():
    # put your actuation code here
    # This function will be executed at each simulation time step

    ####### ADD YOUR CODE HERE ######
    # Hint: Use the error feedback and apply control algorithm here
    #       Provide the resulting actuation as input to the actuator joint
    
    # Example psuedo code:
    #   x1 = error_state_1; # Error in states w.r.t desired setpoint
    #   x2 = error_state_2;
    #   x3 = error_state_3;
    #   x4 = error_state_4;
    #   k = [gain_1 , gain_1, gain_3, gain_4];      # These gains will be generated by control algorithm. For ex: LQR, PID, etc.
    #   U = -k[1]*x1 +k[2]*x2 -k[3]*x3 +k[4]*x4;    # +/- Sign convention may differ according to implementation
    #   Set_joint_actuation(U);                     # Provide this calculated input to system's actuator

    #################################
    global K, theta, theta_dot, alpha, alpha_dot, beta_dot,beta
    global pivot, elbow_motor
    current_state = np.array([alpha_dot, alpha, theta_dot, theta], dtype=np.float16)
    state_error = current_state - target_state
    U = np.dot(K, state_error)
    sim.setJointTargetForce(elbow_motor, U)

def sysCall_sensing():
    # put your sensing code here
    # This function will be executed at each simulation time step
    
    ####### ADD YOUR CODE HERE ######
    # Hint: Take feedback here & do the error calculation
    
    #################################
    global elbow_motor, elbow_free, pivot
    global theta, theta_dot, alpha, alpha_dot, beta_dot, prev_theta, beta
    theta = sim.getJointPosition(elbow_free)
    alpha = sim.getJointPosition(pivot)
    theta_dot = sim.getJointVelocity(elbow_free)
    alpha_dot = sim.getJointVelocity(pivot)
def sysCall_cleanup():
    # do some clean-up here
    # This function will be executed when the simulation ends
    
    ####### ADD YOUR CODE HERE ######
    # Any cleanup (if required) to take the scene back to it's original state after simulation
    # It helps in case simulation fails in an unwanted state.
    #################################
    pass

# See the user manual or the available code snippets for additional callback functions and details

