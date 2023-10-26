#python
import numpy as np
def sysCall_init():
    # do some initialization here
    global drive_motor, front_motor, roller, spherical_joint, reference_frame, bike_respondable
    global target_state, K, U
    global theta, theta_dot, alpha, alpha_dot
    theta = 0.0
    theta_dot = 0.0
    alpha = 0.0
    alpha_dot = 0.0
    U = 0.0
    reference_frame = sim.getObject("/reference_frame")
    drive_motor = sim.getObject("/reference_frame/drive_motor")
    front_motor = sim.getObject("/reference_frame/front_motor")
    roller = sim.getObject("/reference_frame/front_motor/roller")
    spherical_joint = sim.getObject("/reference_frame/spherical_joint")
    bike_respondable = sim.getObject("/reference_frame/bike_respondable")
    #cylinder = sim.getObject("/reference_frame/Cylinder")
    
    target_state = np.array([0.0, 0.0, 0.0 , 0.0], dtype=np.float16)
    K = np.array([ -1.04377 , -1.00000 ,  0.16676  , 8.13505], dtype=np.float16)

def sysCall_actuation():
    # put your actuation code here
    global K, theta, theta_dot, alpha, alpha_dot, U
    global drive_motor, front_motor
    current_state = np.array([alpha_dot, alpha, theta_dot, theta])
    state_error = current_state - target_state
    U = np.dot(K, state_error)#*945 #*972.44556
    #print(U)
    sim.setJointTargetForce(front_motor, U)

def sysCall_sensing():
    # put your sensing code here
    global drive_motor, front_motor, roller, spherical_joint, reference_frame, bike_respondable
    global theta, theta_dot, alpha, alpha_dot, beta_dot, prev_theta, beta
    theta = sim.getJointPosition(front_motor)
    #alpha = sim.getObjectPosition(cylinder, reference_frame)
    alpha = sim.getObjectOrientation(bike_respondable,reference_frame)
    theta_dot = sim.getJointVelocity(front_motor)
    alpha_dot = sim.getObjectVelocity(bike_respondable,reference_frame)
    print(alpha_dot)
    alpha = alpha[1]
    alpha_dot = alpha_dot[1][0]
    #print(f"alpha{alpha}")
    #print(f"alpha_dot{alpha_dot}")
    #alpha_dot = sim.getObjectVelocity(cylinder, reference_frame)
    #alpha = alpha[2]
    #alpha_dot = alpha_dot[0][2]
    #print(f"alpha {alpha}")
    #print(f"alpha_dot {alpha_dot}")

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details