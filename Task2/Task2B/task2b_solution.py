#python
import numpy as np
def sysCall_init():
    # do some initialization here
    global drive_motor, front_motor, reference_frame, bike_respondable
    global K, U
    global theta, theta_dot, alpha, alpha_dot
    global previous, current, yaw_setpoint, target_state, error
    error = 0.0
    theta = 0.0
    theta_dot = 0.0
    alpha = 0.0
    alpha_dot = 0.0
    U = 0.0
    yaw_setpoint = 0.0
    previous = 0.0
    reference_frame = sim.getObject("/reference_frame")
    front_motor = sim.getObject("/reference_frame/front_motor")
    bike_respondable = sim.getObject("/reference_frame/bike_respondable")
    drive_motor = sim.getObject("/reference_frame/drive_motor")
    target_state = np.array([0.0, 0.0, 0.0 , 0.0], dtype=np.float16)
    K = np.array([-1.2134 , -1.0000 ,  0.8260  , 6.1130], dtype=np.float16)

def sysCall_actuation():
    # put your actuation code here
    global drive_motor, front_motor
    global K, U
    global theta, theta_dot, alpha, alpha_dot
    global previous, current, yaw_setpoint, target_state, error
    
    target_state = np.array([0.0, yaw_setpoint + error, 0.0 , 0.0], dtype=np.float16)
    current_state = np.array([alpha_dot, alpha, theta_dot, theta])
    state_error = current_state - target_state
    U = np.dot(K, state_error)
    current = previous + (U)*sim.getSimulationTimeStep()/0.00054
    sim.setJointTargetVelocity(front_motor, current)
    previous = current
    sim.setJointTargetVelocity(drive_motor, 0)
def sysCall_sensing():
    # put your sensing code here
    global reference_frame, bike_respondable
    global theta, theta_dot, alpha, alpha_dot
    global yaw_setpoint, error
    
    angle = sim.getObjectOrientation(bike_respondable,reference_frame)
    velocity = sim.getObjectVelocity(bike_respondable,reference_frame)
    theta = angle[1]
    alpha = angle[2]
    theta_dot = velocity[1][1]
    alpha_dot = velocity[1][2]
    yaw_setpoint = sim.getFloatSignal("yaw_setpoint")
    error = yaw_setpoint - alpha

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details
