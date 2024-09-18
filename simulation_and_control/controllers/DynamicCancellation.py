import numpy as np


def dyn_cancel(dyn_model,q_,qd_, u):
    """
    Perform feedback solely the dynamic cancellation  on a robotic system.

    Parameters:
    - dyn_model (pin_wrapper): The dynamics model of the robot encapsulated within a 'pin_wrapper' object,
                               which provides methods for computing robot dynamics such as mass matrices,
                               Coriolis forces, etc.
    - u (numpy.ndarray): The control input to be applied to the robot, computed by a higher-level controller

    Returns:
    None

    This function computes the control inputs necessary to achieve desired joint positions and velocities by
    applying feedback linearization, using the robot's dynamic model to appropriately compensate for its
    inherent dynamics. The control law implemented typically combines proportional-derivative (PD) control
    with dynamic compensation to achieve precise and stable motion.
    """
    # here i want to add a control that if the kp gains are a numpy vector ill check if the size is good
    #  and then i will create the P matrix and D matrix using the vector a diagonal of the matrices
    # if the size is not good i will raise an error
    # if the kp is a scalar i will create a diagonal matrix with the scalar value
    # same for kd
    if isinstance(kp, np.ndarray):
        if kp.size != dyn_model.getNumberofActuatedJoints():
            raise ValueError("The size of the kp vector is not correct") 
    else:
        kp = np.array([kp] * dyn_model.getNumberofActuatedJoints())
    
    P = np.diag(kp)
    
    if isinstance(kd, np.ndarray):
        if kd.size != dyn_model.getNumberofActuatedJoints():
            raise ValueError("The size of the kd vector is not correct")
    else:
        kd = np.array([kd] * dyn_model.getNumberofActuatedJoints())
    D = np.diag(kd)
 
    # here i compute the feeback linearization tau // the reordering is already done inside compute all teamrs
    dyn_model.ComputeAllTerms(q_, qd_)

    # control 
    
    n = dyn_model.res.c + dyn_model.res.g
    
    tau_FL = dyn_model.res.M @ u + n

    tau_FL = dyn_model._FromPinToExtVec(tau_FL)
  
    return tau_FL