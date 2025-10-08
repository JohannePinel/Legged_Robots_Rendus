import numpy as np
from env.simulation import QuadSimulator, SimulationOptions

from profiles import FootForceProfile

N_LEGS = 4
N_JOINTS = 3


def quadruped_jump():
    # Initialize simulation
    # Feel free to change these options! (except for control_mode and timestep)
    sim_options = SimulationOptions(
        on_rack=False,  # Whether to suspend the robot in the air (helpful for debugging)
        render=True,  # Whether to use the GUI visualizer (slower than running in the background)
        record_video=True,  # Whether to record a video to file (needs render=True)
        tracking_camera=True,  # Whether the camera follows the robot (instead of free)
    )
    simulator = QuadSimulator(sim_options)

    # Determine number of jumps to simulate
    n_jumps = 10  # Feel free to change this number
    jump_duration = 1.0  # TODO: determine how long a jump takes
    n_steps = int(n_jumps * jump_duration / sim_options.timestep)

    # TODO: set parameters for the foot force profile here
    force_profile = FootForceProfile(f0=0.2, f1=0.5, Fx=0, Fy=0, Fz=100)

    for _ in range(n_steps):
        # If the simulator is closed, stop the loop
        if not simulator.is_connected():
            break

        # Step the oscillator
        force_profile.step(sim_options.timestep)

        # Compute torques as motor targets
        # The convention is as follows:
        # - A 1D array where the torques for the 3 motors follow each other for each leg
        # - The first 3 elements are the hip, thigh, calf torques for the FR leg.
        # - The order of the legs is FR, FL, RR, RL (front/rear,right/left)
        # - The resulting torque array is therefore structured as follows:
        # [FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf]
        tau = np.zeros(N_JOINTS * N_LEGS)

        # TODO: implement the functions below, and add potential controller parameters as function parameters here
        tau += nominal_position(simulator)
        tau += apply_force_profile(simulator, force_profile)
        tau += gravity_compensation(simulator)

        on_ground = simulator.get_foot_contacts()
        #print("Initial robot base position:", simulator.get_base_position())

        # If touching the ground, add virtual model
        on_ground = QuadSimulator.get_foot_contacts(simulator)  # TODO: how do we know we're on the ground?
        if on_ground.any():
            tau += virtual_model(simulator)


        # Set the motor commands and step the simulation
        simulator.set_motor_targets(tau)
        simulator.step()

    # Close the simulation
    simulator.close()

    # OPTIONAL: add additional functions here (e.g., plotting)
def nominal_position(
    simulator: QuadSimulator,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    # TODO: compute nominal position torques for leg_id
    """
    kpCartesian = np.diag([20,20,25])
    kdCartesian = np.diag([10,10,10])
    des_foot_pos = np.array([[0.1,0.2, -0.2],[0.1,0.2, -0.2],[0.1,0.2, -0.23],[0.1,0.2, -0.14]]) 

    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):
        reel_pos = simulator.get_motor_angles(leg_id)
        reel_vit = simulator.get_motor_torques(leg_id)

        #J, pos = simulator.get_jacobian_and_position(leg_id) #jacobian for each foot
        tau_i = (kpCartesian @ (des_foot_pos[leg_id] - reel_pos) + kdCartesian @ (-reel_vit))

        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i
    """
    kpCartesian = np.diag([20,20,25])
    kdCartesian = np.diag([10,10,10])
    des_foot_pos = np.array([[0.0 ,0.0, -0.0],[0.0 ,-0.0, -0.0],[0.0 ,0.0, -0.0],[0.0, -0.0, -0.0]])

    kpJoint = np.array([1,1,1])
    kdJoint = np.array([0.1,0.1,0.1])

    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):


        # TODO: compute virtual model torques for leg_id
        tau_i_cart = np.zeros(3)
        

        J, pos = simulator.get_jacobian_and_position(leg_id) #jacobian for each foot
        
        foot_vel = J@ simulator.get_motor_velocities(leg_id)
       
        tau_i_cart = J.T @ (kpCartesian @ (des_foot_pos[leg_id] - pos) + kdCartesian @ (-foot_vel))
        """
        tau_i_joint = np.zeros(3)
        reel_pos = simulator.get_motor_angles(leg_id)
        reel_vit = simulator.get_motor_velocities(leg_id)
        tau_i_joint = (kpJoint @ (des_foot_pos[leg_id] - reel_pos) + kdJoint @ (-reel_vit))
        """
        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i_cart #+ tau_i_joint

    return tau


def virtual_model(
    simulator: QuadSimulator,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    """
    kpCartesian = np.diag([20,20,25])
    kdCartesian = np.diag([10,10,10])
    des_foot_pos = np.array([[0.0 ,0.0, -0.0],[0.0 ,-0.0, -0.0],[0.0 ,0.0, -0.0],[0.0, -0.0, -0.0]])

    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):

        # TODO: compute virtual model torques for leg_id
        tau_i = np.zeros(3)

        J, pos = simulator.get_jacobian_and_position(leg_id) #jacobian for each foot
        
        foot_vel = J@ simulator.get_motor_velocities(leg_id)
       
        tau_i = J.T @ (kpCartesian @ (des_foot_pos[leg_id] - pos) + kdCartesian @ (-foot_vel))

        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i
    """

    R = simulator.get_base_orientation_matrix()
    P = np.array([
        [1,1,-1,-1],
        [-1,1,-1,1],
        [0,0,0,0]
    ]) 
    P = R @ P
    K_vcm = 0.2
    """
    F_vcm = np.array([
        [0,0,0,0],
        [0,0,0,0],
        K_vcm*([0,0,1]@P)
    ])
    """
    F_vcm = np.zeros((3,4))
    F_vcm[2,:] = K_vcm*([0,0,1] @ P)
    tau_vcm_i = np.zeros(3)
    tau_vcm = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):
        #reel_pos = simulator.get_motor_angles(leg_id)
        #reel_vit = simulator.get_motor_velocities(leg_id)

        J, pos = simulator.get_jacobian_and_position(leg_id) #jacobian for each foot

        F_vcm_i = F_vcm[:, leg_id]  # take the force for each leg
        tau_vcm_i = J.T @ F_vcm_i

        # Store in torques array
        tau_vcm[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_vcm_i

    #tau += tau_vcm
    tau = tau_vcm
    return tau


def gravity_compensation(
    simulator: QuadSimulator,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):

        # TODO: compute gravity compensation torques for leg_id
        tau_i = np.zeros(3)
        J, _= simulator.get_jacobian_and_position(leg_id) #jacobian for each foot

        tau_i = J.T @ (-np.array([0, 0, 9.81*simulator.get_mass()]))

        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i

    return tau


def apply_force_profile(
    simulator: QuadSimulator,
    force_profile: FootForceProfile,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):
        # TODO: compute force profile torques for leg_id

        F_foot = force_profile.force()
        J, _= simulator.get_jacobian_and_position(leg_id)     # shape (3, N_JOINTS)

        tau_i = J.T @ F_foot  
        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i

    return tau


if __name__ == "__main__":
    quadruped_jump()
