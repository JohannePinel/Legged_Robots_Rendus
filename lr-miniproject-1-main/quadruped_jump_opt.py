import optuna
import numpy as np
from functools import partial
from optuna.trial import Trial
from env.simulation import QuadSimulator, SimulationOptions

from profiles import FootForceProfile

from quadruped_jump import (
    nominal_position,
    gravity_compensation,
    apply_force_profile,
    virtual_model,
)


N_LEGS = 4
N_JOINTS = 3


def quadruped_jump_optimization():
    # Initialize simulation
    # Feel free to change these options! (except for control_mode and timestep)
    sim_options = SimulationOptions(
        on_rack=False,  # Whether to suspend the robot in the air (helpful for debugging)
        render=True,  # Whether to use the GUI visualizer (slower than running in the background)
        record_video=False,  # Whether to record a video to file (needs render=True)
        tracking_camera=True,  # Whether the camera follows the robot (instead of free)
    )
    simulator = QuadSimulator(sim_options)

    # Create a maximization problem
    objective = partial(evaluate_jumping, simulator=simulator)
    sampler = optuna.samplers.TPESampler(seed=42)
    study = optuna.create_study(
        study_name="Quadruped Jumping Optimization",
        sampler=sampler,
        direction="maximize",
    )

    # Run the optimization
    # You can change the number of trials here
    study.optimize(objective, n_trials=15)

    # Close the simulation
    simulator.close()

    # Log the results
    print("Best value:", study.best_value)
    print("Best params:", study.best_params)

    # OPTIONAL: add additional functions here (e.g., plotting, recording to file)
    # E.g., cycling through all the evaluated parameters and values:
    for trial in study.get_trials():
        trial.number  # Number of the trial
        trial.params  # Used parameters
        trial.value  # Resulting objective function value


def evaluate_jumping(trial: Trial, simulator: QuadSimulator) -> float:

    # TODO: pick optimization variables
    # The following function creates an optimization variable with given name and lower and upper bounds
    # You can then plug in the value in your controller
   
    #variable1 = trial.suggest_float(name="variable1", low=0.0, high=1.0)
    f0 = trial.suggest_float(name="f0", low = 0.5, high = 5)
    f1 = trial.suggest_float(name="f1", low = 0.5, high = 5)
    Fx = trial.suggest_float(name="Fx", low = -5, high = 5)
    Fy = trial.suggest_float(name="Fy", low = 50, high = 150)
    Fz = trial.suggest_float(name="Fz", low = 75, high = 200)
     
    # Reset the simulation
    simulator.reset()

    # Extract simulation options
    sim_options = simulator.options

    

    FORWARD_JUMP = 0 #works but not ideal
    LATERAL_JUMP_LEFT = 1 #
    LATERAL_JUMP_RIGHT = 2
    TWIST_CLOCK_JUMP = 3 #works but not ideal
    TWIST_COUNTER_CLOCK_JUMP = 4 #works but not ideal

    jump_type =  LATERAL_JUMP_LEFT

    
    # TODO: set parameters for the foot force profile here
    force_profile = FootForceProfile(f0=f0, f1=f1, Fx=Fx, Fy=Fy, Fz=Fz)

    # Determine number of jumps to simulate
    n_jumps = 6  # Feel free to change this number
    jump_duration = force_profile.impulse_duration() + force_profile.idle_duration()  # TODO: determine how long a jump takes
    n_steps = int(n_jumps * jump_duration / sim_options.timestep)

    total_time = n_steps * sim_options.timestep #to measure fastest controller
    #max_height = -99 # to initialize tracking
    start_pos = simulator.get_base_position()[0] #to measure fastest controller

    for _ in range(n_steps):
        # Step the oscillator
        force_profile.step(sim_options.timestep)

        # Compute torques as motor targets (reuses your controller functions)
        # OPTIONAL: add potential extra controller parameters here
        tau = np.zeros(N_JOINTS * N_LEGS)
        tau += nominal_position(simulator)
        tau += apply_force_profile(simulator, force_profile, jump_type)[0]
        tau += gravity_compensation(simulator)

        # If touching the ground, add virtual model
        on_ground = any(simulator.get_foot_contacts()) # TODO: how do we know we're on the ground?
        if on_ground:
            tau += virtual_model(simulator)
        else :
            if jump_type == 2 :
                tau -= nominal_position(simulator)
                des_foot_position = np.array([[0,-0.4, 0.1],[0,0.1, -0.3],[0,-0.4, 0.1],[0,0.1, -0.2]])
                tau += nominal_position(simulator, des_foot_position)
            elif jump_type == 1 :
                tau -= nominal_position(simulator)
                des_foot_position = np.array([[0,-0.1, -0.3],[0,0.4, 0.1],[0,-0.1, -0.2],[0,0.4, 0.1]])
                tau += nominal_position(simulator, des_foot_position)
            elif jump_type == 0 :
                tau -= nominal_position(simulator)
                des_foot_position = np.array([[0.075,-0.0838, -0.275],[0.075,0.0838, -0.275],[0.025,-0.0838, -0.475],[0.025,0.0838, -0.475]])
                tau += nominal_position(simulator, des_foot_position)

        # Set the motor commands and step the simulation
        simulator.set_motor_targets(tau)
        simulator.step()

    # TODO: implement an objective function and return its value
    # Note: the objective function is maximized!
    
    #base_pos = simulator.get_base_position()
    #max_height = max(base_pos[2], max_height)

    #orientation_base = simulator.get_base_orientation_roll_pitch_yaw()
    #min_roll_pitch = min(orientation_base)

    #TO MEASURE FURTHEST
    
    position = simulator.get_base_position()
    roll, pitch, yaw = simulator.get_base_orientation_roll_pitch_yaw()

    if jump_type == FORWARD_JUMP:
        # Further along X
        objective_value = position[0]

    elif jump_type == LATERAL_JUMP_LEFT:
        # Positive Y direction
        objective_value = position[1]

    elif jump_type == LATERAL_JUMP_RIGHT:
        # Negative Y direction (absolute)
        objective_value = -position[1]  # or abs(position[1])

    elif jump_type == TWIST_CLOCK_JUMP:
        # Max clockwise twist → NEGATIVE yaw (assuming right-hand rule)
        objective_value = -yaw

    elif jump_type == TWIST_COUNTER_CLOCK_JUMP:
        # Max counterclockwise twist → POSITIVE yaw
        objective_value = yaw

    return objective_value
    """
    # TO MEASURE FASTEST
    
    end_pos = simulator.get_base_position()[0]
    velocity = (end_pos - start_pos) / total_time 
    return velocity
    """


if __name__ == "__main__":
    quadruped_jump_optimization()
