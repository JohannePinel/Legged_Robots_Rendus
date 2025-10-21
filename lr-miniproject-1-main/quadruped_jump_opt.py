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
    study.optimize(objective, n_trials=50)

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
    f0 = trial.suggest_float(name="f0", low = 2, high = 10)
    #f1 = trial.suggest_float(name="f1", low = 0.5, high = 1.2)
    #Fx = trial.suggest_float(name="Fx", low = 50, high = 200)
    Fy = trial.suggest_float(name="Fy", low = 70, high = 180)
    Fz = trial.suggest_float(name="Fz", low = 70, high = 120)
     
    # Reset the simulation
    simulator.reset()

    # Extract simulation options
    sim_options = simulator.options

    
    # the only 3 type of jumps to opptimze
    FORWARD_JUMP = 0
    LATERAL_JUMP_LEFT = 1 #
    TWIST_CLOCK_JUMP = 3
    speed = False #to optimize the fastest 

    jump_type =  TWIST_CLOCK_JUMP

    
    # TODO: set parameters for the foot force profile here
    force_profile = FootForceProfile(f0=f0, f1=0.2, Fx=0, Fy=Fy, Fz=Fz)

    # Determine number of jumps to simulate
    n_jumps = 1  # Feel free to change this number
    jump_duration = force_profile.impulse_duration() + force_profile.idle_duration()  # TODO: determine how long a jump takes
    n_steps = int((force_profile.idle_duration() + n_jumps * jump_duration) / sim_options.timestep)

    total_time = n_steps * sim_options.timestep #to measure fastest controller
    #max_height = -99 # to initialize tracking
    start_pos = simulator.get_base_position() #to measure fastest controller
    _,_,yaw_start = simulator.get_base_orientation_roll_pitch_yaw() #to measure fastest controller
    yaw_previous = 0
    yaw_offset = 0
    max_roll = 0
    max_pitch = 0
    nbr_step_in_air = 0
    nbr_step_on_ground = 0
    punishment_not_full_contact = 2
    punishment_tilt = 4
    punishment_deviation = 5

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
            nbr_step_on_ground += 1
        else :
            nbr_step_in_air += 1
            if jump_type == FORWARD_JUMP :
                tau -= nominal_position(simulator)
                des_foot_position = np.array([[0.075,-0.0838, -0.275],[0.075,0.0838, -0.275],[0.025,-0.0838, -0.475],[0.025,0.0838, -0.475]])
                tau += nominal_position(simulator, des_foot_position)
            elif jump_type == LATERAL_JUMP_LEFT :
                tau -= nominal_position(simulator)
                des_foot_position = np.array([[0,-0.1, -0.3],[0,0.4, 0.1],[0,-0.1, -0.2],[0,0.4, 0.1]])
                tau += nominal_position(simulator, des_foot_position)
            elif jump_type == TWIST_CLOCK_JUMP :
                tau -= nominal_position(simulator)
                des_foot_position = np.array([[0,-0.2, -0.2],[0,0, -0.2],[0,0, -0.2],[0,0.2, -0.2]]) #sifht d'environ -0.1 pour les jambes avant et + 0.1 pou les jambes arriere
                tau += nominal_position(simulator, des_foot_position)
        #update the yaw with each step ans makes it possible to count turns
        roll, pitch, yaw = simulator.get_base_orientation_roll_pitch_yaw()
        if (yaw - yaw_previous) >np.pi:
            yaw_offset -= 2*np.pi
        elif(yaw - yaw_previous) < -np.pi:
            yaw_offset += 2*np.pi
        yaw_previous = yaw
        yaw_final = yaw + yaw_offset

        if (abs(max_pitch) < abs(pitch)):
            max_pitch = pitch
        if(abs(max_roll) < abs(roll)):
            max_roll = roll
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
    
    if not(speed) and (jump_type == FORWARD_JUMP):
        # Further along X
        objective_value = position[0]
        if not(all(simulator.get_foot_contacts())):
            objective_value -= punishment_not_full_contact
        if (abs(max_roll) or abs(max_pitch) or abs(yaw_final)) > np.pi*0.1:
            objective_value -= punishment_tilt
        


    elif jump_type == LATERAL_JUMP_LEFT:
        # Positive Y direction
        objective_value = position[1]
        if ((abs(yaw_final) or abs(max_pitch)) > np.pi*0.125) or (abs(max_roll) > np.pi*0.5):
            objective_value -= punishment_tilt
        

    elif jump_type == TWIST_CLOCK_JUMP:
        # Max clockwise twist → NEGATIVE yaw (assuming right-hand rule)
        objective_value = -yaw_final #proble yax est en -pi et +pi et prends en compte que la position finale
        if (abs(max_roll) or abs(max_pitch) ) > np.pi*0.25:
            objective_value -= punishment_tilt
        """if (np.sqrt(position[0]**2 + position[1]**2) >1):
            objective_value -= punishment_deviation*(-yaw_final)/(2*np.pi)"""
    # TO MEASURE FASTEST
    
    end_pos = simulator.get_base_position()
    average_velocity = (end_pos - start_pos) / total_time 
    if speed and (jump_type == FORWARD_JUMP):
        # Further along X
        objective_value = average_velocity[0]
        if abs(position[1]) > 0.05*position[0]:
            objective_value -= punishment_deviation
        if (abs(max_roll) or abs(max_pitch) or abs(yaw_final)) > np.pi*0.125:
            objective_value -= punishment_tilt  

    return objective_value
    


if __name__ == "__main__":
    quadruped_jump_optimization()
