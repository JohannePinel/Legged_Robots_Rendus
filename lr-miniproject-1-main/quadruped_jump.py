import numpy as np
import matplotlib.pyplot as plt
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
        record_video=False,  # Whether to record a video to file (needs render=True)
        tracking_camera=True,  # Whether the camera follows the robot (instead of free)
    )
    simulator = QuadSimulator(sim_options)
    
    # TODO: set parameters for the foot force profile here
    
    FORWARD_JUMP = 0 #works but not ideal
    LATERAL_JUMP_LEFT = 1 #
    LATERAL_JUMP_RIGHT = 2
    TWIST_CLOCK_JUMP = 3 #works but not ideal
    TWIST_COUNTER_CLOCK_JUMP = 4 #works but not ideal

    jump_type =  TWIST_COUNTER_CLOCK_JUMP

    if jump_type == 0 :
        #force_profile = FootForceProfile(f0= 3.925879806965068, f1=0.9983689107917987, Fx=100, Fy=45, Fz=100) #force profile for a forward jump
        #Force profile furthest
        force_profile = FootForceProfile(f0 = 1.5802996241239413, f1= 2.1247143086272664, Fx=112.47922456158778, Fy=-1.0524536076375688, Fz=155.73205508790514)
        #Force profile fastest
        #force_profile = FootForceProfile(f0= 3.925879806965068, f1= 0.9983689107917987, Fx = 101.87535655954954, Fy = 58.426286023928334, Fz = 99.75458667119952)
    elif jump_type == 1 :
        #force_profile = FootForceProfile(f0=2, f1=0.5, Fx=100, Fy=100, Fz=100) # pour lateral jump left
        #Force profile furthest
        force_profile = FootForceProfile(f0 = 1.0491720568015048, f1=2.728296095500716, Fx=-4.656114788847816, Fy=140.9320402078782, Fz=107.34749770000212)
        #Force profile fastest
        #force_profile = FootForceProfile(f0 = , f1=, Fx=, Fy=, Fz=)
    elif jump_type == 2 :
        #force_profile = FootForceProfile(f0=2, f1=0.5, Fx=0, Fy=-100, Fz=100) # pour lateral jump RIGHT
        #Force profile furthest
        force_profile = FootForceProfile(f0 = 1.325320294340452, f1=1.8690900933179198, Fx=0.24756431632237863, Fy=-106.80549813578843, Fz=111.40364252475524)
        #Force profile fastest
        #force_profile = FootForceProfile(f0 = , f1=, Fx=, Fy=, Fz=)
    elif jump_type == 3 :
        #force_profile = FootForceProfile(f0= 3.925879806965068, f1=0.9983689107917987, Fx=0, Fy=45, Fz=100) #force profile for a twist stable
        #Force profile furthest
        force_profile = FootForceProfile(f0 = 1.325320294340452, f1=1.8690900933179198, Fx=0.24756431632237863, Fy=57.395876398158684, Fz=93.68437102970628)
        #Force profile fastest
        #force_profile = FootForceProfile(f0 = , f1=, Fx=, Fy=, Fz=)
    elif jump_type == 4 :
        #force_profile = FootForceProfile(f0= 3.925879806965068, f1=0.9983689107917987, Fx=0, Fy=45, Fz=100) #force profile for a twist stable
        #Force profile furthest
        force_profile = FootForceProfile(f0 = 1.325320294340452, f1=1.8690900933179198, Fx= 0.24756431632237863, Fy=57.395876398158684, Fz=93.68437102970628)
        #Force profile fastest
        #force_profile = FootForceProfile(f0 = , f1=, Fx=, Fy=, Fz=)

    # Determine number of jumps to simulate
    n_jumps = 6  # Feel free to change this number
    jump_duration = force_profile.impulse_duration() + force_profile.idle_duration()  # TODO: determine how long a jump takes
    n_steps = int(n_jumps * jump_duration / sim_options.timestep)
    # allocate storage for per-step per-foot force vectors (Fx,Fy,Fz)
    
    foot_forces = np.zeros((n_steps, N_LEGS, 3))
    tau_rec = np.zeros((n_steps, N_LEGS, 3))
    recorded_steps = 0
    
    yaw_previous = 0
    yaw_offset = 0

    for step in range(n_steps):
        # If the simulator is closed, stop the loopS
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
        tau += apply_force_profile(simulator, force_profile, jump_type)[0]
        tau += gravity_compensation(simulator)
        # If touching the ground, add virtual model
        on_ground = any(simulator.get_foot_contacts())  # true que quand les 4 pieds touhent le sol# TODO: how do we know we're on the ground?
        if on_ground:
            tau += virtual_model(simulator)
        else :
            if jump_type == 0 :
                tau -= nominal_position(simulator)
                des_foot_position = np.array([[0.075,-0.0838, -0.275],[0.075,0.0838, -0.275],[0.025,-0.0838, -0.475],[0.025,0.0838, -0.475]])
                tau += nominal_position(simulator, des_foot_position)
            elif jump_type == 1 :
                tau -= nominal_position(simulator)
                des_foot_position = np.array([[0,-0.1, -0.3],[0,0.4, 0.1],[0,-0.1, -0.2],[0,0.4, 0.1]])
                tau += nominal_position(simulator, des_foot_position)
            elif jump_type == 2 :
                tau -= nominal_position(simulator)
                des_foot_position = np.array([[0,-0.4, 0.1],[0,0.1, -0.3],[0,-0.4, 0.1],[0,0.1, -0.2]])
                tau += nominal_position(simulator, des_foot_position)
            elif jump_type == 3 :
                tau -= nominal_position(simulator)
                des_foot_position = np.array([[0,-0.2, -0.2],[0,0, -0.2],[0,0, -0.2],[0,0.2, -0.2]]) #sifht d'environ -0.1 pour les jambes avant et + 0.1 pou les jambes arriere
                tau += nominal_position(simulator, des_foot_position)
            elif jump_type == 4 :
                tau -= nominal_position(simulator)
                des_foot_position = np.array([[0,0, -0.2],[0,0.2, -0.2],[0,-0.2, -0.2],[0,0, -0.2]])
                tau += nominal_position(simulator, des_foot_position)
            
        # to record the value of the applied force and tau for each leg in each direction
        
        for i in range(N_LEGS):
            for j in range(N_JOINTS):
                foot_forces[recorded_steps, i, j] = apply_force_profile(simulator, force_profile, jump_type)[1][3*i + j] 
                tau_rec[recorded_steps, i, j] = tau[3*i + j]  
        

        # Set the motor commands and step the simulation
        simulator.set_motor_targets(tau)
        simulator.step()
        
        recorded_steps += 1
        


    # Close the simulation
    simulator.close()

    # Trim recorded arrays in case the loop exited early
    
    foot_forces = foot_forces[:recorded_steps]
    tau_rec = tau_rec[:recorded_steps]
    

    # Plot force components per foot vs simulation step
    
    steps = np.arange(recorded_steps)
    foot_names = ['FR', 'FL', 'RR', 'RL']
    comp_labels_F = ['Fx [N]', 'Fy [N]', 'Fz [N]']
    

    # Now rows = components (Fx,Fy,Fz) and columns = feet (FR,FL,RR,RL)
    
    fig, axs = plt.subplots(len(comp_labels_F), N_LEGS, figsize=(12, 9), sharex=True)
    if recorded_steps > 0:
        for comp in range(3):
            for leg_id in range(N_LEGS):
                ax = axs[comp, leg_id]
                ax.plot(steps, foot_forces[:, leg_id, comp], label=f'{comp_labels_F[comp]} {foot_names[leg_id]}', color=f'C{leg_id}')
                # set column titles to foot names
                if comp == 0:
                    ax.set_title(foot_names[leg_id])
                # set row y-labels to component labels
                if leg_id == 0:
                    ax.set_ylabel(comp_labels_F[comp])
                ax.grid(True)

        axs[-1, 0].set_xlabel('Simulation step')
        plt.suptitle('Per-foot force components over simulation steps')
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.show()
    else:
        print("No simulation data recorded - nothing to plot.")

    # --- New: plot motor torque components recorded in tau_rec ---
    # tau_rec has shape (steps, N_LEGS, 3) and stores per-leg per-joint torque values
    comp_labels_tau = ['tau hip [Nm]', 'tau thigh [Nm]', 'tau calf [Nm]']

    # Now rows = components (hip,thigh,calf) and columns = feet (FR,FL,RR,RL)
    fig2, axs2 = plt.subplots(len(comp_labels_tau), N_LEGS, figsize=(12, 9), sharex=True)
    if recorded_steps > 0:
        for comp in range(3):
            for leg_id in range(N_LEGS):
                ax = axs2[comp, leg_id]
                ax.plot(steps, tau_rec[:, leg_id, comp], label=f'{comp_labels_tau[comp]} {foot_names[leg_id]}', color=f'C{leg_id+4}')
                # set column titles to foot names
                if comp == 0:
                    ax.set_title(foot_names[leg_id])
                # set row y-labels to component labels
                if leg_id == 0:
                    ax.set_ylabel(comp_labels_tau[comp])
                ax.grid(True)

        axs2[-1, 0].set_xlabel('Simulation step')
        plt.suptitle('Per-foot motor torque components over simulation steps')
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.show()
    else:
        print("No torque data recorded - nothing to plot.")
    

def nominal_position(
    simulator: QuadSimulator,
    des_foot_pos = np.array([[0,-0.0838, -0.275],[0,0.0838, -0.275],[0,-0.0838, -0.2],[0,0.0838, -0.2]]), #position juste en dessous des hanche
    kpCartesian = np.diag([400,400,200]),# valeur arbitraire
    kdCartesian = np.diag([50,50,30]),# valeur arbitraire
    kdJoint = np.diag([0.1,0.1,0.1])# valeur arbitraire
    
        
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    # TODO: compute nominal position torques for leg_id


    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):
        J, pos = simulator.get_jacobian_and_position(leg_id) #jacobian for each foot
        
        foot_vel = J@ simulator.get_motor_velocities(leg_id)
       
        tau_i = J.T @ (kpCartesian @ (des_foot_pos[leg_id] - pos) + kdCartesian @ (-foot_vel))
        tau_i += kdJoint @ (-simulator.get_motor_velocities(leg_id))
        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i
    
    return tau



def virtual_model(
    simulator: QuadSimulator,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    R = simulator.get_base_orientation_matrix()
    R_world_mat = np.array([[1,1,-1,-1], [-1,1,-1,1],[0,0,0,0]])
    P = R@R_world_mat
    k_vmc = 200
    F_vmc = np.array([[0,0,0,0],[0,0,0,0], k_vmc*([0,0,1]@P)])
    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):

        # TODO: compute virtual model torques for leg_id
        J, _ = simulator.get_jacobian_and_position(leg_id)
        tau_i = J.T @ F_vmc[:, leg_id]

        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i
    
    return tau


def gravity_compensation(
    simulator: QuadSimulator,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    gnd_contact = simulator.get_foot_contacts()
    # est ce qu'il faut juste diviser par 4 ou savoir sur combien de pied exactement il repose ???git
    if sum(gnd_contact) != 0:
        foot_div = 1/sum(gnd_contact) #permet de savoir sur cobien de pied doit reposer le robot
    else:
        foot_div = 0
    for leg_id in range(N_LEGS):

        # TODO: compute gravity compensation torques for leg_id
        tau_i = np.zeros(3)
        J, _= simulator.get_jacobian_and_position(leg_id) #jacobian for each foot
        if gnd_contact[leg_id]:
            tau_i = J.T @ (-np.array([0, 0, 9.8*simulator.get_mass()*foot_div]))
            #if leg_id <=1 :
                #tau_i = 2*tau_i # because we have more weight in the front
        else:
            tau_i = 0
        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i
   
    return tau


def apply_force_profile(
    simulator: QuadSimulator,
    force_profile: FootForceProfile,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
    jump_type
) -> np.ndarray:
    # All motor torques are in a single array

    tau = np.zeros(N_JOINTS * N_LEGS)
    F_foot = np.zeros(N_JOINTS*N_LEGS)
    
    for leg_id in range(N_LEGS):
        F_foot_i = force_profile.force() #F_foot[0] pour Fx, F_foot[1] pour Fy, F_foot[2] pour Fz
        if jump_type == 0:            # FORWARD_JUMP
            if leg_id in [0, 1]:
                #print(F_foot_i[2],"forward", leg_id)
                F_foot_i[2] *= 1.3
                #print(F_foot_i[2],"forward", leg_id)
        
        if jump_type == 1:            # LATERAL JUMP LEFT
            if leg_id == (2):  # pattes arrière
                F_foot_i[1] *= 0.95 #to try and compensate the drift

        if jump_type == 2:            # LATERAL JUMP RIGHT
            if leg_id == (3):  # pattes arrière
                F_foot_i[1] *= 0.95 #to try and compensate the drift

        if jump_type == 3:          #TWIST CLOCKWISE JUMP       
            if leg_id in [0, 1]:           #Jambe 0, -Fx et -Fy
                F_foot_i[1] = -F_foot_i[1]  #ne fait que changer les pied avec id 0 alors que ca devrait le faire pour 0 et 1 mais le saut fonctionne qund meme
                
        if jump_type == 4:          #TWIST COUNTERCLOCKWISE JUMP       
            if leg_id in [2, 3]:           #Jambe 0, -Fx et -Fy
                F_foot_i[1] = -F_foot_i[1] #ne fait que changer les pied avec id 2 alors que ca devrait le faire pour 2 et 3 mais le saut fonctionne quand meme

        # TODO: compute force profile torques for leg_id
        J,_ = simulator.get_jacobian_and_position(leg_id)
        tau_i = J.T @ F_foot_i
        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i
        F_foot[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = F_foot_i
    
    return [tau, F_foot] #ajout de F_foot en sortie pour pouvoir l'afficher dans un graph


if __name__ == "__main__":
    quadruped_jump()