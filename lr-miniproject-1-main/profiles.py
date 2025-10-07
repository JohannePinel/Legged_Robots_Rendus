import numpy as np


class FootForceProfile:
    """Class to generate foot force profiles over time using a single CPG oscillator"""

    def __init__(self, f0: float, f1: float, Fx: float, Fy: float, Fz: float):
        """
        Create instance of foot force profile with its arguments.

        Args:
            f0 (float): Frequency of the impulse (Hz)
            f1 (float): Frequency between impulses (Hz)
            Fx (float): Foot force amplitude in X direction (N)
            Fy (float): Foot force amplitude in Y direction (N)
            Fz (float): Foot force amplitude in Z direction (N)
        """
        self.theta = 0
        self.f0 = f0
        self.f1 = f1
        self.F = np.array([Fx, Fy, Fz])

    def step(self, dt: float):
        """
        Step the oscillator by a single timestep.

        Args:
            dt (float): Timestep duration (s)
        """

        if 0 <= self.theta <= np.pi :            # je definie fi (donnée)
            fi = self.f1
        else :
            fi = self.f0

        self.theta += 2 * np.pi * fi * dt        # theta = theta_t + theta_point*dt  et la definition de theta_point est dans la donnee

    def phase(self) -> float:
        """Get oscillator phase in [0, 2pi] range."""
        self.theta = self.theta % (2*np.pi)       # je fais modulo 2pi sur theta
        return self.theta

    def force(self) -> np.ndarray:
        """
        Get force vector of the force profile at the current timestep.

        Returns:
            np.ndarray: An R^3 array [Fx, Fy, Fz]
        """
        if np.sin(self.theta) < 0 :                 # conditions de la donnée 
            Force = self.F * np.sin(self.theta)
        else : 
            Force = np.array([0.0, 0.0, 0.0])
        return Force                                # pas sure du return np.ndarray 

    def impulse_duration(self) -> float:
        """Return impulse duration in seconds."""
        impulse_sec = 1/self.f0                     # je suppute que le temps c'est 1 sur la freq
        return impulse_sec

    def idle_duration(self) -> float:
        """Return idle time between impulses in seconds"""
        idle_sec = 1/self.f1
        return idle_sec
