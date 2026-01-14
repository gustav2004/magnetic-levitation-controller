from __future__ import annotations
from typing import Callable, Iterable, List, Tuple, Optional
import numpy as np

class Dynamics:
    def __init__(
            
        self,
            *,
             # physical parameters
            m: float = 0.05,
            g: float = 9.82, 
            R: float = 3.0,         # coil resist.
            L: float = 0.01,        # coild induct.
            k: float =1e-4,         # magnetic force constant

            # inital conditions
            y0: float = 0.1,  
            ydot0: float = 0.0,
            i0: float = 0.0,        # initial current
            
            # numerics and safety
            dt_default: float = 1e-4,
            y_floor: float = 1e-3,

            # voltage limits (for controller)
            u_inp_min: float = -24.0,
            u_inp_max: float = 24.0   
        ):

        # store parameters
        self.m, self.g, self.R, self.L, self.k = m, g, R, L, k
        self.dt_default = dt_default
        self.y_floor = y_floor
        self.u_inp_min, self.u_inp_max = u_inp_min, u_inp_max
    
        # store default initial conditions
        self.default_state: List[float] = [y0, ydot0, i0]

        # current state (copy of defaults)
        self.state: List[float] = [y0, ydot0, i0]
        
        # disturbance function (can be set by user)
        self.disturbance_fn: Optional[Callable[[float], float]] = None
        self.current_time: float = 0.0
    
    def reset(self, state=None):    # use to change state-values or reset to default

        if state is None:
            state = self.default_state   # default values
        
        self.state = list(state)    # list() makes sure we have a list here
        self.current_time = 0.0     # reset time tracking
        return self.state
    
    # disturbance function that applies a force to the object, allowing more complex control scenarios
    
    def set_disturbance(self, disturbance_fn: Optional[Callable[[float], float]]):
        self.disturbance_fn = disturbance_fn

    # ----------- help funcs ----------------
    def _sat(self, x:float, low: float, high: float) -> float:
        return max(low, min(x, high))       
    
    def _mag_force(self, i: float, y: float) -> float:      # calcs the mag force on the ball
        dist = max(y, self.y_floor)
        return self.k * (i**2) / (dist**2)
    
    # -------------- dynamics -------------------
    def derivatives(self, state, u_inp, disturbance: float = 0.0):
        y, ydot, i = state              
        u_inp = self._sat(u_inp, self.u_inp_min, self.u_inp_max)

        # physics                   
        Fmag = self._mag_force(i, y)                                    # uses helper to calc mag force
        yddot = (Fmag - self.m * self.g + disturbance) / self.m         # acceleration in y (with disturbance)
        idot = (-self.R * i + u_inp) / self.L                           # diffeq. for RL circuit

        return [ydot, yddot, idot]

    def step(self, u_inp: float, dt: float = None) -> List[float]:
                
        if dt is None:
            dt = self.dt_default

        # Get disturbance force at current time
        disturbance = 0.0
        if self.disturbance_fn is not None:
            disturbance = self.disturbance_fn(self.current_time)

        def f(state, u, dist=disturbance):                # get the derivatives with disturbance
            return self.derivatives(state, u, dist)        

        y, ydot, i = self.state                                 # unpack

        # runge kutta method

        k1 = np.array(f(self.state, u_inp))
        k2 = np.array(f(self.state + 0.5 * dt * k1, u_inp))
        k3 = np.array(f(self.state + 0.5 * dt * k2, u_inp))
        k4 = np.array(f(self.state + dt * k3, u_inp))

        state_new = np.array(self.state) + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        y_new, ydot_new, i_new = state_new

        y_clamped = max(float(y_new), self.y_floor)     # makes sure ball does not go through floor, important

        self.state = [y_clamped, ydot_new, i_new]       # update the state values
        self.current_time += dt                          # increment time tracker
        return self.state
    
    def simulate(
            self,
            u_fn: Callable[[float, List[float]], float],   # this means that u_fn is a function of t and the state and returns a float
            T: float,
            dt: float = None,
    ) -> Tuple[List[float], List[float], List[float], List[float], List[float]]:

        # here we "run the differential equations" step by step and log (time, y, ydot, i, u) 

        if dt is None:
            dt = self.dt_default
        
        t = 0.0
        time: List[float] = []
        y_log: List[float] = []
        ydot_log: List[float] = []
        i_log: List[float] = []
        u_log: List[float] = []

        steps = int(T / dt)
        for _ in range(steps):
            time.append(t)
            y, ydot, i = self.state
            y_log.append(y)
            ydot_log.append(ydot)
            i_log.append(i)

            u = float(u_fn(t, self.state))
            u_log.append(u)

            self.step(u, dt)
            t += dt

        return time, y_log, ydot_log, i_log, u_log