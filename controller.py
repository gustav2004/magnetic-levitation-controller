
class PIDcontroller:
        def __init__(self, *, setpoint, kp, kd, ki, u_min, u_max):
            self.setpoint = setpoint        # desired position
            self.kp = kp                    # proportional gain
            self.kd = kd                    # derivative gain
            self.ki = ki                    # integral gain
            self.u_min = u_min              # minimum control output
            self.u_max = u_max              # maximum control output
            self.integral = 0.0             # integral term

        def compute_control(self, y, ydot, dt):
            """Compute the control signal based on current position and velocity."""
        
            error = self.setpoint - y
            
            P = self.kp * error         # calculates prop. term
            
            self.integral += error * dt     # accumalative error
            I = self.ki * self.integral     # calc integral term
            
            D = - self.kd *ydot             # use derivative onb measurement for der. term
            
            # Compute total control signal
            u = P + I + D
            
            # Clamp to make sure not outside of allowed voltage range
            u = max(self.u_min, min(u, self.u_max)) 
            
            return u
        
        def reset(self):
            """Reset controller state."""
            self.integral = 0.0