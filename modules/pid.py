class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0
    
    def compute(self, process_var, dt):
        error = self.setpoint - process_var

        # Proportional
        P_out = self.Kp * error

        # Integral
        self.integral += error * dt
        I_out = self.Ki * self.integral

        # Derivative
        derivative = (error - self.previous_error) / dt
        D_out = self.Kd * derivative

        # Compute
        output = P_out + I_out + D_out

        # Update
        self.previous_error = error
        
        return output