import matplotlib.pyplot as plt
import numpy as np

from pid import PIDController

'''
    ... process deteksi titik 
    ... titik hasil deteksi dijadikan setpoint atau process_var
'''

# Initialize
setpoint = 100
pid = PIDController(1.0, 1.0, 0.05, setpoint)

# Sim parameters
time = np.linspace(0, 50, 200)
dt = time[1]-time[0]

# Initial temperature
process_var = 300
process_val = []

for t in time:
    # PID control output
    control_output = pid.compute(process_var, dt)
    
    # Simulate process dynamics (heating rate proportional to control output)
    process_var += control_output * dt - 0.1 * (process_var - 20) * dt  # Heat loss
    
    # Store the process variable
    process_val.append(process_var)

# Plot results
plt.figure(figsize=(10, 6))
plt.plot(time, process_val, label='Process Variable (Temperature)')
plt.axhline(y=setpoint, color='r', linestyle='--', label='Setpoint')
plt.xlabel('Time (s)')
plt.ylabel('Temperature')
plt.title('PID Controller Simulation')
plt.legend()
plt.grid()
plt.show()