import numpy as np
import matplotlib.pyplot as plt
import os

class RocketPIDController:
    def __init__(self, kp: float, ki: float, kd: float, dt: float, max_thrust: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_thrust = max_thrust
        
        self.integral_error = 0.0
        self.prev_error = 0.0

    def compute(self, target_alt: float, current_alt: float) -> float:
        error = target_alt - current_alt
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup clamping
        self.integral_error += error * self.dt
        i_term = self.ki * self.integral_error
        
        # Derivative term
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative
        
        self.prev_error = error
        
        # Total control output (Thrust demand)
        control_output = p_term + i_term + d_term
        
        # Saturate thrust between 0 and maximum engine capacity
        return np.clip(control_output, 0.0, self.max_thrust)


def run_simulation():
    # Simulation Settings
    dt = 0.01          # Time step (seconds)
    total_time = 40.0  # Total run time (seconds)
    time_steps = int(total_time / dt)
    
    # Rocket Physics Constants
    mass = 50.0        # Mass in kg
    gravity = 9.81     # Gravity m/s^2
    rho = 1.225        # Air density kg/m^3
    cd = 0.5           # Drag coefficient
    area = 0.1         # Cross-sectional area m^2
    max_thrust = 1500.0# Max thrust N
    
    # Target altitude
    target_altitude = 500.0  # Meters
    
    # Instantiate PID (Tuned initial gains)
    pid = RocketPIDController(kp=18.0, ki=1.2, kd=25.0, dt=dt, max_thrust=max_thrust)
    
    # State tracking arrays
    time_hist = np.linspace(0, total_time, time_steps)
    alt_hist = np.zeros(time_steps)
    vel_hist = np.zeros(time_steps)
    thrust_hist = np.zeros(time_steps)
    
    # Initial Conditions
    alt = 0.0
    vel = 0.0
    
    for i in range(time_steps):
        # Calculate Drag
        drag = 0.5 * rho * (vel ** 2) * cd * area * np.sign(vel)
        
        # PID calculates thrust demand
        thrust = pid.compute(target_alt=target_altitude, current_alt=alt)
        
        # Net acceleration: F_net = Thrust - Drag - Weight
        accel = (thrust - drag - (mass * gravity)) / mass
        
        # Euler Integration
        vel += accel * dt
        alt += vel * dt
        
        # Ground constraint (rocket cannot drop below surface)
        if alt < 0:
            alt = 0
            vel = 0
            
        # Record state
        alt_hist[i] = alt
        vel_hist[i] = vel
        thrust_hist[i] = thrust

    # Plot Results
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    ax1.plot(time_hist, alt_hist, label="Rocket Altitude", color="b", linewidth=2)
    ax1.axhline(target_altitude, color="r", linestyle="--", label="Target Altitude (500m)")
    ax1.set_ylabel("Altitude (m)")
    ax1.set_title("1D Rocket Altitude PID Control Dynamics")
    ax1.grid(True)
    ax1.legend()
    
    ax2.plot(time_hist, thrust_hist, label="Engine Thrust (N)", color="g", linewidth=1.5)
    ax2.axhline(mass * gravity, color="k", linestyle=":", label="Hover Thrust Requirement (m*g)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Thrust (N)")
    ax2.grid(True)
    ax2.legend()
    
    plt.tight_layout()
    plt.savefig("plots/altitude_step_response.png")
    plt.show()

if __name__ == "__main__":

    plt.tight_layout()
    
    # Create the directory if it doesn't exist
    os.makedirs("plots", exist_ok=True)
    
    plt.savefig("plots/altitude_step_response.png")
    plt.show()
    run_simulation()