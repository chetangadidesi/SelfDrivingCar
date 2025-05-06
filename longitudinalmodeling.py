import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Vehicle():
    def __init__(self):
 
        # ==================================
        #  Parameters
        # ==================================
    
        #Throttle to engine torque
        self.a_0 = 400
        self.a_1 = 0.1
        self.a_2 = -0.0002
        
        # Gear ratio, effective radius, mass + inertia
        self.GR = 0.35
        self.r_e = 0.3
        self.J_e = 10
        self.m = 2000
        self.g = 9.81
        
        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.01
        
        # Tire force 
        self.c = 10000
        self.F_max = 10000
        
        # State variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0
        
        self.sample_time = 0.01
        
    def reset(self):
        # reset state variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0

class Vehicle(Vehicle):
    def step(self, throttle, alpha):
        
        # --- Engine Torque ---
        T_e = throttle * (self.a_0 + self.a_1 * self.w_e + self.a_2 * self.w_e**2)

        # --- Load Forces ---
        F_aero = self.c_a * self.v**2
        R_x = self.c_r1 * self.v
        F_g = self.m * self.g * np.sin(alpha)
        F_load = F_aero + R_x + F_g

        # --- Wheel Angular Velocity ---
        w_w = self.GR * self.w_e

        # --- Slip Ratio ---
        if self.v > 0.1:  # Avoid division by zero
            s = (w_w * self.r_e - self.v) / self.v
        else:
            s = 0

        # --- Tire Force ---
        if abs(s) < 1:
            F_x = self.c * s
        else:
            F_x = self.F_max

        # --- Engine Dynamics ---
        self.w_e_dot = (T_e - self.GR * self.r_e * F_load) / self.J_e
        self.w_e += self.w_e_dot * self.sample_time

        # --- Vehicle Dynamics ---
        self.a = (F_x - F_load) / self.m
        self.v += self.a * self.sample_time
        self.x += self.v * self.sample_time
        pass


sample_time = 0.01
time_end = 100
model = Vehicle()

t_data = np.arange(0,time_end,sample_time)
v_data = np.zeros_like(t_data)

# throttle percentage between 0 and 1
throttle = 0.2

# incline angle (in radians)
alpha = 0

for i in range(t_data.shape[0]):
    v_data[i] = model.v
    model.step(throttle, alpha)
plt.title('Throttle @ 20% with slope of 0 degree')
plt.plot(t_data, v_data)
plt.show()

time_end = 20
t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)

# reset the states
model.reset()

def angle(i, alpha, x):
    if x < 60:
        alpha[i] = np.arctan(3/60)
    elif x < 150:
        alpha[i] = np.arctan(9/90)
    else:
        alpha[i] = 0

throttle = np.zeros_like(t_data)
alpha = np.zeros_like(t_data)

#throttle depends on time and alpha depends on distance travelled (model.x)
for i in range(t_data.shape[0]):
    if t_data[i] < 5:
        throttle[i] = 0.2 + ((0.5 - 0.2)/5)*t_data[i]
        angle(i, alpha, model.x)
    elif t_data[i] < 15:
        throttle[i] = 0.5
        angle(i, alpha, model.x)
    else:
        throttle[i] = ((0 - 0.5)/(20 - 15))*(t_data[i] - 20)
        angle(i, alpha, model.x)
    
    #call the step function and update x_data array
    model.step(throttle[i], alpha[i])
    x_data[i] = model.x
    v_data[i] = model.v
    #w_e_data[i] = model.w_e


# Plot x vs t for visualization
plt.title('Varied Slope and Throttle Inputs for Test Case')
plt.plot(t_data, x_data)
plt.xlabel("Time (s)")
plt.ylabel("Position x(t) [m]")
plt.show()