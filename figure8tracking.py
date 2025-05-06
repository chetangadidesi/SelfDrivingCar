import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Bicycle():
    def __init__(self):
        self.xc = 0  # position in x direction
        self.yc = 0  # position in y direction
        self.theta = 0  # heading angle
        self.delta = 0 # steering angle
        self.beta = 0 # side slip angle
        self.L = 2  # length
        self.lr = 1.2 # length from rear axle to CG 
        self.w_max = 1.22 # max steering angle rate
        self.sample_time = 0.01

    def reset(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0
        pass

class BicycleModel(Bicycle):
    def step(self,v,w):
        dt = self.sample_time # setting sample time to dt
        delta_dot = w # we know d/dt (delta) = w
        self.delta += delta_dot * dt
        self.delta = np.clip(self.delta, -self.w_max, self.w_max)

        self.beta = np.arctan((self.lr / self.L) * np.tan(self.delta))

        xc_dot = v * np.cos(self.theta + self.beta)  # derivative of xc
        yc_dot = v * np.sin(self.theta + self.beta)  # derivative of yc
        theta_dot = (v / self.L) * np.cos(self.beta) * np.tan(self.delta)  # derivative of theta

        self.xc += xc_dot * dt  # update xc position
        self.yc += yc_dot * dt  # update yc position
        self.theta += theta_dot * dt  # update orientation (theta)

# Simulation PLOTTING CIRCLE PATH
sample_time = 0.01
time_end = 20
model = BicycleModel()

model.delta = np.arctan(2 / 10)  # Steering angle
t_data = np.arange(0, time_end, sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)

for i in range(t_data.shape[0]):
    x_data[i] = model.xc
    y_data[i] = model.yc

    if model.delta < np.arctan(2/10):
        model.step(np.pi, model.w_max)
    else:
        model.step(np.pi, 0)  # simulate with constant velocity and no steering change

# Plot the result with adjusted axis limits
plt.axis('equal')
plt.plot(x_data, y_data, label='Bicycle Model')
plt.xlim([-15, 15])  # Adjust x-axis limit
plt.ylim([0, 20])    # Adjust y-axis limit
plt.legend()
plt.show()


# PLOTTING SQUARE PATH

sample_time = 0.01
time_end = 60
model.reset()

t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)


# maintain velocity at 4 m/s
v_data = np.zeros_like(t_data)
v_data[:] = 4 

w_data = np.zeros_like(t_data)

w_data[670:670+100] = 0.753
w_data[670+100:670+100*2] = -0.753
w_data[2210:2210+100] = 0.753
w_data[2210+100:2210+100*2] = -0.753
w_data[3670:3670+100] = 0.753
w_data[3670+100:3670+100*2] = -0.753
w_data[5220:5220+100] = 0.753
w_data[5220+100:5220+100*2] = -0.753

for i in range(t_data.shape[0]):
    x_data[i] = model.xc
    y_data[i] = model.yc
    model.step(v_data[i], w_data[i])
plt.figure(2)   
plt.axis('equal')
plt.plot(x_data, y_data,label='Bicycle Model')
plt.legend()
plt.show()

# PLOTTING FIGURE 8

sample_time = 0.01
time_end = 36
model.reset()

t_data = np.arange(0, time_end, sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)
v_data = np.zeros_like(t_data)
w_data = np.zeros_like(t_data)


# Set initial position to bottom of the first circle
R = 8
model.xc = 0
model.yc = 0
model.theta = np.pi / 2  # Facing left along circle
v = np.pi  # Speed
L = model.L
target_delta = np.arctan(L / R)  # Desired steering angle

# Simulate
for i in range(t_data.shape[0]):
    x_data[i] = model.xc
    y_data[i] = model.yc
    v_data[i] = v

    # First 15 seconds: steer left
    if t_data[i] < 20:
        desired_delta = target_delta
    # Next 15 seconds: steer right
    else:
        desired_delta = -target_delta

    # Ramp up/down to desired delta
    if abs(model.delta - desired_delta) > 1e-3:
        w = model.w_max if desired_delta > model.delta else -model.w_max
    else:
        w = 0

    w_data[i] = w
    model.step(v, w)

# Plot the result
plt.figure(3)
plt.axis('equal')
plt.plot(x_data, y_data, label='Path')
plt.scatter(x_data[0], y_data[0], color='red', label='Start')
plt.scatter(x_data[-1], y_data[-1], color='green', label='End')
plt.legend()
plt.title("Figure-8 Path")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.grid(True)
plt.show()