import matplotlib.pyplot as plt
import numpy as np


def main():
    setpoint = 0 #0 theta at finished
    theta = 1.5
    v_theta = 0
    state = [theta, v_theta]

    kp = 20  # p gain
    ki = 0  # i gain
    kd = 60# d gain
    
    g = 9.81 #gravity
    m = 1.0 #mass
    L = 1.0 #lenght of rod
    kf = 0.1 #friction constant

    previous_error = 0
    integral = 0
    dt = 0.001  # time step

    time_steps = []
    state_values = []
    control_values = []
    setpoint_values = []

    for i in range(10000):
        control, error, integral = pid_control(setpoint, state, kp, ki, kd, previous_error, integral, dt)
        #calculate control
        a_theta = pendulum_output_sim(state, control, m, L, g, kf)
        state[1] += a_theta * dt #update state, more velocity based on accel
        state[0] += state[1] * dt #update state, more theta based on velocity

        previous_error = error

        time_steps.append(i*dt)
        state_values.append(state.copy()) #must be .copy because this is an address if its just state, im appending the address of the object, state, over and over again
        control_values.append(control)
        setpoint_values.append(setpoint)
    
    #plotting thing that i lowk chatted
    plt.figure()
    plt.plot(time_steps, [s[0] for s in state_values], label="theta")
    plt.plot(time_steps, setpoint_values, label="setpoint", linestyle="--")
    plt.xlabel("time (s)")
    plt.ylabel("angle (rad)")
    plt.legend()
    plt.show()

    return time_steps, state_values, control_values, setpoint_values, L, dt

def pid_control(setpoint, state, kp, ki, kd, previous_error, integral, dt):
    error = setpoint - state[0] #state.theta
    integral += error*dt
    derivative = (error - previous_error)/dt
    control = kp * error + ki * integral + kd * derivative #combined output
    return control, error, integral    

def pendulum_output_sim(state, control, m, L, g, kf):
    I = m * L**2
    torque_f = -kf * state[1]
    torque_g = g * m * L * np.sin(state[0])
    torque_i = control

    a_theta = (torque_i + torque_f + torque_g)/I
    
    return a_theta

if __name__ == "__main__":
    main()
