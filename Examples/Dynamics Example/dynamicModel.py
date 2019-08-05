import numpy as np
import math


class R5M(object):

    # density of water (kg/L)
    water_density = 1.0;  #1.023 for seawater

    dof = 4
    # DH parameters (mm,rads)
    d = 1e-3*np.array([46.2,0.,0.,-180.0,0.])
    a = 1e-3*np.array([20,150.71,20,0.,0.])
    alpha = np.array([math.pi/2,math.pi,-math.pi/2,math.pi/2,0.])
    theta_offset = np.array([math.pi,-math.atan(145.3/40),-math.atan(145.3/40),math.pi/2,-math.pi])
    # mass of each link in kg
    m = np.array([0.341,0.194,0.429,0.115,0.333])
    # COM in m
    r_m = np.array([[-0.075,-0.006,-0.003],[0.005,-0.001,0.016],[0.073,-0.000,-0.000],[0.017,-0.026,-0.002],[0.000,0.003,-0.098]])
    # buoyancy of each link in kg
    b = water_density*np.array([0.202,0.018,0.203,0.025,0.155])
    # COB in m
    r_b = np.array([[-0.075,-0.006,-0.003],[-0.001,-0.003,0.032],[0.073,-0.000,-0.002],[0.003,0.001,-0.017],[0.000,0.003,-0.098]])
    # Moment of inertia in kg.m^2
    Io = np.array([[[0.000099,0.000139,0.000115],[0.000139,0.002920,0.000003],[0.000115,0.000003,0.002934]],[[0.000189,0.000005,0.000054],[0.000005,0.000213,0.000003],[0.000054,0.000003,0.000067]],[[0.000087,-0.000076,-0.000010],[-0.000076,0.003190,0.000000],[-0.000010,0.0000000,0.003213]],[[0.000120,-0.000061,-0.000001],[-0.000061,0.000062,0.000000],[-0.000001,0.000000,0.000156]],[[0.003709,0.000002,-0.000004],[0.000002, 0.003734,-0.000074],[-0.000004,-0.000074,0.000079]]])

    # torque (Nm) to current (mA) coefficient
    kt = np.array([90.6,90.6,90.6,50,50])
    # static friction current
    I_static = np.array([43.0,43.0,43.0,43.0,43.0])
    # static friction torque
    t_static = I_static*(1/kt)

    def torque_to_current_map(self,torque,joint):
        if -self.t_static[joint] <= torque <= self.t_static[joint]:
            deadband = 0.0
        elif torque >= 0:
            deadband =  self.I_static[joint]
        else:
            deadband= -self.I_static[joint]

        current = self.kt[joint]*torque + deadband
        return current

    def current_to_torque_map(self,current,joint):
        if current >= self.I_static[joint]:
            deadband = self.I_static[joint]
        elif current <= -self.I_static[joint]:
            deadband= -self.I_static[joint]
        else:
            deadband = current

        torque = (1/self.kt[joint])*(current - deadband)
        return torque

    def current_error_to_torque_map(self,current_measured,current_expected,joint):
        current_error = current_measured - current_expected
        if current_error >= self.I_static[joint]:
            deadband = self.I_static[joint]
        elif current_error <= -self.I_static[joint]:
            deadband= -self.I_static[joint]
        else:
            deadband = current_error

        torque_error = (1/self.kt[joint])*(current_error - deadband)
        return torque_error

