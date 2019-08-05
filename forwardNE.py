import numpy as np
import math


class FNE():

    def __init__(self, dof, d, a, alpha, theta_offset, m, r_m, Io):

        self.dof = dof
        self.d = d
        self.a = a
        self.alpha = alpha
        theta = np.array([math.pi,math.pi/2,math.pi,0,0])
        self.theta_offset = theta_offset
        self.theta = self.theta_offset + theta
        self.dtheta = np.zeros(self.dof)
        self.ddtheta = np.zeros(self.dof)
        self.joint_pos = np.zeros((3,self.dof + 1))
        self.joint_axis = np.zeros((3, self.dof))
        self.jacobian = np.zeros((6, self.dof))
        self.base_force = np.zeros((6, 1))
        self.m = m
        self.r_m = np.transpose(r_m)
        self.Io = Io
        self.f = np.zeros((3,self.dof))
        self.n = np.zeros((3,self.dof))
        self.e_z = np.array([[0],[0],[1.0]])
        self.tau = np.zeros((self.dof,1))
        self.omega = np.zeros((3,self.dof))
        self.domega = np.zeros((3,self.dof))
        self.acc = np.zeros((3,self.dof))
        self.dds = np.zeros((3,self.dof))

        pass

    # forward Newton Euler method for dynamics
    def run_FNE(self,theta,dtheta,ddtheta):
        self.theta = np.add(self.theta_offset,np.array(theta))
        self.dtheta = np.array(dtheta)
        self.ddtheta = np.array(ddtheta)
        # base angular velocity, angular acceleration, and linear acceleration
        omega0 = np.zeros((3,1)), domega0 = np.zeros((3, 1)), acc0 = np.array([[0],[0],[9.81]])
        # end effector force
        f_end = np.zeros((3, 1)), n_end = np.zeros((3, 1))

        self.forward_recursion(self.theta,self.alpha,self.a,self.d,omega0,domega0,acc0,self.dtheta,self.ddtheta,self.r_m)
        self.backward_recursion(self.theta,self.alpha,self.a,self.d,f_end,n_end,self.r_m,self.m,self.Io)

        return None

    # link transform using standard DH parameters
    def link_tranform(self,theta,alpha,a,d):
        Rx = np.array([[1,0,0,0],[0,math.cos(alpha),-math.sin(alpha),0],[0,math.sin(alpha),math.cos(alpha),0],[0,0,0,1]])
        Rz = np.array([[math.cos(theta),-math.sin(theta),0,0],[math.sin(theta),math.cos(theta),0,0],[0,0,1,0],[0,0,0,1]])
        Tx = np.array([[1,0,0,a],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        Tz = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]])
        T = Rz@Tz@Tx@Rx
        return T

    # calculate velocities/accelerations using forward recursion
    def forward_recursion(self,theta,alpha,a,d,omega0,domega0,acc0,dtheta,ddtheta,r_m):

        T = self.link_tranform(0,0,0,0) @ self.link_tranform(theta[0],0,0,0)

        for i in range(self.dof):
            next_R_prev = np.transpose(T[0:3,0:3])
            prev_P_next = T[0:3,-1]
            if i > 0:
                omega_prev = self.omega[:,i-1:i]
                domega_prev = self.domega[:,i-1:i]
                acc_prev = self.acc[:,i-1:i]
            else:
                omega_prev = omega0
                domega_prev = domega0
                acc_prev = acc0

            # angular velocity
            self.omega[:,i:i+1] = (next_R_prev @ omega_prev) + (self.e_z * dtheta[i])
            # angular acceleration
            self.domega[:,i:i+1] = (next_R_prev @ domega_prev) + np.cross(next_R_prev @ omega_prev,self.e_z * dtheta[i],axis = 0) + (self.e_z * ddtheta[i])
            # linear accleration of joint
            self.acc[:,i:i+1] = next_R_prev @ (acc_prev + np.cross(domega_prev, prev_P_next,axis = 0) + np.cross(omega_prev, np.cross(omega_prev, prev_P_next,axis = 0),axis = 0))
            # linear acceleration of COM
            self.dds[:,i:i+1] = self.acc[:,i:i+1] + np.cross(self.domega[:,i:i+1], r_m[:,i+1:i+2],axis = 0) + np.cross(self.omega[:,i:i+1],np.cross(self.omega[:,i:i+1],r_m[:,i+1:i+2],axis = 0),axis = 0)
            T = self.link_tranform(0, alpha[i], a[i], d[i]) @ self.link_tranform(theta[i+1],0,0,0)

        return None

    # calculate forces/torques on joint using backwards recursion
    def backward_recursion(self,theta,alpha,a,d,f_end,n_end,r_m,m,Io):

        for i in reversed(range(self.dof)):
            T =  self.link_tranform(0,alpha[i],a[i],d[i]) @ self.link_tranform(theta[i+1],0,0,0)
            prev_R_next = T[0:3,0:3]
            prev_P_next = T[0:3,-1]
            if i < self.dof-1:
                f_next = self.f[:,i+1:i+2]
                n_next = self.n[:,i+1:i+2]
            else:
                f_next = f_end
                n_next = n_end


            # force due to acc of COM
            f_hat = m[i+1]*self.dds[:,i:i+1]
            # add force on joint due to previous joint
            self.f[:,i:i+1] = prev_R_next @ f_next + f_hat
            # torque due to angular acceleration/velocity
            n_hat = Io[i+1,:,:] @ self.domega[:,i:i+1] + np.cross(self.omega[:,i:i+1], Io[i+1,:,:] @ self.omega[:,i:i+1],axis = 0)
            # add force due to previous joint
            self.n[:,i:i+1] = (prev_R_next @ n_next) + n_hat + np.cross(r_m[:,i+1:i+2], f_hat,axis = 0) + np.cross(prev_P_next,prev_R_next @ f_next,axis = 0)
            # torque along joint axis
            self.tau[i] = np.transpose(self.e_z) @ self.n[:,i:i+1]

        # calculate base force
        T = self.link_tranform(theta[0],0,0,0)
        prev_R_next = T[0:3, 0:3]
        self.base_force = np.array([[prev_R_next@self.f[:,0]],[prev_R_next@self.n[:,0]]])
        return None

    # calculate forward kinematics using DH parameters
    def forward_kinematics(self,theta,alpha,a,d):

        T = np.identity(4)

        for i in range(self.dof):
            self.joint_pos[:, i] = T[0:3, -1]
            self.joint_axis[:, i] = T[0:3, 2]
            T = T @ self.link_tranform(theta[i], alpha[i], a[i], d[i])

        self.joint_pos[:, -1] = T[0:3, -1]

        return None

    # calculate manipulator Jacobian
    def calculate_jacobian(self,theta,alpha,a,d):
        self.forward_kinematics(theta,alpha,a,d)
        T = np.identity(4)
        endPos = self.joint_pos[:,-1]
        for i in range(self.dof):
            self.jacobian[0:3, i] = np.cross(self.joint_axis[:, i],endPos - self.joint_pos[:,i])
            self.jacobian[3:6, i] = self.joint_axis[:, i]

        return None

    # calculate least squares approximation of end effector force
    def estimate_end_force(self,tau_error):
        self.calculate_jacobian(self.theta,self.alpha,self.a,self.d)
        endForce = np.linalg.pinv(np.transpose(self.jacobian[0:3,:])) @ tau_error
        return endForce

