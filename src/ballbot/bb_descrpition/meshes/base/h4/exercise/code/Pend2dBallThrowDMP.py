from math import pi, sin, cos
from matplotlib import pyplot as plt
import numpy as np


class Pend2dBallThrowDMP():
    def __init__(self):
        self.numBasis = 5
        self.numTrajSteps = 100
        self.dimJoints = 2
        self.dt = 0.01
        self.lengths = np.ones(self.dimJoints)
        self.masses = np.ones(self.dimJoints)
        self.initState = np.array([-pi, 0., 0., 0.])
        self.ballGoal = [2., 1.]
        self.releaseStep = 50
        self.Kp = 1400.


    def getDesiredTrajDMP(self, dmp_w):
        alphaz = 8./3.
        alpha  = 25.
        beta   = alpha/4.
        tau    = 1.
        Ts     = 1.
        g      = self.initState[::2]

        C = np.exp(-alphaz*np.arange(self.numBasis)/(self.numBasis-1)*Ts)
        H = 0.5 / (0.65 * np.diff(C)**2)
        H = np.append(H,H[-1])

        q = np.zeros((self.numTrajSteps, 2*self.dimJoints))
        q[0,:] = self.initState
        x = np.ones(self.numTrajSteps)

        for i in range(self.numTrajSteps-1):
            psi = np.exp(-H*(x[i]-C)**2)
            f = np.dot(dmp_w.T, psi) * x[i] / np.sum(psi)
            qdd_des = (alpha * (beta * ( g - q[i,::2] ) - ( q[i,1::2] / tau ) ) + f.T ) * tau**2
            q[i+1,1::2] = q[i,1::2] + qdd_des * self.dt
            q[i+1,::2] = q[i,::2] + q[i+1,1::2] * self.dt
            xd = -alphaz*x[i]*tau
            x[i+1] = x[i] + xd * self.dt

        return q


    def transitionFunction(self, x, action):
        xnew = np.zeros(x.shape)
        xnew[1::2] = x[1::2] + (action / self.masses) * self.dt
        xnew[::2] = x[::2] + xnew[1::2] * self.dt
        return xnew


    def getForwardKinematics(self, theta):
        y = np.zeros((theta.shape[0], 2))[0]
        for i in range(self.dimJoints):
            y += np.array([sin(np.sum(theta[:i+1])), cos(np.sum(theta[:i+1]))]) * self.lengths[i]
        return y


    def getJacobian(self, theta):
        si = self.getForwardKinematics(theta)
        J = np.zeros((2, self.dimJoints))
        for j in range(self.dimJoints):
            pj = np.array([0.0, 0.0])
            for i in range(j):
                pj += np.array([sin(sum(theta[:i+1])), cos(sum(theta[:i+1]))]) * self.lengths[i]
            pj = -(si - pj)
            J[np.ix_([0, 1], [j])] = np.mat([-pj[1], pj[0]]).T
        return [J, si]


    def simulateSystem(self, des_q):
            K = np.zeros((self.dimJoints, 2*self.dimJoints))
            K[:,::2] = self.Kp * np.eye(self.dimJoints)
            K[:,1::2] = 2 * np.sqrt(self.Kp) * np.eye(self.dimJoints)

            q = np.zeros((des_q.shape[0], 2*self.dimJoints))
            q[0,:] = self.initState

            b  = np.zeros((des_q.shape[0], 2))
            bd = np.zeros((des_q.shape[0], 2))
            b[0,:] = self.getForwardKinematics(q[0,:])

            u = np.zeros((des_q.shape[0], self.dimJoints))

            for i in range(des_q.shape[0]-1):
                u[i,:]    = np.dot(K, (des_q[i,:] - q[i,:]).T)
                q[i+1,:]  = self.transitionFunction(q[i,:], u[i,:])
                if i > self.releaseStep:
                    bd[i+1,:] = bd[i,:]
                    bd[i+1,1] = bd[i+1,1] - 10 * self.dt
                    b[i+1,:]  = b[i,:] + bd[i,:] * self.dt
                else:
                    b[i+1,:]  = self.getForwardKinematics(q[i+1,::2])
                    bd[i+1,:] = np.dot(self.getJacobian(q[i+1,::2])[0], q[i+1,1::2].T)

            return [q, u, b, bd]


    def getReward (self, theta):
        q_des = self.getDesiredTrajDMP(np.reshape(theta,(-1,self.numBasis)).T)
        data_traj = self.simulateSystem(q_des)

        uFactor = -1e-4
        uCost = uFactor * np.linalg.norm(data_traj[1])**2
        distFactor = -1e4
        b_diff = self.ballGoal - data_traj[2][-1,:]
        rCost = np.dot(b_diff,b_diff)*distFactor
        return uCost + rCost


    def getJointsInTaskSpace(self, q):
        x1 = np.array(self.lengths[0] * np.array([sin(q[0]), cos(q[0])]))
        x2 = x1 + np.array(self.lengths[1] * np.array([sin(q[2] + q[0]), cos(q[2] + q[0])]))
        return x1, x2


    def visualize(self, q, line):
        lw = 4.0
        fs = 26
        mp1, mp2 = self.getJointsInTaskSpace(q)
        thisx = [0, mp1[0], mp2[0]]
        thisy = [0, mp1[1], mp2[1]]
        line.set_data(thisx, thisy)


    def animate_fig(self, theta):
        q_des = self.getDesiredTrajDMP(np.reshape(theta,(-1,self.numBasis,)).T)
        data_traj = self.simulateSystem(q_des)
        q = data_traj[0]
        b = data_traj[2]
        dim = sum(self.lengths)
        t = [1, 25, 50, 75, 100]
        f, axs = plt.subplots(1,len(t),figsize=(20,5))
        for i in range(len(t)):
            plt.subplot(1,len(t),i+1)
            plt.title('Timestep ' + str(t[i]))
            mp1, mp2 = self.getJointsInTaskSpace(q[t[i]-1,:])
            thisx = [0, mp1[0], mp2[0]]
            thisy = [0, mp1[1], mp2[1]]
            plt.plot(thisx,thisy)
            plt.plot(b[t[i]-1,0],b[t[i]-1,1],marker='x',markersize=18)
            plt.axis((-dim,dim,-dim,dim))
