import abc
import numpy as np
import rbdl
import ExoHumanModel
import time
from lib.GaitCore.Core import Point
from std_msgs.msg import Float32MultiArray
from threading import Thread


class ExoHuman(ExoHumanModel.ExoHumanModel):

    def __init__(self, client, mass, height):
        super(ExoHuman, self).__init__(client, mass, height)
        self._handle = self._client.get_obj_handle('Hiprob')
        self.q = 22 * [0.0]
        self.qd = 22 * [0.0]
        time.sleep(2)
        self._state = (self._q, self._qd)
        # self._updater.start() this has to be uncommented


    def update_torque(self, tau):
        tau[4] *= -1
        tau[10] *= -1
        tau[1] = 0.0
        tau[3] = 0.0
        tau[5] = 0.0
        tau[7] = 0.0
        tau[9] = 0.0
        tau[11] = 0.0
        tau[12] = 0.0
        tau[13] = 0.0
        tau[14] = 0.0
        tau[15] = 0.0
        tau[16] = 0.0
        tau[17] = 0.0
        tau[18] = 0.0
        tau[19] = 0.0
        tau[20] = 0.0
        tau[21] = 0.0
        super(ExoHuman, self).update_torque(tau)

    @property
    def q(self):
        return self._q

    @property
    def qd(self):
        return self._qd

    @q.setter
    def q(self, value):
        value[4] *= -1
        value[10] *= -1
        self._q = np.array(value)

    @qd.setter
    def qd(self, value):
        value[4] *= -1
        value[10] *= -1
        self._qd = np.array(value)

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = np.concatenate(value)

    def dynamic_model(self, total_mass, height):

        model = rbdl.Model()
        bodies = {}
        mass = {}
        com = {}
        inertia = {}

        bodies["Right"] = {}


        bodies["Right"] = {}
        bodies["Left"] = {}
        segments = ["thighrob", "shankrob", "footrob"]

        mass["Hiprob"] = 2.37

        mass["Rightthighrob"] = 2.11
        mass["Leftthighrob"] = 2.11
        mass["Rightshankrob"] = 1.28
        mass["Leftshankrob"] = 1.28
        mass["Rightfootrob"] = 0.86
        mass["Leftfootrob"] = 0.86

        parent_dist = {}
        parent_dist["Hiprob"] = np.array([0.0, 0.0, 0.0])

        parent_dist["Leftthighrob"] = np.array([0.237, -0.124, -0.144])
        parent_dist["Leftshankrob"] = np.array([0.033, -0.03, -0.436])
        parent_dist["Leftfootrob"] = np.array([0.02, -0.027, -0.39])

        parent_dist["Rightthighrob"] = np.array([-0.237, -0.124,  -0.144])
        parent_dist["Rightshankrob"] = np.array([0.033, -0.03,  -0.436])
        parent_dist["Rightfootrob"] = np.array([0.02, -0.027,  -0.39])

        inertia["Hiprob"] = np.diag([ 0.0,0.0,0.0])

        inertia["Leftthighrob"] = np.diag([0.0, 0.0, 0.07])
        inertia["Leftshankrob"] = np.diag([0.18, 0.18, 0.0])
        inertia["Leftfootrob"] = np.diag([0.07, 0.07, 0.0])

        inertia["Rightthighrob"] = np.diag([0.0, 0.00, 0.07])
        inertia["Rightshankrob"] = np.diag([0.18, 0.18, 0.0])
        inertia["Rightfootrob"] = np.diag([0.07, 0.07, 0.0])

        com["Hiprob"] = np.array([0.00, -0.02, 0.18])
        com["Leftthighrob"] = np.array([0.02, 0.01,  -0.09])
        com["Leftshankrob"] = np.array([-0.02, -0.007, 0.06])
        com["Leftfootrob"] = np.array([0.08, -0.06, 0.04])

        com["Rightthighrob"] = np.array([-0.02, 0.01, -0.09])
        com["Rightshankrob"] = np.array([0.02, -0.007, 0.06])
        com["Rightfootrob"] = np.array([0.08, -0.06, 0.04])


        hip_body = rbdl.Body.fromMassComInertia(mass["Hiprob"], com["Hiprob"], inertia["Hiprob"])
        for segs in segments:
            bodies["Right" + segs] = rbdl.Body.fromMassComInertia(mass["Right" + segs], com["Right" + segs], inertia["Right" + segs])
            bodies["Left" + segs] = rbdl.Body.fromMassComInertia(mass["Left" + segs], com["Left" + segs], inertia["Left" + segs])

        xtrans = rbdl.SpatialTransform()
        xtrans.r = np.array([0.0, 0.0, 0.0])
        xtrans.E = np.eye(3)
        
        self.Hiprob = model.AddBody(0, xtrans, rbdl.Joint.fromJointType("JointTypeFixed"), hip_body,"Hiprob")
        joint_rot_z =  rbdl.Joint.fromJointType("JointTypeRevoluteX")

        xtrans.r = parent_dist["Leftthighrob"]
        self.Leftthighrob = model.AddBody(self.Hiprob, xtrans, joint_rot_z, bodies["Leftthighrob"], "Leftthighrob")
        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["Leftshankrob"]
        self.Leftshankrob = model.AddBody(self.Leftthighrob, xtrans, joint_rot_z, bodies["Leftshankrob"], "Leftshankrob")
        xtrans.r = parent_dist["Leftfootrob"]
        self.leftfootrob = model.AddBody(self.Leftshankrob, xtrans, joint_rot_z, bodies["Leftfootrob"], "Leftfootrob")

        xtrans.r = parent_dist["Rightthighrob"]
        self.Rightthighrob = model.AddBody(self.Hiprob, xtrans, joint_rot_z, bodies["Rightthighrob"], "Rightthighrob")
        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["Rightshankrob"]
        self.Rightshankrob = model.AddBody(self.Rightthighrob, xtrans, joint_rot_z, bodies["Rightshankrob"], "Rightshankrob")
        xtrans.r = parent_dist["Rightfootrob"]
        self.right_foot = model.AddBody(self.Rightshankrob, xtrans, joint_rot_z, bodies["Rightfootrob"], "Rightfootrob")


        model.gravity = np.array([0, 0, -9.81])


        return model