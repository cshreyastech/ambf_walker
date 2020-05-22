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
        self._updater.start()


    def update_torque(self, tau):
        tau[4] *= -1
        tau[10] *= -1
        tau[1] = 0.0
        tau[3] = 0.0
        tau[5] = 0.0
        tau[7] = 0.0
        tau[9] = 0.0
        tau[11] = 0.0
        tau[12] *= -1
        tau[13] *= -1
        tau[14] *= -1
        tau[15] *= -1
        tau[16] = 0.0
        tau[17] *= -1
        tau[18] *= -1
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
        value[12] *= -1
        value[13] *= -1
        value[14] *= -1
        value[15] *= -1
        value[17] *= -1
        value[18] *= -1
        self._q = np.array(value)

    @qd.setter
    def qd(self, value):
        value[4] *= -1
        value[10] *= -1
        value[12] *= -1
        value[13] *= -1
        value[14] *= -1
        value[15] *= -1
        value[17] *= -1
        value[18] *= -1
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

        # bodies["Right"] = {}
        # bodies["Left"] = {}
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
        self.Leftfootrob = model.AddBody(self.Leftshankrob, xtrans, joint_rot_z, bodies["Leftfootrob"], "Leftfootrob")

        xtrans.r = parent_dist["Rightthighrob"]
        self.Rightthighrob = model.AddBody(self.Hiprob, xtrans, joint_rot_z, bodies["Rightthighrob"], "Rightthighrob")
        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["Rightshankrob"]
        self.Rightshankrob = model.AddBody(self.Rightthighrob, xtrans, joint_rot_z, bodies["Rightshankrob"], "Rightshankrob")
        xtrans.r = parent_dist["Rightfootrob"]
        self.Lightfootrob = model.AddBody(self.Rightshankrob, xtrans, joint_rot_z, bodies["Rightfootrob"], "Rightfootrob")

        
###########################################################################################
        # # bodies["head"] = {}
        # # bodies["Left"] = {}
           
        # segments = ["thigh", "shank", "foot"]

        # mass["body"] = 2.37
        # mass["head"] = 1.0
        # mass["Crutch1"] = 1.0
        # mass["Crutch2"] = 1.0


        # mass["rightthigh"] = 2.11
        # mass["leftthigh"] = 2.11
        # mass["rightshank"] = 1.28
        # mass["leftshank"] = 1.28
        # mass["rightfoot"] = 0.86
        # mass["leftfoot"] = 0.86

        # parent_dist = {}
        # parent_dist["body"] = np.array([0.0, -0.1273, 0.72])
        # parent_dist["head"] = np.array([0.0, -0.1273, 0.72])

        # parent_dist["Crutch1"] = np.array([0.493, -0.8677, -0.3804])
        # parent_dist["Crutch2"] = np.array([-0.4642, -0.8677, -0.3804])

        # parent_dist["leftthigh"] = np.array([0.0043, -0.1536, 0.0923])
        # parent_dist["leftshank"] = np.array([0.1466, -0.1308, -0.3343])
        # parent_dist["leftfoot"] = np.array([0.1466, -0.1014, -0.7709])

        # parent_dist["rightthigh"] = np.array([-0.0066, -0.1536, 0.0923])
        # parent_dist["rightshank"] = np.array([-0.149, -0.1308, -0.3343])
        # parent_dist["rightfoot"] = np.array([-0.149, -0.1014, -0.7709])


        # inertia["body"] = np.diag([ 0.0,0.0,0.0])
        # inertia["head"] = np.diag([ 0.0,0.0,0.0])

        # inertia["leftthigh"] = np.diag([0.0, 0.0, 0.07])
        # inertia["leftshank"] = np.diag([0.18, 0.18, 0.0])
        # inertia["leftfoot"] = np.diag([0.07, 0.07, 0.0])

        # inertia["rightthigh"] = np.diag([0.0, 0.00, 0.07])
        # inertia["rightshank"] = np.diag([0.18, 0.18, 0.0])
        # inertia["rightfoot"] = np.diag([0.07, 0.07, 0.0])

        # com["Crutch1"] = np.array([0.0, 0.0, 0.0])
        # com["Crutch2"] = np.array([0.0, 0.0, 0.0])

        # com["leftthigh"] = np.array([0.1009, 0.0088, -0.1974])
        # com["leftshank"] = np.array([-0.0305, 0.0174, -0.2118])
        # com["leftfoot"] = np.array([-0.0305, -0.0833, -0.0036])

        # com["rightthigh"] = np.array([-0.1009, 0.0088, -0.1974])
        # com["rightshank"] = np.array([0.0305, 0.0174, -0.2118])
        # com["rightfoot"] = np.array([0.0305, -0.0833, -0.0036])

        # # hip_body = rbdl.Body.fromMassComInertia(mass["Hiprob"], com["Hiprob"], inertia["Hiprob"])
        # for segs in segments:
        #     bodies["right" + segs] = rbdl.Body.fromMassComInertia(mass["right" + segs], com["right" + segs], inertia["right" + segs])
        #     bodies["left" + segs] = rbdl.Body.fromMassComInertia(mass["left" + segs], com["left" + segs], inertia["left" + segs])

        # xtrans = rbdl.SpatialTransform()
        # xtrans.r = np.array([0.0, 0.0, 0.0])
        # xtrans.E = np.eye(3)

        # # xtrans.r = parent_dist["humanbody"]
        # # self.humanbody = model.AddBody(self.Hiprob, xtrans, joint_rot_z, bodies["humanbody"], "humanbody")

        # # bodies["head"] = rbdl.Body.fromMassComInertia(mass["head"], com["head"], inertia["head"])
        # # xtrans.E = np.eye(3)
        # # xtrans.r = parent_dist["head"]
        # # self.head = model.AddBody(self.Hiprob, xtrans, joint_rot_z, bodies["head"], "head")




        # xtrans.r = parent_dist["leftthigh"]
        # self.leftthigh = model.AddBody(self.Hiprob, xtrans, joint_rot_z, bodies["leftthigh"], "leftthigh")
        # xtrans.E = np.eye(3)
        # xtrans.r = parent_dist["leftshank"]
        # self.leftshank = model.AddBody(self.leftthigh, xtrans, joint_rot_z, bodies["leftshank"], "leftshank")
        # xtrans.r = parent_dist["leftfoot"]
        # self.leftfoot = model.AddBody(self.leftshank, xtrans, joint_rot_z, bodies["leftfoot"], "leftfoot")

        # xtrans.r = parent_dist["rightthigh"]
        # self.rightthigh = model.AddBody(self.Hiprob, xtrans, joint_rot_z, bodies["rightthigh"], "rightthigh")
        # xtrans.E = np.eye(3)
        # xtrans.r = parent_dist["rightshank"]
        # self.rightshank = model.AddBody(self.rightthigh, xtrans, joint_rot_z, bodies["rightshank"], "rightshank")
        # xtrans.r = parent_dist["rightfoot"]
        # self.rightfoot = model.AddBody(self.Rightshankrob, xtrans, joint_rot_z, bodies["rightfoot"], "rightfoot")


        model.gravity = np.array([0, 0, -9.81])

        return model
