import abc
import numpy as np
import rbdl
import time
from std_msgs.msg import Float32MultiArray
from threading import Thread
import os
import sys

if "../" not in sys.path:
  sys.path.append("../")
from lib.GaitCore.Core import Point

class ExoHumanRBDL():
    def __init__(self):
        # print("ExoHumanRBDL")
        self.rbdl_model_ = self.dynamic_model()


    def dynamic_model(self):

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
        xtrans.r = parent_dist["Rightthighrob"]
        self.Rightthighrob = model.AddBody(self.Hiprob, xtrans, joint_rot_z, bodies["Rightthighrob"], "Rightthighrob")

        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["Leftshankrob"]
        self.Leftshankrob = model.AddBody(self.Leftthighrob, xtrans, joint_rot_z, bodies["Leftshankrob"], "Leftshankrob")
        xtrans.r = parent_dist["Leftfootrob"]
        self.Leftfootrob = model.AddBody(self.Leftshankrob, xtrans, joint_rot_z, bodies["Leftfootrob"], "Leftfootrob")

        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["Rightshankrob"]
        self.Rightshankrob = model.AddBody(self.Rightthighrob, xtrans, joint_rot_z, bodies["Rightshankrob"], "Rightshankrob")
        xtrans.r = parent_dist["Rightfootrob"]
        self.Lightfootrob = model.AddBody(self.Rightshankrob, xtrans, joint_rot_z, bodies["Rightfootrob"], "Rightfootrob")

        
###########################################################################################
        segments = ["thigh", "shank", "foot"]

        mass["body"] = 2.37
        mass["head"] = 1.0
        mass["Crutch1"] = 1.0
        mass["Crutch2"] = 1.0


        mass["rightthigh"] = 2.11
        mass["leftthigh"] = 2.11
        mass["rightshank"] = 1.28
        mass["leftshank"] = 1.28
        mass["rightfoot"] = 0.86
        mass["leftfoot"] = 0.86

        parent_dist = {}
        parent_dist["body"] = np.array([0.0, -0.1273, 0.72])
        parent_dist["head"] = np.array([0.0, -0.1273, 0.72])

        parent_dist["Crutch1"] = np.array([0.493, -0.8677, -0.3804])
        parent_dist["Crutch2"] = np.array([-0.4642, -0.8677, -0.3804])

        parent_dist["leftthigh"] = np.array([0.0043, -0.1536, 0.0923])
        parent_dist["leftshank"] = np.array([0.1466, -0.1308, -0.3343])
        parent_dist["leftfoot"] = np.array([0.1466, -0.1014, -0.7709])

        parent_dist["rightthigh"] = np.array([-0.0066, -0.1536, 0.0923])
        parent_dist["rightshank"] = np.array([-0.149, -0.1308, -0.3343])
        parent_dist["rightfoot"] = np.array([-0.149, -0.1014, -0.7709])


        inertia["body"] = np.diag([ 0.0,0.0,0.0])
        inertia["head"] = np.diag([ 0.0,0.0,0.0])

        inertia["Crutch1"] = np.diag([ 0.0,0.0,0.0])
        inertia["Crutch2"] = np.diag([ 0.0,0.0,0.0])

        inertia["leftthigh"] = np.diag([0.0, 0.0, 0.07])
        inertia["leftshank"] = np.diag([0.18, 0.18, 0.0])
        inertia["leftfoot"] = np.diag([0.07, 0.07, 0.0])

        inertia["rightthigh"] = np.diag([0.0, 0.00, 0.07])
        inertia["rightshank"] = np.diag([0.18, 0.18, 0.0])
        inertia["rightfoot"] = np.diag([0.07, 0.07, 0.0])

        com["body"] = np.array([0.0, 0.0, 0.0])
        com["head"] = np.array([0.0, 0.0, 0.0])

        com["Crutch1"] = np.array([0.0, 0.0, 0.0])
        com["Crutch2"] = np.array([0.0, 0.0, 0.0])

        com["leftthigh"] = np.array([0.1009, 0.0088, -0.1974])
        com["leftshank"] = np.array([-0.0305, 0.0174, -0.2118])
        com["leftfoot"] = np.array([-0.0305, -0.0833, -0.0036])

        com["rightthigh"] = np.array([-0.1009, 0.0088, -0.1974])
        com["rightshank"] = np.array([0.0305, 0.0174, -0.2118])
        com["rightfoot"] = np.array([0.0305, -0.0833, -0.0036])

        # hip_body = rbdl.Body.fromMassComInertia(mass["Hiprob"], com["Hiprob"], inertia["Hiprob"])
        for segs in segments:
            bodies["right" + segs] = rbdl.Body.fromMassComInertia(mass["right" + segs], com["right" + segs], inertia["right" + segs])
            bodies["left" + segs] = rbdl.Body.fromMassComInertia(mass["left" + segs], com["left" + segs], inertia["left" + segs])

        xtrans = rbdl.SpatialTransform()
        xtrans.r = np.array([0.0, 0.0, 0.0])
        xtrans.E = np.eye(3)


        xtrans.r = parent_dist["leftthigh"]
        self.leftthigh = model.AddBody(self.Hiprob, xtrans, joint_rot_z, bodies["leftthigh"], "leftthigh")
        xtrans.r = parent_dist["rightthigh"]
        self.rightthigh = model.AddBody(self.Hiprob, xtrans, joint_rot_z, bodies["rightthigh"], "rightthigh")

        bodies["body"] = rbdl.Body.fromMassComInertia(mass["body"], com["body"], inertia["body"])
        bodies["Crutch1"] = rbdl.Body.fromMassComInertia(mass["Crutch1"], com["Crutch1"], inertia["Crutch1"])
        bodies["Crutch2"] = rbdl.Body.fromMassComInertia(mass["Crutch2"], com["Crutch2"], inertia["Crutch2"])

        bodies["head"] = rbdl.Body.fromMassComInertia(mass["head"], com["head"], inertia["head"])

        xtrans.r = parent_dist["body"]
        self.body = model.AddBody(self.Hiprob, xtrans, joint_rot_z, bodies["body"], "body")
        xtrans.r = parent_dist["Crutch1"]
        self.Crutch1 = model.AddBody(self.Hiprob, xtrans, joint_rot_z, bodies["Crutch1"], "Crutch1")
        xtrans.r = parent_dist["Crutch2"]
        self.Crutch2 = model.AddBody(self.Hiprob, xtrans, joint_rot_z, bodies["Crutch2"], "Crutch2")

        xtrans.r = parent_dist["head"]
        self.head = model.AddBody(self.body, xtrans, joint_rot_z, bodies["head"], "head")


        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["leftshank"]
        self.leftshank = model.AddBody(self.leftthigh, xtrans, joint_rot_z, bodies["leftshank"], "leftshank")
        xtrans.r = parent_dist["leftfoot"]
        self.leftfoot = model.AddBody(self.leftshank, xtrans, joint_rot_z, bodies["leftfoot"], "leftfoot")

        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["rightshank"]
        self.rightshank = model.AddBody(self.rightthigh, xtrans, joint_rot_z, bodies["rightshank"], "rightshank")
        xtrans.r = parent_dist["rightfoot"]
        self.rightfoot = model.AddBody(self.rightshank, xtrans, joint_rot_z, bodies["rightfoot"], "rightfoot")


        model.gravity = np.array([0, 0, -9.81])
        # print(bodies)

        return model
    def check_model(self):
        n_joints = len(self.rbdl_model_.mBodies)

        for joint_id in range(n_joints):
            joint_name = self.rbdl_model_.GetBodyName(joint_id)
            joint_parent_id = self.rbdl_model_.GetParentBodyId(joint_id)
            joint_parent_name = self.rbdl_model_.GetBodyName(joint_parent_id)

            # print("Joint ID: {}, Name: {}, Parent ID: {}, Parent Name: {}".format(joint_id, joint_name, joint_parent_id, joint_parent_name))
            print("Joint ID: {}, Name: {}, Parent Name: {}".format(joint_id, joint_name, joint_parent_name))

    



if __name__ == "__main__":
    EXOHUMANRBDL = ExoHumanRBDL()
    EXOHUMANRBDL.check_model()

    # print(rbdl_model.mBodies[0])
    
    
    # print(rbdl_model.mBodies[0])
    # base_body_name = "Hiprob"
    # # print(rbdl_model.GetBodyId("ssdfsdsdfdssf"))
    # joint_id = 1
    # print(rbdl_model.GetBodyName(rbdl_model.GetParentBodyId(joint_id)))
    
