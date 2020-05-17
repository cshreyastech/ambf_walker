# Import the Client from ambf_client package
from ambf_client import Client
import os
import sys
if "../" not in sys.path:
  sys.path.append("../")

from Utlities import Plotter
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from Model import ExoHuman
from Controller import DynControllerExoHuman
import time
from Controller import PDController

def get_traj(q0, qf, v0, vf, tf, dt):

    b = np.array([q0, v0, qf, vf]).reshape((-1,1))
    A = np.array([[1.0, 0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0],
                  [1.0, 0.0, tf ** 2, tf ** 3],
                  [0.0, 0.0, 2 * tf, 3 * tf * 2]])

    x = np.linalg.solve(A, b)
    q = []
    qd = []
    qdd = []

    for t in np.linspace(0, tf, tf/dt):
        q.append(x[0] + x[1] * t + x[2] * t * t + x[3] * t * t * t)
        qd.append(x[1] + 2*x[2] * t + 3*x[3] * t * t)
        qdd.append(2*x[2] + 6*x[3] * t)

    return q, qd, qdd

if __name__ == "__main__":
    pub = rospy.Publisher('qd',Float32MultiArray, queue_size=1)
    pub_goal = rospy.Publisher('goal', Float32MultiArray, queue_size=1)
    msg_vel = Float32MultiArray()
    msg_goal = Float32MultiArray()


    _client = Client()
    _client.connect()
    rate = rospy.Rate(1000)
    EXOHUMAN =  ExoHuman.ExoHuman(_client, 56, 1.56)
    
    # num_joints = EXOHUMAN._handle.get_num_joints() # Get the number of joints of this object
    # children_names = EXOHUMAN._handle.get_children_names() # Get a list of children names belonging to this obj

    # print(num_joints)
    # print(children_names)

    Kp = np.zeros((22, 22))
    Kd = np.zeros((22, 22))

    Kp_hip = 100.0
    Kd_hip = 2.0

    Kp_knee = 125.0
    Kd_knee = 1.0

    Kp_ankle = 750.0
    Kd_ankle = 0.4

    Kp[0, 0] = Kp_hip
    Kd[0, 0] = Kd_hip
    
    Kp[2, 2] = Kp_knee
    Kd[2, 2] = Kd_knee

    Kp[4, 4] = Kp_ankle
    Kd[4, 4] = Kd_ankle

    Kp[6, 6] = Kp_hip
    Kd[6, 6] = Kd_hip

    Kp[8, 8] = Kp_knee
    Kd[8, 8] = Kd_knee

    Kp[10, 10] = Kp_ankle
    Kd[10, 10] = Kd_ankle

    body_controller = PDController.PDController(np.array([2000]), np.array([100]) )
    crl = DynControllerExoHuman.DynControllerExoHuman(EXOHUMAN, Kp, Kd)

    dt = 0.001
    tf = 2.0
    tf2 = 10.0
    q_hip, qd_hip, qdd_hip = get_traj(0.0, -0.3, 0.0, 0.0, tf,dt)
    q_knee, qd_knee, qdd_knee = get_traj(0.0, 0.20, 0.0, 0., tf, dt)
    q_ankle, qd_ankle, qdd_ankle = get_traj(-0.349, 0.157+0.1, 0.0, 0.0, tf, dt)

    count = 0
    EXOHUMAN._handle.set_rpy(0, 0, 0)
    EXOHUMAN._handle.set_pos(0, 0, .1)
    time.sleep(5)
    height = 0.1
    count2 = 0

    while not rospy.is_shutdown():

        count = min(count, int(tf/dt)-1)
        if count == 1999:
            if height < -0.1:
                EXOHUMAN._handle.set_rpy(0.25, 0, 0)
                EXOHUMAN._handle.set_pos(0, 0, height)
            else:
                height -= 0.001
                EXOHUMAN._handle.set_rpy(0.25, 0, 0)
                EXOHUMAN._handle.set_pos(0, 0, height)

        else:
            EXOHUMAN._handle.set_rpy(0.25, 0, 0.0)
            EXOHUMAN._handle.set_pos(0, 0, height)

        q_d = np.array([q_hip[count].item(), 0, q_knee[count].item(), 0, q_ankle[count].item(), 0,
                        q_hip[count].item(), 0, q_knee[count].item(), 0, q_ankle[count].item(), 0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        qd_d = np.array([qd_hip[count].item(), 0, qd_knee[count].item(), 0, qd_ankle[count].item(), 0,
                         qd_hip[count].item(), 0, qd_knee[count].item(), 0, qd_ankle[count].item(), 0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        qdd_d = np.array([qdd_hip[count].item(), 0, qdd_knee[count].item(), 0, qdd_ankle[count].item(), 0,
                          qdd_hip[count].item(), 0, qdd_knee[count].item(), 0, qdd_ankle[count].item(), 0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        crl.calc_tau(q_d, qd_d, qdd_d)
        msg_vel.data = EXOHUMAN.q
        msg_goal.data = q_d
        #leg_plot.update()
        pub.publish(msg_vel)
        pub_goal.publish(msg_goal)
        count += 1
        rate.sleep()

    _client.clean_up()