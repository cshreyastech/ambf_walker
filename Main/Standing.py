import rospy
from Simulation import AMBF
from Utlities import Read_Mocap
from Controller import PD_Controller
import numpy as np
from std_msgs.msg import Float64MultiArray
import time as clock
from lib.Python import RMP_runner
from Tkinter import *
import ambf_msgs.msg as ambf



def calculate_gain(Ku, Tu):
    Td = Tu / 8.0
    Kp = 0.8 * Ku
    Kd = (Ku * Tu) / 10.0
    return Kp, Kd

def go(count):

    q = sim.q
    qd = sim.qd

    for ii in xrange(6):
        q_goal[ii] = 0
        qd_goal[ii] = 0
        qdd_goal[ii] = 0

    aq = qdd_goal + Controller.calc(q_goal - q, qd_goal - qd)
    tau = sim.calculate_dynamics(q_d, qd_d, aq)

    for i in xrange(0, 6):
        cmd[i] = tau[i + 1]

    sim.send_command(cmd)

    dt = sim.dt

    clock.sleep(0.01)
    #print dt*1000
    #root.after(10,go)

path = "/home/nathaniel/catkin_ws/src/AMBF_Walker/config/"
sim = AMBF.AMBF("/ambf/env", 52, 1.57)
Lhip_runner = RMP_runner.RMP_runner(path + "hip_left.xml")
Lknee_runner = RMP_runner.RMP_runner(path + "knee_left.xml")
Lankle_runner = RMP_runner.RMP_runner(path + "ankle_left.xml")

Rhip_runner = RMP_runner.RMP_runner(path + "hip_right.xml")
Rknee_runner = RMP_runner.RMP_runner(path + "knee_right.xml")
Rankle_runner = RMP_runner.RMP_runner(path + "ankle_right.xml")

traj_hip = rospy.Publisher("traj_hip", Float64MultiArray, queue_size=1)
traj_knee = rospy.Publisher("traj_knee", Float64MultiArray, queue_size=1)
traj_ankle = rospy.Publisher("traj_ankle", Float64MultiArray, queue_size=1)

q_d = np.asarray([0.0] * 7)
qd_d = np.asarray([0.0] * 7)
qdd_d = np.asarray([0.0] * 7)
q_goal = np.asarray([0.0] * 7)
qd_goal = np.asarray([0.0] * 7)
qdd_goal = np.asarray([0.0] * 7)
cmd = np.asarray([0.0] * 6)
dt = 0

#root = Tk()


gains = ( [ 510,36.8],[565,42.89],[354,45.2],[ 510,36.8],[565,42.89],[354,45.2])

Kp = np.array([0,gains[0][0],gains[1][0],gains[2][0],gains[3][0],gains[4][0],gains[5][0] ])
Kd = np.array([0,gains[0][1],gains[1][1],gains[2][1],gains[3][1],gains[4][1],gains[5][1] ])

Controller = PD_Controller.PDController(Kp, Kd)


if __name__ == "__main__":

    count = 0
    while 1:
        go(count)
        count+=1

    #root.after(1, go)
    #root.mainloop()









