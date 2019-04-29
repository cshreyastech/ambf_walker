import rospy
import numpy as np
import ambf_msgs.msg as ambf
from Model import Exoskeleton


class AMBF(Exoskeleton.Exoskeleton):

    def __init__(self, name_space, mass, height):
        """

        :param name_space: name space of the the body
        """

        super(AMBF, self).__init__(mass, height)
        rospy.init_node("AMBF_Walker")
        self.time = 0
        self.sub = rospy.Subscriber(name_space + "/body/State", ambf.ObjectState, self.joint_callback)
        self.tau_pub = rospy.Publisher(name_space + "/body/Command", ambf.ObjectCmd, queue_size=1)
        self._time = 0
        self._dt = 0

    @property
    def time(self):
        return self._time

    @time.setter
    def time(self, value):
        self._time = value

    @property
    def dt(self):
        return self._dt

    @dt.setter
    def dt(self, value):
        self._dt = value

    def send_command(self, tau):
        cmd = ambf.ObjectCmd()
        cmd.joint_cmds = tau
        cmd.publish_joint_names = True
        cmd.publish_joint_positions = True

        self.tau_pub.publish(cmd)

    def joint_callback(self, msg):
        """
        callback fnc for the joint listener
        :type msg: ambf.ObjectState
        :param msg:
        :return:
        """

        # this checks if we have the joint states enable and if they are
        # not then they are are enables
        if not msg.joint_positions:
            msg = ambf.ObjectCmd()
            msg.publish_joint_names = True
            msg.publish_joint_positions = True
            self.tau_pub.publish(msg)
            return None

        q = np.array([0.0] * 7)
        qd = np.array([0.0] * 7)
        q[1:] = np.asarray(msg.joint_positions)
        q = np.round(q, 3)

        # Correct the joint angles
        # q[1] *= -1
        # q[2] *= -1
        # q[4] *= -1
        # q[5] *= -1

        if self.time == 0:
            self.time = msg.sim_time

        qd = (q - self.q) / (msg.sim_time - self.time + 0.0000001)

        self.dt = msg.wall_time - self.time
        self.time = msg.wall_time
        self.update_joints(q, qd)
