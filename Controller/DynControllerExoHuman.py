
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from Model import ExoHumanModel
import PDController
import random

class DynControllerExoHuman():


    def __init__(self, model, kp, kd):
        """

        :param model:
        :param kp:
        :param kd:
        """
        self._model = model
        self.pdController = PDController.PDController(kp, kd)
        self.pub = rospy.Publisher('tau',Float32MultiArray, queue_size=1)

    def set_gains(self, kp, kd):
        """

        :param kp:
        :param kd:
        :return:
        """
        self.pdController.kp = kp
        self.pdController.kd = kd

    def calc_tau(self, q=None, qd=None, qdd=None ):
        """

        :param q:
        :param qd:
        :param qdd:
        :return:
        """
        aq = np.zeros(22)
        if q is not None and qd is not None:
            e = q - self._model.q
            ed = qd - self._model.qd
            aq = self.pdController.get_tau(e, ed)
            if qdd is not None:
                aq = qdd + aq
        tau = self._model.calculate_dynamics(aq)

    
        print(tau)
        msg = Float32MultiArray()
        msg.data = tau.tolist()
        self.pub.publish(msg)
        self._model.update_torque(tau)


