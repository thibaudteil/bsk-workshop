import numpy as np

from Basilisk.architecture import sysModel
from Basilisk.architecture.messaging import AttGuidMsg
from Basilisk.architecture.messaging import CmdTorqueBodyMsg
from Basilisk.architecture.messaging import CmdTorqueBodyMsgPayload
from Basilisk.architecture.messaging import CmdTorqueBodyMsgReader
from Basilisk.architecture.messaging import NavAttMsgPayload
from Basilisk.architecture.messaging import NavAttMsgReader
from Basilisk.utilities import RigidBodyKinematics as rbk
import ipdb

class SunRepulse(sysModel.SysModel):

    def __init__(self):
        super(SunRepulse, self).__init__()

        self.angle_min = 0.0
        self.sHat_B_avoid = np.array([0.0, 0.0, 1.0])
        self.eHat_180_B = np.array([1.0, 0.0, 0.0])
        self.min_unit_vector_magnitude = 0.95
        self.small_angle = 0.01
        self.K_proportional_gain = 1.0

        self.navigation_in_msg = NavAttMsgReader()
        self.attitude_torque_out_msg = CmdTorqueBodyMsg()
        self.base_torque_in_msg = CmdTorqueBodyMsgReader()
        self.attitude_guidance_out_msg = AttGuidMsg()
    
    def UpdateState(self, current_sim_nanos):
        """
        The updateState method is called periodically at the rate specified by 
        the task to which the model is attached.

        :param current_sim_nanos: current simulation time in nano-seconds
        :return: none
        """
        # ipdb.set_trace()
        navigation_msg = self.navigation_in_msg()
        sNorm = np.linalg.norm(navigation_msg.vehSunPntBdy)
        sigma_BR = np.zeros(3)

        if sNorm > self.min_unit_vector_magnitude:
            ct_S_normalized = np.dot(self.sHat_B_avoid, navigation_msg.vehSunPntBdy) / sNorm
            ct_S_normalized = np.sign(ct_S_normalized) * 1.0 if abs(ct_S_normalized) > 1.0 else ct_S_normalized
            sun_angle_err = np.arccos(ct_S_normalized)

            if sun_angle_err > self.angle_min:
                sigma_BR = np.zeros(3)
            else:
                if np.pi - sun_angle_err < self.small_angle:
                    e_hat = self.eHat_180_B
                else:
                    e_hat = np.cross(navigation_msg.vehSunPntBdy, self.sHat_B_avoid)
                sun_maneuver_vector = e_hat / np.linalg.norm(e_hat)
                mrp_remain_BR = np.tan(sun_angle_err * 0.25) * sun_maneuver_vector
                mrp_command_BR = np.tan(self.angle_min * 0.25) * sun_maneuver_vector
                sigma_BR = mrp_command_BR - mrp_remain_BR
                rbk.MRPswitch(sigma_BR, 1.0)
        
        repulse_torque = self.K_proportional_gain * sigma_BR
        # print("repulse torque: ")
        # print(repulseTorque)
        attitude_torque_out = CmdTorqueBodyMsgPayload() 

        attitude_torque_out.torqueRequestBody = (np.array(self.base_torque_in_msg().torqueRequestBody) + repulse_torque).tolist()
        # print("self.baseTorqueInMsg().torqueRequestBody: ")
        # print(self.baseTorqueInMsg().torqueRequestBody)
        # print("attTorqueOut.torqueRequestBody: ")
        # print(attTorqueOut.torqueRequestBody)
        self.attitude_torque_out_msg.write(attitude_torque_out, current_sim_nanos, self.moduleID)

    
    def Reset(self, current_sim_nanos):
        """
        The Reset method is used to clear out any persistent variables that need to get changed
        when a task is restarted.  This method is called once after selfInit,
        but it should be written to allow the user to call it multiple times.
        :param current_sim_nanos: current simulation time in nano-seconds
        :return: none
        """
        if np.linalg.norm(self.sHat_B_avoid) < 0.1:
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"sHat_B_avoid: is not unit norm")
        else:
            v1 = np.array([1.0, 0.0, 0.0])
            self.sHat_B_avoid = self.sHat_B_avoid / np.linalg.norm(self.sHat_B_avoid)
            self.eHat_180_B = np.cross(self.sHat_B_avoid, v1)
            if np.linalg.norm(self.eHat_180_B) < 0.1:
                v1 = np.array([0.0, 1.0, 0.0])
                self.eHat_180_B = np.cross(self.sHat_B_avoid, v1)
            self.eHat_180_B = self.eHat_180_B / np.linalg.norm(self.eHat_180_B)
