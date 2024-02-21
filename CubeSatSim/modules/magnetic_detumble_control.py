import os
import numpy as np
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def v3_tilde(vector):
    return [[0, -vector[2], vector[1]], 
            [vector[2], 0 , -vector[0]], 
            [-vector[1], vector[0], 0]]


class MagneticDetumbleControl(sysModel.SysModel):
    """
    Python module to implement a detumble algorithm using magneto torque rods
    """
    def __init__(self):
        super(MagneticDetumbleControl, self).__init__()

        # Derivative gain term used in control
        self.P = 1
        # Input messages
        self.navAttInMsg = messaging.NavAttMsgReader()
        self.tamSensorInMsg = messaging.TAMSensorBodyMsgReader()
        self.mtbConfigInMsg = messaging.MTBArrayConfigMsgReader()
        
        # Output body torque message name
        self.cmdTorqueOutMsg = messaging.MTBCmdMsg_C()
        
        return

    def Reset(self, CurrentSimNanos):

        return

    def UpdateState(self, CurrentSimNanos):

        # read input messages
        navAttMsgBuffer = self.navAttInMsg()
        tamSensorInMsgBuffer = self.tamSensorInMsg()
        mtbConfigInMsgBuffer = self.mtbConfigInMsg()

        # create output message buffer
        cmdTorqueOutMsgBuffer = messaging.MTBCmdMsgPayload()

        # Compute the [Bx][Gt] matrix
        B_tilde_B = v3_tilde(tamSensorInMsgBuffer.tam_B)
        numMTB = mtbConfigInMsgBuffer.numMTB
        Gt_vector_B = mtbConfigInMsgBuffer.GtMatrix_B[:3*numMTB]

        Gt_B = np.zeros([3, numMTB])
        for i in range(numMTB):
            Gt_B[:, i] = [Gt_vector_B[i], Gt_vector_B[i+numMTB], Gt_vector_B[i+2*numMTB]]
        
        # Define the desired torque from the Lyapunov control law
        tau_B = -np.dot(self.P*np.eye(3), navAttMsgBuffer.omega_BN_B) 
        
        # desired moments found using pseudo inverse (least squares)
        pinv = np.linalg.pinv(np.dot(Gt_B.T, Gt_B))
        dipole_command_B = np.dot(np.dot(pinv, Gt_B.T), np.dot(B_tilde_B, tau_B))
                
        # saturate the commands if they are greater than the maximum moment
        max_dipoles = mtbConfigInMsgBuffer.maxMtbDipoles[0:numMTB]
        
        for i in range(numMTB):
            if np.abs(dipole_command_B[i]) > max_dipoles[i] : dipole_command_B[i] = np.sign(dipole_command_B[i])*max_dipoles[i]

        # compute control solution
        cmdTorqueOutMsgBuffer.mtbDipoleCmds = dipole_command_B.tolist()
        messaging.MTBCmdMsg_C_write(cmdTorqueOutMsgBuffer, self.cmdTorqueOutMsg, self.moduleID, CurrentSimNanos)

        return
    