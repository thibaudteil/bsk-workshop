
import math
import os
import sys
import numpy as np
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (hillPoint, inertial3D, attTrackingError, mrpFeedback,
                                    rwMotorTorque, tamComm, mtbMomentumManagement,
                                    velocityPoint, mrpSteering, rateServoFullNonlinear,
                                    sunSafePoint, cssWlsEst)
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import fswSetupRW
from Basilisk.utilities import macros as mc

path = os.getcwd()
sys.path.append(path + '/../modules')
import magnetic_detumble_control

class CubeSat_fsw:
    def __init__(self, SimBase, fswRate):
        self.vcMsg = None
        self.fswRwConfigMsg = None
        self.cmdTorqueMsg = None
        self.cmdTorqueDirectMsg = None
        self.attRefMsg = None
        self.attGuidMsg = None
        self.cmdRwMotorMsg = None
        self.mtbCmdMsg = None

        self.processName = SimBase.FSWProcessName
        self.processTasksTimeStep = mc.sec2nano(fswRate)

        self.inertial3D = inertial3D.inertial3D()
        self.inertial3D.ModelTag = "inertial3D"

        self.hillPoint = hillPoint.hillPoint()
        self.hillPoint.ModelTag = "hillPoint"

        self.sunSafePoint = sunSafePoint.sunSafePoint()
        self.sunSafePoint.ModelTag = "sunSafePoint"

        self.velocityPoint = velocityPoint.velocityPoint()
        self.velocityPoint.ModelTag  = "velocityPoint"

        self.cssWlsEst = cssWlsEst.cssWlsEst()
        self.cssWlsEst.ModelTag = "cssWlsEst"

        self.trackingError = attTrackingError.attTrackingError()
        self.trackingError.ModelTag = "trackingError"

        self.mrpFeedbackControl = mrpFeedback.mrpFeedback()
        self.mrpFeedbackControl.ModelTag = "mrpFeedbackControl"

        self.mrpFeedbackRWs = mrpFeedback.mrpFeedback()
        self.mrpFeedbackRWs.ModelTag = "mrpFeedbackRWs"

        self.mrpSteering = mrpSteering.mrpSteering()
        self.mrpSteering.ModelTag = "MRP_Steering"

        self.rateServo = rateServoFullNonlinear.rateServoFullNonlinear()
        self.rateServo.ModelTag = "rate_servo"

        self.rwMotorTorque = rwMotorTorque.rwMotorTorque()
        self.rwMotorTorque.ModelTag = "rwMotorTorque"
        
        self.tam_com_module = tamComm.tamComm()
        self.tam_com_module.ModelTag = "tamComm"
        
        # python detumble controller
        self.detumble_module = magnetic_detumble_control.MagneticDetumbleControl()
        self.detumble_module.ModelTag = "magnetic_detumble"
        
        self.mtb_momentum_management_module = mtbMomentumManagement.mtbMomentumManagement()
        self.mtb_momentum_management_module.ModelTag = "mtbMomentumManagement"   
        
        self.setupGatewayMsgs(SimBase)

        self.InitAllFSWObjects(SimBase)

        SimBase.fswProc.addTask(SimBase.CreateNewTask("inertial3DPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("hillPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mtb_mom_management_task", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mtb_detubmle_task", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("sunSafePointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("velocityPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpSteeringRWsTask", self.processTasksTimeStep), 10)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackRWsTask", self.processTasksTimeStep), 10)

        SimBase.AddModelToTask("inertial3DPointTask", self.inertial3D, 10)
        SimBase.AddModelToTask("inertial3DPointTask", self.trackingError, 9)

        SimBase.AddModelToTask("hillPointTask", self.hillPoint, 10)
        SimBase.AddModelToTask("hillPointTask", self.trackingError, 9)

        SimBase.AddModelToTask("mtb_mom_management_task", self.tam_com_module, 7)
        SimBase.AddModelToTask("mtb_mom_management_task", self.mtb_momentum_management_module, 6)
        
        SimBase.AddModelToTask("mtb_detubmle_task", self.tam_com_module, 8)
        SimBase.AddModelToTask("mtb_detubmle_task", self.detumble_module, 6)
        
        SimBase.AddModelToTask("sunSafePointTask", self.cssWlsEst, 10)
        SimBase.AddModelToTask("sunSafePointTask", self.sunSafePoint, 9)

        SimBase.AddModelToTask("velocityPointTask", self.velocityPoint, 10)
        SimBase.AddModelToTask("velocityPointTask", self.trackingError, 9)

        SimBase.AddModelToTask("mrpSteeringRWsTask", self.mrpSteering, 10)
        SimBase.AddModelToTask("mrpSteeringRWsTask", self.rateServo, 9)
        SimBase.AddModelToTask("mrpSteeringRWsTask", self.rwMotorTorque, 8)

        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.mrpFeedbackRWs, 5)
        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.rwMotorTorque, 4)

        SimBase.fswProc.disableAllTasks()

        SimBase.createNewEvent("initiateStandby", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'standby'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.setAllButCurrentEventActivity('initiateStandby', True)"
                                ])

        SimBase.createNewEvent("initiateAttitudeGuidance", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'inertial3D'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('inertial3DPointTask')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.setAllButCurrentEventActivity('initiateAttitudeGuidance', True)"
                                ])


        SimBase.createNewEvent("initiateHillPoint", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'hillPoint'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('hillPointTask')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.setAllButCurrentEventActivity('initiateHillPoint', True)"
                                ])

        SimBase.createNewEvent("initiateSunSafePoint", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'sunSafePoint'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('sunSafePointTask')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.setAllButCurrentEventActivity('initiateSunSafePoint', True)"
                                ])

        SimBase.createNewEvent("initiateVelocityPoint", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'velocityPoint'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('velocityPointTask')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.setAllButCurrentEventActivity('initiateVelocityPoint', True)"])

        SimBase.createNewEvent("init_mtb_momentum_management", self.processTasksTimeStep, True,
                                ["self.modeRequest == 'mtb_momentum_management'"],
                                ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('inertial3DPointTask')",
                                "self.enableTask('mtb_mom_management_task')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.setAllButCurrentEventActivity('init_mtb_momentum_management', True)"])
        
        SimBase.createNewEvent("init_mtb_detumble", self.processTasksTimeStep, True,
                                ["self.modeRequest == 'mtb_detumble'"],
                                ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('mtb_detubmle_task')",
                                "self.setAllButCurrentEventActivity('init_mtb_detumble', True)"])

    def SetInertial3DPointGuidance(self):
        self.inertial3D.sigma_R0N = [0.2, 0.4, 0.6]
        messaging.AttRefMsg_C_addAuthor(self.inertial3D.attRefOutMsg, self.attRefMsg)

    def SetHillPointGuidance(self, SimBase):
        self.hillPoint.transNavInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.transOutMsg)
        self.hillPoint.celBodyInMsg.subscribeTo(SimBase.DynModels.EarthEphemObject.ephemOutMsgs[0])  # earth
        messaging.AttRefMsg_C_addAuthor(self.hillPoint.attRefOutMsg, self.attRefMsg)

    def SetSunSafePointGuidance(self, SimBase):
        self.sunSafePoint.imuInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.attOutMsg)
        self.sunSafePoint.sunDirectionInMsg.subscribeTo(self.cssWlsEst.navStateOutMsg)
        self.sunSafePoint.sHatBdyCmd = [0.0, 0.0, 1.0]
        messaging.AttGuidMsg_C_addAuthor(self.sunSafePoint.attGuidanceOutMsg, self.attGuidMsg)

    def SetVelocityPointGuidance(self, SimBase):
        self.velocityPoint.transNavInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.transOutMsg)
        self.velocityPoint.celBodyInMsg.subscribeTo(SimBase.DynModels.EarthEphemObject.ephemOutMsgs[0])
        self.velocityPoint.mu = SimBase.DynModels.gravFactory.gravBodies['earth'].mu
        messaging.AttRefMsg_C_addAuthor(self.velocityPoint.attRefOutMsg, self.attRefMsg)

    def SetAttitudeTrackingError(self, SimBase):
        self.trackingError.attNavInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.attOutMsg)
        self.trackingError.attRefInMsg.subscribeTo(self.attRefMsg)
        messaging.AttGuidMsg_C_addAuthor(self.trackingError.attGuidOutMsg, self.attGuidMsg)
        
    def setTAMComm(self, SimBase):
        self.tam_com_module.dcm_BS = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        self.tam_com_module.tamInMsg.subscribeTo(SimBase.DynModels.tam_module.tamDataOutMsg)

    def setMagneticDetumble(self, SimBase):
        self.detumble_module.P = 1E6
        self.detumble_module.navAttInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.attOutMsg)
        self.detumble_module.tamSensorInMsg.subscribeTo(self.tam_com_module.tamOutMsg)
        self.detumble_module.mtbConfigInMsg.subscribeTo(SimBase.DynModels.mtbParamsInMsg)

        messaging.MTBCmdMsg_C_addAuthor(self.detumble_module.cmdTorqueOutMsg, self.mtbCmdMsg)

        
    def setMtbMomManagement(self, SimBase):
        self.mtb_momentum_management_module.wheelSpeedBiases = [800. * mc.rpm2radsec, 600. * mc.rpm2radsec,
                                                        400. * mc.rpm2radsec, 200. * mc.rpm2radsec]
        self.mtb_momentum_management_module.cGain = 0.003
        self.mtb_momentum_management_module.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mtb_momentum_management_module.mtbParamsInMsg.subscribeTo(SimBase.DynModels.mtbParamsInMsg)
        self.mtb_momentum_management_module.tamSensorBodyInMsg.subscribeTo(self.tam_com_module.tamOutMsg)
        self.mtb_momentum_management_module.rwSpeedsInMsg.subscribeTo(SimBase.DynModels.rwStateEffector.rwSpeedOutMsg)
        self.mtb_momentum_management_module.rwMotorTorqueInMsg.subscribeTo(self.cmdRwMotorMsg)
        
        messaging.MTBCmdMsg_C_addAuthor(self.mtb_momentum_management_module.mtbCmdOutMsg, self.mtbCmdMsg)


    def SetCSSWlsEst(self, SimBase):
        cssConfig = messaging.CSSConfigMsgPayload()
        totalCSSList = []
        nHat_B_vec = [
            [0.0, 0.707107, 0.707107],
            [0.707107, 0., 0.707107],
            [0.0, -0.707107, 0.707107],
            [-0.707107, 0., 0.707107],
            [0.0, -0.965926, -0.258819],
            [-0.707107, -0.353553, -0.612372],
            [0., 0.258819, -0.965926],
            [0.707107, -0.353553, -0.612372]
        ]
        for CSSHat in nHat_B_vec:
            CSSConfigElement = messaging.CSSUnitConfigMsgPayload()
            CSSConfigElement.CBias = 1.0
            CSSConfigElement.nHat_B = CSSHat
            totalCSSList.append(CSSConfigElement)
        cssConfig.cssVals = totalCSSList

        cssConfig.nCSS = len(nHat_B_vec)
        self.cssConfigMsg = messaging.CSSConfigMsg().write(cssConfig)

        self.cssWlsEst.cssDataInMsg.subscribeTo(SimBase.DynModels.CSSConstellationObject.constellationOutMsg)
        self.cssWlsEst.cssConfigInMsg.subscribeTo(self.cssConfigMsg)

    def SetMRPFeedbackControl(self, SimBase):
        self.mrpFeedbackControl.guidInMsg.subscribeTo(self.attGuidMsg)
        self.mrpFeedbackControl.vehConfigInMsg.subscribeTo(self.vcMsg)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.mrpFeedbackControl.cmdTorqueOutMsg, self.cmdTorqueDirectMsg)

        self.mrpFeedbackControl.K = 3.5
        self.mrpFeedbackControl.Ki = -1.0  # Note: make value negative to turn off integral feedback
        self.mrpFeedbackControl.P = 30.0
        self.mrpFeedbackControl.integralLimit = 2. / self.mrpFeedbackControl.Ki * 0.1

    def SetMRPFeedbackRWA(self, SimBase):
        self.mrpFeedbackRWs.Ki = -1  # make value negative to turn off integral feedback
        self.mrpFeedbackRWs.K = 0.0001
        self.mrpFeedbackRWs.P = 0.002
        self.mrpFeedbackRWs.integralLimit = 2. / self.mrpFeedbackRWs.Ki * 0.1

        self.mrpFeedbackRWs.vehConfigInMsg.subscribeTo(self.vcMsg)
        self.mrpFeedbackRWs.rwSpeedsInMsg.subscribeTo(SimBase.DynModels.rwStateEffector.rwSpeedOutMsg)
        self.mrpFeedbackRWs.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mrpFeedbackRWs.guidInMsg.subscribeTo(self.attGuidMsg)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.mrpFeedbackRWs.cmdTorqueOutMsg, self.cmdTorqueMsg)

    def SetMRPSteering(self):
        self.mrpSteering.K1 = 0.05
        self.mrpSteering.ignoreOuterLoopFeedforward = False
        self.mrpSteering.K3 = 0.75
        self.mrpSteering.omega_max = 1.0 * mc.D2R
        self.mrpSteering.guidInMsg.subscribeTo(self.attGuidMsg)

    def SetRateServo(self, SimBase):
        self.rateServo.guidInMsg.subscribeTo(self.attGuidMsg)
        self.rateServo.vehConfigInMsg.subscribeTo(self.vcMsg)
        self.rateServo.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.rateServo.rwSpeedsInMsg.subscribeTo(SimBase.DynModels.rwStateEffector.rwSpeedOutMsg)
        self.rateServo.rateSteeringInMsg.subscribeTo(self.mrpSteering.rateCmdOutMsg)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.rateServo.cmdTorqueOutMsg, self.cmdTorqueMsg)

        self.rateServo.Ki = 5.0
        self.rateServo.P = 150.0
        self.rateServo.integralLimit = 2. / self.rateServo.Ki * 0.1
        self.rateServo.knownTorquePntB_B = [0., 0., 0.]

    def SetVehicleConfiguration(self):
        vehicleConfigOut = messaging.VehicleConfigMsgPayload()
        vehicleConfigOut.ISCPntB_B = [0.02 / 3, 0., 0.,
                                     0., 0.1256 / 3, 0.,
                                     0., 0., 0.1256 / 3]
        self.vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    def SetRWConfigMsg(self, SimBase):
        self.fswRwConfigMsg = SimBase.DynModels.rwFactory.getConfigMessage()

    def SetRWMotorTorque(self):
        controlAxes_B = [
            1.0, 0.0, 0.0
            , 0.0, 1.0, 0.0
            , 0.0, 0.0, 1.0
        ]
        self.rwMotorTorque.controlAxes_B = controlAxes_B
        self.rwMotorTorque.vehControlInMsg.subscribeTo(self.cmdTorqueMsg)
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.rwMotorTorque.rwMotorTorqueOutMsg, self.cmdRwMotorMsg)
        self.rwMotorTorque.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)

    # Global call to initialize every module
    def InitAllFSWObjects(self, SimBase):
        self.SetVehicleConfiguration()
        self.SetRWConfigMsg(SimBase)
        self.SetInertial3DPointGuidance()
        self.SetHillPointGuidance(SimBase)
        self.SetCSSWlsEst(SimBase)
        self.SetSunSafePointGuidance(SimBase)
        self.SetVelocityPointGuidance(SimBase)
        self.SetAttitudeTrackingError(SimBase)
        self.SetMRPFeedbackControl(SimBase)
        self.SetMRPFeedbackRWA(SimBase)
        self.SetMRPSteering()
        self.SetRateServo(SimBase)
        self.SetRWMotorTorque()
        
        self.setTAMComm(SimBase)
        self.setMtbMomManagement(SimBase)
        self.setMagneticDetumble(SimBase)

    def setupGatewayMsgs(self, SimBase):
        self.cmdTorqueMsg = messaging.CmdTorqueBodyMsg_C()
        self.cmdTorqueDirectMsg = messaging.CmdTorqueBodyMsg_C()
        self.attRefMsg = messaging.AttRefMsg_C()
        self.attGuidMsg = messaging.AttGuidMsg_C()
        self.cmdRwMotorMsg = messaging.ArrayMotorTorqueMsg_C()
        self.mtbCmdMsg = messaging.MTBCmdMsg_C()

        self.zeroGateWayMsgs()

        # connect gateway FSW effector command msgs with the dynamics
        SimBase.DynModels.extForceTorqueObject.cmdTorqueInMsg.subscribeTo(self.cmdTorqueDirectMsg)
        SimBase.DynModels.rwStateEffector.rwMotorCmdInMsg.subscribeTo(self.cmdRwMotorMsg)
        SimBase.DynModels.mtb_effector.mtbCmdInMsg.subscribeTo(self.mtbCmdMsg)


    def zeroGateWayMsgs(self):
        self.cmdTorqueMsg.write(messaging.CmdTorqueBodyMsgPayload())
        self.cmdTorqueDirectMsg.write(messaging.CmdTorqueBodyMsgPayload())
        self.attRefMsg.write(messaging.AttRefMsgPayload())
        self.attGuidMsg.write(messaging.AttGuidMsgPayload())
        self.cmdRwMotorMsg.write(messaging.ArrayMotorTorqueMsgPayload())
        self.mtbCmdMsg.write(messaging.MTBCmdMsgPayload())
