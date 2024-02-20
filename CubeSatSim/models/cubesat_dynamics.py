import numpy as np
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.simulation import ephemerisConverter
from Basilisk.simulation import (spacecraft, extForceTorque, simpleNav, eclipse,
                                 reactionWheelStateEffector, coarseSunSensor,
                                 magneticFieldWMM, magnetometer, MtbEffector)
from Basilisk.simulation import thrusterDynamicEffector
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros as mc
from Basilisk.utilities import simIncludeRW, simIncludeGravBody
from Basilisk.utilities import simIncludeThruster
from Basilisk.utilities import unitTestSupport as sp

bskPath = __path__[0]


class CubeSat_dynamics():
    def __init__(self, SimBase, dynRate):
        self.sun = None
        self.earth = None
        self.moon = None
        self.epochMsg = None
        self.RW1 = None
        self.RW2 = None
        self.RW3 = None
        self.RW4 = None

        self.processName = SimBase.DynamicsProcessName
        self.taskName = "DynamicsTask"
        self.processTasksTimeStep = mc.sec2nano(dynRate)

        SimBase.dynProc.addTask(SimBase.CreateNewTask(self.taskName, self.processTasksTimeStep))

        self.scObject = spacecraft.Spacecraft()
        self.gravFactory = simIncludeGravBody.gravBodyFactory()
        self.rwFactory = simIncludeRW.rwFactory()
        self.extForceTorqueObject = extForceTorque.ExtForceTorque()
        self.simpleNavObject = simpleNav.SimpleNav()
        self.eclipseObject = eclipse.Eclipse()
        self.CSSConstellationObject = coarseSunSensor.CSSConstellation()
        self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
        self.thrustersDynamicEffector = thrusterDynamicEffector.ThrusterDynamicEffector()
        self.EarthEphemObject = ephemerisConverter.EphemerisConverter()
        
        self.mag_field_module = magneticFieldWMM.MagneticFieldWMM()
        self.tam_module = magnetometer.Magnetometer()
        self.mtb_effector = MtbEffector.MtbEffector()

        self.InitAllDynObjects()

        SimBase.AddModelToTask(self.taskName, self.scObject, 201)
        SimBase.AddModelToTask(self.taskName, self.simpleNavObject, 109)
        SimBase.AddModelToTask(self.taskName, self.gravFactory.spiceObject, 200)
        SimBase.AddModelToTask(self.taskName, self.EarthEphemObject, 198)
        SimBase.AddModelToTask(self.taskName, self.CSSConstellationObject, 108)
        SimBase.AddModelToTask(self.taskName, self.eclipseObject, 204)
        SimBase.AddModelToTask(self.taskName, self.rwStateEffector, 301)
        SimBase.AddModelToTask(self.taskName, self.extForceTorqueObject, 300)
        
        SimBase.AddModelToTask(self.taskName, self.mag_field_module, 199)
        SimBase.AddModelToTask(self.taskName, self.tam_module, 210)
        SimBase.AddModelToTask(self.taskName, self.mtb_effector, 213)

    def setSpacecraftHub(self):
        self.scObject.ModelTag = "bskSat"
        # -- Crate a new variable for the sim sc inertia I_sc. Note: this is currently accessed from FSWClass
        self.I_sc = [0.02 / 3, 0., 0.,
                     0., 0.1256 / 3, 0.,
                     0., 0., 0.1256 / 3]
        self.scObject.hub.mHub = 10.0  # kg - spacecraft mass
        self.scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
        self.scObject.hub.IHubPntBc_B = sp.np2EigenMatrix3d(self.I_sc)

    def setGravityBodies(self):
        timeInitString = "2024 FEB 28 00:00:00.0"
        gravBodies = self.gravFactory.createBodies(['sun', 'earth', 'moon'])
        gravBodies['earth'].isCentralBody = True
        self.sun = 0
        self.earth = 1
        self.moon = 2

        self.gravFactory.addBodiesTo(self.scObject)
        self.gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                              timeInitString,
                                              epochInMsg=True)
        self.epochMsg = self.gravFactory.epochMsg

        self.gravFactory.spiceObject.zeroBase = 'Earth'

        self.EarthEphemObject.addSpiceInputMsg(self.gravFactory.spiceObject.planetStateOutMsgs[self.earth])

    def setMagneticField(self):
        self.mag_field_module.ModelTag = "WMM"
        self.mag_field_module.dataPath = bskPath + '/supportData/MagneticField/'
        epochMsg = sp.timeStringToGregorianUTCMsg('2019 June 27, 10:23:0.0 (UTC)')
        self.mag_field_module.epochInMsg.subscribeTo(epochMsg)
        self.mag_field_module.addSpacecraftToModel(self.scObject.scStateOutMsg) 
        
    def setEclipseObject(self):
        self.eclipseObject.ModelTag = "eclipseObject"
        self.eclipseObject.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
        # add all celestial objects in spiceObjects except for the sun (0th object)
        for c in range(1, len(self.gravFactory.spiceObject.planetStateOutMsgs)):
            self.eclipseObject.addPlanetToModel(self.gravFactory.spiceObject.planetStateOutMsgs[c])
        self.eclipseObject.addSpacecraftToModel(self.scObject.scStateOutMsg)

    def setExternalForceTorqueObject(self):
        self.extForceTorqueObject.ModelTag = "externalDisturbance"
        self.scObject.addDynamicEffector(self.extForceTorqueObject)

    def setSimpleNavObject(self):
        self.simpleNavObject.ModelTag = "SimpleNavigation"
        self.simpleNavObject.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)

    def setMagnetoTorqueBar(self):
        self.tam_module.ModelTag = "TAM_sensor"
        self.tam_module.scaleFactor = 1.0
        self.tam_module.senNoiseStd = [0.0,  0.0, 0.0]
        self.tam_module.stateInMsg.subscribeTo(self.scObject.scStateOutMsg)
        self.tam_module.magInMsg.subscribeTo(self.mag_field_module.envOutMsgs[0])
 
    def setMtbConfig(self):
        # mtbConfigData message
        self.mtbConfigParams = messaging.MTBArrayConfigMsgPayload()
        self.mtbConfigParams.numMTB = 4

        # row major toque bar alignments
        self.mtbConfigParams.GtMatrix_B =[
            1., 0., 0., 0.70710678,
            0., 1., 0., 0.70710678,
            0., 0., 1., 0.]
        maxDipole = 0.2
        self.mtbConfigParams.maxMtbDipoles = [maxDipole]*self.mtbConfigParams.numMTB
        self.mtbParamsInMsg = messaging.MTBArrayConfigMsg().write(self.mtbConfigParams)
        
    def setMtbEffector(self):
        self.mtb_effector.ModelTag = "MtbEff"
        self.mtb_effector.mtbParamsInMsg.subscribeTo(self.mtbParamsInMsg)
        self.mtb_effector.magInMsg.subscribeTo(self.mag_field_module.envOutMsgs[0])
        
        self.scObject.addDynamicEffector(self.mtb_effector)
        
    def setReactionWheelDynEffector(self):
        rwPosVector = [[0.1, 0.1, 0.22],
                       [0.1, -0.1, 0.22],
                       [-0.1, -0.1, 0.22],
                       [-0.1, 0.1, 0.22]
                       ]
        
        beta = 52. * np.pi / 180.
        Gs = np.array([
                [0.,            0.,             np.cos(beta), -np.cos(beta)],
                [np.cos(beta),  np.sin(beta),  -np.sin(beta), -np.sin(beta)],
                [np.sin(beta), -np.cos(beta),   0.,             0.]])

        # create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments
        self.RW1 = self.rwFactory.create('BCT_RWP015', 
                                    Gs[:, 0], 
                                    Omega_max=5000.,
                                    useRWfriction=False
                               )
        self.RW2 = self.rwFactory.create('BCT_RWP015', 
                                    Gs[:, 1], 
                                    Omega_max=5000., 
                                    useRWfriction=False
                               )
        self.RW3 = self.rwFactory.create('BCT_RWP015', 
                                    Gs[:, 2], 
                                    Omega_max=5000.,
                                    useRWfriction=False
                               )

        self.RW4 = self.rwFactory.create('BCT_RWP015', 
                                    Gs[:, 3], 
                                    Omega_max=5000., 
                                    useRWfriction=False
                               )
        self.rwFactory.addToSpacecraft("RWA", self.rwStateEffector, self.scObject)


    def setCSSConstellation(self):
        self.CSSConstellationObject.ModelTag = "cssConstellation"

        def setupCSS(cssDevice):
            cssDevice.fov = 80. * mc.D2R         # half-angle field of view value
            cssDevice.scaleFactor = 2.0
            cssDevice.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
            cssDevice.stateInMsg.subscribeTo(self.scObject.scStateOutMsg)
            cssDevice.sunEclipseInMsg.subscribeTo(self.eclipseObject.eclipseOutMsgs[0])
            cssDevice.this.disown()

        # setup CSS sensor normal vectors in body frame components
        nHat_B_List = [
            [0.0, 0.707107, 0.707107],
            [0.707107, 0., 0.707107],
            [0.0, -0.707107, 0.707107],
            [-0.707107, 0., 0.707107],
            [0.0, -0.965926, -0.258819],
            [-0.707107, -0.353553, -0.612372],
            [0., 0.258819, -0.965926],
            [0.707107, -0.353553, -0.612372]
        ]
        numCSS = len(nHat_B_List)

        # store all
        cssList = []
        for nHat_B, i in zip(nHat_B_List, list(range(1,numCSS+1))):
            CSS = coarseSunSensor.CoarseSunSensor()
            setupCSS(CSS)
            CSS.ModelTag = "CSS" + str(i)
            CSS.nHat_B = np.array(nHat_B)
            cssList.append(CSS)

        # assign the list of CSS devices to the CSS array class
        self.CSSConstellationObject.sensorList = coarseSunSensor.CSSVector(cssList)


    def InitAllDynObjects(self):
        self.setMtbConfig()
        
        self.setSpacecraftHub()
        self.setGravityBodies()
        self.setExternalForceTorqueObject()
        self.setSimpleNavObject()
        self.setEclipseObject()
        self.setCSSConstellation()
        self.setMagneticField()

        self.setReactionWheelDynEffector()
        self.setMtbEffector()
        self.setMagnetoTorqueBar()

