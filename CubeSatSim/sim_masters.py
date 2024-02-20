import inspect
import os
import sys
from Basilisk.utilities import SimulationBaseClass

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/models')


class Sim(SimulationBaseClass.SimBaseClass):
    def __init__(self, fswRate=0.1, dynRate=0.05):
        self.dynRate = dynRate
        self.fswRate = fswRate
        # Create a sim module as an empty container
        SimulationBaseClass.SimBaseClass.__init__(self)

        self.DynModels = []
        self.FSWModels = []
        self.DynamicsProcessName = None
        self.FSWProcessName = None
        self.dynProc = None
        self.fswProc = None
        
        self.oneTimeRWFaultFlag = 0
        self.oneTimeFaultTime = -1
        self.repeatRWFaultFlag = 0

        self.dynamics_added = False
        self.fsw_added = False

    def get_DynModel(self):
        assert (self.dynamics_added is True), "It is mandatory to use a dynamics model as an argument"
        return self.DynModels

    def set_DynModel(self, dynModel):
        self.dynamics_added = True
        self.DynamicsProcessName = 'DynamicsProcess'  # Create simulation process name
        self.dynProc = self.CreateNewProcess(self.DynamicsProcessName)  # Create process
        self.DynModels = dynModel.CubeSat_dynamics(self, self.dynRate)  # Create Dynamics and FSW classes

    def get_FswModel(self):
        assert (self.fsw_added is True), "A flight software model has not been added yet"
        return self.FSWModels

    def set_FswModel(self, fswModel):
        self.fsw_added = True
        self.FSWProcessName = "FSWProcess"  # Create simulation process name
        self.fswProc = self.CreateNewProcess(self.FSWProcessName)  # Create process
        self.FSWModels = fswModel.CubeSat_fsw(self, self.fswRate)  # Create Dynamics and FSW classes


class Scenario(object):
    def __init__(self):
        self.name = "scenario"

    def configure_initial_conditions(self):
        """
            Developer must override this method
        """
        pass

    def log_outputs(self):
        """
            Developer must override this method
        """
        pass

    def pull_outputs(self):
        """
            Developer must override this method
        """
        pass
