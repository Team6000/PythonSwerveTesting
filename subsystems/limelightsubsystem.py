from wpilib import Timer
from commands2 import Subsystem
from ntcore import NetworkTableInstance

class LimelightSubsystem(Subsystem):
    def __init__(self,limelight_name="limelight"):
        super().__init__()

        instance = NetworkTableInstance.getDefault()
        self.table = instance.getTable(limelight_name)

        self.pipelineIndexRequest = self.table.getDoubleTopic("pipeline").publish()
        self.pipelineIndex = self.table.getDoubleTopic("getpipe").getEntry(-1)

        self.ledMode = self.table.getIntegerTopic("ledMode").getEntry(-1)
        self.camMode = self.table.getIntegerTopic("camMode").getEntry(-1)
        self.tx = self.table.getDoubleTopic("tx").getEntry(0.0)
        self.ty = self.table.getDoubleTopic("ty").getEntry(0.0)
        self.ta = self.table.getDoubleTopic("ta").getEntry(0.0)
        self.botpose_orb_wpiblue = self.table.getDoubleArrayTopic("botpose_orb_wpiblue").getEntry([])
        self.botpose_tagcount = self.table.getDoubleTopic("botpose_tagcount").getEntry(0)

        self.hb = self.table.getIntegerTopic("hb").getEntry(0)
        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False

    def setPipeline(self, index: int):
        self.pipelineIndexRequest.set(float(index))

    def getPipeline(self) -> int:
        return int(self.pipelineIndex.get(-1))

    def getA(self) -> float:
        return self.ta.get()

    def getX(self) -> float:
        return self.tx.get()

    def getY(self) -> float:
        return self.ty.get()

    def getHB(self) -> float:
        return self.hb.get()

    def hasDetection(self) -> bool:
        if self.getX() != 0.0 and self.heartbeating:
            return True
        else:
            return False

    def getSecondsSinceLastHeartbeat(self) -> float:
        return Timer.getFPGATimestamp() - self.lastHeartbeatTime

    def getBotPose(self):
        return self.botpose_orb_wpiblue.get()

    def getBotPoseTagCount(self):
        return int(self.botpose_tagcount.get())

    def periodic(self) -> None:
        now = Timer.getFPGATimestamp()
        heartbeat = self.getHB()
        if heartbeat != self.lastHeartbeat:
            self.lastHeartbeat = heartbeat
            self.lastHeartbeatTime = now

        heartbeating = now < self.lastHeartbeatTime + 5 # no heartbeat for 5s => stale camera
        if heartbeating != self.heartbeating:
            if heartbeating:
                print("Limelight is UPDATING")
            else:
                print("Limelight is NO LONGER UPDATING")
        self.heartbeating = heartbeating
