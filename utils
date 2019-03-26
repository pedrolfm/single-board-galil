import numpy

class Target:
    def __init__(self):
        self.HT_RAS_zFrame = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0 ; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')
        self.HT_RAS_Target = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0 ; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')
        self.HT_zFrame_Target = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0 ; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        # Transformation between Z frame and Robot - defined from the robot design
        self.HT_rz = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0 ; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')
        self.ready = False


    def getTargetRAS(self):
        return numpy.array([self.x , self.y, self.z])

    def setTargetRAS(self,pos):
        self.HT_RAS_Target = pos

    def defineTargetRobot(self,zFrameMatrix):
        self.HT_zRAS = zFrameMatrix
        RASTarget = numpy.matrix('0.0; 0.0 ; 0.0 ; 1.0')
        RobotTarget  = numpy.matrix('0.0; 0.0 ; 0.0 ; 1.0')

        # TODO: Add the Transformation between Z frame and Robot - defined from the robot design
        inv_HT_RAS_zFrame = numpy.linalg.inv(self.HT_RAS_zFrame)
        self.HT_zFrame_Target = inv_HT_RAS_zFrame*self.HT_RAS_Target
        self.x = self.HT_zFrame_Target[0,0]
        self.y = self.HT_zFrame_Target[1,0]
        self.z = self.HT_zFrame_Target[2,0]
        if numpy.linalg.det(self.HT_zFrame_Target[0:3,0:3]) == 1.0:
            self.ready = True
            return 1
        else:
            return 0





