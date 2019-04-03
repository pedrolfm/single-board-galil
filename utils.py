import numpy

D1 = 13.52

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

        # TODO: Add the Transformation between Z frame and Robot - defined from the robot design
        inv_HT_RAS_zFrame = numpy.linalg.inv(self.HT_RAS_zFrame)
        self.HT_zFrame_Target = inv_HT_RAS_zFrame*self.HT_RAS_Target

        #Compensate the bias due to the piezo motion:
        self.x = self.HT_zFrame_Target[0,3] + self.piezo[0]
        self.y = self.HT_zFrame_Target[1,3] + self.piezo[1]
        self.z = self.HT_zFrame_Target[2,3]
        if numpy.linalg.det(self.HT_zFrame_Target[0:3,0:3]) == 1.0:
            self.ready = True
            return 1
        else:
            return 0


    def getInsertionAngle(self):

        self.teta = numpy.atan2(-self.HT_RAS_Target[2,0],numpy.sqrt(self.HT_RAS_Target[0,0]^2+self.HT_RAS_Target[1,0]^2))
        self.gamma = numpy.atan2(self.HT_RAS_Target[1,0]/cos(teta),self.HT_RAS_Target[0,0]/cos(teta))
        self.phi = numpy.atan2(self.HT_RAS_Target[2,1]/cos(teta),self.HT_RAS_Target[2,2]/cos(teta))

        return #numpy.array([teta , gamma, phi])


    def definePositionPiezo(self):
        self.getInsertionAngle()

        #We need to check again the orientation of the movement.
        self.piezo = numpy.array([D1*numpy.tan(self.gamma) , D1 * numpy.tan(self.teta)])

        #Caclulate the bias:
        D3 = D2 - numpy.sqrt(self.piezo[0]^2+self.piezo[1]^2+D1^2)
        self.piezo_bias = numpy.array([D3*numpy.sin(self.gamma) + self.piezo[0] , D3*numpy.sin(self.teta)])








