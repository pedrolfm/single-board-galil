import numpy

D1 = 13.52
D2 = 44.50

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
        self.HT_RAS_Target_angle = pos
        self.HT_RAS_Target[0,3] = pos[0,3] 
        self.HT_RAS_Target[1,3] = pos[1,3]
        self.HT_RAS_Target[2,3] = pos[2,3]
    def defineTargetRobot(self,zFrameMatrix):
        self.HT_RAS_zFrame = zFrameMatrix
        self.getInsertionAngle()
        print(self.phi)
        print(self.teta)
        print(self.gamma)
        self.definePositionPiezo()
        self.HT_zRAS = zFrameMatrix

        # TODO: Add the Transformation between Z frame and Robot - defined from the robot design
        inv_HT_RAS_zFrame = numpy.linalg.inv(self.HT_RAS_zFrame)
        self.HT_zFrame_Target = inv_HT_RAS_zFrame*self.HT_RAS_Target
        print(inv_HT_RAS_zFrame)
        print("em cima ta o inversion")
        print(self.HT_RAS_zFrame)
        print("em cima ta o normal")
        #Compensate the bias due to the piezo motion:
        self.x = self.HT_zFrame_Target[0,3] + self.piezo[0]
        self.y = self.HT_zFrame_Target[1,3] + self.piezo[1]
        self.z = self.HT_zFrame_Target[2,3]
        print(self.HT_zFrame_Target)
        if numpy.linalg.det(self.HT_zFrame_Target[0:3,0:3]) == 1.0:
            self.ready = True
            return 1
        else:
            return 0


    def getInsertionAngle(self):

        self.teta = numpy.arctan2(-self.HT_RAS_Target_angle[2,0],numpy.sqrt(self.HT_RAS_Target_angle[0,0]*self.HT_RAS_Target_angle[0,0]+self.HT_RAS_Target_angle[1,0]*self.HT_RAS_Target_angle[1,0]))
        self.gamma = numpy.arctan2(self.HT_RAS_Target_angle[1,0]/numpy.cos(self.teta),self.HT_RAS_Target_angle[0,0]/numpy.cos(self.teta))
        self.phi = numpy.arctan2(self.HT_RAS_Target_angle[2,1]/numpy.cos(self.teta),self.HT_RAS_Target_angle[2,2]/numpy.cos(self.teta))

        return


    def definePositionPiezo(self):
        self.getInsertionAngle()
        print(self.HT_RAS_Target_angle)
        print("teste")
        #We need to check again the orientation of the movement.
        self.piezo = numpy.array([D1*numpy.tan(self.teta) , D1 * numpy.tan(self.phi)])
        print(self.piezo)
        #Caclulate the bias:
        D3 = D2 - numpy.sqrt(self.piezo[0]*self.piezo[0]+self.piezo[1]*self.piezo[1]+D1*D1)
        self.piezo_bias = numpy.array([D3*numpy.sin(self.gamma) + self.piezo[0] , D3*numpy.sin(self.teta)])
        self.piezo_bias = numpy.array([D3*numpy.sin(self.teta) + self.piezo[0] , D3*numpy.sin(self.phi)])
        #TODO CHECK BIAS
