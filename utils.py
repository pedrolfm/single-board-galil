import numpy

D1 = 13.52
D2 = 44.50

class Target:
    def __init__(self):
        self.ht_zframe_base = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0 ; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0') #TODO: Input numbers
        self.ht_RAS_zframe = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0 ; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')
        self.ht_RAS_target = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0 ; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')
        self.ht_zframe_target = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0 ; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')
        self.ht_RAS_target_angle = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0 ; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.ready = False


    def get_target_RAS(self):
        return numpy.array([self.x , self.y, self.z])

    def set_target_RAS(self,pos):
        self.ht_RAS_target = pos

    def set_target_RAS_angle(self,ang):
        self.ht_RAS_target_angle = ang

    def define_target_robot(self,zframe_matrix):
        rotation = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 0.0 1.0 0.0 ; 0.0 -1.0 0.0 0.0; 0.0 0.0 0.0 1.0')
        translation = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 50.0 ; 0.0 0.0 1.0 -100.0; 0.0 0.0 0.0 1.0')
        
        self.ht_RAS_zFrame = zframe_matrix #TODO: Name convention
        self.get_insertion_angle()
        self.define_position_piezo()
        print(translation*self.ht_RAS_zframe*rotation)
        inv_ht_RAS_zFrame = numpy.linalg.inv(translation*self.ht_RAS_zframe*rotation)
        self.ht_zframe_target = inv_ht_RAS_zFrame*self.ht_RAS_target
        print('target location')
        print(self.ht_zframe_target)
        #Compensate the bias due to the piezo motion and angulation:

        diff_horizontal = self.ht_zframe_target[2,3]*numpy.tan(self.phi)
        diff_vertical = self.ht_zframe_target[2, 3] * numpy.tan(self.teta)
        print("Difference: %f, %f." % (diff_horizontal, diff_vertical))

        self.x = self.ht_zframe_target[0,3] + self.piezo[1] + diff_horizontal
        self.y = self.ht_zframe_target[1,3] + self.piezo[0] + diff_vertical
        self.z = self.ht_zframe_target[2,3]

        return 1



    def get_insertion_angle(self):

        self.teta = numpy.arctan2(-self.ht_RAS_target_angle[2,0],numpy.sqrt(self.ht_RAS_target_angle[0,0]*self.ht_RAS_target_angle[0,0]+self.ht_RAS_target_angle[1,0]*self.ht_RAS_target_angle[1,0]))
        self.gamma = numpy.arctan2(self.ht_RAS_target_angle[1,0]/numpy.cos(self.teta),self.ht_RAS_target_angle[0,0]/numpy.cos(self.teta))
        self.phi = numpy.arctan2(self.ht_RAS_target_angle[2,1]/numpy.cos(self.teta),self.ht_RAS_target_angle[2,2]/numpy.cos(self.teta))

        return


    def define_position_piezo(self):
        self.get_insertion_angle()


        self.piezo = numpy.array([D1*numpy.tan(self.teta) , D1 * numpy.tan(self.phi)])
        print(self.piezo)
        #Caclulate the bias:
        D3 = D2 - numpy.sqrt(self.piezo[0]*self.piezo[0]+self.piezo[1]*self.piezo[1]+D1*D1)
        self.piezo_bias = numpy.array([D3*numpy.sin(self.gamma) + self.piezo[0] , D3*numpy.sin(self.teta)])
        self.piezo_bias = numpy.array([D3*numpy.sin(self.teta) + self.piezo[0] , D3*numpy.sin(self.phi)])
        #TODO CHECK BIAS
