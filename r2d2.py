import pybullet
import time
import pybullet_data
import math
import numpy


def distance(centerPos1, centerPos2, r1, r2):
    p1 = numpy.array(centerPos1); p2 = numpy.array(centerPos2)
    sq_dist = numpy.sum((p1 - p2)**2, axis =0)
    dist = numpy.sqrt(sq_dist)
    return dist - r1 - r2




class GravityMovement:
    gravityPosition = [0,0,0]
    position = [0,0,0]
    mass = 0
    sphere = 0
    id = -1
    upp_limit = [5, 5, 7]
    down_limit = [-5, -5, -7]
    r_hor = 0
    gravity = 1
    def __init__(self, mass, sphere, id, position, gravity):
        self.position = position
        self.gravity = gravity
        self.gravityPosition = [x + self.gravity for x in position]
        self.mass = mass
        self.sphere = sphere
        self.id = id
        self.r_hor = self.gravity
        self.sphereUid = pybullet.createMultiBody(self.mass,
                                     self.sphere,
                                     self.id,
                                     self.position,
                                     )
    def setGrav(self, pos):
        self.gravityPosition = [x + self.gravity for x in pos]
    def getCenter(self):
        return self.position
    def setRoll(self):
        rollini = numpy.arctan2(self.position[2], self.r_hor)
        phiini = numpy.arctan2(self.position[1],self.position[0])
        thetaini = 0
        roll = -rollini
        phi = phiini - numpy.pi - numpy.pi/2
        theta = thetaini
        q = pybullet.getQuaternionFromEuler((roll,theta,phi))
        return q
    def rotate(self):
        pos, rot = pybullet.getBasePositionAndOrientation(self.sphereUid)
        qw = rot[3]
        if abs(qw) != 1:
            rot.angle = math.acos(qw)* 114.6
            value = math.sqrt(1 - qw * qw)
            rot.axis = tuple ([i / value for i in rot[:3]])


    def getGravityField(self):
            return self.gravity
    def moveNeutron(self):
        pass

    def moveUp(self):
        self.position = [self.position[0] - 0.005, self.position[1] + 0.002, self.position[2] + 0.005]
        self.setGrav(self.position)
        pybullet.resetBasePositionAndOrientation(self.sphereUid, self.position, self.setRoll())
    def moveCurvyUp(self):
        self.position = [self.position[0] + 0.005, self.position[1] - 0.005, self.position[2] + 0.004]
        self.setGrav(self.position)
        pybullet.resetBasePositionAndOrientation(self.sphereUid, self.position, self.setRoll())
    def moveDown(self):
        self.position = [self.position[0] + 0.006, self.position[1] + 0.00005, self.position[2] - 0.005]
        self.setGrav(self.position)
        pybullet.resetBasePositionAndOrientation(self.sphereUid, self.position, self.setRoll())
    def moveCurvyBack(self):
        self.position = [self.position[0] - 0.005, self.position[1] - 0.001, self.position[2] + 0.004]
        self.setGrav(self.position)
        pybullet.resetBasePositionAndOrientation(self.sphereUid, self.position, self.setRoll())
    def moveCurvyDown(self):
        self.position = [self.position[0] - 0.005, self.position[1] - 0.005, self.position[2] - 0.007]
        self.setGrav(self.position)
        pybullet.resetBasePositionAndOrientation(self.sphereUid, self.position, self.setRoll())

    def moveCurvyForward(self):
        self.position = [self.position[0] + 0.006, self.position[1] + 0.005, self.position[2] - 0.001]
        self.setGrav(self.position)
        pybullet.resetBasePositionAndOrientation(self.sphereUid, self.position, self.setRoll())
    def moveCurvyUpward(self):
        self.position = [self.position[0] - 0.002, self.position[1] - 0.0085, self.position[2] - 0.001]
        self.setGrav(self.position)
        pybullet.resetBasePositionAndOrientation(self.sphereUid, self.position, self.setRoll())
    def moveCurvyBackwards(self):
        self.position = [self.position[0] - 0.003, self.position[1] + 0.002, self.position[2] + 0.002]
        self.setGrav(self.position)
        pybullet.resetBasePositionAndOrientation(self.sphereUid, self.position, self.setRoll())
    def moveToDefault(self):
        if not math.fabs(self.position[0]) < 0.004:
            if self.position[0] < 0:
                self.position[0] = self.position[0] + 0.004
            elif self.position[0] > 0:
                self.position[0] = self.position[0] - 0.004
        else:
            self.position[0] = 0
        if not math.fabs(self.position[1]) < 0.008:
            if self.position[1] < 0:
                self.position[1] = self.position[1] + 0.008
            elif self.position[1] > 0:
                self.position[1] = self.position[1] - 0.008
        else:
            self.position[1] = 0
        if not math.fabs(self.position[2]) < 0.0012:
            if self.position[2] < 0:
                self.position[2] = self.position[2] + 0.0012
            elif self.position[2] > 0:
                self.position[2] = self.position[2] - 0.0012
        else:
            self.position[2] = 0
        self.setGrav(self.position)
        pybullet.resetBasePositionAndOrientation(self.sphereUid, self.position, self.setRoll())

    def moveX(self):
        self.position = [self.position[0] + 0.007, self.position[1] + 0.002, self.position[2] - 0.0001]
        self.setGrav(self.position)
        pybullet.resetBasePositionAndOrientation(self.sphereUid, self.position, self.setRoll())

    def movecurvyAround(self):
        self.position = [self.position[0] + 0.0031, self.position[1] - 0.006, self.position[2] - 0.002]
        self.setGrav(self.position)
        pybullet.resetBasePositionAndOrientation(self.sphereUid, self.position, self.setRoll())
    def moveY(self):
        self.position = [self.position[0] - 0.004, self.position[1], self.position[2] - 0.002]
        self.setGrav(self.position)
        pybullet.resetBasePositionAndOrientation(self.sphereUid, self.position, self.setRoll())






physicsClient = pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.resetDebugVisualizerCamera(cameraDistance = 2,
                                    cameraYaw = 10,
                                    cameraPitch = 5,
                                    cameraTargetPosition = [0, -4, 1])

sphere = pybullet.createCollisionShape(pybullet.GEOM_SPHERE, radius = 0.6)
sphere2 = pybullet.createCollisionShape(pybullet.GEOM_SPHERE, radius = 1)
sphere3 = pybullet.createCollisionShape(pybullet.GEOM_SPHERE, radius = 0.2)
spherePos1 = [2,-2,0.2]
spherePos2 = [-2.5,-1,2.5]
spherePos3 = [0,0,0]


mass = 10
visualShapeId = pybullet.createVisualShape(shapeType = pybullet.GEOM_SPHERE,
                                           rgbaColor = [1,0.47,0.97,1],
                                           radius = 0.2
                                              )
visualShapeId2 = pybullet.createVisualShape(shapeType = pybullet.GEOM_SPHERE,
                                           rgbaColor = [0.26,0.8,0.9,1],
                                           radius = 1
                                              )
visualShapeId3 = pybullet.createVisualShape(shapeType = pybullet.GEOM_SPHERE,
                                           rgbaColor = [1,0.7,0.1,1],
                                           radius = 0.5
                                              )

link_Masses = [1]
linkCollisionShapeIndices = [sphere2]
linkVisualShapeIndices = [-1]
linkPositions = [[0, 0, 0.11]]
linkOrientations = [[0, 0, 0, 1]]
linkInertialFramePositions = [[0, 0, 0]]
linkInertialFrameOrientations = [[0, 0, 0, 1]]
indices = [0]
jointTypes = [pybullet.JOINT_SPHERICAL]
axis = [[0, 0, 1]]

pybullet.setGravity(0,0,0)
cubeStartPos = [0,0,0.5]
cubeOrientation = pybullet.getQuaternionFromEuler([0,0,0])
planetGravity = 1; neutronGravity = 1.5; sunGravity = 1.3

#pybullet.changeDynamics(sphereUid, -1, spinningFriction=0.1, rollingFriction =0.001, linearDamping = 0.0)
#pybullet.changeDynamics(sphereUid2, -1, spinningFriction =0.0001, rollingFriction = 0.1, linearDamping = 1)
#pybullet.changeDynamics(sphereUid3, -1, spinningFriction = 1, rollingFriction = 1, linearDamping = 0.5)

planet = GravityMovement(mass, sphere3, visualShapeId, spherePos3, planetGravity)
Neutron = GravityMovement(mass, sphere2, visualShapeId2, spherePos2, neutronGravity)
Sun = GravityMovement(mass, sphere, visualShapeId3, spherePos1, sunGravity)
neutronFlag = 1; sunFlag = 0


for i in range(100000):
    pybullet.stepSimulation()
    time.sleep(1./240.)
    #Neutron.rotate(); Sun.rotate()
    print("STARTING CYCLE NEUTRON_FLAG: ",neutronFlag, " SUN_FLAG: ",sunFlag," PLANET POSITION: ",planet.position[0],planet.position[1], planet.position[2])
    if neutronFlag == 1 and sunFlag == 0:
        Distance = distance(planet.getCenter(), Neutron.getCenter(), planet.getGravityField(), Neutron.getGravityField())
        if Distance > 0:
            if planet.position[0] < 0 and planet.position[1] < 0 and planet.position[2] < 0:
                planet.moveToDefault()
            else:
                planet.moveUp()
            print("MOVING UP NOT YET IN GRAVITY")

        else:
            print("CAUGHT GRAVITY")
            if planet.position[2] <= Neutron.position[2] and planet.position[0] >= Neutron.position[0]:
                print("MOVING CURVY UP")
                planet.moveCurvyUp()
            elif planet.position[0] >= Neutron.position[0]and planet.position[2] >= Neutron.position[2]:
                print("MOVING CURVY BACK")
                planet.moveCurvyBack()
            elif planet.position[0] <= Neutron.position[0] and planet.position[2] >= Neutron.position[2]:
                print("MOVING CURVY DOWN")
                planet.moveCurvyDown()
            elif planet.position[0] <= Neutron.position[0] and planet.position[2] <= Neutron.position[2]:
                print("FINAL MOVEMENT BEFORE EXITING DOWN")
                planet.moveDown()
                neutronFlag = 0

    if neutronFlag == 0 and sunFlag == 0:
        print("EXITING GRAVITY")
        if planet.position[0] <= Neutron.position[0] and planet.position[2] <= Neutron.position[2]:
            print("STILL BEHIND THE NEUTRON ON THE WAY OUT")
            planet.moveDown()
        else:
            print("ON ITS WAY OUT TO SUN GRAVITY")
            planet.moveX()
            sunFlag = 1

    if sunFlag == 1 and neutronFlag == 0:
        print("ON THE SUN'S GRAVITY")
        Distance = distance(planet.getCenter(), Sun.getCenter(), planet.getGravityField(), Sun.getGravityField())
        if Distance > 0:
            print("NOT YET CAUGHT BY SUN")
            planet.moveX()
        else:
            if  planet.position[2] >= Sun.position[2] and planet.position[0] <= Sun.position[0]:
                print("MOVING CURVY FORWARD AROUND THE SUN")
                planet.moveCurvyForward() #*****
            elif planet.position[0] >= Sun.position[0] and planet.position[2] >= Sun.position[2]:
                print("THE SWOOP")
                planet.movecurvyAround() #**************
            elif planet.position[0] >= Sun.position[0] and planet.position[2] <= Sun.position[2]:
                print("MOVING AROUND BACK")
                planet.moveCurvyUpward()  # ****
                if planet.position[1] < Sun.position[1]:
                    planet.moveY()
            elif planet.position[2] <= Sun.position[2] and planet.position[0] <= Sun.position[0]:
                print("MOVING UP BACKWARDS")
                planet.moveCurvyBackwards()
               # sunFlag = 0
                neutronFlag = 1

    if sunFlag == 1 and neutronFlag == 1:
        if planet.position[2] < 0 or planet.position[0] > 0 or planet.position[1] < 0:
            planet.moveToDefault()
        else:
            sunFlag = 0


pybullet.disconnect()
