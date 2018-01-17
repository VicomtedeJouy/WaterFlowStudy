import math

import Rhino
import rhinoscriptsyntax as rs
from Rhino import Geometry as g
import Rhino.Geometry.Collections as c
import Rhino.Geometry.Intersect.Intersection as s
from scriptcontext import sticky as st
import random as r


class RainDrop(object):
    def __init__(self, point3d, facadeMesh):
        self.pos = point3d   # position of the flow
        self.mesh = facadeMesh  # surface mesh of the facade
        self.edges = [facadeMesh.TopologyEdges.EdgeLine(i) for i in range(facadeMesh.TopologyEdges.Count)]
        self.mpos = self.mesh.ClosestMeshPoint(self.pos, 0.)   # mesh position of the flow
        self.wind = windVect    # vector representing the wind force
        self.stepsize = stepSize    # interval of steps on the mesh
        self.state = 'on'   # if the flow is on or off the facade or finished
        self.maxsteps = maxSteps    # max numbers of iterations
        self.count = 0  # step advancement
        self.check = False  # True if the flow has to be checked, false otherwise
        
        self.speed = 4  # we will consider thqt it is the same as the weight for now
        
        self.indexList = len(dropList)
        
        # data collection
        self.step = 0.
        self.points = [self.pos]  # flow points history
        self.curveTemp = [self.pos]  # stock a journey on a srf, then back to 0 when changes
        self.waterPath = [] # gets the different curves of a water path
        self.waterPoints = []   # gets the different points of a water path
        self.dirtCoef = 0
        self.dirtTemp = []
        self.dirtPath = []
        
        dropList.append(self)

    def __del__(self):
        finishedPath.append(self.waterPath)
        ind = self.indexList
        dropList.remove(drop)
        for i in range(ind, len(dropList)):
            self.drops[i].indexList -= 1
        
    def __add__(self, drop):
        self.speed += drop.speed
        del drop


    ### MOVEMENT AREA ###
    
    def move(self):
        count = self.speed
        
        while count > 0:
            self.nextStep()
            
            self.checkStep()
            
            self.updateData()
            
            nextDisplay(display, self)
            
            if self.state == 'finished':
                del self
                break
                
            count -= 1

    def nextStep(self):
        if self.state == 'on':  # if the waterflow is on the facade
            self.nextStepOn()
        if self.state == 'off': # if the waterflow is off the facade (did not used if self.check == 'off' to gain a test)
            self.check = False  # stop checking until it reaches the facade again
            self.nextStepOff()
            self.count = 0   # re initialisation of the count to be sure that it is not overtaken

    def nextStepOn(self):        
        newPlane = rs.PlaneFromNormal(self.pos, self.mesh.NormalAt(self.mpos))
        
        # figure out how much to rotate it to put it in the slope
        deltaAngle = g.Vector3d.VectorAngle( newPlane.XAxis, g.Vector3d(0., 0., -1.), newPlane )
        # rotate it in the plane
        newPlane.Rotate( deltaAngle, newPlane.ZAxis, newPlane.Origin)
        # set the movement vector
        downVect = newPlane.XAxis
        downVect.Unitize()
        downVect = rs.VectorScale(downVect, self.stepsize)
        spacePoint = g.Point3d.Add(self.pos, downVect)
        # find next point
        newPoint = self.mesh.ClosestPoint(spacePoint)
        newPoint = self.randomDir(newPoint, newPlane)
        self.updatePos(newPoint)

    def nextStepOff(self):
        # find the next intersection with the mesh
        self.pos = g.Point3d.Add(self.pos, g.Vector3d(0., 0., -1.) * self.stepsize/10)
        ray = g.Ray3d(self.pos, g.Vector3d(0., 0., -1.))
        num = s.MeshRay(self.mesh, ray)
        if num > 0: 
            # if the water drops on the facade
            newPoint = ray.PointAt(num)
            
            self.state = 'landing'
            self.updateData(newPoint)
            
            self.state = 'on'
            self.updatePos(newPoint)
        else:
            self.state = 'finished'

    def updatePos(self, newPoint):
        # update the position of the drop on the facade, and all the values that comes with it
        self.points.append(newPoint)
        self.pos = newPoint
        self.mpos = self.mesh.ClosestMeshPoint(self.pos, 0.)
        
    def pop(self):
        # get the last position off (because an error would occure otherwise)
        self.points.pop()
        self.pos = self.points[-1]
        self.mpos = self.mesh.ClosestMeshPoint(self.pos, 0.)
    
    ### CHECK AREA ###
    
    def checkStep(self):
        if self.count > 3:
            self.check = True   # begin to check
        if self.check == True:  # check falling and accumulation conditions
            self.accumulationCheck()
            self.anglesCheck()
    
    def anglesCheck(self):
        vect1 = rs.VectorUnitize(rs.VectorCreate(self.points[-2], self.points[-3]))
        vect2 = rs.VectorUnitize(rs.VectorCreate(self.points[-1], self.points[-2]))
        norm = self.mesh.NormalAt(self.mpos)
        
        # check the tolerance angle
        #alpha = g.Vector3d.VectorAngle(vect1, vect2) 
        alpha = math.acos(min(1., rs.VectorDotProduct(vect1, vect2)))
        if alpha > angleTol and norm.Z < 0 :  # if the angle between 2 moves larger than tolerance
            self.pop()
            self.state = 'off'  # the waterflow is off the facade
            edgePoints.append(self.pos)
        
        # check the drop angle
        #alpha = g.Vector3d.VectorAngle(vect2, g.Vector3d(0., 0., -1.))
        alpha = math.acos(-vect2.Z)
        if alpha > angleDrop and norm.Z < 0 : # if the geometry is too steep
            self.pop()
            self.state = 'off'  # the waterflow is off the facade
            edgePoints.append(self.pos)
            
    def accumulationCheck(self):
        d12 = self.points[-1].DistanceTo(self.points[-2])
        norm0 = rs.VectorUnitize(self.mesh.NormalAt(self.mpos))
        
        # special check when the water drop stops
        if d12 == 0.:
            self.pop()
            self.edgeCheck(norm0)
        norm1 = rs.VectorUnitize(self.mesh.NormalAt(self.mesh.ClosestMeshPoint(self.points[-2], 0.)))
        
        # check accumulation conditions (distance, height and stepsize (in order))
        if self.points[-1].Z > self.points[-2].Z or d12 < self.stepsize/20.:
            # verify which case : stuck on an edge or in a basin
            if norm0.Z > 0.99 or (norm1 != norm0 and norm1.Z >= 0 and norm0.Z >= 0):
                # the water drop is in a basin
                self.pop()
                self.state = 'finished'
                criticalPoints.append(self.pos)
            else:
                # the water drop is on an edge    
                self.pop()
                self.state = 'off'
                edgePoints.append(self.pos)
                
    def edgeCheck(self, norm):
        # check if the drop is close to a mesh edge
        for i in range(len(self.edges)):
            line = self.edges[i]
            if line.DistanceTo(self.pos, True) < self.stepsize/50.:
                # it is in a basin
                if norm.Z >= 0.:
                    self.state = 'finished'
                    criticalPoints.append(self.pos)
                # it is on an edge
                else:
                    self.state = 'off'
                    edgePoints.append(self.pos)
        self.state = 'finished'
        criticalPoints.append(self.pos)
    
    
    ### DATA STORAGE ###
    
    def updateData(self, *point):
        self.count += 1
        # update the lists of data that are recorded during the resolution
        if self.state == 'on':
            self.curveTemp.append(self.pos)
            self.dirtCoef += self.getDirtCoef()
            self.dirtTemp.append(self.dirtCoef)
            self.step = g.Line(self.points[-2], self.points[-1])
            
        if self.state == 'landing':
            # creates new lists to collect data
            self.curveTemp = [point[0]]
            self.dirtTemp = []
            
        if self.state == 'finished' or self.state == 'off':
            # collect the datas
            self.waterPath.append(g.PolylineCurve(self.curveTemp))
            self.waterPoints.append(self.curveTemp)
            self.dirtPath.append(self.dirtTemp)
    
    
    ### FONCTIONNALITY AREA ###
    
    def randomDir(self, point, plane):
        # check if the drop has hit a flat srf and return a random direction if so
        if point == self.pos and self.count < 3:
            random = r.uniform(0., 2* math.pi)
            vect = g.Vector3d(1., 0., 0.) * self.stepsize
            vect.Rotate(random, plane.ZAxis)
            point = self.mesh.ClosestPoint(self.pos + vect)
            return point
        
        # check if the drop is riding verticaly
        if rs.VectorUnitize(plane.XAxis).Z > -.5:
            return point
        
        random = r.randint(0, 9)
        # 3 choices : downward, or two side shifting
        if random < 8:
            return point
        else:
            YAxis = rs.VectorUnitize(rs.VectorUnitize(plane.YAxis))
            ratio = 0.18
            if random == 8:
                # first direction is chosen
                point = point + YAxis * self.stepsize/5.
                return self.mesh.ClosestPoint(point)
            if random == 9:
                # first direction is chosen
                point = point - YAxis * self.stepsize/5.
                return self.mesh.ClosestPoint(point)
                
    def getDirtCoef(self):
        vectDown = rs.VectorCreate(self.points[-1], self.points[-2])
        if vectDown.Length == 0.:   print self.count
        alpha = math.acos(-rs.VectorUnitize(vectDown).Z) * 180/math.pi
        norm = rs.VectorUnitize(self.mesh.NormalAt(self.mpos))
        if alpha <= 90 and norm.Z > 0:
            coef = 1-(1-(alpha/90)**2)**.5
            return coef
        if alpha <= 90 and norm.Z <= 0.:
            return 0
        if alpha > 90:
            return 1



def randomPoint():
    N = len(startPoints)-1
    index = r.randint(0, N)
    return startPoints[index]

def initialization(nbDrops):
    new = []
    for i in range(nbDrops):
        pt = randomPoint()
        new.append(pt)
    return new
    
def nextDisplay(display, drop):
    if drop.state == 'on':
        display.append(g.PolylineCurve(drop.curveTemp))
    waterPath.append(g.PolylineCurve(drop.curveTemp))

# Creation of the drop list
if 'dropList' not in st:
    st['dropList'] = []
    st['waterPath'] = []
    
dropList = st['dropList']   # drop list that is used to keep the drops between each steps
finishedPath = st['waterPath'] # drop paths history

global dropList
global finishedPath

display = []
waterPath = []  # new list used every iteration to keep the datas
edgePoints = []
criticalPoints = []

global display
global waterPath
global edgePoints
global criticalPoints

delDropList = []

nbDrops = int(nbDrops)
angleTol = mat[0]   # angle of change of direction tolerance 
angleDrop = mat[1]  # angle for water drops

# addition of the new drops to the list
def initialiseRainDrops(nbDrops, maxDrops):
    newPoints  = initialization(nbDrops)
    if len(dropList) < maxDrops - nbDrops:
        for pt in newPoints:
            RainDrop(pt, facadeMesh)

# loop on the drops
def main(nbDrops, maxDrops):
    
    initialiseRainDrops(nbDrops, maxDrops)
    
    for drop in dropList:
        
        drop.move()


out = len(dropList)

if bool == True:
    main(nbDrops, maxDrops)

st['dropList'] = dropList
st['waterPath'] = finishedPath

del dropList
del waterPath

if bool == False:
    del st['dropList']
    del st['waterPath']