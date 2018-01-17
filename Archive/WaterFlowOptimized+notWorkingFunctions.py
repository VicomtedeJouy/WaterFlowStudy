import math

import Rhino
import rhinoscriptsyntax as rs
from Rhino import Geometry as g
import Rhino.Geometry.Collections as c
import Rhino.Geometry.Intersect.Intersection as s


# OBJECT : waterdrop
class RainDrop(object):
    def __init__(self, point3d, facadeMesh, edges, windvect, stepsize, maxsteps, plane):
        self.pos = point3d   # position of the flow
        self.mesh = facadeMesh  # surface mesh of the facade
        self.edges = [line for line in edges if isinstance(line, g.Line)]
        #self.edges = [facadeMesh.TopologyEdges.EdgeLine(i) for i in range(facadeMesh.TopologyEdges.Count)]
        self.mpos = self.mesh.ClosestMeshPoint(self.pos, 0.)   # mesh position of the flow
        self.wind = windvect    # vector representing the wind force
        self.stepsize = stepsize    # interval of steps on the mesh
        self.points = [self.pos]  # flow points history
        self.curveTemp = [self.pos]  # stock a journey on a srf, then back to 0 when changes
        self.waterPath = [] # gets the different curves of a water path
        self.state = 'on'   # if the flow is on or off the facade or finished
        self.maxsteps = maxsteps    # max numbers of iterations
        self.count = 0  # step counter
        self.check = False  # True if the flow has to be checked, false otherwise
        self.plane = plane  # plane that define the ground

    def __del__(self):
        del self.mesh
        del self.edges
        del self.points
        del self.curveTemp
        del self.waterPath

    def nextStep(self):
        newPlane = rs.PlaneFromNormal(self.pos, self.mesh.NormalAt(self.mpos))
        # create a vector from newFrame XAxis
        downVect = newPlane.XAxis
        rs.VectorUnitize(downVect)
        # figure out how much to rotate it.
        deltaAngle = g.Vector3d.VectorAngle( downVect, g.Vector3d(0.0, 0.0, -1.0), newPlane )
        # rotate it in the plane
        downVect.Rotate( deltaAngle, newPlane.ZAxis)
        # set the length
        downVect = rs.VectorScale(downVect, self.stepsize)
        spacePoint = g.Point3d.Add(self.pos, downVect)
        # find next point
        newPoint = self.mesh.ClosestPoint(spacePoint)
        self.updatePos(newPoint)

    def nextSurf(self):
        # find the next intersection with the mesh
        self.pos = g.Point3d.Add(self.pos, g.Vector3d(0., 0., -1.) * self.stepsize/10)
        ray = g.Ray3d(self.pos, g.Vector3d(0., 0., -1.))
        num = s.MeshRay(self.mesh, ray)
        if num > 0: # if it does not exists
            self.nextCrv()
            newPoint = ray.PointAt(num)
            #self.waterPath.append(g.Line(self.pos, newPoint))  # this line can be unhashtagged if we want to follow the water path off the facade
            self.updatePos(newPoint)
            self.state = 'on'   # the waterflow is on the facade again
        else:
            self.finish()
            #self.pos.Transform(g.Transform.PlanarProjection(self.plane))   # these lines can be unhashtagged if we want to display the path of the water to the ground
            #self.points.append(self.pos)

    def nextCrv(self):
        # separates the curves after a 'srf to srf' journey
        self.waterPath.append(g.PolylineCurve(self.curveTemp))
        self.curveTemp = []

    def updatePos(self, newPoint):
        #   change flow position to the new point
        self.points.append(newPoint)
        self.curveTemp.append(newPoint)
        self.pos = newPoint
        self.mpos = self.mesh.ClosestMeshPoint(self.pos, 0.)
        
    def pop(self):
        # get the last position off (because an error would occure otherwise)
        self.points.pop()
        self.curveTemp.pop()
        self.pos = self.points[-1]
        self.mpos = self.mesh.ClosestMeshPoint(self.pos, 0.)
        
    def finish(self):
        # finish the water path
        self.waterPath.append(g.PolylineCurve(self.curveTemp))
        self.state = 'finished'
            
    def anglesCheck(self):
        vect1 = rs.VectorCreate(self.points[-2], self.points[-3])
        vect2 = rs.VectorCreate(self.points[-1], self.points[-2])
        norm = self.mesh.NormalAt(self.mpos)
        alpha = g.Vector3d.VectorAngle(vect1, vect2)
        
        # check the tolerance angle
        if alpha > angleTol and norm.Z < 0 :  # if the angle between 2 moves larger than tolerance
            self.state = 'off'  # the waterflow is off the facade
            edgePoints.append(self.pos)
        
        # check the drop angle
        alpha = g.Vector3d.VectorAngle(vect2, g.Vector3d(0., 0., -1.))
        if alpha > (math.pi/2-angleDrop) and norm.Z < 0 : # if the geometry is too steep
            self.state = 'off'  # the waterflow is off the facade
            
    def accumulationCheck(self):
        d12 = self.points[-1].DistanceTo(self.points[-2])
        norm0 = rs.VectorUnitize(self.mesh.NormalAt(self.mpos))
        
        # special check when the water drop stops
        if d12 == 0.:
            self.pop()  # get the last point out, to avoid 'invalid curves'
            self.dropStop(norm0)
        
        norm1 = rs.VectorUnitize(self.mesh.NormalAt(self.mesh.ClosestMeshPoint(self.points[-2], 0.)))
        # check accumulation conditions (distance, height and stepsize (in order))
        if self.points[-1].Z > self.points[-2].Z or d12 < self.stepsize/20.:
            # verify which case : stuck on an edge or in a basin
            if norm0.Z > 0.99 or (norm1 != norm0 and norm1.Z >= 0 and norm0.Z >= 0):   # the water drop is in a basin
                self.finish()
                self.basinAccumulation()
                criticalPoints.append(self.pos)
            else:    # the water drop is on an edge
                self.state = 'off'
                edgePoints.append(self.pos)
                
    def dropStop(self, norm):
        # the drop has stopped
        # it is in a basin
        if norm.Z > 0.:
            self.finish()
            self.basinAccumulation()
            criticalPoints.append(self.pos)
        # it is on an edge
        else:
            self.state = 'off'
            edgePoints.append(self.pos)
    
    def basinAccumulation(self):
        index = self.closeEdge()
        if index == -1:
            return
        edge = self.edges[index]
        edgeList.append(edge)
        potential = []  # potential leak of water
        pt1 = edge.PointAt(0)
        pt2 = edge.PointAt(1)
        if pt1.Z == pt2.Z:  # if the edge is flat, then we start drops on each side
            potential.append([pt1, rs.VectorCreate(pt1, pt2)])
            potential.append([pt2, rs.VectorCreate(pt2, pt1)])
        else:   # the edge is supposed to be flat anyway...
            if pt1.Z > pt2.Z: #if it is not flat, we only add to the bottom part
                pt1, pt2 = pt2, pt1
            potential.append([pt1, rs.VectorCreate(pt1, pt2)])
        for el in potential:
            if self.wallCheck(el[0], el[1]):
                accuPoints.append(el[0])
                accu.append(el[0])
                
    def wallCheck(self, point, dir):
        dist = self.stepsize*0.05
        dir = rs.VectorUnitize(dir) * dist
        up = g.Vector3d.ZAxis * dist
        pt1 = point + up
        pt1 = pt1 + dir
        pt2 = pt1 + dir
        pt3 = point + 2*dir
        line1 = g.Line(pt1, pt2)
        line2 = g.Line(pt2, pt3)
        test = [s.MeshLine(self.mesh, line1)[1], s.MeshLine(self.mesh, line2)[1]]
        print test
        #if test == [None, None]:
        #    return True
        return False
        
    def closeEdge(self):
        # return the index of the closest edge (with an arbitrary condition to qvoid conflicts)
        for i in range(len(self.edges)):
            line = self.edges[i]
            if line.DistanceTo(self.pos, True) < self.stepsize/20.:
                return i
        return -1


def impactPoint(startPoint, facadeMesh, windVect):
    # does the rain line touch the facade?
    ray = g.Ray3d(startPoint, windVect)
    num = s.MeshRay(facadeMesh, ray)
    if num <= 0:    # the rainLine does not touch the facade
        return
    startPointDrop = ray.PointAt(num)    # impact point
    impactPoints.append(startPointDrop)
    rainLines.append(g.Line(startPoint, startPointDrop))
    return startPointDrop

def pathDrawing(drop):
    # loop that draw the water path
    while drop.state != 'finished' and drop.count < drop.maxsteps:
        drop.count += 1
        if drop.count > 5:
            drop.check = True   # begin to check
        if drop.check == True:  # check falling and accumulation conditions
            drop.accumulationCheck()
            drop.anglesCheck()
        if drop.state == 'on':  # if the waterflow is on the facade
            drop.nextStep()
        if drop.state == 'off': # if the waterflow is off the facade (did not used if self.check == 'off' to gain a test)
            drop.check = False  # stop checking until it reaches the facade again
            drop.nextSurf()
            drop.count = 0   # re initialisation of the count to be sure that it is not overtaken
    
    # creation of the curves
    curves = []
    for cu in drop.waterPath:
        curves.append(cu)
    return curves

def main(startPoint, facadeMesh, edges, windVect, stepSize, maxSteps, endPlane):
    # draw the path from the rain that hit the facade
    for pt in startPoint:
        if impactPoint(pt, facadeMesh, windVect) != None:
            startPointDrop = impactPoint(pt, facadeMesh, windVect)
            drop = RainDrop(startPointDrop, facadeMesh, edges, windVect, stepSize, maxSteps, endPlane)
            temp = pathDrawing(drop)
            del drop
            if len(temp) != 0:
                for cu in temp:
                    fCrvs.append(cu)
    
    # draw the path from the water that has accumulate on the facade
    stop = len(startPoint)
    count = 0
    while len(accuPoints) > 0 and count < stop:
        count += 1
        pt = accuPoints.pop()
        drop = RainDrop(pt, facadeMesh, edges, windVect, stepSize, maxSteps, endPlane)
        temp = pathDrawing(drop)
        del drop
        if len(temp) != 0:
            for cu in temp:
                fCrvs.append(cu)
    # return the curves
    return fCrvs



fCrvs = []
impactPoints = []
rainLines = []
criticalPoints = [] # points where the program has stopped because of flatness
edgePoints = [] # points where the water drops dropped from the facade
edgeList = []
accu = []
accuPoints = []

global criticalPoints
global edgePoints
global edgeList
global accu
global accuPoints
global impactPoints
global rainLines

angleTol = mat[0]   # angle of change of direction tolerance 
angleDrop = mat[1]  # angle for water drops

fCrvs = main(startPoint, facadeMesh, edges, windVect, stepSize, maxSteps, endPlane)