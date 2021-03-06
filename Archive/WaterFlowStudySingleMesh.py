import math

import Rhino
import rhinoscriptsyntax as rs
from Rhino import Geometry as g
import Rhino.Geometry.Collections as c
import Rhino.Geometry.Intersect.Intersection as s


# OBJECT : waterdrop
class RainDrop(object):
    def __init__(self, point3d, facadeMesh, windvect, stepsize, maxsteps, plane):
        self.pos = point3d   # position of the flow
        self.mesh = facadeMesh  # surface mesh of the facade
        self.edges = [facadeMesh.TopologyEdges.EdgeLine(i) for i in range(facadeMesh.TopologyEdges.Count)]
        self.mpos = self.mesh.ClosestMeshPoint(self.pos, 0.)   # mesh position of the flow
        self.wind = windvect    # vector representing the wind force
        self.stepsize = stepsize    # interval of steps on the mesh
        self.points = [self.pos]  # flow points history
        self.curveTemp = [self.pos]  # stock a journey on a srf, then back to 0 when changes
        self.waterPath = [] # gets the different curves of a water path
        self.state = 'on'   # if the flow is on or off the facade or finished
        self.maxsteps = maxsteps    # max numbers of iterations
        self.check = False  # True if the flow has to be checked, false otherwise
        self.plane = plane  # plane that define the ground

    def nextStep(self):
        newPlane = rs.PlaneFromNormal(self.pos, self.mesh.NormalAt(self.mpos))
        # create a vector from newFrame XAxis
        downVect = newPlane.XAxis
        downVect.Unitize()
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
        
    def finish(self):
        # finish the water path
        self.waterPath.append(g.PolylineCurve(self.curveTemp))
        self.state = 'finished'
        
    def pop(self):
        # get the last position off (because an error would occure otherwise)
        self.points.pop()
        self.curveTemp.pop()
        self.pos = self.points[-1]
        self.mpos = self.mesh.ClosestMeshPoint(self.pos, 0.)
            
    def anglesCheck(self):
        vect1 = rs.VectorCreate(self.points[-2], self.points[-3])
        vect2 = rs.VectorCreate(self.points[-1], self.points[-2])
        norm = self.mesh.NormalAt(self.mpos)
        alpha = g.Vector3d.VectorAngle(vect1, vect2)
        # check the tolerance angle 
        if alpha > angleTol and rs.VectorDotProduct(norm, g.Vector3d(0., 0., 1.)) < 0 :  # if the angle between 2 moves larger than tolerance
            self.state = 'off'  # the waterflow is off the facade
            edgePoints.append(self.pos)
        # check the drop angle 
        alpha = g.Vector3d.VectorAngle(vect2, g.Vector3d(0., 0., -1.))
        if alpha > (math.pi/2-angleDrop) and rs.VectorDotProduct(norm, g.Vector3d(0., 0., 1.)) < 0 : # if the geometry is too steep
            self.state = 'off'  # the waterflow is off the facade
            
    def accumulationCheck(self):
        d12 = self.points[-1].DistanceTo(self.points[-2])
        norm0 = rs.VectorUnitize(self.mesh.NormalAt(self.mpos))
        # special check when the water drop stops
        if d12 == 0.:
            self.pop()
            #weirdos.append(self.pos)
            self.edgeCheck(norm0)
        norm1 = rs.VectorUnitize(self.mesh.NormalAt(self.mesh.ClosestMeshPoint(self.points[-2], 0.)))
        # check accumulation conditions (distance, height and stepsize (in order))
        if self.points[-1].Z > self.points[-2].Z or d12 < self.stepsize/20.:
            # verify which case : stuck on an edge or in a basin
            if norm0.Z > 0.99 or (norm1 != norm0 and norm1.Z >= 0 and norm0.Z >= 0):   # the water drop is in a basin
                self.finish()
                criticalPoints.append(self.pos)
            else:    # the water drop is on an edge
                self.state = 'off'
                edgePoints.append(self.pos)
                
    def edgeCheck(self, norm):
        # check if the drop is close to a mesh edge
        for i in range(len(self.edges)):
            line = self.edges[i]
            if line.DistanceTo(self.pos, True) < self.stepsize/50.:
                # it is in a basin
                if norm.Z >= 0.:
                    self.finish()
                    criticalPoints.append(self.pos)
                # it is on an edge
                else:
                    self.state = 'off'
                    edgePoints.append(self.pos)



def pathDrawing(startPoint, facadeMesh, windVect, stepSize, maxSteps, endPlane):
    # does the rain line touch the facade?
    ray = g.Ray3d(startPoint, windVect)
    num = s.MeshRay(facadeMesh, ray)
    if num <= 0:    # the rainLine does not touch the facade
        return [], [], []
    startPointDrop = ray.PointAt(num)    # impact point
    
    # creation and initialization of the RainDrop OBJECT
    drop = RainDrop(startPointDrop, facadeMesh, windVect, stepSize, maxSteps, endPlane)
    i = 0
    
    # loop that draw the water path
    while drop.state != 'finished' and i < drop.maxsteps:
        i += 1
        if i > 5:
            drop.check = True   # begin to check
        if drop.check == True:  # check falling and accumulation conditions
            drop.accumulationCheck()
            drop.anglesCheck()
        if drop.state == 'on':  # if the waterflow is on the facade
            drop.nextStep()
        if drop.state == 'off': # if the waterflow is off the facade (did not used if self.check == 'off' to gain a test)
            drop.check = False  # stop checking until it reaches the facade again
            drop.nextSurf()
            i = 0   # re initialisation of the count to be sure that it is not overtaken
    
    # creation of the curves
    curves = []
    for cu in drop.waterPath:
        curves.append(cu)
    if len(drop.waterPath) == 0:
        print 'missed'
    return curves, startPointDrop, g.Line(startPoint, startPointDrop)

def main(startPoint, facadeMesh, windVect, stepSize, maxSteps, endPlane):
    for pt in startPoint:
        temp, imPt, rLines = pathDrawing(pt, facadeMesh, windVect, stepSize, maxSteps, endPlane)
        
        if len(temp) != 0:
            for cu in temp:
                fCrvs.append(cu)
            impactPoints.append(imPt)
            rainLines.append(rLines)
    return fCrvs, impactPoints, rainLines



fCrvs = []
impactPoints = []
rainLines = []
criticalPoints = [] # points where the program has stopped because of flatness
edgePoints = [] # points where the water drops dropped from the facade

global criticalPoints
global edgePoints

angleTol = mat[0]   # angle of change of direction tolerance 
angleDrop = mat[1]  # angle for water drops

fCrvs, impactPoints, rainLines = main(startPoint, facadeMesh, windVect, stepSize, maxSteps, endPlane)