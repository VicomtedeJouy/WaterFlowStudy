import math

import Rhino
import rhinoscriptsyntax as rs
from Rhino import Geometry as g
import Rhino.Geometry.Collections as c
import Rhino.Geometry.Intersect.Intersection as s


# OBJECT : waterdrop
class RainDrop(object):
    def __init__(self, point3d, ind, facadeMesh, windvect, stepsize, maxsteps, plane):
        self.pos = point3d   # position of the flow
        self.ind = ind
        self.mesh = facadeMesh  # surface mesh of the facade
        self.mpos = self.mesh.ClosestMeshPoint(self.pos, self.ind)   # mesh position of the flow
        self.wind = windvect
        self.antol = self.mesh.mat[self.ind][0]   # tolerance angle
        self.androp = self.mesh.mat[self.ind][1] # drop angle
        self.stepsize = stepsize    # interval of steps on the mesh
        self.points = [self.pos]  # flow points history
        self.curveTemp = [self.pos]  # stock a journey on a srf, then back to 0 when changes
        self.waterPath = [] # gets the different curves of a water path
        self.state = 'on'   # if the flow is on or off the facade or finished
        self.maxsteps = maxsteps    # max numbers of iterations
        self.check = False  # True if the flow has to be checked, false otherwise
        self.plane = plane  # plane that define the ground

    def nextStep(self):
        newPlane = rs.PlaneFromNormal(self.pos, self.mpos.Mesh.NormalAt(self.mpos))
        # create a vector from newFrame XAxis
        downVect = newPlane.XAxis
        # figure out how much to rotate it.
        deltaAngle = g.Vector3d.VectorAngle( downVect, g.Vector3d(0.0, 0.0, -1.0), newPlane )
        # rotate it in the plane
        downVect.Rotate( deltaAngle, newPlane.ZAxis)
        # set the length
        downVect = rs.VectorScale(downVect, self.stepsize)
        spacePoint = g.Point3d.Add(self.pos, downVect)
        # find next point
        newPoint, id = self.mesh.ClosestPoint(spacePoint)

        if newPoint.Z > self.pos.Z: # if higher
            self.state = 'off'
            print 'higher', self.ind
        #if newPoint == self.pos: # if the water drop stops
        #    self.state = 'off'
        #    print 'stop'
        #elif self.checkTolerance(newPoint) == True: # if too close
        #    self.state = 'finished'
        else:
            self.updatePos(newPoint, id)

    def nextSurf(self):
        # find the next intersection with the mesh
        self.pos = g.Point3d.Add(self.pos, g.Vector3d(0., 0., -1.) * self.stepsize/10)
        ray = g.Ray3d(self.pos, g.Vector3d(0., 0., -1.))
        results = self.mesh.MeshRay(ray)
        if not results: # if it does not exists
            self.nextCrv()
            #self.pos.Transform(g.Transform.PlanarProjection(self.plane))
            #self.points.append(self.pos)
            self.state = 'finished' # the waterflow has ended its path
        else :  # if it does
            self.nextCrv()
            newPoint = results[0]
            id = results[1]
            self.updatePos(newPoint, id)
            self.state = 'on'   # the waterflow is on the facade again

    def nextCrv(self):
        # separates the curves after a 'srf to srf' journey
        self.waterPath.append(g.PolylineCurve(self.curveTemp))
        self.curveTemp = []

    def updatePos(self, newPoint, id):
        #   change flow position to the new point
        self.points.append(newPoint)
        self.curveTemp.append(newPoint)
        self.pos = newPoint
        self.ind = id
        self.mpos = self.mesh.ClosestMeshPoint(self.pos, self.ind)
        
    def tol(self):
        #   tolerance angle check function
        vect1 = rs.VectorCreate(self.points[-2], self.points[-3])
        vect2 = rs.VectorCreate(self.points[-1], self.points[-2])
        norm = self.mesh.NormalAt(self.mpos)
        alpha = g.Vector3d.VectorAngle(vect1, vect2)
        if alpha > self.antol and rs.VectorDotProduct(norm, g.Vector3d(0., 0., 1.)) < 0 :  # if the angle between 2 moves larger than tolerance
            self.state = 'off'  # the waterflow is off the facade
        
    def drop(self):
        #   drop angle check function
        move = rs.VectorCreate(self.points[-1], self.points[-2])
        alpha = g.Vector3d.VectorAngle(move, g.Vector3d(0., 0., -1.))
        norm = self.mesh.NormalAt(self.mpos)
        if alpha > (math.pi/2-self.androp) and rs.VectorDotProduct(norm, g.Vector3d(0., 0., 1.)) < 0 : # if the geometry is too steep
            self.state = 'off'  # the waterflow is off the facade
            
    def toldrop(self):
        vect1 = rs.VectorCreate(self.points[-2], self.points[-3])
        vect2 = rs.VectorCreate(self.points[-1], self.points[-2])
        norm = self.mpos.Mesh.NormalAt(self.mpos)
        alpha = g.Vector3d.VectorAngle(vect1, vect2)
        if alpha > self.antol and rs.VectorDotProduct(norm, g.Vector3d(0., 0., 1.)) < 0 :  # if the angle between 2 moves larger than tolerance
            self.state = 'off'  # the waterflow is off the facade
        alpha = g.Vector3d.VectorAngle(vect2, g.Vector3d(0., 0., -1.))
        if alpha > (math.pi/2-self.androp) and rs.VectorDotProduct(norm, g.Vector3d(0., 0., 1.)) < 0 : # if the geometry is too steep
            self.state = 'off'  # the waterflow is off the facade



# OBJECT : list of mesh, to be used like a single mesh
class FacadeMesh(object):
    def __init__(self, mesh, mat):
        self.length = len(mesh)
        self.obj = [mesh[i] for i in range(self.length)]
        self.mat = [mat[i] for i in range(self.length)]
        
    #   return the index of the mesh in the list
    def ind(self, mesh):
        return self.obj.index(mesh)
        
    #   rewriting these methods to use them on facadeMesh as if it was a single mesh
    def ClosestPoint(self, point):
        index = 0
        pt = self.obj[0].ClosestPoint(point)
        d = point.DistanceTo(pt)
        for i in range(1, self.length):
            ptTemp = self.obj[i].ClosestPoint(point)
            dd = point.DistanceTo(ptTemp)
            if dd < d:
                index = i
                d = dd
                pt = ptTemp
        return pt, i  #   return both the point and the index of the mesh it is on
         
    def ClosestMeshPoint(self, point, ind):
        return self.obj[ind].ClosestMeshPoint(point, 0.)
         
    def NormalAt(self, mPoint, ind):
        return self.obj[ind].NormalAt(mPoint)
        
    def MeshRay(self, ray):
        startPt = ray.Position
        l = []
        for i in range(self.length):
            num = s.MeshRay(self.obj[i], ray)
            if num > 0:
                pt = ray.PointAt(num)
                l.append([ray.PointAt(num), i])
        if not l:   #   if the list is empty, return nothing
            return([])
        else:   #   return the closeset point and the index of the mesh it is on
            tempDist = [startPt.DistanceTo(l[i][0]) for i in range(len(l))]
            index = tempDist.index(min(tempDist))
            return l[index]



def main(startPoint, facadeMesh, windVect, stepSize, maxSteps, endPlane):
    # does the rain line touch the facade?
    init = start(facadeMesh, startPoint, windVect)
    if not init:    # the rainLine does not touch the facade
        return [], [], []
    startPointDrop = init[0]    # impact point
    ind = init[1]   # index of the mesh the water is on
    
    # creation and initialization of the RainDrop OBJECT
    drop = RainDrop(startPointDrop, ind, facadeMesh, windVect, stepSize, maxSteps, endPlane)
    i = 0
    
    # loop that draw the water path
    while drop.state != 'finished' and i < drop.maxsteps:
        i += 1
        if i > 4:
            drop.check = True   # begin to check
        if drop.check == True:  # check falling conditions
            drop.toldrop()
        if drop.state == 'on':  # if the waterflow is on the facade
            drop.nextStep()
        else: # if the waterflow is off the facade (did not used if self.check == 'off' to gain a test)
            drop.check = False  # stop checking until it reaches the facade again
            drop.nextSurf()
            i = 0   # re initialisation of the count to be sure that it is not overtaken
    print ind
    # creation of the curves
    curves = []
    for cu in drop.waterPath:
        curves.append(cu)
    return curves, startPointDrop, g.Line(startPoint, startPointDrop)


def start(facadeMesh, point, windVect):
    ray = g.Ray3d(point, windVect)
    return facadeMesh.MeshRay(ray)



fCrvs = []
impactPoints = []
rainLines = []

facadeMesh = FacadeMesh(facadeMesh, mat)

for pt in startPoint:
    temp, imPt, rLines = main(pt, facadeMesh, windVect, stepSize, maxSteps, endPlane)
    if len(temp) != 0:
        for cu in temp:
            fCrvs.append(cu)
        impactPoints.append(imPt)
        rainLines.append(rLines)
