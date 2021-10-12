
from mymath import *

class Material(object):
    #difuse represents colors with a V3
    def __init__(self, diffuse = None, specular = 1, type = 0, ior = 1, texture = None):
        self.diffuse = diffuse
        self.specular = specular
        self.type = type
        self.ior = ior
        self.texture = texture


class Intersect(object):
    def __init__(self, distance, sceneObj, point, normal, texCoords):
        self.distance = distance
        self.refObj = sceneObj
        self.point = point
        self.normal = normal
        self.texCoords = texCoords

class Sphere(object):
    def __init__(self, center, radius, material):
        self.center = center
        self.radius = radius
        self.material = material

    def ray_intersect(self, orig, dir):
        L = sub(self.center, orig)
        l = L.x*L.x+L.y*L.y+L.z*L.z
        tca = dot(L, dir)
        d = (l - tca**2) 

        if d > self.radius*self.radius:
            return None

        thc = (self.radius**2 - d) ** 0.5
        t0 = tca - thc
        t1 = tca + thc

        if t0 < 0:
            t0 = t1

        if t0 < 0:
            return None

        point = segmentoRecta(orig, t0, dir)
        normal = norm(sub(point, self.center))
        return Intersect( distance = t0, point = point, normal=normal, sceneObj=self )


class Plane(object):
    def __init__(self, position, normal, material = Material()):
        self.position = position
        self.normal = norm(normal) 
        self.material = material

    def ray_intersect(self, orig, dir):
        #t = (( planePos - origRayo) dot planeNormal) / (dirRayo dot planeNormal)
        denom = dot(dir, self.normal)

        if abs(denom) > 0.0001:
            num = dot(sub(self.position, orig), self.normal)
            t = num / denom
            if t > 0:
                # P = O + t * D
                hit = sum(orig, mul(dir, t))

                return Intersect(distance = t,
                                 point = hit,
                                 normal = self.normal,
                                 texCoords = None,
                                 sceneObj = self)
        return None

class AABB(object):
    # Axis Aligned Bounding Box
    def __init__(self, position, size, material = Material()):
        self.position = position
        self.size = size
        self.material = material
        self.planes = []

        self.boundsMin = [0,0,0]
        self.boundsMax = [0,0,0]

        halfSizeX = size[0] / 2
        halfSizeY = size[1] / 2
        halfSizeZ = size[2] / 2

        #Sides
        self.planes.append(Plane( sum(position, V3(halfSizeX,0,0)), V3(1,0,0), material))
        self.planes.append(Plane( sum(position, V3(-halfSizeX,0,0)), V3(-1,0,0), material))

        # Up and down
        self.planes.append(Plane( sum(position, V3(0,halfSizeY,0)), V3(0,1,0), material))
        self.planes.append(Plane( sum(position, V3(0,-halfSizeY,0)), V3(0,-1,0), material))

        # Front and Back
        self.planes.append(Plane( sum(position, V3(0,0,halfSizeZ)), V3(0,0,1), material))
        self.planes.append(Plane( sum(position, V3(0,0,-halfSizeZ)), V3(0,0,-1), material))

        #Bounds
        epsilon = 0.001
        for i in range(3):
            self.boundsMin[i] = self.position[i] - (epsilon + self.size[i]/2)
            self.boundsMax[i] = self.position[i] + (epsilon + self.size[i]/2)


    def ray_intersect(self, orig, dir):
        intersect = None
        t = float('inf')

        uvs = None

        for plane in self.planes:
            planeInter = plane.ray_intersect(orig, dir)
            if planeInter is not None:
                # Si estoy dentro de los bounds
                if planeInter.point[0] >= self.boundsMin[0] and planeInter.point[0] <= self.boundsMax[0]:
                    if planeInter.point[1] >= self.boundsMin[1] and planeInter.point[1] <= self.boundsMax[1]:
                        if planeInter.point[2] >= self.boundsMin[2] and planeInter.point[2] <= self.boundsMax[2]:
                            #Si soy el plano mas cercano
                            if planeInter.distance < t:
                                t = planeInter.distance
                                intersect = planeInter

                                u, v = 0, 0

                                if abs(plane.normal[0]) > 0:
                                    # mapear uvs para eje X, uso coordenadas en Y y Z.
                                    u = (planeInter.point[1] - self.boundsMin[1]) / (self.boundsMax[1] - self.boundsMin[1])
                                    v = (planeInter.point[2] - self.boundsMin[2]) / (self.boundsMax[2] - self.boundsMin[2])

                                elif abs(plane.normal[1]) > 0:
                                    # mapear uvs para eje Y, uso coordenadas en X y Z.
                                    u = (planeInter.point[0] - self.boundsMin[0]) / (self.boundsMax[0] - self.boundsMin[0])
                                    v = (planeInter.point[2] - self.boundsMin[2]) / (self.boundsMax[2] - self.boundsMin[2])

                                elif abs(plane.normal[2]) > 0:
                                    # mapear uvs para eje Z, uso coordenadas en X y Y.
                                    u = (planeInter.point[0] - self.boundsMin[0]) / (self.boundsMax[0] - self.boundsMin[0])
                                    v = (planeInter.point[1] - self.boundsMin[1]) / (self.boundsMax[1] - self.boundsMin[1])

                                uvs = (u,v)


        if intersect is None:
            return None

        return Intersect(distance = intersect.distance,
                         point = intersect.point,
                         normal = intersect.normal,
                         texCoords = uvs,
                         sceneObj = self)
