
from mymath import *
from math import pi, sin, cos, tan
from obj import Obj, texture
from figures import AABB, Intersect, Material, Plane, Sphere


OPAQUE = 0
REFLECTIVE = 1
TRANSPARENT = 2
MAXRECURSION = 3


#Renderer

class Render(object):
    STEPS = 1
    camPos = V3(0, 0, 0)
    scene = []
    fov = 60
    ambientLight = None
    directionalLight = None
    pointLights = []
    envmap = None

    def __init__(self):
        self.clear_color = color(0,0,0)
        self.draw_color = color(255,255,233)
    
    def glClear(self):
        self.framebuffer = [
            [self.clear_color for x in range(self.width)] 
            for y in range(self.height)
        ]


    def glCreateWindow(self, width, height): #width and height from window are renderer
        self.width = width
        self.height = height
        self.framebuffer = []
        self.glViewPort(0, 0, width, height)
        self.glClear()
    
    def point(self, x,y, col=None):
        if col == None:
            return
        self.framebuffer[int(y)][int(x)] = color(col[0]*255,  col[1]*255, col[2]*255) 

    def glInit(self):
        pass

    def glViewPort(self, x, y, width, height):
        self.x_VP = x
        self.y_VP = y
        self.width_VP = width
        self.height_VP = height


    def glClearColor(self, r, g, b):
        self.clear_color = color(int(round(r*255)),int(round(g*255)),int(round(b*255)))

    def glColor(self, r,g,b):
        self.draw_color = color(int(round(r*255)),int(round(g*255)),int(round(b*255)))

    def glVertex(self, x, y):
        xPixel = round((x+1)*(self.width_VP/2)+self.x_VP)
        yPixel = round((y+1)*(self.height_VP/2)+self.y_VP)
        self.point(xPixel, yPixel)
    
    def glVertex(self, x, y):
        xPixel = round((x+1)*(self.width_VP/2)+self.x_VP)
        yPixel = round((y+1)*(self.height_VP/2)+self.y_VP)
        self.point(xPixel, yPixel)

    def glFinish(self, filename):
        f = open(filename, 'bw')

        #file header
        f.write(char('B'))
        f.write(char('M'))
        f.write(dword(14 + 40 + self.width * self.height * 3))
        f.write(dword(0))
        f.write(dword(14 + 40))

        #image header
        f.write(dword(40))
        f.write(dword(self.width))
        f.write(dword(self.height))
        f.write(word(1))
        f.write(word(24))   
        f.write(dword(0))
        f.write(dword(24))
        f.write(dword(self.width * self.height * 3))
        f.write(dword(0))
        f.write(dword(0))
        f.write(dword(0)) 
        f.write(dword(0))

        # pixel data

        for x in range(self.width):
            for y in range(self.height):
                f.write(self.framebuffer[x][y])
        
        f.close()
    

    def raytracing(self):
        for y in range(0, self.height, self.STEPS):
            for x in range(0, self.width , self.STEPS):
                Px = 2 * ((x + 0.5) / self.width) - 1
                Py = 2 * ((y + 0.5) / self.height) - 1
                t = tan( (self.fov * pi / 180) / 2)
                r = t * self.width / self.height

                Px *= r
                Py *= t
                direction = V3(Px, Py, -1)
                direction = norm(direction)
                self.point(x, y, self.cast(self.camPos, direction))
    
    def cast(self, o = V3(0, 0, 0), d = V3(0, 0, -1), recursion = 0):
        intersection = self.intersect(o, d)
        final_color = None

        if intersection == None or recursion >= MAXRECURSION:
            if self.envmap:
                return self.envmap.getColor(d)
            return V3(1, 0, 1)
        if intersection != None:
            material = intersection.refObj.material
            object_color = material.diffuse
            ambientColor = V3(0, 0, 0)
            dirLightColor = V3(0, 0, 0)
            pointColor = V3(0, 0, 0)
            lightingColor = V3(0, 0, 0)
            pointSpecular = V3(0, 0, 0)
            dirSpecular = V3(0, 0, 0)

            view_dir = sub(self.camPos, intersection.point)
            view_dir = norm(view_dir)
            if self.ambientLight:
                ambientColor = V3(self.ambientLight['color'].x * self.ambientLight['strength'],
                                self.ambientLight['color'].y * self.ambientLight['strength'],
                                self.ambientLight['color'].z * self.ambientLight['strength'],)

            if self.directionalLight:
                shadow_int = 0
                light_dir = norm(mul(self.directionalLight['dir'], -1))
                intensity = max(0, dot(intersection.normal, light_dir)) * self.directionalLight['intensity']
                dirLightColor = V3(self.directionalLight['color'].x * intensity,
                                    self.directionalLight['color'].y * intensity,
                                    self.directionalLight['color'].z * intensity, )
                reflect = self.reflect(intersection.normal, light_dir)

                specular_intensity = self.directionalLight['intensity'] * (max(0, dot(view_dir, reflect)) ** material.specular)
                specular_color = V3(specular_intensity * self.directionalLight['color'].x,
                                    specular_intensity * self.directionalLight['color'].y,
                                    specular_intensity * self.directionalLight['color'].z,)

                shadow_intersect = self.intersect(intersection.point, mul(light_dir, -1), intersection.refObj)
                if shadow_intersect:
                    shadow_int = 1
                dirSpecular = mul(specular_color, (1-shadow_int))
                dirLightColor = mul(dirLightColor, (1-shadow_int))


            for pointLight in self.pointLights:
                diffuseColor = V3(0, 0, 0)
                specular_color = V3(0, 0, 0)
                shadow_int = 0


                light_dir = norm(sub(pointLight['position'], intersection.point))
                intensity = max(0, dot(light_dir, intersection.normal)) * pointLight['intensity']
                diffuseColor = mul(pointLight['color'] , intensity)

                reflect = self.reflect(intersection.normal, light_dir)
                specular_intensity = pointLight['intensity'] * (max(0, dot(view_dir, reflect)) ** material.specular)
                specular_color = mul(pointLight['color'] , specular_intensity)

                shadow_intersect = self.intersect(intersection.point, light_dir, intersection.refObj)
                light_distance = length(sub(pointLight['position'], intersection.point))
                if shadow_intersect and shadow_intersect.distance < light_distance:
                    shadow_int = 1
                pointSpecular = sum(pointSpecular, mul(specular_color, (1-shadow_int)))
                pointColor = sum(pointColor, mul(diffuseColor, (1-shadow_int)))
            

            if material.type == OPAQUE:
                lightingColor = sum(sum(ambientColor, dirLightColor), pointColor)
                lightingColor = sum(lightingColor, sum(dirSpecular, pointSpecular))
                if material.texture:
                    object_color = material.texture.getTextureColor(intersection.texCoords[0], intersection.texCoords[1])
                final_color = V3(max(min(lightingColor.x * object_color.x, 1), 0),
                                max(min(lightingColor.y * object_color.y, 1),0),
                                max(min(lightingColor.z * object_color.z, 1),0))
            
            if material.type == REFLECTIVE:
                reflect = self.reflect(intersection.normal, mul(light_dir, 1))
                reflectColor = self.cast(intersection.point, reflect, recursion=recursion+1)
                final_color = sum(sum(reflectColor, dirSpecular), pointSpecular)
                final_color = V3(final_color[0] * object_color[0],
                                final_color[1] * object_color[1],
                                final_color[2] * object_color[2],)
            
            if material.type == TRANSPARENT:
                outside = dot(d, intersection.normal) < 0
                bias = 0.001
                kr = self.fresnel(intersection.normal, d, material.ior)
                refractColor = V3(0, 0, 0)

                reflect = self.reflect(intersection.normal, mul(light_dir, 1))
                reflectOrig = V3(0, 0, 0)
                if outside:
                    reflectOrig = V3(intersection.point[0] - bias,
                                    intersection.point[1] - bias,
                                    intersection.point[2] - bias, )
                else:  
                    reflectOrig = V3(intersection.point[0] + bias,
                                    intersection.point[1] + bias,
                                    intersection.point[2] + bias, )
                reflectColor = self.cast(reflectOrig, reflect, recursion=recursion+1)
                
                if kr < 1:
                    refract = self.refract(intersection.normal, d, material.ior )
                    if outside:
                        reflectOrig = V3(intersection.point[0] - bias * intersection.normal[0],
                                        intersection.point[1] - bias* intersection.normal[1],
                                        intersection.point[2] - bias* intersection.normal[2], )
                    else:  
                        reflectOrig = V3(intersection.point[0] + bias * intersection.normal[0],
                                        intersection.point[1] + bias* intersection.normal[1],
                                        intersection.point[2] + bias* intersection.normal[2], )
                    refractColor = self.cast(reflectOrig, refract, recursion=recursion+1)

                final_color = sum(sum(sum(mul(reflectColor, kr), mul(refractColor, (1 - kr))) , dirSpecular), pointSpecular)
                final_color = V3(final_color[0] * object_color[0],
                                final_color[1] * object_color[1],
                                final_color[2] * object_color[2],)

        final_color = V3(min(max(final_color[0], 0), 1), min(max(final_color[1], 0), 1), min(max(final_color[2], 0), 1))
            
            
            
            
        return final_color

    def intersect(self, o, d, origObj = None):
        depth = float('inf')
        intersection = None

        for obj in self.scene:
            if origObj == obj:
                continue
            intersect = obj.ray_intersect(o, d)
            if intersect != None:
                if intersect.distance < depth:
                    intersection = intersect
                    depth = intersect.distance

        return intersection

    def reflect(self, normal, dir):
        # R = 2 * ( N . L) * N - L
        reflect = 2*dot(normal, dir)
        reflect = mul(normal, reflect)
        reflect = sub(reflect, dir)
        reflect = norm(reflect)
        return reflect

    def fresnel(self, n, d, ior):
        cosi = max(-1, min(1, dot(d, n)))
        etai = 1
        etat = ior

        if cosi > 0:
            etai, etat, = etat, etai
        
        sint = etai/etat * (max(0, 1- cosi*cosi))**0.5

        if sint >= 1:
            return 1
        
        cost = max(0, 1 - sint*sint) **0.5
        cosi = abs(cosi)
        Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost))
        Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost))
        return (Rs * Rs + Rp * Rp) / 2


    def refract(self, n, d, ior):
        cosi = max(-1, min(1, dot(d, n)))
        etai = 1
        etat = ior

        if cosi < 0:
            cosi = -cosi
        else:
            etai, etat = etat, etai
            n =  mul(n, -1)
        
        eta = etai/etat
        k = 1 - eta*eta * (1-(cosi*cosi))

        if k < 0:
            return None
        
        R = sum(mul(d, eta), mul(n, eta*cosi-k**0.5))
        return norm(R)


r = Render()
res = 256
r.glCreateWindow(res, res)
#r.STEPS = 3
r.envmap = texture('env_map.bmp')
r.ambientLight = { 'color': V3(0, 0, 1), 'strength': 0.2}
r.directionalLight = { 'color': V3(1, 1, 1), 'intensity': 0.3, 'dir': V3(0, 1, -3)}
r.pointLights.append({'color': V3(1, 1, 1), 'intensity': 0.5, 'position': V3(10, 100, -200)})

ventana = Material(V3(0.8, 0.8, 0.8), specular=256, type=TRANSPARENT, texture=None, ior=2.56)
                    #138, 131, 113
honda = Material(V3(138/255, 131/255, 113/255), specular=256, type=REFLECTIVE, texture=None)
asiento = Material(V3(0, 0, 0), specular=1, type=OPAQUE, texture=None)
llantas = Material(V3(0, 0, 0), specular=32, type=OPAQUE, texture=None)

bricks = Material(V3(1, 0, 0), specular=32, type=OPAQUE, texture=texture('brick.bmp'))

acera = Material(V3(0.8, 0.8, 0.8), specular=2, type=OPAQUE, texture=None)
piso = Material(V3(0.7, 0.7, 0.7), specular=2, type=OPAQUE, texture=None)


centrocarro = 0
r.scene.append(AABB(V3(centrocarro, 0, -120), V3(30, 20, 80), ventana) )
r.scene.append(AABB(V3(centrocarro-20, 0, -120), V3(10, 40, 80), honda) )
r.scene.append(AABB(V3(centrocarro+20, 0, -120), V3(10, 40, 80), honda) )
r.scene.append(AABB(V3(centrocarro, 15, -120), V3(30, 10, 80), honda) )
r.scene.append(AABB(V3(centrocarro, -15, -120), V3(30, 10, 80), honda) )
r.scene.append(AABB(V3(centrocarro, -5, -120), V3(30, 15, 80), honda) )

r.scene.append(AABB(V3(centrocarro-10, -20, -150), V3(7, 30, 10), asiento) )
r.scene.append(AABB(V3(centrocarro+10, -20, -150), V3(7, 30, 10), asiento) )

r.scene.append(AABB(V3(centrocarro-25, -30, -130), V3(15, 20, 10), asiento) )
r.scene.append(AABB(V3(centrocarro+25, -30, -130), V3(15, 20, 10), asiento) )

#carros del fondo
r.scene.append(AABB(V3(40, 0, -200), V3(90, 40, 40), honda) )
r.scene.append(AABB(V3(60, 0, -260), V3(90, 40, 40), honda) )
r.scene.append(AABB(V3(80, 0, -320), V3(90, 40, 40), honda) )

#k
r.scene.append(AABB(V3(-80, 60, -190), V3(10, 40, 400), bricks) )
r.scene.append(AABB(V3(-80, 0, -190), V3(10, 40, 400), bricks) )

#acera
r.scene.append(AABB(V3(0, -15, -30), V3(100, 2, 10), acera) )
r.scene.append(AABB(V3(5, -10, -30), V3(10, 5, 10), bricks) )


r.scene.append(Plane(V3(0, -100, 0), V3(0, 1, 0), acera) )


r.raytracing()
r.glFinish('out.bmp')
