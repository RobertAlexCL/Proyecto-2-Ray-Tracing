import struct
from mymath import *
from math import pi, sin, cos, tan, atan2, acos

def try_int(s, base=10, val=None):
  try:
    return int(s, base)
  except ValueError:
    return val

class Obj(object):
    def __init__(self, filename):
        with open(filename) as f:
            self.lines = f.read().splitlines()

        self.vertices = []
        self.texcoords = []
        self.normals = []
        self.vfaces = []
        self.read()

    def read(self):
        for line in self.lines:
            if line:
                prefix, value = line.split(' ', 1)

                if prefix == 'v':
                    self.vertices.append(list(map(float, value.split(' '))))
                elif prefix == 'vt':
                    self.texcoords.append(list(map(float, value.split(' '))))
                elif prefix == 'vn': 
                    self.normals.append(list(map(float, value.split(' '))))
                elif prefix == 'f':
                    self.vfaces.append([list(map(int , face.split('/'))) for face in value.split(' ')])


class texture(object):

    def __init__(self, filename):
        self.filename = filename
        with open(self.filename, "rb") as image:
            image.seek(10)
            headerSize = struct.unpack('=l', image.read(4))[0]

            image.seek(14 + 4)
            self.width = struct.unpack('=l', image.read(4))[0]
            self.height = struct.unpack('=l', image.read(4))[0]

            image.seek(headerSize)

            self.pixels = []

            for y in range(self.height):
                self.pixels.append([])
                for x in range(self.width):
                    b = ord(image.read(1)) / 255
                    g = ord(image.read(1)) / 255
                    r = ord(image.read(1)) / 255

                    self.pixels[y].append((r,g,b))
    
    def getColor(self, dir):
        dir = norm(dir)

        x = int(((atan2( dir[2], dir[0]) / (2 * pi)) + 0.5) * self.width )
        y = int(acos(-dir[1]) / pi * self.height)

        return V3(self.pixels[y][x][0], self.pixels[y][x][1], self.pixels[y][x][2])

    
    def getTextureColor(self, tx, ty):
        if 0<=tx<1 and 0<=ty<1:
            x = int(tx * self.width)
            y = int(ty * self.height)
            return V3(self.pixels[y][x][0], self.pixels[y][x][1], self.pixels[y][x][2])
        else:
            return V3(0, 0, 0)