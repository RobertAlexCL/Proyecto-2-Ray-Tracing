import struct
from collections import namedtuple


V2 = namedtuple('Point2', ['x', 'y'])
V3 = namedtuple('Point3', ['x', 'y', 'z'])
V4 = namedtuple('Point4', ['x', 'y', 'z', 'w'])




def sum(v0, v1):
  return V3(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z)

def mul(v0, k):
  return V3(v0.x * k, v0.y * k, v0.z *k)

def sub(v0, v1):
  return V3(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z)

def dot(v0, v1):
  return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z

def length(v0):
  return (v0.x**2 + v0.y**2 + v0.z**2)**0.5

def cross(v0, v1):
  return V3(
    v0.y * v1.z - v0.z * v1.y,
    v0.z * v1.x - v0.x * v1.z,
    v0.x * v1.y - v0.y * v1.x,
  )


def bcenntric(A, B, C, P):
  
  bary = cross(
    V3(C.x - A.x, B.x - A.x, A.x - P.x), 
    V3(C.y - A.y, B.y - A.y, A.y - P.y)
  )

  if abs(bary[2]) < 1:
    return -1, -1, -1   

  return (
    1 - (bary[0] + bary[1]) / bary[2], 
    bary[1] / bary[2], 
    bary[0] / bary[2]
  )


def norm(v0):
  v0length = length(v0)

  if not v0length:
    return V3(0, 0, 0)

  return V3(v0.x/v0length, v0.y/v0length, v0.z/v0length)

def bbox(*vertices):
  xs = [ vertex.x for vertex in vertices ]
  ys = [ vertex.y for vertex in vertices ]
  xs.sort()
  ys.sort()

  return V2(xs[0], ys[0]), V2(xs[-1], ys[-1])

def trans_lineal(M, vec):
  x = 0
  y = 0
  z = 0
  v2 = [vec[0], vec[1], vec[2], 1]
  v = [0, 0, 0, 0]
  for i in range(4):
    for j in range(4):
      v[i] += v2[j] * M[i][j]

  x = v[0]/v[3]
  y = v[1]/v[3]
  z = v[2]/v[3]
  return V3(x, y, z)

def dir_trans_lineal(M, vec):
  x = 0
  y = 0
  z = 0
  v2 = [vec[0], vec[1], vec[2], 0]
  v = [0, 0, 0, 0]
  for i in range(4):
    for j in range(4):
      v[i] += v2[j] * M[i][j]

  x = v[0]
  y = v[1]
  z = v[2]
  return V3(x, y, z)

def matrixmul(M1, M2):
  M3 = [[0 for i in range(4)] for j in range(4)]
  for i in range(4):
    for j in range(4):
      for k in range(4):
        M3[i][j] += M1[i][k]*M2[k][j] 
  return M3

def transpose(M):
  T = [[M[j][i] for j in range(4)] for i in range(4)]
  return T

def det2x2(M):
  return M[0][0]*M[1][1]-M[0][1]*M[1][0]

def det3x3(M):
  det = 0
  sign = 1
  for i in range(3):
    subm = [[M[j][k] for k in range(3) if k != i] for j in range(1, 3)]
    det += M[0][i]*det2x2(subm)*sign
    sign *= -1
  return det

def det4x4(M):
  det = 0
  sign = 1

  for i in range(4):
    subm = [[M[j][k] for k in range(4) if k != i] for j in range(1, 4)]
    det += M[0][i]*det3x3(subm)*sign
    sign *= -1
  return det

def inv(M):
  cofactors = [[0, 0, 0, 0],
  [0, 0, 0, 0],
  [0, 0, 0, 0],
  [0, 0, 0, 0],]

  determinant = det4x4(M)
  if determinant == 0:
    return

  for i in range(4):
    for j in range(4):
      subm = [[M[x][y] for y in range(4) if y != j] for x in range(4) if x != i]
      cofactors[i][j] = det3x3(subm) * ((-1)**(i+j))
  
  adjugate = transpose(cofactors)

  inv = [[adjugate[i][j]/determinant for j in range(4)] for i in range(4)]

  return inv


def segmentoRecta(o, t, dir):
  return V3(o.x + t*dir.x, o.y + t*dir.y, o.z + t*dir.z)










#bmp functions

def char(c):
    return struct.pack('=c', c.encode('ascii'))

def word(c):
    return struct.pack('=h', c)

def dword(c):
    return struct.pack('=l', c)

def color(r, g, b):
    return bytes([int(g),int(r),int(b)])


