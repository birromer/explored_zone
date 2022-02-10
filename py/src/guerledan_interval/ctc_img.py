from pyibex.image import *
from pyibex import Interval, IntervalVector, SepCtcPair, pySIVIA
from vibes import vibes
import os
import numpy as np
import sys
import matplotlib.pyplot as plt
sys.setrecursionlimit(50000)

def Fcolor(img,i,j):
    num_rows, num_cols = img.shape
    if(img[i,j] == 0):
        img[i,j] = 1
        if(i+1 < num_rows): Fcolor(img,i+1,j)
        if(j+1 < num_cols): Fcolor(img,i,j+1)
        if(j-1 > 0): Fcolor(img,i,j-1)
    else:
        return


world = IntervalVector([[-20,40.0],[-20,40.0]])

#pixel precision
pixel_x = 0.1
pixel_y = -0.1
dt = 0.1
#nb pixel per image
npx = int((world[0].ub() - world[0].lb())/abs(pixel_x))
npy = int((world[1].ub() - world[1].lb())/abs(pixel_y))

#create image
img = np.zeros((npx, npy), dtype=np.int64)

#draw line 1
#x = y for x in [10,20]
for t in np.arange(10,20,dt):
    xi = Interval(t,t+dt)
    yi = xi
    it_x = (xi - world[0].lb())/pixel_x
    it_y = (yi - world[1].ub())/pixel_y
    for i in range(int(it_x.lb()),int(it_x.ub()) + 1):
        for j in range(int(it_y.lb()),int(it_y.ub()) +1):
            img[i,j] = -1

#draw line 2
#y = 10 for x in [10,25]
for t in np.arange(10,25,dt):
    xi = Interval(t,t+dt)
    yi = Interval(10.0,10.0)
    it_x = (xi - world[0].lb())/pixel_x
    it_y = (yi - world[1].ub())/pixel_y
    for i in range(int(it_x.lb()),int(it_x.ub()) + 1):
        for j in range(int(it_y.lb()),int(it_y.ub()) +1):
            img[i,j] = -1

#draw line 3
#y = -2*x + 60 for x in [20,25]
for t in np.arange(20,25,dt):
    xi = Interval(t,t+dt)
    yi = -2*xi + 60
    it_x = (xi - world[0].lb())/pixel_x
    it_y = (yi - world[1].ub())/pixel_y
    for i in range(int(it_x.lb()),int(it_x.ub()) + 1):
        for j in range(int(it_y.lb()),int(it_y.ub()) +1):
            img[i,j] = -1
plt.imshow(img)
plt.show()
#create image of exterior(to contract interior)
img_in = img.copy()
Fcolor(img_in,0,0)
img_in[img < 0] = 1

#create image of interior(to contract exterior)
img_out = np.ones((npx, npy), dtype=np.int64) - img_in
img_out[img < 0] = 1

img_out = img_out.cumsum(0).cumsum(1) #images integrales
img_in = img_in.cumsum(0).cumsum(1)

ctcOut = CtcRaster(img_out, world[0].lb(), world[1].ub(), pixel_x, pixel_y) #contract the exterior
ctcIn = CtcRaster(img_in, world[0].lb(), world[1].ub(), pixel_x, pixel_y) #contract the interior
sep = SepCtcPair(ctcIn, ctcOut)

vibes.beginDrawing()
vibes.newFigure('CtcImage')
X0 = world
pySIVIA(X0, sep, 0.5)
vibes.axisEqual()
