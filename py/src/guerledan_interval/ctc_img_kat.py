from pyibex.image import *
from pyibex import Interval, IntervalVector, SepCtcPair, pySIVIA
from vibes import vibes
import os
import numpy as np
import sys
from matplotlib import image
from matplotlib import pyplot

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

#load image

img = image.imread('trajectoire_traite.png')
npx = img.shape[0]
npy = img.shape[1]
print(img.shape)
for i in range(len(img)):
    for j in range(len(img[0])):
        if img[i][j] == 1.:
            img[i][j] = -1
        else:
            img[i][j] = 0
"""
img2 = np.zeros((npy, npx), dtype=np.int64)
for i in range(len(img2)):
    for j in range(len(img2[0])):
        img2[i][j] = img[j][i]
npx = img2.shape[0]
npy = img2.shape[1]
"""

#create image of exterior(to contract interior)
img_in = img2.copy()
Fcolor(img_in,0,0)
img_in[img2 < 0] = 1

#create image of interior(to contract exterior)
img_out = np.ones((npx, npy), dtype=np.int64) - img_in
img_out[img2 < 0] = 1

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
