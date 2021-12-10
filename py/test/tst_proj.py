import numpy as np
from projection import *
import pylab as plt

R   = np.array([[1,0,0],[0,1,0],[0,0,1]])
pos = np.array([[0,0,1]]).T


sortie = []
entrer = [[-1,1],[-1,-1],[1,-1],[1,1],[-1,1]]

for i in range(len(entrer)):
	pos_img = np.array(entrer[i])
	sortie.append(proj(pos,R,pos_img))

sortie = np.array(sortie).T
plt.plot(sortie[0],sortie[1],'b')
plt.plot([pos[0][0]],[pos[1][0]],'ro')
plt.show()
