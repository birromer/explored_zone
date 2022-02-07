from pyproj import Proj
import csv
import pandas
import numpy as np

myProj = Proj("+proj=utm +zone=30U, +ellps=WGS84 +datum=WGS84 +units=m +no_defs")

x_init = 498752
y_init = 5.33854e+06

data = np.loadtxt('/home/robot/ros/explored_zone/src/viz_intervals/trapeze2.txt')
col0 = data[:, 0]
col1 = data[:, 1]
col2 = data[:, 2]
col3 = data[:, 3]
col4 = data[:, 4]
col5 = data[:, 5]
col6 = data[:, 6]
col7 = data[:, 7]
col8 = data[:, 8]
col9 = data[:, 9]

col10 = data[:, 10] #lon
col11 = data[:, 11] #lat

lon0, lat0 = myProj(col0 + x_init, col1 + y_init, inverse=True)
lon1, lat1 = myProj(col2 + x_init, col3 + y_init, inverse=True)
lon2, lat2 = myProj(col4 + x_init, col5 + y_init, inverse=True)
lon3, lat3 = myProj(col6 + x_init, col7 + y_init, inverse=True)
lon4, lat4 = myProj(col8 + x_init, col9 + y_init, inverse=True)
lon5, lat5 = myProj(col11 + x_init, col10 + y_init, inverse=True)
#tableau = np.hstack((lon0, lat0, lon1, lat1, lon2, lat2, lon3, lat3, lon4, lat4, col10, col11))
#print(tableau)
#np.savetxt('converti.txt', tableau, delimiter = ' ', newline = '\n')

with open('converti2.txt', 'w') as f:
    writer = csv.writer(f, delimiter='\t')
    #writer.writerows(zip(lon0, lat0, lon1, lat1, lon2, lat2, lon3, lat3, lon4, lat4, col10, col11))
    writer.writerows(zip(lon0, lat0, lon1, lat1, lon2, lat2, lon3, lat3, lon4, lat4, lon5, lat5))


