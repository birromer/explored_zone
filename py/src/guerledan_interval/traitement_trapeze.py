from matplotlib import image

from matplotlib import pyplot
import cv2
import numpy as np 
# load image as pixel array

image = image.imread('trajectoire.png')
im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
print(im_gray)
#th, im_gray_th_otsu = cv2.threshold(im_gray, 128, 192, cv2.THRESH_OTSU)
#th, im_th = cv2.threshold(im_gray, 1, 255, cv2.THRESH_BINARY)
for i in range(len(im_gray)):
    for j in range(len(im_gray[0])):
        if im_gray[i][j] == 1.:
            im_gray[i][j] = 0
        else:
            im_gray[i][j] = 255
            #im_gray[i+1][j] = 255
            #im_gray[i+1][j+1] = 255
            #im_gray[i][j+1] = 255
            #im_gray[i][j-1] = 255
            #im_gray[i-1][j-1] = 255
            #im_gray[i-1][j] = 255
            #im_gray[i-1][j+1] = 255
            #im_gray[i+1][j-1] = 255
            
#print(im_gray[0:3])
#cv2.imshow("gray", im_gray)
#cv2.waitKey(0) 
im_gray = np.uint8(im_gray)
edged = cv2.Canny(im_gray, 30, 200) 
cv2.waitKey(0) 
  
contours, hierarchy = cv2.findContours(edged,  
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
  
cv2.imshow('Canny Edges After Contouring', edged) 

  
print("Number of Contours found = " + str(len(contours))) 
# Find the index of the largest contour
"""areas = [cv2.contourArea(c) for c in contours]
max_index = np.argmax(areas)
cnt=contours[max_index]"""
cv2.drawContours(image, contours, -1, (255, 255, 255), 2) 

image = np.uint8(image)
edged = cv2.Canny(image, 30, 200)   
contours, hierarchy = cv2.findContours(edged,  
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 

cv2.drawContours(image, contours, -1, (255, 255, 255), 2) 

image = np.uint8(image)
edged = cv2.Canny(image, 30, 200)   
contours, hierarchy = cv2.findContours(edged,  
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
areas = [cv2.contourArea(c) for c in contours]
max_index = np.argmax(areas)
cnt=contours[max_index]  
cv2.drawContours(image, cnt, -1, (0, 0, 255), 2) 

img = np.ones((image.shape), dtype=np.int64)
img = np.uint8(img)
cv2.drawContours(img, cnt, -1, (255, 255, 255), 2) 
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


"""im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
print(im_gray)
for i in range(len(im_gray)):
    for j in range(len(im_gray[0])):
        if im_gray[i][j] == 255:
            im_gray[i][j] = 1
        elif im_gray[i][j] != 1:
            im_gray[i][j] = -1"""
img2 = np.zeros((img.shape[1], img.shape[0]), dtype=np.int64)
for i in range(len(img2)):
    for j in range(len(img2[0])):
        img2[i][j] = img[j][i]
img2 = np.uint8(img2)
print(img[:])
#cv2.imshow('Contours', im_gray) 
cv2.imshow('test', img)

cv2.waitKey(0) 
cv2.destroyAllWindows() 
cv2.imwrite('trajectoire_traite.png', img)
