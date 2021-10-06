import cv2
import numpy as np
import time
#import traitement

import os

calib = np.load('calib_parameters.npz')
mtx   = calib['mtx']
dist  = calib['dist']
rvecs = calib['rvecs']
tvecs = calib['tvecs']


temps = []

devs = os.listdir('/dev')
vid_indices = [int(dev[-1]) for dev in devs 
               if dev.startswith('video')]
vid_indices = sorted(vid_indices)
print(vid_indices)

index = int(input("Entrer video : "))
save  = input("Save ? (y/n) : ")=="y"

cam = cv2.VideoCapture(index)
ret_val, img = cam.read()
print(img.shape)


fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 15.0, (img.shape[1],img.shape[0]))
dst = img

def show_webcam():
    t0 = time.time()
    while True:
        ret_val, img = cam.read()
        #cv2.imshow('my webcam', img)
        dst=img
        #dst = cv2.undistort(img, mtx, dist, None)
        cv2.imshow('modif', dst)
        if save:
            temps.append(time.time()-t0)
            out.write(dst)
        if cv2.waitKey(1) == 27: 
            break  # esc to quit
    out.release()
    if save:
        np.savetxt("log_time.txt",np.array(temps))
    cv2.destroyAllWindows()


def main():
    show_webcam()


if __name__ == '__main__':
    main()
