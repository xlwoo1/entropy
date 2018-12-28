from open3d import *
import numpy as np
import matplotlib.pyplot as plt
import time

rgb = read_point_cloud("./rgb2.pcd")
seg = read_point_cloud("./seg2.pcd")
label = (np.asarray(seg.colors)[:,0]*255/16).astype("uint8")

rgb1 = read_point_cloud("./rgb6.pcd")
seg1 = read_point_cloud("./seg6.pcd")
label1 = (np.asarray(seg1.colors)[:,0]*255/16).astype("uint8")

def objectList(rgb,label):

    start = time.time()
    rgbcolor = np.asarray(rgb.colors)
    rgbpoints = np.asarray(rgb.points)
    rgbarray = np.concatenate((rgbcolor,rgbpoints),axis = 1)

    labelcount = np.unique(label).astype("uint8")
    print(labelcount)
    newset = [[] for i in range( len(labelcount))]
    print(time.time()-start)
    for id1,pixels in enumerate(label):

        newset[pixels-1].append(rgbarray[id1])

                
    print(time.time()-start)            
    return newset


# start = time.time()

list1 = objectList(rgb,label)

# print(time.time()-start)
list2 = objectList(rgb1,label1)
# print(list1[5])