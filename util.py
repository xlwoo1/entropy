import numpy as np
from open3d import *
import copy
import time

def is_empty(any_structure):
    if np.any(any_structure):
        return False
    else:
        return True

def iVal(pcd,pcd2):
    temp = np.asarray(pcd.colors)*255/16
    temp = temp.astype("uint8")
    test = np.concatenate((pcd.points,temp),axis = 1)

    test2 = np.concatenate((pcd2.points,pcd2.colors),axis = 1)

    test[:][3:5] = (test[:][3:5]*255/16).astype("uint8")
    count = [0,0,0,0,0]
    
    return test,test2

def fillObjectArray(test,test2):#,object1,object2,object3,object4,object5):
    object1 = []
    object2 = []
    object3 = []
    object4 = []
    object5 = []
    for idx,pixels in enumerate(test):
        if pixels[4] == 4:
            if is_empty(object1):
                object1 = [test2[idx]]
                object1 = np.asarray(object1)
            else:
    #             print(object1)
                object1 = np.append(object1,[test2[idx]],axis = 0)

    #                 print(object1[:][n-4][:])
    #                 print(np.asarray([pixels]))
    #                 print(np.asarray(object1[:][:][:]).shape,
    #                       np.asarray([pixels]).shape)
    #             object1 = object1[n-4].append(pixels)
        elif pixels[4] == 5:
            if is_empty(object2):
                object2 = [test2[idx]]
                object2 = np.asarray(object2)

            else:
                object2 = np.append(object2,[test2[idx]],axis = 0)

    #             print(object1)
    #             object1 = object1[n-4].append(pixels)

        elif pixels[4] == 6:
            if is_empty(object3):
                object3 = [test2[idx]]
                object3 = np.asarray(object3)

            else:
                object3 = np.append(object3,[test2[idx]],axis = 0)

    #             print(object1)
    #             object1 = object1[n-4].append(pixels)
        elif pixels[4] == 7:
            if is_empty(object4):
                object4 = [test2[idx]]
                object4 = np.asarray(object4)

            else:
                object4 = np.append(object4,[test2[idx]],axis = 0)

    #             print(object1)
    #             object1 = object1[n-4].append(pixels)
        elif pixels[4] == 8:
            if is_empty(object5):
                object5 = [test2[idx]]
                object5 = np.asarray(object5)

            else:
                object5 = np.append(object5,[test2[idx]],axis = 0)
    return object1,object2,object3,object4,object5
                
def array2PC(abcd):
    abc = PointCloud()


    x = np.array_split(abcd,2,axis = 1)
    # print (np.asarray(x).shape)
    #     print(x[0])
    # x[1] = x[1]*16/255
    abc.points = Vector3dVector(x[0])
    abc.colors = Vector3dVector(x[1])
    return abc

