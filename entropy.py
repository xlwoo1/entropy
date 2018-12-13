from open3d import *
from util import *
from pc_util import *
import numpy as np
import copy
import time

def base(voxel_size,source,target):
    source, target, source_down, target_down, source_fpfh, target_fpfh, radius = \
            prepare_dataset(voxel_size,source,target)

    result_ransac = execute_fast_global_registration(source_down, target_down,
            source_fpfh, target_fpfh, voxel_size)
#     print(result_ransac)
#     draw_registration_result(source_down, target_down,
#             result_ransac.transformation)

    result_icp = refine_registration(source_down, target_down,
            source_fpfh, target_fpfh, voxel_size, radius,result_ransac.transformation)
    
    theta_x = np.arctan(result_icp.transformation[2,1]/result_icp.transformation[2,2])
    theta_y = -np.arcsin(result_icp.transformation[2,0])
    theta_z = np.arctan(result_icp.transformation[1,0]/result_icp.transformation[0,0])
    translation = result_icp.transformation[:3,3]
    # print(result_icp)
    # draw_registration_result(source, target, result_icp.transformation)

    return result_icp,theta_x,theta_y,theta_z,translation

def maxTrans(abc):
    cde=np.asarray(abc.points)
# print(cde)
# print(cde[:,0])
    minx = np.amin(cde[:,0])
    miny = np.amin(cde[:,1])
    minz = np.amin(cde[:,2])

    maxx = np.amax(cde[:,0])
    maxy = np.amax(cde[:,1])
    maxz = np.amax(cde[:,2])

    eud = np.sqrt((maxx-minx)**2+(maxy-miny)**2+(maxz-minz)**2)
    return eud


def process1(seg1,seg2,rgb1,rgb2):    # voxel_size = 0.003 # means 5cm for the dataset

    pcdArrayS1,pcdArrayR1 = iVal(seg1,rgb1)
    pcdArrayS2,pcdArrayR2 = iVal(seg2,rgb2)

    object11,object12,object13,object14,object15=fillObjectArray(pcdArrayS1,pcdArrayR1)
    object21,object22,object23,object24,object25=fillObjectArray(pcdArrayS2,pcdArrayR2)
    # object31,object32,object33,object34,object35=fillObjectArray(pcdArrayS3,pcdArrayR3)
    # object41,object42,object43,object44,object45=fillObjectArray(pcdArrayS4,pcdArrayR4)
    # object51,object52,object53,object54,object55=fillObjectArray(pcdArrayS5,pcdArrayR5)
    # object61,object62,object63,object64,object65=fillObjectArray(pcdArrayS6,pcdArrayR6)
    # object71,object72,object73,object74,object75=fillObjectArray(pcdArrayS7,pcdArrayR7)

              
    object11pc = array2PC(object11)
    object12pc = array2PC(object12)
    object13pc = array2PC(object13)
    object14pc = array2PC(object14)
    object15pc = array2PC(object15)

    object21pc = array2PC(object21)
    object22pc = array2PC(object22)
    object23pc = array2PC(object23)
    object24pc = array2PC(object24)
    object25pc = array2PC(object25)
  
    voxel_size = 0.003 # means 5cm for the dataset


    _,x1,y1,z1,t1 = base(voxel_size,object11pc,object21pc)
    _,x2,y2,z2,t2 = base(voxel_size,object12pc,object22pc)
    _,x3,y3,z3,t3 = base(voxel_size,object13pc,object23pc)
    _,x4,y4,z4,t4 = base(voxel_size,object14pc,object24pc)
    _,x5,y5,z5,t5 = base(voxel_size,object15pc,object25pc)
    
    x = np.asarray([x1,x2,x3,x4,x5])
    y = np.asarray([y1,y2,y3,y4,y5])
    z = np.asarray([z1,z2,z3,z4,z5])
    t = np.concatenate([[t1],[t2],[t3],[t4],[t5]],axis = 0)
    print(x,y,z,t)
    return x,y,z,t

def entropyExt(x,y):
    x = np.absolute(x)
    xprob = x/y
    hx = (np.log((2*xprob)+1))
    for idx,value in enumerate(hx):
        if value>1:
            hx[idx] = 0
    hx = sum(hx)
    return hx
# lx = np.where(xprob>=0,np.log(xprob),0)
# hx = -sum(xprob*lx)
# print(x,xprob,hx)#,lx,hx)
def entropyMain(x,y,z,t):
    transTest = np.linalg.norm(t,axis = 1)
    # print(transTest)
    hx = entropyExt(x,3.14)
    hy = entropyExt(y,3.14)
    hz = entropyExt(z,3.14)
    ht = entropyExt(transTest,maxt)
    # print(hx,hy,hz,ht)
    return hx,hy,hz,ht

if __name__ == "__main__":
    start = time.time()

    seg1 = read_point_cloud("seg1.pcd")
    seg2 = read_point_cloud("seg2.pcd")
    seg3 = read_point_cloud("seg3.pcd")
    seg4 = read_point_cloud("seg4.pcd")
    seg5 = read_point_cloud("seg5.pcd")
    seg6 = read_point_cloud("seg6.pcd")
    seg7 = read_point_cloud("seg7.pcd")

    # rgb0 = read_point_cloud("rgb0.pcd")
    rgb1 = read_point_cloud("rgb1.pcd")
    rgb2 = read_point_cloud("rgb2.pcd")
    rgb3 = read_point_cloud("rgb3.pcd")
    rgb4 = read_point_cloud("rgb4.pcd")
    rgb5 = read_point_cloud("rgb5.pcd")
    rgb6 = read_point_cloud("rgb6.pcd")
    rgb7 = read_point_cloud("rgb7.pcd")

    pcdArrayS1,pcdArrayR1 = iVal(seg1,rgb1)

    object11,object12,object13,object14,object15=fillObjectArray(pcdArrayS1,pcdArrayR1)
    
    object11pc = array2PC(object11)
    object12pc = array2PC(object12)
    object13pc = array2PC(object13)
    object14pc = array2PC(object14)
    object15pc = array2PC(object15)

    maxt1 = maxTrans(object11pc)
    maxt2 = maxTrans(object12pc)
    maxt3 = maxTrans(object13pc)
    maxt4 = maxTrans(object14pc)
    maxt5 = maxTrans(object15pc)
           
    maxt = np.concatenate([[maxt1],[maxt2],[maxt3],[maxt4],[maxt5]])
    x1,y1,z1,t1 = process1(seg1,seg2,rgb1,rgb2)
    x2,y2,z2,t2 = process1(seg2,seg3,rgb2,rgb3)
    x3,y3,z3,t3 = process1(seg3,seg4,rgb3,rgb4)
    x4,y4,z4,t4 = process1(seg4,seg5,rgb4,rgb5)
    x5,y5,z5,t5 = process1(seg5,seg6,rgb5,rgb6)
    x6,y6,z6,t6 = process1(seg6,seg7,rgb6,rgb7)
    hx1,hy1,hz1,ht1 = entropyMain(x1,y1,z1,t1)
    hx2,hy2,hz2,ht2 = entropyMain(x2,y2,z2,t2)
    hx3,hy3,hz3,ht3 = entropyMain(x3,y3,z3,t3)
    hx4,hy4,hz4,ht4 = entropyMain(x4,y4,z4,t4)
    hx5,hy5,hz5,ht5 = entropyMain(x5,y5,z5,t5)
    hx6,hy6,hz6,ht6 = entropyMain(x6,y6,z6,t6)

    hx = hx1+hx2+hx3+hx4+hx5+hx6
    hy = hy1+hy2+hy3+hy4+hy5+hy6
    hz = hz1+hz2+hz3+hz4+hz5+hz6
    ht = ht1+ht2+ht3+ht4+ht5+ht6

    print(time.time()-start)
    print(hx,hy,hz,ht)


    


    

    
