import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
#import tensorflow as tf
#import pymssql as sql
import laspy
from sklearn.metrics import mean_squared_error
from PIL import Image
#import cv2
import csv
import pclpy
import treetool.seg_tree as seg_tree
import treetool.utils as utils
import treetool.tree_tool as tree_tool
import pandas as pd
from scipy.optimize import linear_sum_assignment

def average(l):
    return sum(l) / len(l)

def getRMSE(filename):
    DBH = []
    with open(filename, 'r') as data:
        csvReader = csv.reader(data)

        for line in csvReader:
            DBH.append(line[3])
        
        for i in len(DBH):
            print(DBH[i])
    
    #print("El promedio de DBH es ")
    #print(average(DBH))

    
# Conectarse al servidor
#conn = sql.connect(server='localhost', user='sa', password='Bvf09081999+', database='ProyectoD')
#cursor = conn.cursor(as_dict=True)

jpg = "jpg"

# Funcion con la que se crea la nube de puntos
# image: Direccion de la imagen .jpg
def createPC(image):
    image = image.replace("/home/bradly/Escritorio/proyecto/","")
    color_raw = o3d.io.read_image(image)
    
    print(image)

    image = image.replace("jp","pn")

    print(image)

    depth_raw = o3d.io.read_image(image)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw)
    print(rgbd_image)

    plt.subplot(1, 2, 1)
    plt.title("greyscale")
    plt.imshow(rgbd_image.color)
    plt.subplot(1, 2, 2)
    plt.title("depth image")
    plt.imshow(rgbd_image.depth)
    plt.show()

    pc = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    pc.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
    o3d.visualization.draw_geometries([pc])

    print(pc)
    
    #Guardar
    image = image.replace(".png","")
    o3d.io.write_point_cloud(image + ".pcd",pc) 


# Funcion con la que se visualiza la nube de puntos
# image: Direccion de la imagen .jpg
def visualizePC(image):
    image = image.replace("/home/bradly/Escritorio/proyecto/","")
    #image = image.replace(".jpg","")
   
    pc = o3d.io.read_point_cloud(image)
    print(pc)
    o3d.visualization.draw_geometries([pc])

   
    file1 = open(image, 'r')
    Lines = file1.readlines()
    
    filew = open("./proyecto/prueba2PM.xyz", "a")

    count = 0
    # Strips the newline character
    for line in Lines:
        
        for i in range(0, len(line)):
            
            # Check each char
            # is blank or not
            if line[i] == " ":
                count += 1
                if count == 2 :
                    filew.write(line[i + 1 : i + 34])
                    #print(line[i])
                    
                    
                elif count == 3:
                    filew.write("\n")
        count = 0
    filew.close()


def ubicarArbol(listaNP):
    counterArbol = 0
    counterPts = 0
    while len(listaNP) > 0:
        arbol = "arbolesprueba/arbolprueba" + str(counterArbol) + ".xyz"

        filew = open(arbol, "a")
        deletePos = [0]
        listaPC = [listaNP[0]]
        i = 0
        puntosNP = [listaNP[0][0] + 0.4, listaNP[0][1] + 0.4, listaNP[0][2] + 0.3, listaNP[0][0] - 0.4, listaNP[0][1] - 0.4, listaNP[0][2] - 0.3]

        for i in range(1, len(listaNP)):
            if (listaNP[i][0] < puntosNP[0] and listaNP[i][0] >= puntosNP[3]) and (listaNP[i][1] < puntosNP[1] and listaNP[i][1] >= puntosNP[4]) and (listaNP[i][2] < puntosNP[2] and listaNP[i][2] >= puntosNP[5]):
                deletePos.append(i)
                listaPC.append(listaNP[i])
                counterPts += 1

        listaPC = np.array(listaPC)

        if counterPts >= 500:
            for i in range(0, len(listaPC)):
                filew.write(str(listaPC[i][0]))
                filew.write(" ")
                filew.write(str(listaPC[i][1]))
                filew.write(" ")
                filew.write(str(listaPC[i][2]))
                filew.write("\n")
            filew.close()
            counterArbol += 1
        
        counterPts = 0
        listaNP = np.delete(listaNP, deletePos, 0)


### de https://www.open3d.org/docs/0.9.0/tutorial/Advanced/pointcloud_outlier_removal.html
def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
    #o3d.visualization.draw_geometries([inlier_cloud])


        

# Funcion con la que se visualiza la nube de puntos xyz
# image: Direccion de la imagen .xyz
def visualizeXYZ(image):


    image = image.replace("/home/bradly/Escritorio/proyecto/","")
    #image = image.replace(".jpg","")

    
    #pc = o3d.geometry.PointCloud()
    #pc.points = o3d.utility.Vector3dVector(image)

    #cargar imagen
    pc = o3d.io.read_point_cloud(image)

    #seleccionar json para cortar imagen
    vol = o3d.visualization.read_selection_polygon_volume("cropped.json")
    tree = vol.crop_point_cloud(pc)

    #eliminacion del ruido del point cloud
    cl, ind = tree.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.5)
    
    #clustering del pointcloud 
    #https://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html#Convex-hull
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(tree.cluster_dbscan(eps=0.08, min_points=100, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    tree.colors = o3d.utility.Vector3dVector(colors[:, :3])


    listCrop = np.asarray(tree.points)

    #ubicarArbol(listCrop)
    """
    filew = open("croppedPrueba.xyz", "a")

    i = 0

    for i in range(0, len(listCrop)):
        filew.write(str(listCrop[i][0]))
        filew.write(" ")
        filew.write(str(listCrop[i][1]))
        filew.write(" ")
        filew.write(str(listCrop[i][2]))
        filew.write("\n")
    filew.close()
"""
    
    print("puntos mas largo = ")
    print(np.amax(np.asarray(pc.points), axis=0))
    print("puntos mas corto = " )
    print(np.amin(np.asarray(pc.points), axis=0))
    print("diferencia en X = " )
    x = np.amax(np.asarray(pc.points), axis=0)[0] - np.amin(np.asarray(pc.points), axis=0)[0]
    print(abs(x))   
    print("diferencia en Y = " )
    y = np.amax(np.asarray(pc.points), axis=0)[1] - np.amin(np.asarray(pc.points), axis=0)[1]
    print(abs(y))   
    
    print("relacion entre X y Y = " )
    print(x/y)

    print((np.asarray(pc.points).size // 3))
    #CAMBIAR
    """"
    hdr = laspy.LasHeader(point_format=3, version="1.2")

    allx = []
    ally = []
    allz = []

    for i in range(0, (np.asarray(pc.points).size // 3)):
        allx.append(np.asarray(pc.points)[i][0])
        ally.append(np.asarray(pc.points)[i][1])
        allz.append(np.asarray(pc.points)[i][2])


    xmin = np.floor(np.min(allx))
    ymin = np.floor(np.min(ally))
    zmin = np.floor(np.min(allz))

    hdr.offsets = [xmin,ymin,zmin] 
    hdr.scales = [0.0001,0.0001,0.0001] 

    las = laspy.LasData(hdr)

    las.x = allx
    las.y = ally
    las.z = allz

    las.write("output.las")

    """

    print(np.asarray(pc.points)[1][1])
    #print("valores de la imagen gris: " )
    #img = cv2.imread("image.png", 0)
    #for i in range (img.shape[0]): 
        #for j in range (img.shape[1]): 
            #print(img[i][j])
    #plt.hist(img.ravel(),256,[0,256])
    #plt.show()
    
    PointCloud = pclpy.pcl.PointCloud.PointXYZ()

    file_directory = r'prueba1pm.pcd'
    pclpy.pcl.io.loadPCDFile(file_directory,PointCloud)
    PointCloudV = seg_tree.voxelize(PointCloud.xyz, 0.06)
    utils.open3dpaint(PointCloudV, reduce_for_vis = False  , voxel_size = 0.1)

    PointCloudV.shape, PointCloud.xyz.shape

    My_treetool = tree_tool.treetool(PointCloudV)

    print("Paso 1")
    My_treetool.step_1_remove_floor()
    utils.open3dpaint([My_treetool.non_ground_cloud,My_treetool.ground_cloud],reduce_for_vis = True  , voxel_size = 0.1)

    print("Paso 2")
    My_treetool.step_2_normal_filtering(verticality_threshold=0.04, curvature_threshold=0.06, search_radius=0.12)
    utils.open3dpaint([My_treetool.non_ground_cloud.xyz, My_treetool.non_filtered_points.xyz + My_treetool.non_filtered_normals * 0.1, My_treetool.non_filtered_points.xyz + My_treetool.non_filtered_normals * 0.2], reduce_for_vis = True , voxel_size = 0.1)

    utils.open3dpaint([My_treetool.filtered_points.xyz, My_treetool.filtered_points.xyz + My_treetool.filtered_normals * 0.05, My_treetool.filtered_points.xyz + My_treetool.filtered_normals * 0.1], reduce_for_vis = True , voxel_size = 0.1)

    print("Paso 3")
    My_treetool.step_3_euclidean_clustering(tolerance=0.2, min_cluster_size=40, max_cluster_size=6000000)
    utils.open3dpaint(My_treetool.cluster_list,reduce_for_vis = True  , voxel_size = 0.1)

    print("Paso 4")
    My_treetool.step_4_group_stems(max_distance=0.4)
    utils.open3dpaint(My_treetool.complete_Stems,reduce_for_vis = True  , voxel_size = 0.1)

    print("Paso 5")
    My_treetool.step_5_get_ground_level_trees(lowstems_height=5, cutstems_height=5)
    utils.open3dpaint(My_treetool.low_stems,reduce_for_vis = True  , voxel_size = 0.1)

    print("Paso 6")
    My_treetool.step_6_get_cylinder_tree_models(search_radius=0.1)
    utils.open3dpaint([i['tree'] for i in My_treetool.finalstems] + My_treetool.visualization_cylinders,reduce_for_vis = True  , voxel_size = 0.1)

    print("Paso 7")
    My_treetool.step_7_ellipse_fit()

    My_treetool.save_results(save_location = '/home/bradvm/Descargas/Pointclud1-main/proyecto/resultados.csv')
    #display_inlier_outlier(tree, ind)
    o3d.visualization.draw_geometries_with_editing([tree])
    o3d.visualization.draw_geometries_with_editing([pc])
    
    getRMSE("resultadosPM1.csv")
   
    #downpc = pc.voxel_down_sample(voxel_size=0.05)
    #downpc.estimate_normals(
    #search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
    #print(pc.points)
    #o3d.visualization.draw_geometries([downpc])
    

# Funcion con la que se crea la nube de puntos
# image1: Direccion de la imagen .jpg del LiDAR
# image2: Direccion de la imagen .jpg del sistema
def score(image1, image2):
    image1 = image1.replace("/home/bradly/Escritorio/proyecto/","")
    image1 = image1.replace(".jpg","")

    pc1 = o3d.io.read_point_cloud(image1 + ".pcd")

    image2 = image2.replace("/home/bradly/Escritorio/proyecto/","")
    image2 = image2.replace(".jpg","")

    pc2 = o3d.io.read_point_cloud(image2 + ".pcd")

    lista1 = []
    lista2 = []

    for x in range (len(np.asarray(pc1.points))):
        lista1.append(np.asarray(pc1.points)[x])
        lista2.append(np.asarray(pc2.points)[x])

    MSE = mean_squared_error(lista1, lista2)

    query = "INSERT INTO resultados (Imagen, Resultado) " + "VALUES ('" + image2 + "', " + str(MSE) +")"

# Cuando exista una base de datos se descomenta
    #cursor.execute(query)
    #conn.commit()
    #cursor.close()
    #conn.close()

    return MSE

# Funcion con la que se crea la nube de puntos
# image1: Direccion de la imagen .jpg del LiDAR
# image2: Direccion de la imagen .jpg del sistema
# Por el momento esta seccion no funciona 




# Para utilizar el pymssql 
# Se utilizó la siguiente guia https://learn.microsoft.com/en-us/sql/connect/python/pymssql/step-3-proof-of-concept-connecting-to-sql-using-pymssql?view=sql-server-ver16

# Para la creación, visualización y guardado de nubes de puntos se utilizó lo siguiente
# https://www.youtube.com/watch?v=vGr8Bg2Fda8&list=PLkmvobsnE0GEZugH1Di2Cr_f32qYkv7aN&index=5
# http://www.open3d.org/docs/0.8.0/tutorial/Basic/rgbd_images/redwood.html
# http://www.open3d.org/docs/release/python_api/open3d.io.write_point_cloud.html
# http://www.open3d.org/docs/release/python_api/open3d.visualization.draw_geometries.html

# Para obtener el mean squared error se utilizó
# https://www.geeksforgeeks.org/python-mean-squared-error/

# Para la comprensión e intento de implementar Machine Learning con redes neuronales se utilizó
# https://www.youtube.com/watch?v=7eh4d6sabA0
# https://www.youtube.com/watch?v=iX_on3VxZzk
# https://www.mygreatlearning.com/blog/types-of-neural-networks/

# Como dataset se utilizó el siguiente
# http://redwood-data.org/3dscan/dataset.html

