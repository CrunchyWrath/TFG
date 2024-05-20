"""
MIT License

Copyright (c) 2021 porteratzo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import csv
import pclpy
import treetool.seg_tree as seg_tree
import treetool.utils as utils
import treetool.tree_tool as tree_tool
import pandas as pd
import math

# Funcion para obtener la altura de los arboles
# Utiliza la nube de puntos sin suelo y el valor de cada X y Y
# dentro del CSV para buscar el Z mas alto
# resultCSV = Archivo CSV con los resultados obtenidos
# NGP =  nube de puntos non ground points que tiene la nube original sin suelo
# searchRadius = radio de busqueda para el Z mas alto

def getHeight(resultCSV, NGP, searchRadius):
    CSVX = []
    CSVY = []
    CSVZ = 0.0
    CSVHeight = []
    
    with open(resultCSV, 'r') as data:
        csvReader = csv.reader(data)
        csvReader
        csvReader
        for line in csvReader:
            CSVX.append(line[1])
            CSVY.append(line[2])
            
    
    for i in range(1, len(CSVX)):
        for j in range(len(NGP)):
            if (float(NGP[j][0]) - float(CSVX[i]) <= searchRadius and float(NGP[j][0]) - float(CSVX[i]) >= -searchRadius) and (float(NGP[j][1]) - float(CSVY[i]) <= searchRadius and float(NGP[j][1]) - float(CSVY[i]) >= -searchRadius):
                if (float(NGP[j][2]) >= CSVZ):
                    CSVZ = float(NGP[j][2])
                   
        CSVHeight.append(CSVZ)
        CSVZ = 0.0

    df = pd.read_csv(resultCSV)
    df['Height'] = CSVHeight
    df.to_csv(resultCSV, index = False)

# Funcion para obtener el RMSE y el error total de los resultados
# Ademas de crear un Scatter plot y un box plot
# Params
# FileCSV = resultados obtenidos en CSV de las pruebas
# FileTXT = archivos de texto con valores reales de la muestras de control 

def getRMSE(fileCSV, fileTXT):
    CSVDBH = []
    CSVHeight = []
    CSVX = []
    CSVY = []
    RealDBH = []
    RealHeight = []
    simDBH = []
    simHeight = []
    errorDBH = []
    errorHeight = []
    with open(fileCSV, 'r') as data:
        csvReader = csv.reader(data)

        for line in csvReader:
            
            CSVX.append(line[1])
            CSVY.append(line[2])
            CSVDBH.append(line[4])
            CSVHeight.append(line[5])

    with open(fileTXT) as file:
        for line in file:
            for i in range (1, len(CSVDBH)):
                if (float(line.split()[1]) / float(CSVX[i]) > 0.95 and float(line.split()[1]) / float(CSVX[i]) < 1.05) and (float(line.split()[2]) / float(CSVY[i]) > 0.95 and float(line.split()[2]) / float(CSVY[i]) < 1.05):
                    RealDBH.append(float(line.split()[4]))
                    simDBH.append(float(CSVDBH[i]))
                    RealHeight.append(float(line.split()[3]))
                    simHeight.append(float(CSVHeight[i]))
                    break
    
    DBHTotalError = 0
    HeightTotalError = 0
    for i in range(len(simDBH)):
        errorDBH.append( abs((RealDBH[i] - simDBH[i]) / simDBH[i]) * 100 )        
        errorHeight.append( abs((RealHeight[i] - simHeight[i]) / simHeight[i]) * 100 ) 
        DBHTotalError += abs((RealDBH[i] - simDBH[i]) / simDBH[i]) * 100
        HeightTotalError += abs((RealHeight[i] - simHeight[i]) / simHeight[i]) * 100 

 

    MSE_DBH = np.square(np.subtract(RealDBH,simDBH)).mean()
    RMSE_DBH = math.sqrt(MSE_DBH)

    MSE_Height = np.square(np.subtract(RealHeight,simHeight)).mean()
    RMSE_Height = math.sqrt(MSE_Height)

    DBHTotalError = (DBHTotalError / len(errorDBH))
    HeightTotalError = (HeightTotalError / len(errorHeight))

    print("Para")
    print(fileCSV)

    print("Arboles encontrados")
    print(len(simDBH))
     
    print("RMSE_Height")
    print(RMSE_Height)
    print("RMSE_DBH")
    print(RMSE_DBH)

    print("Error DBH")
    print(DBHTotalError)
    print("Error Height")
    print(HeightTotalError)
    print(" ")
  

    plt.title('DBH Box plot')
    plt.ylabel('Error %')
    plt.boxplot(errorDBH)
    plt.show()

    plt.title('Height Box plot')
    plt.ylabel('Error %')
    plt.boxplot(errorHeight)
    plt.show()


    plt.title('DBH Scatter plot')
    plt.xlabel('Real value')
    plt.ylabel('Result value')
    plt.scatter(RealDBH, simDBH)
    plt.show()

    plt.title('Height Scatter plot')
    plt.xlabel('Real value')
    plt.ylabel('Result value')
    plt.scatter(RealHeight, simHeight)
    plt.show()
    
# Funcion con la que se visualiza y se extraen las metricas de los arboles
# image = direccion del archivo .pcd
# HRadius = Radio de busqueda para la altura
# DBHRadius = Radio de busqueda de vecinos para el DBH
# verticality = umbral en radianes para ver la verticalidad de cada punto, esto se obtiene haciendo el producto
# punto entre la normal del punto y el vector vertical (0,0,1)
# curvature = umbral de 0 a 1 para definir la curvatura de cada punto 
# voxelize = tamaño de la hoja a la hora de voxalizar
def visualizeXYZ(image, HRadius, DBHRadius, verticality, curvature, voxelize):

    PointCloud = pclpy.pcl.PointCloud.PointXYZ()

    file_directory = image
    pclpy.pcl.io.loadPCDFile(file_directory,PointCloud)
    PointCloudV = seg_tree.voxelize(PointCloud.xyz, voxelize)
    utils.open3dpaint(PointCloudV, reduce_for_vis = False  , voxel_size = 0.1)

    PointCloudV.shape, PointCloud.xyz.shape

    My_treetool = tree_tool.treetool(PointCloudV)

    print("Paso 1")
    My_treetool.step_1_remove_floor()
    utils.open3dpaint([My_treetool.non_ground_cloud,My_treetool.ground_cloud],reduce_for_vis = True  , voxel_size = 0.1)

    print("Paso 2")
    My_treetool.step_2_normal_filtering(verticality_threshold=verticality, curvature_threshold=curvature, search_radius=DBHRadius)
    utils.open3dpaint([My_treetool.non_ground_cloud.xyz, My_treetool.non_filtered_points.xyz + My_treetool.non_filtered_normals * 0.1, My_treetool.non_filtered_points.xyz + My_treetool.non_filtered_normals * 0.2], reduce_for_vis = True , voxel_size = 0.1)
    utils.open3dpaint([My_treetool.filtered_points.xyz, My_treetool.filtered_points.xyz + My_treetool.filtered_normals * 0.05, My_treetool.filtered_points.xyz + My_treetool.filtered_normals * 0.1], reduce_for_vis = True , voxel_size = 0.1)
    
    print("Paso 3")
    My_treetool.step_3_euclidean_clustering(tolerance=0.1, min_cluster_size=40, max_cluster_size=6000000)
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
    image = image.replace(".pcd",".csv")
    image = image.replace("/Datasets","/Results")
    image = image.replace("/3D-forest-classic-data","/Results")

   
    My_treetool.save_results(save_location = image )
    
    getHeight(image, My_treetool.non_ground_cloud.xyz, HRadius)

    getRMSE("./Results/EURO1.csv", "./realValues/TLS_Benchmarking_Plot_1_LHD.txt")
    #getRMSE("./Results/EURO2.csv", "./realValues/TLS_Benchmarking_Plot_2_LHD.txt")
    #getRMSE("./Results/EURO3.csv", "./realValues/TLS_Benchmarking_Plot_3_LHD.txt")
    #getRMSE("./Results/EURO4.csv", "./realValues/TLS_Benchmarking_Plot_4_LHD.txt")
    getRMSE("./Results/EURO5.csv", "./realValues/TLS_Benchmarking_Plot_5_LHD.txt")
    #getRMSE("./Results/EURO6.csv", "./realValues/TLS_Benchmarking_Plot_6_LHD.txt")


# Como dataset se utilizó el siguiente
# https://github.com/VUKOZ-OEL/3d-forest-classic-data
# https://laserscanning.fi/results-available-for-international-benchmarking-of-terrestrial-laser-scanning-methods/
# https://drive.google.com/drive/folders/1AhYd8pwCrTAJCV4OIc9MWot8tjNgIJSx

# Pagina para transformar de XYZ a PCD
# https://imagetostl.com/convert/file/xyz/to/pcd



"""
Los derechos de autor del uso de la herramienta Treetool pertenecen a porteratzo

Repositorio de la herramienta: https://github.com/porteratzo/TreeTool/tree/master

Copyright (c) 2021 porteratzo
"""