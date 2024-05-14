from tkinter import Tk     # from tkinter import Tk for Python 3.x
from tkinter.filedialog import askopenfilename
import os.path

import pcu

Tk().withdraw() # Cierra la pestaña de tkinter
filename = askopenfilename() # Obtiene el path del archivo

temp = filename

temp = temp.replace("jpg", "pcd") 

temp1 = filename




# Comentar o descomentar si se desea probar la funcion de puntajes 
#filename2 = askopenfilename()

#temp2 = filename2

#temp2 = temp2.replace("jpg", "pcd") 

#print(pcu.score(filename, filename2))


#Comentar o descomentar si se desea probar el guardado y visualización
# Se verifica si existe el point cloud
if os.path.exists(temp1):
    #pcu.visualizePC(filename)
    pcu.visualizeXYZ(filename)
    
# De no existir se crea el point cloud
else:
    pcu.createPC(filename)




# No funciona le machine learning
#pcu.ML(filename, filename2)