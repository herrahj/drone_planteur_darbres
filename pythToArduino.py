import numpy as np
import random
from pyfirmata import ArduinoMega
import time
import serial
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,LinearSegmentedColormap



board=ArduinoMega('/dev/cu.usbmodem14101')
sensor= serial.Serial('/dev/cu.usbmodem14201', 9600)

sensx=board.get_pin('d:12:o')
motx=board.get_pin('d:3:p')
sensy=board.get_pin('d:13:o')
moty=board.get_pin('d:11:p')
##Résolution
L=8
##Résolution
X=[];Y=[];Z=[]
## Données initiales
angledroite=1 #a definir
anglemontee=2 #a definir
Carte=np.zeros((L,L))
p=0.4 #pas de vis A MODIFER
sens=1
ytemp=0.
commande=[0,1]
ligne=False
colonne=True
pos=[0,0]
vy=0.3 #vitesse du drone selon y
vx=0.6 #vitesse du drone selon x
dt=0.05 #1 microseconde correspond au temps d'échantillonage Arduino
Vitessex= 1.5 #m.s-1
Vitessey= 2  #m.s-1
## Données initiales

## Programmes simulants le mouvement du drone

    
def setPosInitiale():
    global pos
    sensx.write(1)
    #motx.write(1);
    sensy.write(1)
    moty.write(1);
    time.sleep(20)
    moty.write(0);
    motx.write(0);
    sensy.write(0)
    sensx.write(0)
    pos=[0,0]
    
def retourCaseDepart():
    global pos
    global commande
    global sens
    global ytemp
    global ligne
    global colonne
    (x,y)=pos
    ##
    part=min((x)/Vitessex,(y)/Vitessey)
    sensx.write(0)
    sensy.write(1)
    motx.write(vx)
    moty.write(vy)
    time.sleep(part)
    if part==(x+1)/Vitessex:
        motx.write(0)
        time.sleep((y)/Vitessey-part)
        moty.write(0)
    else:
        moty.write(0)
        time.sleep((x+1)/Vitessex-part)
        motx.write(0)
    ##
    sens=1
    ytemp=0.
    commande=[0,1]
    ligne=False
    colonne=True
    pos=[0,0]
    return pos
    
def Mesure():
    sensor.flushInput()
    while sensor.readline()=='':
        Mesure()
    distance = sensor.readline()
    return 14-float(distance)
    
def setPosFinale():
    sensx.write(0)
    motx.write(1);
    sensy.write(0)
    moty.write(1);
    time.sleep(20)
    moty.write(0);
    motx.write(0);
    sensy.write(0)
    sensx.write(0)
    
#def longueurX():
    #start = time.time()
    
    #end = time.time()
    #print(end - start)
    
def movex(): #fait déplacer le drone fictif sur l'axe des x
    global Carte
    global pos
    global X
    global Y
    global Z
    
    #pos=getPos()
    (x,y)=pos
    if (int(x)<L) :
        sensx.write(1)
        motx.write(vx)
        time.sleep(dt)
        motx.write(0)
        pos[0]+=dt*Vitessex*vx
    z=max(Mesure(),0)
    X.append(x);Y.append(y);Z.append(z)
    return pos

def movey(): #fait déplacer le drone fictif sur l'axe des y
    
    global Carte
    global pos
    global sens
    global ytemp
    global X
    global Y
    global Z
    
    #pos=getPos()
    (x,y)=pos
    ytemp=y
    if (0<=y<L-1) and sens==1:
        sensy.write(0)
        moty.write(vy)
        time.sleep(dt)
        moty.write(0)
        pos[1]+=dt*Vitessey*vy
    if (1<=y<L) and sens==-1:
        sensy.write(1)
        moty.write(vy)
        time.sleep(dt)
        moty.write(0)
        pos[1]-=dt*Vitessey*vy
    z=max(Mesure(),0)
    X.append(x);Y.append(y);Z.append(z)
    return pos
## Programmes simulants le mouvement du drone  
    
def estDansCase(): #Verifie que la commande correspond à la sortie
    
    global pos
    global commande
    
    (x,y)=pos;(xc,yc)=commande
    return int(x)==xc and int(y)==yc
    

def mouvementligne(): #fait changer la commande en accord avec la position du drone
    
    global pos
    global sens
    global ytemp
    
    (x,y)=pos
    if ytemp!=y and x<L-1 and (int(y)==0 or int(y)==L-1):
        commande[0]+=1
        while (not estDansCase()): #tant que le drone n'est pas dans la case de commande, on attend
            #anglex=0     #a utiliser sur le drone réel
            #angley=angledroite    #a utiliser sur le drone réel
            pos=movex()
        ytemp=y #casse la boucle pour que mouvementligne ne se relance pas
        sens=-sens  #on inverse le sens
        commande[1]+=sens
        print ('commande' + str(commande))
        
def mouvementcolonne():
    
    global pos
    global sens
    global commande
    
    #angley=0    #a utiliser sur le drone réel
    y=pos[1]
    while (not estDansCase()) and 0<=y<L: #tant que le drone n'est pas dans la case de commande, on attend
        pos=movey()
        #anglex=sens*anglemontee #a utiliser sur le drone réel
    if 1<=commande[1]<=L-2:
        commande[1]+=sens
    

        

#def carteEstSondee(): #on verfie que toute les cases de la carte sont sondées
    #for i in range (L):
        #for j in range (l):
            #if not ijEstSondee(i,j):
                #return False
    #return True
        
def setCarte(x,y,z,m):
    global Carte
    n=len(x)
    for i in range (n):
        if z[i]>m:
            Carte[int(x[i]),int(y[i])]=0
        else:
            Carte[int(x[i]),int(y[i])]=1
            
def sondage():
    
    global X
    global Y
    global Z
    global Carte
    
    for i in range (L*L-1):
        mouvementcolonne()
        mouvementligne()
    retourCaseDepart()
    M=sum(Z)/len(Z)
    maxz=max(Z)
    sol=np.array([[0,1,0],[1,0,0]])
    sol=ListedColormap(sol)
    fig=plt.figure()
    ax=fig.add_subplot(projection='3d')
    surf=ax.plot_trisurf(X,Y,Z,cmap = sol,vmin=M-maxz,vmax=M+maxz)
    bar=fig.colorbar(surf, shrink=0.5, aspect=5)
    bar.set_label("Limite de plantage : " +str("%.2f" % M))
    plt.show()
    plt.savefig('scan3D.png',bbox_inches='tight')
    setCarte(X,Y,Z,M)
    
    






