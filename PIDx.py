Kp=1
Kd=1
Ki=1
def movex(C,posPrec,err):#C est une liste [xc,yc], err=sum(xc-x)
    global Carte
    global pos
    global X
    global Y
    global Z
    [xc,yc]=posPrec
    pos=getPos() #renvoyé par le capteur de position
    (x,y)=pos
    err+=(xc-x)
    if xc>x: #le moteur avance
        sensx.write(0)
        motx.write(min(abs(vx*(Kp*(xc-x)+Kd*(xc-x)/dt+Ki*err)),1))
    if xc<x: #le moteur recule
        sensx.write(1)
        motx.write(min(abs(vx*(Kp*(xc-x)+Kd*(xc-x)/dt+Ki*err)),1))
    else:
        pass
    Carte[int(x),int(y)]=estPlantable()
    z=Mesure()
    X.append(x);Y.append(y);Z.append(z)
    return pos
