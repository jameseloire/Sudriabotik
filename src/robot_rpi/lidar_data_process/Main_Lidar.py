
import math
import rospy
import numpy
import serial
import serial.tools.list_ports
#import time

from sensor_msgs.msg import LaserScan
rospy.init_node('Node')

def com_uart():
	ports=list(serial.tools.list_ports.comports())
	print(ports)
	
	for p in ports:
		
		if 'USB' in p.description:
			
			
			mData=serial.Serial(p.device,1000000)
			#print(mData.is_open)
			print(mData.name)
			#print(mData)
			#print("test")
			Data=com_uart()
			#print(Data)
			line=str(Data.readline())
			#print('test3')
			print(line)
			return mData.name
			


def envoie_donnee(port,donnee):
	COM=port
	BAUD=1000000
	
	donnee_conv=bytes(donnee,"utf-8")
	ser=serial.Serial(COM,BAUD,timeout=0.1)
	ser.write(donnee_conv)



def distance(x,y,angle):
    test=0
    teta=angle*math.pi/180
    
    x_plateau=1.8
    y_plateau=1.1
    
    if angle<90 and angle>=0:
        distance1=(x_plateau-x)/math.cos(teta)                   #calcul hypothenuse par rapport a teta et x    
        distance2=(y_plateau-y)/math.cos((math.pi/2)-teta)      #calcul hypothenuse par rapport a pi/2-teta et y 
        #print("distance1:",distance1)
        #print("distance2:",distance2)
        
        
        compare1=distance1*math.sin(teta)               #calcul distance coté opposé de teta 
        compare2=distance2*math.sin((math.pi/2)-teta)   #calcul distance coté opposé de pi/2-teta 
        
        if (2-y)>compare1:       
            distance=distance1
            test=test+1
            
        if (3-x)>compare2:
            distance=distance2
            test=test+1
            
        if compare1==compare2:
            distance=distance1
            test=test+1
            
        if test>1:
        	distance=distance1   
            #return "probleme code"
        
    if angle >= 90 and angle < 180:
        
        distance1=x/math.cos(math.pi-teta)
        distance2=(y_plateau-y)/math.cos(teta-(math.pi/2))
        
        #print("distance1:",distance1)
        #print("distance2:",distance2)
        
        compare1=distance1*math.sin(math.pi-teta)               #calcul distance coté opposé de teta 
        compare2=distance2*math.sin(teta-(math.pi/2))   #calcul distance coté opposé de pi/2-teta 
    
        if (2-y)>compare1:
            distance=distance1
            test=test+1
        
        if x>compare2:
            distance=distance2
            test=test+1
            
        if compare1==compare2:
            distance=distance1
            test=test+1
            
        if test>1:   
        	distance=distance1    
             #return "probleme code"
         
    

    if angle >= 180 and angle < 270:
        distance1=x/math.cos(teta-(math.pi))
        distance2=y/math.cos(3/2*math.pi-teta)
        
        #print("distance1:",distance1)
        #print("distance2:",distance2)
        
        compare1=distance1*math.sin(teta-(math.pi))               #calcul distance coté opposé de teta 
        compare2=distance2*math.sin(3/2*math.pi-teta)   #calcul distance coté opposé de pi/2-teta 
        
        
        if y>compare1:
            distance=distance1
            test=test+1
        
        if x>compare2:
            distance=distance2
            test=test+1
            
        if compare1==compare2:
            distance=distance1
            test=test+1
            
        if test>1:    
        	distance=distance1   
             #return "probleme code"
         
            
    if angle >= 270 and angle < 360:
        
        distance1=y/math.cos(teta-3/2*math.pi)
        distance2=(x_plateau-x)/math.cos(2*math.pi-teta)
        
        #print("distance1:",distance1)
        #print("distance2:",distance2)
        
        
        compare1=distance1*math.sin(teta-3/2*math.pi)     #calcul distance coté opposé de teta 
        compare2=distance2*math.sin(2*math.pi-teta)      #calcul distance coté opposé de pi/2-teta 
    
        
        if 3-x>compare1:
            distance=distance1
            test=test+1
        
        if y>compare2:
            distance=distance2
            test=test+1
            
        if compare1==compare2:
            distance=distance1
            test=test+1
            
        if test>1:
        	distance=distance1    
             #return "probleme code"
        
       
    return distance
	

def position_robot(liste,angle):
    stop=0
    test=0
    milieu=0
    robot1=[]
    n=0
    for i in range(0,len(liste)):
        if n==0:
            if stop==0:
                
                if liste[i]!='None':
                   
                    robot1.append(liste[i])
                    
                    while test==0:
                        memoire_debut=i
                        test=1
                
                if test==1 and liste[i]=='None':
                    memoire_fin=i-1
                    stop=1
                if i == len(liste)-1:
                    if liste[i]!='None':
                        
                        memoire_fin=i
                
                if i<(len(liste)-1):        
                	if liste[i+1]!='None' and liste[i]!='None':
		                #print('titi')
		                if liste[i]>(liste[i+1]+0.1) or liste[i]<(liste[i+1]-0.1):
		                        n=1
		                        memoire_fin=i
    
    distance_robot1=sum(robot1)/len(robot1)
    
    milieu=memoire_debut+((memoire_fin-memoire_debut)/2)
    
    milieu=round(milieu)
    
    angle_robot1=milieu*0.7894+angle
    
    return distance_robot1,angle_robot1,milieu,memoire_debut,memoire_fin


def remplacement(liste):
    testeur=0
    
    for i in range(0,len(liste)):
        if testeur==0:
            temp=liste[i]
            if liste[i]!='None':
                liste[i]='None'
                
                if i<len(liste)-1:
                    if liste[i+1]=='None':
                        testeur=1
                        
                    if liste[i+1]!='None':
                        if temp>(liste[i+1]+0.1) or temp<(liste[i+1]-0.1):
                            testeur=1
    return liste





def def_CO(xR, yR, theta, D_L):
    
    #0 Convertir les angles de degré à radians
    if(theta == 360):
        theta=0
    theta = theta * (math.pi / 180)
    
    #0.2 Convertion des distances de mètres en mm
    xR  *= 100
    yR  *= 100
    D_L *= 100

    #1 Trouver les co de E dans le repère du robot
    #le repère robot a la même orientation que le repère Plateau de jeu
    xE_R = 0
    yE_R = 0

    #le theta utilisé est celui du LIDAR, qui tourne dans le sens horaire
    if(0 <= theta < math.pi/2):
        yE_R =  D_L*math.sin(theta)
        xE_R =  D_L*math.cos(theta)

    elif(math.pi/2 <= theta < math.pi):
        yE_R =  D_L*math.sin(theta)
        xE_R = -D_L*math.cos(theta)
    
    elif(math.pi <= theta < (3*math.pi)/2):
        yE_R = -D_L*math.sin(theta)
        xE_R = -D_L*math.cos(theta)

    elif((3*math.pi)/2 <= theta <= math.pi*2):
        yE_R = -D_L*math.sin(theta)
        xE_R =  D_L*math.cos(theta)

    #2 trouver les coordonnées de xE et Ye en les translatant 
    # vers l'origine du plateau de jeu
    xE = 0
    yE = 0

    if(0 <= theta < math.pi/2):
        xE = xR + xE_R
        yE = yR + yE_R
    
    elif(math.pi/2 <= theta < math.pi):
        xE = xR - xE_R
        yE = yR + yE_R

    elif(math.pi <= theta < (3*math.pi)/2):
        xE = xR - xE_R
        yE = yR - yE_R
    
    elif((3*math.pi)/2 <= theta < 2*math.pi):
        xE = xR + xE_R
        yE = yR - yE_R

    Co_E =[xE, yE]
    return (Co_E)



def angle_mort(liste):
	debut_angle_mort=210 #220
	fin_angle_mort=242 #238
	total=fin_angle_mort-debut_angle_mort
	while debut_angle_mort!=fin_angle_mort+1:
		liste[debut_angle_mort]="None"
		debut_angle_mort+=1
	return liste
	
	


def main(msg):
    x_robot=0.1
    y_robot=0.1
    teta_robot=90.0
    #time.sleep(1)
    #envoie_donnee("/dev/ttyUSB1","pos get rob\r")
    chaine_pos=""
    
    #ser=serial.Serial("/dev/ttyUSB1","115200",timeout=0.1)
    #envoie_donnee("/dev/ttyUSB1","pos get rob\r")
    #while len(chaine_pos)<45:
    	#print("test")
    	#envoie_donnee("/dev/ttyUSB1", "pos get rob\r")
    	#chaine_pos=chaine_pos+ser.read().decode("utf-8")
    	#CHAINE=CHAINE+chaine_pos.decode("utf-8")
    	#print(chaine_pos)
    #print(chaine_pos)

    ### treter la tramme pour pos x y z ;;;
    if len(chaine_pos)>=45:
    	x_robot=chaine_pos[3:12]
    	y_robot=chaine_pos[19:28]
    	teta_robot=chaine_pos[38:45]
    	
    	float(x_robot)
    	float(y_robot)
    	float(teta_robot)
    ###
       
    teta_lidar=teta_robot
    tableau_data=msg
    donnee_finale=["lidar1"]
    liste_data=list(tableau_data.ranges)
    
    #liste_data=angle_mort(liste_data)
    #port_name=def_com()
    #print(liste_data)
    
    for i in range(0,len(liste_data)):
    	x=numpy.isnan(liste_data[i])
    	if x==True:
            liste_data[i]='None'
            
   # print (liste_data)        
    liste_data=angle_mort(liste_data)  
    #print (liste_data)  
   
             
    
    for i in range(0,len(tableau_data.ranges)):
        if (teta_lidar<360):  
            L_max=distance(x_robot,y_robot,teta_lidar)
            teta_lidar=teta_lidar+0.7894
        
        if (teta_lidar>=360):
            L_max=distance(x_robot,y_robot,teta_lidar-360)
            teta_lidar=teta_lidar+0.7894
            
        if tableau_data.ranges[i]!="nan":
            convert=float(tableau_data.ranges[i])
            
            if L_max<convert:
                liste_data[i]='None'
                
    #for i in range(0,len(liste_data)):        
    	#if i>=1 and i<len(liste_data)-1:
    		#if liste_data[i-1]!="None" and liste_data[i+1]!="None":
    			#liste_data[i]=liste_data[i-1]
    
     
    
   
         
   
    suppression=0
    #print(liste_data[len(liste_data)-2])
   
    if liste_data[len(liste_data)-2]!="None" and (liste_data[0])!="None":
    	suppression=1
    
    tmp=0
    tab_dist=[]
    tab_angle=[]
    
    while tmp!=len(liste_data):
        
        tmp=0 
        pos_mid=position_robot(liste_data,teta_robot)
        liste_remp=liste_data
        vide=remplacement(liste_remp)
        
        tab_dist.append(pos_mid[0])
        if pos_mid[1]<360:    
        	tab_angle.append(pos_mid[1])
        if pos_mid[1]>=360:
        	tab_angle.append(pos_mid[1]-360)
        
        
        
        for i in range(0,len(vide)):
        
            if vide[i]=='None':
                
                tmp=tmp+1
    #print(vide)
    #print(pos_mid)
    #print(tab_dist)
    #print(tab_angle)
    #print(liste_remp[len(liste_remp)-3])
    if suppression==1:
    	tab_dist.pop()
    	tab_angle.pop()
    
    #print(tab_dist)
    #print(tab_angle)
    
    msg_final="comp1 all"
    compteur_msg=0
    if len(tab_dist)==len(tab_angle):
        for i in range(0,len(tab_dist)):
            pos_final=def_CO(x_robot,y_robot,tab_angle[i],tab_dist[i])
            #donnee_finale.append(pos_final)
            #print('pos finale: -->',pos_final)
            msg_final=msg_final+" "+str(int(pos_final[0]))+" "+str(int(pos_final[1]))
            compteur_msg=compteur_msg+2
            
            
    if compteur_msg==4:
    	msg_final=msg_final+" 0 0"
    
    if compteur_msg==2: 
    	msg_final=msg_final+" 0 0 0 0"
    	
    if compteur_msg==0:
    	msg_final=msg_final+" 0 0 0 0 0 0"
	   
    
    msg_final+="\r"
    print(msg_final)
    #envoie_donnee("/dev/ttyUSB1",msg_final)
    
    
sub=rospy.Subscriber('/scan',LaserScan,main)
rospy.spin()    
    	
    
    

        
