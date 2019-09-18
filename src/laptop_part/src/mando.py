# Import libraries
import rospy
import socket
from struct import *
from sensor_msgs.msg import Joy
from benjie.msg import *


def callback(data):
	
	global Start 
	global Select 
	global X 
	global pulsadoStart
	global pulsadoSelect 
	global pulsadoX 
	global cambio
   
	# If start button is pushed, change the actual state
	if data.buttons[7] == 1 and pulsadoStart == 0:
		pulsadoStart = 1
		Start = not Start
		cambio = 1
	if data.buttons[7] == 0:
		pulsadoStart = 0
	# If select button is pushed, change the actual state
	if data.buttons[6] == 1 and pulsadoSelect == 0:
		pulsadoSelect = 1
		Select = not Select
		cambio = 1
	if data.buttons[6] == 0:
		pulsadoSelect = 0
	# If X button is pushed, change the actual state	
	if data.buttons[2] == 1 and pulsadoX == 0:
		pulsadoX = 1
		X = not X
		cambio = 1
	if data.buttons[2] == 0:
		pulsadoX = 0
	# Convert joysticks movement to velocity of the motors      
	velF = 100*data.axes[1]
	velD = 100*data.axes[2]
	velm1 = velF - velD	
	velm0 = velF + velD # 1 = Left. 0 = Right
	# Saturate the velocity of the motors  
	if velm1 > 100:
	   velm1 = 100
	if velm1 < -100:
	   velm1 = -100
	if velm0 > 100:
	   velm0 = 100
	if velm0 < -100:
	   velm0 = -100
	# Create a deadzone between -10 and 10
	if velm1 > -10 and velm1 < 10:
	   velm1 = 0
	if velm0 > -10 and velm0 < 10:
	   velm0 = 0
	# Print the velocity of the motors  on the screen  
	rospy.loginfo("Motor I:"+str(int(velm1)) + " Motor Der:"+str(int(velm0)))
	# Convert velocity to two's complement 
	if velm0 < 0:
	   velm0 = 256 - abs(velm0) 
	if velm1 < 0:
	   velm1 = 256 - abs(velm1)
	# Send X, start or select if some of them were pushed
	if cambio == 1:
		s.sendall(pack('ccccQh',chr(2),chr(Start),chr(Select),chr(X),0,0))
		rospy.loginfo("Paquete"+str(2)+str(Start)+str(Select)+str(X))
	else:
		# if not, send velocities 
		s.sendall(pack('cccQhc',chr(1),chr(int(velm0)),chr(int(velm1)),0,0,chr(0)))
	cambio = 0
	 
 
def callbackKinect(arr):
		# Send markers info
        if (len(arr.marcadorArray) == 2):
			msg = pack('cc',chr(4),chr(5))
			if (arr.marcadorArray[0].id == 2): 
				msg = msg + pack('<hhh',arr.marcadorArray[0].cx,arr.marcadorArray[0].cy,arr.marcadorArray[0].alpha)
				msg = msg + pack('<hhh',arr.marcadorArray[1].cx,arr.marcadorArray[1].cy,arr.marcadorArray[1].alpha)
			else:
				msg = msg + pack('<hhh',arr.marcadorArray[1].cx,arr.marcadorArray[1].cy,arr.marcadorArray[1].alpha)
				msg = msg + pack('<hhh',arr.marcadorArray[0].cx,arr.marcadorArray[0].cy,arr.marcadorArray[0].alpha)
        elif(arr.marcadorArray[0].id == 2):
			msg = pack('cc',chr(3),chr(5))
			msg = msg + pack('<hhh',arr.marcadorArray[0].cx,arr.marcadorArray[0].cy,arr.marcadorArray[0].alpha)
			msg = msg + pack('<hhh',0,0,0)
		s.send(msg)

	
	

def start():
	global pub
	global s
	global Start 
	global Select 
	global X 
	global pulsadoStart
	global pulsadoSelect 
	global pulsadoX 
	global cambio
	Start = 0
	Select = 0
	X = 0
	pulsadoStart = 0
	pulsadoSelect = 0
	pulsadoX = 0
	cambio = 0
	# Init a socket
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	# IP with the WiFi of the robot
	ip = "192.168.1.1"
	port = 1000
	rospy.loginfo("Conectando\n")
	s.connect((ip,port))
	rospy.loginfo("Conectado")
 

	rospy.Subscriber("joy", Joy, callback)
	rospy.Subscriber("SEPA_node", MarcadorArray , callbackKinect)
	rospy.init_node('Joy2SEPA')
	rospy.spin()
 
if __name__ == '__main__':
	start()
