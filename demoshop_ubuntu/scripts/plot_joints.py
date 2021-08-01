#! /usr/bin/python

import rospy, math
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class JointRecorder(object):
    def __init__(self):
	self.joint_data = np.zeros((7,1))
	self.counter = 1;
	rospy.Subscriber("/joint_states", JointState, self.callback)

    def callback(self, msg):
	self.counter += 1
	#emgArr = msg.data
	currdata = np.asarray(msg.position).reshape((7,1))
	# currdata = np.transpose(currdata)
	# print(currdata)
	self.joint_data = np.concatenate((self.joint_data, currdata), axis = 1)

    def Plot(self):
	fig=plt.figure()
        # fig.show()
        ax=fig.add_subplot(111)
	print(self.counter, self.joint_data.shape)
        ax.plot(np.arange(self.counter),self.joint_data[0],color='b',label = "Shoulder Pan")
        ax.plot(np.arange(self.counter),self.joint_data[1],color='r',label = "Shoulder Lift")
        ax.plot(np.arange(self.counter),self.joint_data[2],color='g',label = "Elbow")
        ax.plot(np.arange(self.counter),self.joint_data[3],color='c',label = "Wrist 1")
        ax.plot(np.arange(self.counter),self.joint_data[4],color='m',label = "Wrist 2")
        ax.plot(np.arange(self.counter),self.joint_data[5],color='y',label = "Wrist 3")
        ax.plot(np.arange(self.counter),self.joint_data[6],color='k',label = "Finger")
	plt.legend(loc='upper left')
        plt.draw()
	fig.savefig('/home/gopika/joint_plots/output.png') 

if __name__ == '__main__':
    rospy.init_node('joint_recorder')
    #rospy.init_node('myo-raw-Plot', anonymous=True)
    
    # Publish to the turtlesim movement topic
    '''
    myodata = np.zeros((8,1))
    counter = 0;
   
    def strength(emgArr1):
	emgArr=emgArr1.data
	# Define proportional control constant:
	K = 0.005
	currdata = np.asarray(emgArr)
	myodata = np.concatenate((myodata, currdata), axis = 1)
	
    counter += 1
    rospy.Subscriber("/myo_raw/myo_emg", EmgArray, strength)
    '''
    my_node = JointRecorder()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    my_node.Plot()
