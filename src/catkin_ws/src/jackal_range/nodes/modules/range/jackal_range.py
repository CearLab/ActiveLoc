#!/usr/bin/env python
# license removed for brevity

# general import
import rospy
import actionlib
import serial
import time
import sys
import message_filters 
import math
import roslib
import random
import numpy as np
from collections import deque
import tf_conversions
import tf2_ros
import colorsys

# message import
import geometry_msgs.msg
from geometry_msgs.msg import Point
import nav_msgs.msg
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

# custom message import
from jackal_range.msg import RD_recap as RD

# UWB range
RANGE = 100

class JackalRange:
    # SERIAL BUFFER LENGTHS
    BUFSIZE_SET = 6
    BUFSIZE_GET = 4

    # anchors ID dictionary
    ID = ['AN0','AN1','AN2','AN3']    
    
    # rate
    RATE = 10 #(Hz)

    def __init__(self):
        # number of anchors
        self.NUM_A = len(self.ID)
        
        # get namespace
        self.namespace_handler(None)
        
        # define transform
        self.trans = np.array([0.0, 0.0, 0.0])
        self.rot = np.array([0.0, 0.0, 0.0, 1.0])   
        matrix = tf_conversions.transformations.quaternion_matrix(self.rot)
        self.DCM = matrix[:3,:3]
        self.trans = np.array(self.trans).T                

        # general anchors_params with anchor positions
        self.anchors_params = [
            ['UKN', self.ID[0], 0.0, 0.0, 0.0],
            ['UKN', self.ID[1], 0.0, 0.0, 0.0],
            ['UKN', self.ID[2], 0.0, 0.0, 0.0],
            ['UKN', self.ID[3], 0.0, 0.0, 0.0]
        ]

        # general anchors_params with anchor positions
        self.tag_params = [
            ['UKN', self.ID[0], 0.0],
            ['UKN', self.ID[1], 0.0],
            ['UKN', self.ID[2], 0.0],
            ['UKN', self.ID[3], 0.0]
        ]

    # write on serial
    # You can add methods for writing on serial here
    
    # define timers
    def timers(self):        
        # Create a buffer and a listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))  # 10 seconds cache time
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback_tf)            
    
    # get the namespace
    def namespace_handler(self, NS):                 
        
        # get the namespace        
        if NS is None:
            self.namespace = rospy.get_namespace()
            rospy.loginfo('got namespace from ros: {}'.format(self.namespace))
        elif NS.startswith('_'):            
            NS = None
            rospy.loginfo('namespace is private, using ROS namespace')
        else:
            self.namespace = NS
            rospy.loginfo('got namespace from args: {}'.format(self.namespace))                    
                
        # remove all leading and trailing slashes
        self.namespace = self.namespace.strip('/')
        rospy.loginfo('final name space: {}'.format(self.namespace))    

    # callback for the transform
    def timer_callback_tf(self,event):                
            
        # define frames of the transformation
        target_frame = f"{self.namespace}/base_link"
        source_frame = f"{self.namespace}/right_tag"        
        rospy.loginfo_once('source_frame: {}'.format(source_frame))
        rospy.loginfo_once('target_frame: {}'.format(target_frame))
        try:          
            
            # Define target time for the transformation
            target_time = rospy.Time.now() - rospy.Duration(1.0)  # 1 seconds ago  
            
            # Lookup the transform, allowing extrapolation
            transform = self.tf_buffer.lookup_transform(
                target_frame,  # target frame
                source_frame,  # source frame
                target_time,     # time at which you want the transform
                rospy.Duration(1.0)  # timeout for looking up the transform
            )               
            self.trans = np.array(( transform.transform.translation.x, 
                                    transform.transform.translation.y, 
                                    transform.transform.translation.z))
            self.rot = np.array((   transform.transform.rotation.x,
                                    transform.transform.rotation.y,
                                    transform.transform.rotation.z,
                                    transform.transform.rotation.w))
            rospy.loginfo_once("Transform found: translation %s, rotation %s", str(self.trans), str(self.rot))
            matrix = tf_conversions.transformations.quaternion_matrix(self.rot)
            self.DCM = matrix[:3,:3]
            self.trans = np.array(self.trans).T
            rospy.loginfo_once("Transform available")
            rospy.loginfo_once("DCM: \n {}".format(self.DCM))
            rospy.loginfo_once("trans: {}".format(self.trans))            
            self.tf_available = True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:            
            rospy.sleep(0.1)
            self.tf_available = False
            rospy.logwarn("Transformation error raised: %s", e)
    
    # callback for the transform
    def sort_points_clockwise(self,points):
        
        # Compute the distances from the origin
        distances = np.sqrt(points[:,0]**2 + points[:,1]**2)

        # Find the point closest to the origin        
        origin = np.mean(points, axis=0)
        points_shifted = points - origin
        
        # debug
        # rospy.logwarn('points: ' + str(points))
        # rospy.logwarn('points_shifted: ' + str(points_shifted))

        # Compute the angles from the origin
        angles = np.arctan2(points_shifted[:,1], points_shifted[:,0])

        # Sort the points by the angles in clockwise order
        sorted_indices = np.argsort(-angles)
        sorted_indices = np.roll(sorted_indices, shift=1, axis=0)
        sorted_points = points[sorted_indices]

        return sorted_points, sorted_indices

    # write on serial
    # INPUT: 
    #   srl: Serial object
    #   data: array with data to write
    #   sleepTime: how much to wait after sending the data
    #   print_flag: boolean to print or not debug
    def serialWrite(self,srl, data, sleepTime, print_flag):
        
        # flush the serial
        srl.flushInput()
        srl.flushOutput()
        
        # debug
        rospy.logdebug('flushed stdin stdout')

        # split the data to write in chunks of limited size
        # this has been done after troubleshooting and observing that too long data are missed by the serial
        chunks = [data[i:i + self.BUFSIZE_SET] for i in range(0, len(data)+1, self.BUFSIZE_SET)]
        
        # check chunks
        rospy.logdebug(chunks)

        # send the chunks
        for ck in chunks:
            
            # write in UTF-8
            srl.write(ck.encode('utf-8'))
            
            # if debug, print
            if print_flag:
                print('Send: ' + ck)

            # get the time t0
            t0 = time.time()
            
            # get elapsed time from t0
            dt = time.time() - t0
            
            # wait for response for at most sleepTime
            while dt < sleepTime:
                
                # update the elapsed time
                dt = time.time() - t0
                
                # if the serial is waiting, read the data and stream them
                if (srl.in_waiting > 0):
                    data = srl.readline().decode().strip()
                    
                    # if debug
                    if print_flag:
                        print('Receive: ' + data)

        # open and close the serial (good practice)
        srl.close()
        srl.open()
        
    # read the distances (it is a TAG running this)
    # INPUT:
    #   srl: Serial object
    #   stopTime: upper bound on waiting for data
    #   id: Anchor ID I want to read (from ID global variable)
    # OUTPUT:
    #   A: anchor position (array)
    #   D: distance measured
    #   SUCC: boolean, success of the reading (the anchor might not be available)
    #   NID: network ID
    def serialRead(self, srl, stopTime):

        # start timer
        t0 = time.time()
        
        # init outputs
        A = np.zeros((self.NUM_A,3))
        D = np.zeros(self.NUM_A)
        SUCC = np.zeros(self.NUM_A)
        NID = ['' for _ in range(self.NUM_A)]                

        # read for for the duration of stopTime
        # compute elapsed time
        dt = time.time() - t0
        
        # init flag describing if data have been received or not
        empty = True
        
        # keep waiting until either there is no time left or we read something
        while (dt < stopTime) or (empty == True):
            
            # update elapsed time
            dt = time.time() - t0
            
            # if the serial is waiting
            if (srl.in_waiting > 0):

                # get data and split in chunks separated by a comma (lec mode in the UWB antennas, check datasheet)
                data = srl.readline().decode().strip()
                
                # if we got data
                if len(data) > 0:
                    
                    # update empty flag
                    empty = False
                    
                    # split with comma (lec mode)
                    items = data.split(',')                    

                    # find if anchor was read
                    try:                                                
                        
                        # cycle over the anchors
                        for id_num in range(len(self.ID)):
                        
                            id = self.ID[id_num]
                            if id in items:                                
                        
                                # in the array of chunks, get the position of the one describing the anchor ID
                                # the lec mode prints: #DISTANCES, Anchor Assigned ID, Anchor Hardware ID, Anchor Position (if set), Distance from the TAG
                                pos = items.index(id)                                

                                # set anchors
                                A[id_num][0] = float(items[pos+2])
                                A[id_num][1] = float(items[pos+3])
                                A[id_num][2] = float(items[pos+4])

                                # set D
                                D[id_num] = float(items[pos+5])    
                                
                                # set network ID (first letter of the Anchor Assigned ID)
                                NID[id_num] = items[pos][0]
                        
                                # set flag and return
                                SUCC[id_num] = 1
                            else:
                                
                                # set the flag and return
                                SUCC[id_num] = 0
                                
                                # set anchors
                                A[id_num][0] = float(-999.0)
                                A[id_num][1] = float(-999.0)
                                A[id_num][2] = float(-999.0)  
                                
                                # set D
                                D[id_num] = float(-1.0)  
                                
                                # set network ID (first letter of the Anchor Assigned ID)
                                NID[id_num] = 'N/A'                                
                        
                        return A,D,SUCC,NID

                    except Exception as e:
                        # if the anchor is not available: debug and set SUCC to zero
                        rospy.logfatal(e)                        

        # default return
        return A,D,SUCC,NID

    # setup the antenna
    # This function basically uses all the commands from the UWB CLI to setup the antenna mode
    def talker_setup(self):    

        # general stuff    
        sleepLong = 4
        sleepShort = 2
        print_flag = 1
        
        # debug
        rospy.logdebug('start setup')    

        # how many args were sent
        argc = len(sys.argv)        

        # check if there are enough params
        if argc < 5:
            rospy.logfatal('Insufficient number of parameters for UWB setup')
        else:

            # Serial port
            SerialPort = sys.argv[1]

            # Baud rate
            BaudRate = sys.argv[2]

            # NetworkID
            NetworkID = sys.argv[3]

            # Label
            label = sys.argv[4]

            # UWB mode (tag, anchor, anchorinit)
            UWBmode = sys.argv[5] 

            # If we setup a TAG we also need the read frequency
            if UWBmode == 'tag':

                if argc < 6:
                    rospy.logfatal('Insufficient number of parameters for UWB setup')

                # Frequency
                freq = sys.argv[6]

            # If we setup an anchor or an anchor initiator we need their position (fixed, just an initialization)
            elif (UWBmode == 'anchor') or (UWBmode == 'anchorinit'):

                if argc < 8:
                    rospy.logfatal('Insufficient number of parameters for UWB setup');

                # Position x
                px = sys.argv[6]

                # Position y
                py = sys.argv[7]

                # Position z
                pz = sys.argv[8]
                
            else:
                rospy.logfatal('Wrong UWBmode')
                
        # debug
        rospy.logdebug('argv read')

        # create the serial object and flush the buffers
        myserial = serial.Serial(SerialPort, BaudRate, timeout=0.5, 
        parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,  bytesize=serial.EIGHTBITS)
        myserial.flushInput()
        myserial.flushOutput()    

        # Check if the serial port is open
        if not myserial.is_open:
            rospy.logfatal('Failed to open serial port!')
        else:
            rospy.logdebug('Connected to: ' + SerialPort)

        # start the configuration (double enter - see UWB datasheet)
        data = '\x0D\x0D'
        self.serialWrite(myserial, data, sleepLong, print_flag)

        # set the mode
        # the double enter is needed every time to check that everything works
        if UWBmode == 'tag':

            # set to tag mode
            rospy.logdebug('Set mode: ' + UWBmode)
            data = 'nmt\x0D'
            self.serialWrite(myserial, data, sleepLong, print_flag)
            data = '\x0D\x0D'
            self.serialWrite(myserial, data, sleepLong, print_flag)

            # set the frequency
            rospy.logdebug('Set frequency')
            data = 'aurs ' + freq + ' ' + freq + '\x0D'
            self.serialWrite(myserial, data, sleepShort, print_flag)

        elif UWBmode == 'anchor':
            
            # set to anchor
            rospy.logdebug('Set mode: ' + UWBmode)
            data = 'nma\x0D'
            self.serialWrite(myserial, data, sleepLong, print_flag)
            self.serialWrite(myserial, '\x0D\x0D', sleepLong, print_flag)

            # set position
            data = 'aps ' + str(px) + ' ' + str(py) + ' ' + str(pz) + '\x0D'
            self.serialWrite(myserial, data, sleepShort, print_flag)

        else:
            
            # set to anchor init
            rospy.logdebug('Set mode: ' + UWBmode)
            data = 'nmi\x0D'
            self.serialWrite(myserial, data, sleepLong, print_flag)
            self.serialWrite(myserial, '\x0D\x0D', sleepLong, print_flag)

            # set position
            data = 'aps ' + str(px) + ' ' + str(py) + ' ' + str(pz) + '\x0D'
            self.serialWrite(myserial, data, sleepShort, print_flag)

        # set the label
        rospy.logdebug('Set label')
        data = 'nls ' + label + ' \x0D'
        self.serialWrite(myserial, data, sleepShort, print_flag)

        # set the NetworkID
        rospy.logdebug('Set NetworkID')
        data = 'nis ' + str(NetworkID) + ' \x0D'
        self.serialWrite(myserial, data, sleepShort, print_flag)

        # see the results (drop the general antenna system info)
        rospy.logdebug('Get info')
        data = 'si\x0D'
        self.serialWrite(myserial, data, sleepShort, print_flag)

        # if we set anchors, see the position
        if UWBmode == 'anchor' or UWBmode == 'anchorinit':
            data = 'apg\x0D'
            self.serialWrite(myserial, data, sleepShort, print_flag)
        
        # Close the serial port
        rospy.logdebug('Close serial')
        myserial.close()

        # default return
        return 0

    # read from the tag the distance and anchor info
    def talker_read(self):    

        # general stuff init            
        sleepLong = 2 #(s)
        stopTime = 0.8/float(self.RATE) #(s) slightly less than freq for overhead
        print_flag = 0
        
        # define message
        msgD = RD()                

        # input vals
        argc = len(sys.argv)  
        
        # check if there are enough params
        if argc < 1:
            rospy.logfatal('Insufficient number of parameters for UWB setup')
        else:

            # Serial port
            SerialPort = sys.argv[1]

            # Baud rate
            BaudRate = sys.argv[2]            
            
            # Tag ID
            TagID = sys.argv[3]
            
        # publisher
        pub = rospy.Publisher('range', RD, queue_size=10)

        # set node rate    
        r = rospy.Rate(self.RATE)   

        # start serial
        myserial = serial.Serial(SerialPort, BaudRate, timeout=0.5, 
        parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,  bytesize=serial.EIGHTBITS)
        # flush buffers
        myserial.flushInput()
        myserial.flushOutput()
        
        # Check if the serial port is open
        if not myserial.is_open:
            rospy.logfatal('Failed to open serial port!')
            return -1
        else:
            rospy.logdebug('Connected to: ' + SerialPort)

        # start the configuration - go to cli mode
        data = '\x0D\x0D'
        self.serialWrite(myserial, data, sleepLong, print_flag)
        
        # reset
        rospy.logwarn('Tag rebooting') 
        data = 'reset\x0D'
        self.serialWrite(myserial, data, sleepLong, print_flag)    
        rospy.sleep(5) 
        rospy.logwarn('Tag rebooted')
        myserial = serial.Serial(SerialPort, BaudRate, timeout=0.5, 
        parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,  bytesize=serial.EIGHTBITS)  
        # flush buffers
        myserial.flushInput()
        myserial.flushOutput()
        
        # Check if the serial port is open
        if not myserial.is_open:
            rospy.logfatal('Failed to open serial port!')
            return -1
        else:
            rospy.logwarn('Connected to: ' + SerialPort)
            
        # start the configuration - go to cli mode
        data = '\x0D\x0D'
        self.serialWrite(myserial, data, sleepLong, print_flag)

        # start reading - send lec (stream of data)
        data = 'lec\x0D'
        self.serialWrite(myserial, data, sleepLong, print_flag)        
        
        # signal
        rospy.loginfo('Start publishing')  
        
        # init the Anchors param
        rospy.set_param('AnchorsInfoReal', self.anchors_params) 
        
        # init the Tag param
        rospy.set_param(TagID + 'Info', self.tag_params)                

        # loop
        while not rospy.is_shutdown():          
            
            try:                                
                
                # get measurement
                A, D, SUCC, NID = self.serialRead(myserial, stopTime)  
                
                # sort everything
                A, sorted_indices = self.sort_points_clockwise(A)
                D = D[sorted_indices]
                SUCC = SUCC[sorted_indices]
                NID = [NID[i] for i in sorted_indices]
                
                # debug
                # rospy.logwarn('A: ' + str(A))
                
                # check readings
                # rospy.logwarn('SUCC: ' + str(SUCC))
                
                # set if success in reading
                if any(element == 1 for element in SUCC): 
                
                    # get anchors info - set param
                    tmp_anchors_params = rospy.get_param('AnchorsInfoReal')
                    
                    # get tag info - set param
                    tmp_tag_params = rospy.get_param(TagID + 'Info')
                    
                    # cycle over the anchors
                    for index in range(self.NUM_A):                                                
                    
                        # update the param
                        tmp_anchors_params[index] = [NID[index], str(self.ID[index]), float(A[index][0]), float(A[index][1]), float(A[index][2])]
                        rospy.set_param('AnchorsInfoReal', tmp_anchors_params)                
                        
                        # update the param
                        tmp_tag_params[index] = [NID[index], str(self.ID[index]), float(D[index])]
                        rospy.set_param('Info', tmp_tag_params)  
                    
                    # now setup the publisher
                    msgD.header.stamp = rospy.Time.now()
                    msgD.N_ID = [row[0] for row in tmp_tag_params]
                    msgD.A_ID = [row[1] for row in tmp_anchors_params]
                    msgD.NUM_A = self.NUM_A
                    
                    # get the XYZ
                    X_POS = np.asarray([row[2] for row in tmp_anchors_params])
                    Y_POS = np.asarray([row[3] for row in tmp_anchors_params])
                    Z_POS = np.asarray([row[4] for row in tmp_anchors_params])
                    
                    # store anchors in sequence
                    A_POS = []
                    for i in range(msgD.NUM_A):
                        A_POS.append(X_POS[i])
                        A_POS.append(Y_POS[i])
                        A_POS.append(Z_POS[i])
                    
                    # rospy.loginfo(type(X_POS))
                    msgD.A_POS = A_POS
                    msgD.T_ID = TagID
                    msgD.D = [row[2] for row in tmp_tag_params]
                    rospy.logwarn_once(f"first range msg:{msgD}")
                    pub.publish(msgD)
                
                
            except Exception as e:                                
                # nothing special                   
                rospy.logfatal(e)
            

        # stop reading
        data = 'lec\x0D'
        self.serialWrite(myserial, data, sleepLong, print_flag)

        # Close the serial port
        rospy.logdebug('Close serial')
        myserial.close()
        
        rospy.logwarn('End of the node')

        return 0                

    # this functions set a parameter with all the anchors info, and publish marker arrays
    # INPUT: 
    #   anchors pos: string with all anchors pos
    #   params: name of the parameters
    #   topic:  name of the published topic
    def anchors_server(self,anchors_pos, topic, params_name):
        
        # general stuff init            
        rate = rospy.Rate(self.RATE)                
        
        # init the Anchors param        
        rospy.set_param(params_name, self.anchors_params)    
            
        # publisher
        pub = rospy.Publisher(topic, MarkerArray, queue_size=10)                        
        
        # init the Anchors param
        rospy.set_param(params_name, self.anchors_params)    
        
        while not rospy.is_shutdown():
            
            # define marker array
            marker_array = MarkerArray()                
            
            # if anchors_pos is not empty, split it
            if anchors_pos != '':
                
                # now split the anchors_pos into the positions
                # split with comma (lec mode)
                items = np.array([float(x) for x in anchors_pos.split(' ')])             
                
            else:
                
                # We should get here in the Anchors Estimate node, so we gather the anchors from the params_name
                self.anchors_params = rospy.get_param(params_name)  
                        
                # now extract the coordinates from anchors_params
                items = np.array([coord for anchor in self.anchors_params for coord in anchor[2:]])                             

            
            # get number of anchors (not rows of the param because maybe we are setting more than 4 anchors at once).         
            N_A = int(len(items)/3)   
            
            # debug
            # rospy.logwarn(items)  
        
            # assign positions in N_A markers
            for i in range(N_A):
                
                # define marker
                marker = Marker()
                
                scale_factor = 0.25
                
                # get position of anchor ith
                position = (items[3*i], items[3*i+1], items[3*i+2])
                orientation = (0.0, 0.0, 0.0, 1.0)
                scale = (scale_factor, scale_factor, scale_factor)                        
                
                # assign marker            
                marker.header.frame_id = "world"
                marker.header.stamp = rospy.Time.now()
                marker.id = i
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                marker.pose.position.x = position[0]
                marker.pose.position.y = position[1]
                marker.pose.position.z = position[2]

                marker.pose.orientation.x = orientation[0]
                marker.pose.orientation.y = orientation[1]
                marker.pose.orientation.z = orientation[2]
                marker.pose.orientation.w = orientation[3]

                marker.scale.x = scale[0]
                marker.scale.y = scale[1]
                marker.scale.z = scale[2]

                color = self.generate_color(i, N_A)
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = 0.5*color[3]

                # keep the markers while the node is running
                marker.lifetime = rospy.Duration()          
                
                # append the last marker  
                marker_array.markers.append(marker)  
                
                # now we set the anchors pos in the info param
                # get anchors info - get param
                tmp_anchors_params = rospy.get_param(params_name)

                # Find the index of the row containing the value
                id = "AN"+str(i)
                
                # set network ID (first letter of the Anchor Assigned ID)
                NID = id[0]
                index = next((i for i, row in enumerate(tmp_anchors_params) if id in row), None)
                tmp_anchors_params[index] = [NID, str(id), float(position[0]), float(position[1]), float(position[2])]
                rospy.set_param(params_name, tmp_anchors_params)                        
                
            # publish the array
            pub.publish(marker_array)                        

    # I want to visualize in RVIZ a line connecting two points
    def publish_line_marker(self,odom,params_name,topic, color):
        
        # timers
        self.timers()
        
        # general stuff init            
        rate = rospy.Rate(self.RATE)
        
        # subscribe to the odom
        sub = rospy.Subscriber(odom, Odometry, lambda msg: self.vis_line_callback(msg, params_name, topic, color))
        
        # spin
        rospy.spin()
        
        
    # callback when odometry is received and publishing the line markers
    def vis_line_callback(self,msg, params_name, topic, color):
        
        # Line publisher
        pub = rospy.Publisher(topic+"/line", MarkerArray, queue_size=10)
        
        # define marker_array with lines
        marker_array_line = MarkerArray()        
        
        # get anchors info - get param    
        try:
            tmp_anchors_params = rospy.get_param(params_name)
        except:
            return
        
        # debug
        rospy.loginfo(tmp_anchors_params)
        
        # N_A
        N_A = len(tmp_anchors_params)
        rospy.loginfo("Number of anchors:" + str(N_A))
        
        # convert color
        # Convert the string to a list of floats
        color = np.asarray([float(x) for x in color.split()])        
        
        # see if you need to generate colors    
        if color[0] == -1:
            color_gen = 1
        else:
            color_gen = 0
            color = tuple(color)     
        
        for i in range(N_A):
            
            # define color of the line
            if color_gen == 1:
                color = self.generate_color(i, N_A)                    
        
            # define marker for line
            marker_line = Marker()    
            
            marker_line.header.frame_id = "world"
            marker_line.header.stamp = rospy.Time.now()
            marker_line.ns = "AN" + str(i) + "/line_markers"
            marker_line.id = i
            marker_line.type = Marker.LINE_STRIP
            marker_line.action = Marker.ADD            
            marker_line.scale.x = 0.01  # Line width
            marker_line.color.r = color[0]
            marker_line.color.g = color[1]
            marker_line.color.b = color[2]
            marker_line.color.a = color[3]
            
            marker_line.pose.orientation.x = 0
            marker_line.pose.orientation.y = 0
            marker_line.pose.orientation.z = 0
            marker_line.pose.orientation.w = 1
            
            # debug
            # rospy.logwarn(tmp_anchors_params[i])

            # Define anchor point
            pointANC = Point()
            pointANC.x = float(tmp_anchors_params[i][2])
            pointANC.y = float(tmp_anchors_params[i][3])
            pointANC.z = float(tmp_anchors_params[i][4])

            # Define UGV point
            pointUGV = Point()
            pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            pos = self.DCM@(pos + self.trans)
            pointUGV.x = pos[0]
            pointUGV.y = pos[1]
            pointUGV.z = pos[2]
            
            # debug
            rospy.loginfo(pointUGV.z)            
            
            # compute distance between the two points            
            # Extract coordinates
            x1, y1, z1 = pointANC.x, pointANC.y, pointANC.z
            x2, y2, z2 = pointUGV.x, pointUGV.y, pointUGV.z
            
            # Compute Euclidean distance
            d = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
            
            # append marker if it's within range
            if d > RANGE:
                pointANC.x = 0.0
                pointANC.y = 0.0
                pointANC.z = 0.0
                pointUGV.x = 0.0
                pointUGV.y = 0.0
                pointUGV.z = 0.0
                
            marker_line.points.append(pointANC)
            marker_line.points.append(pointUGV)                               
                
            marker_array_line.markers.append(marker_line)   
        

        # Publish the marker
        pub.publish(marker_array_line)
        
        
    # ok this is an OCD method. I need the anchors to have incremental colors
    # INPUT
    #   index: number of marker
    #   total_markers: number of anchors
    def generate_color(self,index, total_markers):
        hue = index / float(total_markers)  # Vary the hue between 0 and 1
        saturation = 1.0  # Full saturation
        value = 1.0  # Full brightness
        rgb = colorsys.hsv_to_rgb(hue, saturation, value)
        return rgb + (1.0,)  # Return as (r, g, b, a) tuple with full opacity