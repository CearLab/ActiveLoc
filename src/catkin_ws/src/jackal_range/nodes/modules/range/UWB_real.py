#!/usr/bin/env python
# license removed for brevity

# general import
import rospy
import serial
import time
import sys
import numpy as np

# custom message import
from jackal_range.msg import RD_recap as RD
from range.jackal_range import JackalRange


class UWB_real(JackalRange):
    
    def __init__(self):
        
        # Inherit
        super().__init__()
        
    def timer_callback_tf(self, event):
        pass
    
    def timer_callback(self, event):
        pass

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
        pub = rospy.Publisher('Anchors/range', RD, queue_size=10)

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
        
        # Wait for the reset to complete with a timeout of 10 seconds
        timeout = 10
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            response = myserial.readline().decode().strip()
            if len(response) > 0:
                rospy.logwarn('Reset complete')
                rospy.sleep(1) 
                rospy.logwarn('Tag rebooted')
            break
            rospy.sleep(0.1)
        else:
            rospy.logwarn('Reset timeout')
            
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
        rospy.logwarn_once('Start publishing')

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
                
                # set if success in reading
                if any(element == 1 for element in SUCC): 
                
                    # get anchors info - set param
                    tmp_anchors_params = self.anchors_params
                    
                    # get tag info - set param
                    tmp_tag_params = self.tag_params
                    
                    # cycle over the anchors
                    for index in range(self.NUM_A):                                                
                    
                        # update the param
                        tmp_anchors_params[index] = [NID[index], str(self.ID[index]), float(A[index][0]), float(A[index][1]), float(A[index][2])]
                        tmp_tag_params[index] = [NID[index], str(self.ID[index]), D[index]]
                        
                    self.anchors_params = tmp_anchors_params
                    self.tag_params = tmp_tag_params
                    
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
                    # rospy.logwarn_once(f"first range msg:{msgD}")
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