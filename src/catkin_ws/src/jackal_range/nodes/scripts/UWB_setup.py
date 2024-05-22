#!/usr/bin/env python
# license removed for brevity


import rospy
import sys
import serial
import jackal_range as jr

def talker():

    # ros::init() the node
    rospy.init_node('UWB_setup', anonymous=True)

    # general stuff
    cnt = 0
    rate = 5
    sleepLong = 4
    sleepShort = 2

    argc = len(sys.argv)

    # get tagID - default 7
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

        if UWBmode == 'tag':

            if argc < 6:
                rospy.logfatal('Insufficient number of parameters for UWB setup')

            # Frequency
            freq = sys.argv[6]

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

    myserial = jr.serial.Serial(SerialPort, BaudRate, timeout=0.5, 
    parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,  bytesize=serial.EIGHTBITS)
    myserial.flushInput()
    myserial.flushOutput()
    

    # Check if the serial port is open
    if not myserial.is_open:
        rospy.logfatal('Failed to open serial port!')
    else:
        rospy.loginfo('Connected to: ' + SerialPort)

    # start the configuration
    data = '\x0D\x0D'
    jr.serialWrite(myserial, data, sleepLong)

    # mode
    if UWBmode == 'tag':

        # set to tag
        rospy.loginfo('Set mode: ' + UWBmode)
        data = 'nmt\x0D'
        jr.serialWrite(myserial, data, sleepLong)
        data = '\x0D\x0D'
        jr.serialWrite(myserial, data, sleepLong)

        # set the frequency
        rospy.loginfo('Set frequency')
        data = 'aurs ' + freq + ' ' + freq + '\x0D'
        jr.serialWrite(myserial, data, sleepShort)

    elif UWBmode == 'anchor':
        
        # set to anchor
        rospy.loginfo('Set mode: ' + UWBmode)
        data = 'nma\x0D'
        jr.serialWrite(myserial, data, sleepLong)
        jr.serialWrite(myserial, '\x0D\x0D', sleepLong)

        # set position
        data = 'aps ' + str(px) + ' ' + str(py) + ' ' + str(pz) + '\x0D'
        jr.serialWrite(myserial, data, sleepShort)

    else:
        
        # set to anchor init
        rospy.loginfo('Set mode: ' + UWBmode)
        data = 'nmi\x0D'
        jr.serialWrite(myserial, data, sleepLong)
        jr.serialWrite(myserial, '\x0D\x0D', sleepLong)

        # set position
        data = 'aps ' + str(px) + ' ' + str(py) + ' ' + str(pz) + '\x0D'
        jr.serialWrite(myserial, data, sleepShort)

    # set the label
    rospy.loginfo('Set label')
    data = 'nls ' + label + ' \x0D'
    jr.serialWrite(myserial, data, sleepShort)

    # set the NetworkID
    rospy.loginfo('Set NetworkID')
    data = 'nis ' + str(NetworkID) + ' \x0D'
    jr.serialWrite(myserial, data, sleepShort)

    # see the results
    rospy.loginfo('Get info')
    data = 'si\x0D'
    jr.serialWrite(myserial, data, sleepShort)

    if UWBmode == 'anchor' or UWBmode == 'anchorinit':
        data = 'apg\x0D'
        jr.serialWrite(myserial, data, sleepShort)
    
    # Close the serial port
    rospy.loginfo('Close serial')
    myserial.close()

    return 0

if __name__ == '__main__':
    talker()
