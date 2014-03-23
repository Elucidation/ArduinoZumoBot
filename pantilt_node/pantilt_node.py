import smbus
import rospy
# import pickle
from time import time as tm
from rosserial_arduino.msg import pan_tilt
from std_msgs.msg import UInt8MultiArray
# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04
last_time = 0
pantilt_rate = 0.02 # 20 ms pub rate max

def setPanTilt(pan, tilt):
    global last_time
    # constrain to 0-100 range
    pan = 0 if pan < 0 else 100 if pan > 100 else pan
    tilt = 0 if tilt < 0 else 100 if tilt > 100 else tilt

    # Write 3 bytes to bus, 'm', pan, tilt (m for manual)
    if tm() - last_time > pantilt_rate:
        try:
            bus.write_i2c_block_data(address, ord('m'), [pan, tilt]);
        except IOError:
            print "Pan Tilt Write error (rate too fast)"
        last_time = tm()

def setLEDs(left, right):
    try:
        bus.write_i2c_block_data(address, ord('l'), [left, right]);
    except IOError:
        print "LED Write error (rate too fast)"

def cb_PanTilt(data):
    setPanTilt(data.pan, data.tilt)

def cb_LEDs(msg):
    # setLEDs(data[0], data[1])
    data = msg.data
    # print "Received",data, "of type", type(data)
    try:
        if type(data) == str:
            a, b = ord(data[0]), ord(data[1])
            print "Setting LEDs to",a,b
            setLEDs(a,b)
        else:
            setLEDs(data[0], data[1])
    except Exception, e:
        print e
    # pickle.dump(data, open('testdata', 'w'))

def listener():
    print "Initializing Pan Tilt Node..."
    rospy.init_node("pantilt_node")
    
    rospy.Subscriber("pantilt/set_position", pan_tilt, cb_PanTilt)
    print "Subscribed to 'pantilt/set_position'"

    rospy.Subscriber("set_leds", UInt8MultiArray, cb_LEDs)
    print "Subscribed to 'set_leds'"

    rospy.spin()

if __name__ == '__main__':
    listener()


# while True:
#     try:
#         pan, tilt = map(int, raw_input("Enter Pan/Tilt: ").split())
#         setPanTilt(pan, tilt)
#     except Exception, e:
#         raise e   
