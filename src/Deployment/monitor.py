import socket, time, threading, os, multiprocessing,rospy
from collections import OrderedDict 
from std_msgs.msg import String

def update_device ( device_list ):
    '''
    This function updates the device list 
    '''
    buffer_size = 1024
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sender.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sender.sendto(b'INFO', ('<broadcast>', 60651))
    
    sender.settimeout(3.0)

    print("[INFO]: Device query sent!")

    i = 0
    while True: 
        try:
            info, address = sender.recvfrom(buffer_size)
            info = info.decode("utf-8").split(' ')
            device_list.update( {i: [info[0], str(address[0]), info[1]]} )
            i +=1
            print( info[0], " from ",address )
        except:
            break
    print("[INFO]: Discover finished")


def talker( topic, tpoic_type, data_queue ):

    os.environ['ROS_MASTER_URI'] = "http://localhost:11311"
    pub = rospy.Publisher(topic, tpoic_type, queue_size = 10 )
    rospy.init_node('talker', anonymous=True)
    while not rospy.is_shutdown():
        # print("puted data to ", topic)
        data = data_queue.get()
        # rospy.loginfo(data)
        pub.publish(data)




# --------------------------------------
def callback(data, args):
    print(args[1], " : ", data)
    if( not args[0].full() ):
        args[0].put( data )

def listener(data_queue, address, topic ):
    # subscribe from remote master
    os.environ['ROS_MASTER_URI'] = "http://" + address + ":11311"
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber(name = topic, 
                     data_class =  String, 
                     callback = callback, 
                     callback_args = (data_queue, address)
                     )
    rospy.spin()





# -----------------------------------------------------------------------------------

topic = "chatter"

device_list = {}
update_device(device_list)


print(os.environ['ROS_IP']) #need to det thid first in shell
print(os.environ['ROS_MASTER_URI'])

os.environ['ROS_MASTER_URI'] = "http://localhost:11311"

data_queue_list = [ multiprocessing.Queue() for index, attributes in device_list.items() ]
sub_process_list = [ multiprocessing.Process(target = listener, args = ( data_queue_list[index], attributes[1], topic, ) ) for index, attributes in device_list.items() ]
pub_process_list = [ multiprocessing.Process(target = talker, args = (topic+str(data_queue_list.index(queue)), String, queue,))  for queue in  data_queue_list]

for process in sub_process_list :
	process.start()

for process in pub_process_list :
	process.start()

