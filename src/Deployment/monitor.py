import socket
import time
from collections import OrderedDict 



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



def monitor_device(address):
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.sendto(b'MONITOR' )
    sender.settimeout(3.0)


    while True: 
        try:
            info, address = sender.recvfrom(buffer_size)
            info = info.decode("utf-8")
            print( info, " from ",address )
        except:
            break

    sender.close()



# -----------------------------------------------------------------------------------

device_list = {}

while True:
    command = input(">>> ")
    if( command == "exit" or command == "quit" ):
        break

    elif( command[0:4] == "list" ):
        update_device(device_list)
    
    elif( command[0:7] == "monitor" ):
        print("Which of the following devices do you want to monitor? ")
        print("\n ------------------------ Device List ------------------------")
        for index, attributes in device_list.items():
            print(index, " : name" + attributes[0] + "\t\t\t IP:" + attributes[1] + "\t\t Status:" + attributes[2] )
        print(" -------------------------------------------------------------\n")

        index_list = input("Input the device index, separate by space >>> ").split(' ')
        print('\n')
        try:
            index_list = [int(x) for x in index_list if x != '']
            index_list = list(set(index_list))
        except:
            print("[ERROR]: input not valid")
            continue

        for index in index_list:
            print("[EXC]: Sending command to " + str(device_list[index][1]))
            monitor_device(device_list[index][1])

    else:
        print("[ERROR]: command not vaild")