import socket
import time
from collections import OrderedDict 

from pyftpdlib.authorizers import DummyAuthorizer
from pyftpdlib.handlers import FTPHandler
from pyftpdlib.servers import FTPServer


def update_device ( device_list ):
    buffer_size = 1024
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sender.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sender.sendto(b'INFO', ('<broadcast>', 60651))
    
    sender.sendto(b'INFO', ('10.194.167.146', 60651))
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
     

def ftp_transfering(address): # coming soon ...
    sender = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sender.connect( (address, 60652) )
    # ----------------------------
    authorizer = DummyAuthorizer()
    authorizer.add_user("user", "12345", "/Users/charlie/Desktop", perm="elradfmw")
    authorizer.add_anonymous("/Users/charlie/Desktop", perm="elradfmw")
    handler = FTPHandler
    handler.authorizer = authorizer
    fserver = FTPServer(("0.0.0.0", 60655), handler)
    # ----------------------------
    sender.send(b'FTP')
    print("message sent!")
    fserver.serve_forever() # need to deal with timming !!!!!!!!!!!!!!!!!!!

def git_transfering( address , url ):
    sender = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sender.connect( (address, 60652) )
    sender.send(bytes('GIT' + url, "utf-8"))
    
    #wait for resault
    sender.settimeout(5.0)
    try:
        response, device_address = sender.recvfrom(1024)
        response = response.decode("utf-8")
    except:
        print("[ERROR]: response connection time out, something is woring on the device, SSH for futher information")
        sender.close()
        return
   
    if(response == "SUCCESS"):
        print("[INFO]: GIT clone success in device ", address)
    elif(response[0:4] == "FAIL"):
        print("[ERROR]: GIT clone FAIL in device ", address)
        print("[ERROR]: The device returns : " + response[3:-1])
    else:
        print("[ERROR]: unknow response code, contact administrator.")

    sender.close()
    

def execute_program(address, command):
    sender = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sender.connect( (address, 60652) )
    sender.send(bytes('EXC' + "  "+ command, "utf-8"))
    sender.close()
    #wait for resault 

# ---------------------------------------------------------------------------------------------
# this is a dictionary with key being the index of the device and value being a list of it attributes 
# attributes (list of string): [name of the device , ip address, status]
device_list = {}


while True:
     
    command = input(">>> ")
    if( command == "exit" or command == "quit" ):
        break

    elif( command[0:4] == "list" ):
        update_device(device_list)

    elif( command[0:2] == "ftp" ):
        print("Which of the following devices do you want to tranfer file using FTP? Separate the index by space...")
        for index, attributes in device_list:
            print(index, " : name:" + attributes[0] + "\t\t\t IP:" + attributes[1] + "\t\t Status:" + attributes[2] )

        index_list = input().splite(' ')
        try:
            index_list = [int(x) for x in index_list if x != '']
            index_list = list(set(index_list))
        except:
            print("[ERROR]: input not valid")
            continue

        for index in index_list:
            ftp_transfering(device_list[index][1])

    elif( command[0:3] == "git" ): 
        print("Which of the following devices do you want to tranfer file using GIT? ")
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
            print("[GIT]: Sending command to " + str(device_list[index][1]))
            git_transfering( device_list[index][1] , command[3:len(command)] )

    elif( command[0:6] == "excute" ):
        print("Which of the following devices do you want to tranfer file using GIT? ")
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
            execute_program( device_list[index][1] , command[6:len(command)] )

    else:
        print("[ERROR]: command not vaild")
        
         
  












