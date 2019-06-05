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
     

def git_transfering( address , url ):
    '''
    This function handles git file transfering
    '''
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
    

def execute_command_w_feedback( address: tuple , command: str ):

    sender = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sender.connect( (address, 60652) )
    sender.send(bytes('EXF' + command, "utf-8"))
    
    #wait for resault for 10 sencond 
    sender.settimeout(10.0)
    try:
        response, device_address = sender.recvfrom(1024)
        response = response.decode("utf-8")
    except:
        print("[ERROR]: response connection time out, something is woring on the device, SSH for futher information")
        sender.close()
        return
   
    if(response == "SUCCESS"):
        print("[INFO]: commmand execute successed in device ", address)
    elif(response[0:4] == "FAIL"):
        print("[ERROR]: commmand execute FAIL in device ", address)
        print(" \n\n ------------------------------------------------------")
        print("[ERROR]: The device returns : " + response[3:-1])
        print(" \n\n\n")
    else:
        print(response)
        print("[ERROR]: unknow response code, contact administrator.")

    sender.close()

def execute_program(address, command):
    '''
    this function handles command execution
    '''
    sender = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sender.connect( (address, 60652) )
    sender.send(bytes('EXC' + "  "+ command, "utf-8"))
    sender.close()

def device_selection(device_list):
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
        return 0
    
    for i in index_list:
        if (i >= len(device_list)):
            print("[ERROR]: Index out of range")
            return 0

    return index_list

# ---------------------------------------------------------------------------------------------
# The following is the main function 

# this is a dictionary with key being the index of the device and value being a list of it attributes 
# attributes (list of string): [name of the device , ip address, status]
device_list = {}

# the following while loop is a shell like object that handles uer input 
while True:
     
    command = input(">>> ")
    if( command == "exit" or command == "quit" ):
        break

    elif( command[0:4] == "list" ):
        update_device(device_list)

    elif( command[0:9] == "execute-f" ): 
        index_list = device_selection(device_list)
        if( index_list == 0):
            continue

        for index in index_list:
            print("[EXF]: Sending command to " + str(device_list[index][1]))
            execute_command_w_feedback( device_list[index][1] , command[9:len(command)] )

    elif( command[0:7] == "execute" ):
        index_list = device_selection(device_list)
        if( index_list == 0):
            continue

        for index in index_list:
            print("[EXC]: Sending command to " + str(device_list[index][1]))
            execute_program( device_list[index][1] , command[7:len(command)] )
    
    elif( command[0:4] == "kill"):
        index_list = device_selection(device_list)
        if( index_list == 0):
            continue
        argument = " ls" ##########need attention
        for index in index_list:
            print("[KILL]: Sending command to " + str(device_list[index][1]))
            execute_command_w_feedback( device_list[index][1] , argument )
        
    elif( command[0:3] == "run"):
        index_list = device_selection(device_list)
        if( index_list == 0):
            continue
        argument = "ls" ##########need attention       
        for index in index_list:
            print("[RUN]: Sending command to " + str(device_list[index][1]))
            execute_program( device_list[index][1] , argument )

    elif( command[0:6] == "gitrun"):
        index_list = device_selection(device_list)
        if( index_list == 0):
            continue
        argument = "ls" ##########need attention        
        for index in index_list:
            print("[GITRUN]: Sending command to " + str(device_list[index][1]))
            execute_program( device_list[index][1] , argument )

    else:
        print("[ERROR]: command not vaild")
        
         
  












