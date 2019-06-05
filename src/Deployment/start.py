import argparse, socket, time, threading
from collections import OrderedDict 



def update_device ( device_list: dict ):

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


def execute_command( address: tuple , command: str ):
    sender = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sender.connect( (address, 60652) )
    sender.send(bytes('EXC' + "  "+ command, "utf-8"))
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
        print("[ERROR]: unknow response code, contact administrator.")

    sender.close()

def execute_multi_command(command: str, device_list: dict, mode: int):
    thread_list = []
    if (mode == 0 ):
        for index in range( len(device_list) ):
            thread_list.append( threading.Thread(target=execute_command_w_feedback,args=(device_list[index][1], command)) )
    elif(mode == 1):
        for index in range( len(device_list) ):
            thread_list.append( threading.Thread(target=execute_command,args=(device_list[index][1], command)) )
    else:
        print("[ERROR]: invalid mode")
        return

    for thread in thread_list:
        thread.start()

    for thread in thread_list:
        thread.join()

#-----------------------------------------------------------------------
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-u", "--url",      required = False,   default = "https://github.com/cyphyhouse/CyPyHous3.git", help="path to the git repo, default to be the ONE")
ap.add_argument("-s", "--startfile",   required = True,    help= "the initialzing bash script that we want to run ")
ap.add_argument("-b", "--branch",       required = True,     help=" the git branch that we want to pull")
args = vars(ap.parse_args())

device_list = {}
update_device(device_list)

print("\n ------------------------ Device List ------------------------")
for index, attributes in device_list.items():
    print(index, " : name" + attributes[0] + "\t\t\t IP:" + attributes[1] + "\t\t Status:" + attributes[2] )
print(" -------------------------------------------------------------\n")

command = "rm -rf CyPyHous3"
execute_multi_command(command, device_list, 1)

command = "git clone -b " + args["branch"] + ' '+args["url"]
execute_multi_command(command, device_list, 0)

print("[INFO]: Git clone finished")

command = "cat " + args["startfile"]
execute_multi_command(command, device_list, 1)

print("[INFO]: Execute finished")


