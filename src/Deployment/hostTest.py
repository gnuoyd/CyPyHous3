'''
DANGER: This code contains multiple security loopholes, running in a privatized network is recommanded 
'''
import select, socket, os, threading, subprocess
from multiprocessing import Process, Queue



# --------------------------------------------------------------------------------------------------------------------
def send_response_to_controller( address, device_name, status ):#udp response
    msg = bytes( device_name + ' ' + status,"utf-8" )
    s = socket.socket( socket.AF_INET, socket.SOCK_DGRAM ) 
    s.sendto( msg, address )
    s.close()

def listening_discover(device_name):
    status = "some_status"
    # Preparing socket 
    port = 60651  
    buffer_size = 64 
    discover_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    discover_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    discover_socket.bind(('', port))

    # Listening
    while True:
        instruction, address = discover_socket.recvfrom(buffer_size)
        print( "[DISCOVER]", instruction, " from " , address )
        if( instruction.decode("utf-8") == "INFO" ):
            udp_response = threading.Thread( target=send_response_to_controller, args=(address, device_name, status, ) )
            udp_response.start()
        


# --------------------------------------------------------------------------------------------------------------------

def ftp_dowload (address):
    os.system("wget -r ftp://"+ address[0] +":60655")

    
def git_dowload ( controller_sock, url ):
    print("[GIT]: downloading from ", url)
    result = subprocess.run(['git','clone', url.strip()], cwd="/home/pi/Desktop" , stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print (result.stdout)
  
    print("[GIT]: git return code",result.returncode)
    if(result.returncode == 0):
        controller_sock.send(b"SUCCESS")
    else:
        controller_sock.send(b"FAIL"+ result.stdout[0:1000])

def excute_program ( command ):
    print("[EXC]: Excuting command ", command)
    if(len(command.split()) != 0  ):
        command = command.split("&&")
        for arg in command:
            result = subprocess.run(arg.split(), cwd="/home/pi/Desktop" , stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            print (result.stdout)
            print("[EXC]: excute return code",result.returncode)
    else:
        print("[EXC]: empty command")
    # if(result.returncode == 0):
    #     controller_sock.send(b"SUCCESS")
    # else:
    #     controller_sock.send(b"FAIL"+ result.stdout[0:1000])
    

def connection_handler(controller_sock, address):
    request = controller_sock.recv(1024)
    instruction = request.decode("utf-8")
    print("[INSTRUCTION]: ", instruction, " -- ",request)
    if  ( instruction[0:2] == "FTP" ): 
        ftp_dowload(address)
    elif( instruction[0:3] == "GIT" ):
        git_dowload( controller_sock, instruction[3:len(instruction)] )
    elif( instruction[0:3] == "EXC" ):
        excute_program( instruction[3:len(instruction)] )
    else:
        pass
    controller_sock.close()


def listening_instruction():
    port = 60652
    instruction_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    instruction_socket.bind(('0.0.0.0', port))
    instruction_socket.listen(8) 

    while True:
        controller_sock, address = instruction_socket.accept()
        instructiond_handler = threading.Thread( target=connection_handler, args=(controller_sock,address,) )
        instructiond_handler.start()

    
#-----------------------------------------------------------------------------------------------------------------------
# main

device_name = "some_device"
listening_instruction      = Process(target=listening_instruction)
listening_discover         = Process(target=listening_discover,args=(device_name, ))

listening_instruction.start()
listening_discover.start()
