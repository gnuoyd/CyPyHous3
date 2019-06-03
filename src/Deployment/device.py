'''
DANGER: This code contains multiple security loopholes, running in a privatized network is recommanded 
'''
import select, socket, os, threading, subprocess, shutil, datetime, time
from multiprocessing import Process, Queue




# The following two function handles device discover---------------------------------------------------------

def device_query_handler( address: tuple, device_name: str ):  # no returns 
    '''
    INPUT  - address: ip address that the function send the package to 
             device_name: device name
    OUTPUT - NONE
    FUNCATIONALIATY -  This function sends UDP package that contains device name back to the controller 
    '''
    status = "some_status"
    msg = bytes( device_name + ' ' + status ,"utf-8" )
    s = socket.socket( socket.AF_INET, socket.SOCK_DGRAM ) 
    s.sendto( msg, address )
    s.close()

def device_quey_listener(device_name: str):  # no returns
    '''
    INPUT  - device_name: device Name
    OUTPUT - NONE 
    FUNCATIONALIATY:    This function listening on port 60651 for device query
                        Once a query is captured, it will call the device_query_handler to response
    '''

    # Preparing socket 
    port = 60651  
    buffer_size = 64 
    discover_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    discover_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    discover_socket.bind(('0.0.0.0', port))

    # Listening
    while True:
        instruction, address = discover_socket.recvfrom(buffer_size)
        print( "[DISCOVER]", instruction, " from " , address )
        if( instruction.decode("utf-8") == "INFO" ):
            udp_response = threading.Thread( target=device_query_handler, args=(address, device_name, ) )
            udp_response.start()
        


# The following function handles device instruction execution -----------------------------------------------

def git_dowload ( controller_sock: socket, url: str ): # no returns 
    '''
    INPUT  - controller_sock: the socket that this function later will use to send the execution result back to 
             url: the git url that will be puul from the internet 
    OUTPUT - NONE
    FUNCATIONALIATY - This function handles git clone command, the command will execute on /tmp directory
    '''
    print("[GIT]: downloading from ", url)
    result = subprocess.run(['git','clone', url.strip()], cwd= ( str(os.environ.get('HOME')) + "/tmp" ) , stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print ("[GIT]:", result.stdout)
  
    print("[GIT]: git return code",result.returncode)
    if(result.returncode == 0):
        controller_sock.send(b"SUCCESS")
    else:
        controller_sock.send(b"FAIL"+ result.stdout[0:1000])

def excute_program ( command ):  # no returns 
    '''
    INPUT  - command: the command that will be executed by this function
    OUTPUT - NONE
    FUNCATIONALIATY - This function handles command execution, the command will execute on /tmp directory
    '''
    print("[EXC]: Excuting command ", command)
    if(len(command.split()) != 0  ):
        command = command.split("&&")
        for arg in command:
            result = subprocess.run(arg.split(), cwd= ( str(os.environ.get('HOME')) + "/tmp" ) , stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            print (result.stdout)
            print("[EXC]: excute return code",result.returncode)
    else:
        print("[EXC]: empty command")


def connection_handler(controller_sock: socket, address: tuple): # no returns 
    '''
    INPUT  - controller_sock: the socket that handle the connection to the controller
             address: address of the controller 
    OUTPUT - NONE
    FUNCATIONALIATY - This function handles connection and command parse 
    '''
    request = controller_sock.recv(1024)
    instruction = request.decode("utf-8")
    print("[INSTRUCTION]: ", instruction, " -- ",request)
    if  ( instruction[0:3] == "GIT" ):
        git_dowload( controller_sock, instruction[3:len(instruction)] )
    elif( instruction[0:3] == "EXC" ):
        excute_program( instruction[3:len(instruction)] )
    else:
        pass
    controller_sock.close()


def listening_instruction():
    '''
    INPUT  - NONE
    OUTPUT - NONE
    FUNCATIONALIATY -   This function constently listens on port 60652 for instruction
                        Upon reciving an instruction, it will call the connection handler
                        to parse and handle the request 
    '''
    port = 60652
    instruction_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    instruction_socket.bind(('0.0.0.0', port))
    instruction_socket.listen(8) 

    while True:
        controller_sock, address = instruction_socket.accept()
        instructiond_handler = threading.Thread( target=connection_handler, args=(controller_sock,address,) )
        instructiond_handler.start()

# the following functions handles device monitoring ---------------------------------------------------------

def get_info() -> str:
    '''
    INPUT  - NONE
    OUTPUT - NONE
    FUNCATIONALIATY - this function gets the necessary infomation from the device 
                      and then send back to the controller  
    '''
    msg = "some_info" + str(datetime.datetime.now().time())
    return msg

def monitor_query_handler(address: tuple , parameter: list): # no returns 
    '''
    INPUT  - address: address of the controller 
             parameter: monitering parameter 
    OUTPUT - NONE
    FUNCATIONALIATY - this function sends back infomation of the device back to the 
                      controller, and the dehavior is controller by the parameter 
                      passed in. The first parameter controls the time gap between 
                      each UDP package, the seoncd one control the number of time that 
                      the device will send the information .
    '''
    try:
        gap = int(parameter[0])
        termination = int(parameter[1])
    except:
        print("[ERROR]: monitoring parameter input type error, it can only accept int")

    for i in range(termination):
        info = get_info()
        msg = bytes( device_name + ' ' + info,"utf-8" )
        s = socket.socket( socket.AF_INET, socket.SOCK_DGRAM ) 
        s.sendto( msg, address )
        s.close()
        time.sleep(gap)


def monitoring():
    '''
    INPUT  - NONE
    OUTPUT - NONE
    FUNCATIONALIATY - This function constently monitering the 
    '''
    port = 60653
    buffer_size = 64 
    monitor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    monitor_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    monitor_socket.bind(('', port))

    # Listening
    while True:
        instruction, address = monitor_socket.recvfrom(buffer_size)
        instruction = instruction.decode("utf-8")
        if( instruction[0:7] == "MONITOR" ):
            parameter = instruction[8:].split(' ')
            udp_response = threading.Thread( target=monitor_query_handler, args=(address, parameter,) )
            udp_response.start()

    
#-----------------------------------------------------------------------------------------------------------------------
# main
tmp_path = str(os.environ.get('HOME')) + "/tmp"

if os.path.exists(tmp_path):
    shutil.rmtree(tmp_path)
os.makedirs(tmp_path)

device_name = "some_device"
listening_instruction      = Process(target=listening_instruction)
listening_discover         = Process(target=device_quey_listener,args=(device_name, ))

listening_instruction.start()
listening_discover.start()
