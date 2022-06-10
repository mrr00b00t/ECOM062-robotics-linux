# Programa criado por José Augusto, matrícula 17111416
# Atividade 1 para a disciplina de Robótica
# Tasks:
# a) Obter pose do Dummy.
# b) Escrever uma função que controle as juntas do MTB e as posiciona com ângulos arbitrários.

# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import math
import numpy as np
import random
from functools import reduce

print('Program started')

# termina todas as conexões só pra garantir
sim.simxFinish(-1)

# se conecta com o CoppeliaSIm
# clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

robot_name = 'MTB'
dummy_name = 'Dummy'

has_error = lambda z: reduce(lambda x, y: x and y,
                   map(lambda x: x != sim.simx_error_noerror, z))
    
def move_joints(clientID, jointHandles, jointPositions):
    sim.simxPauseCommunication(clientID, True)
    
    _ = [sim.
         simxSetJointPosition(
            clientID,
            jointHandle,
            jointPosition,
            sim.simx_opmode_oneshot
        ) for (jointHandle, jointPosition) in zip(jointHandles, jointPositions)]
    
    sim.simxPauseCommunication(clientID, False)

if clientID != -1:
    print ('Connected to remote API server')
    
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

    sim.simxAddStatusbarMessage(clientID, 'Funcionando...', sim.simx_opmode_oneshot_wait)

    time.sleep(2)
    
    # Colentando os handles úteis
    error1, robot = sim.simxGetObjectHandle(clientID, robot_name, sim.simx_opmode_oneshot_wait)
    error2, dummy = sim.simxGetObjectHandle(clientID, dummy_name, sim.simx_opmode_oneshot_wait)
    error3, axis1 = sim.simxGetObjectHandle(clientID, robot_name + '_axis1', sim.simx_opmode_oneshot_wait)
    error4, axis2 = sim.simxGetObjectHandle(clientID, robot_name + '_axis2', sim.simx_opmode_oneshot_wait)
    error5, axis3 = sim.simxGetObjectHandle(clientID, robot_name + '_axis3', sim.simx_opmode_oneshot_wait)
    error6, axis4 = sim.simxGetObjectHandle(clientID, robot_name + '_axis4', sim.simx_opmode_oneshot_wait)
    
    errors = [error1, error2, error3, error4, error5, error6]
    
    if has_error(errors):
        print('Erro encontrado. Finalizando programa.')
        
        sim.simxGetPingTime(clientID)
        sim.simxFinish(clientID)
        
        print('Programa finalizado.')
        
        exit(0)
    
    # Cria stream de dados, como recomendado
    error1, position_robot = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
    error2, orientation_robot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming)
    error3, position_dummy = sim.simxGetObjectPosition(clientID, dummy, -1, sim.simx_opmode_streaming)
    error4, orientation_dummy = sim.simxGetObjectOrientation(clientID, dummy, -1, sim.simx_opmode_streaming)
    time.sleep(2)
    
    # Colocando robô em Home
    move_joints(clientID, [axis1, axis2, axis3, axis4], [0, 0, 0, 0])
    time.sleep(2)
    
    args = [
        [0, 0, 0, 0], # comandos atv2.py
        [np.pi/2, -np.pi/2, 0, 0], # comandos atv2.py
        [np.pi/2, -np.pi/2, 0.05, 0], # comandos atv2.py
        [-0.2043048091038573, 2.6823344700137253, 0.1, 3.141592653589793], # saída atv3.py
    ]
    
    counter = 0
    while(counter <= 130):
        counter += 1
        
        error1, (xd, yd, zd) = sim.simxGetObjectPosition(clientID, dummy, -1, sim.simx_opmode_buffer)
        error2, (ad, bd, gd) = sim.simxGetObjectOrientation(clientID, dummy, -1, sim.simx_opmode_buffer)
        
        error3, (xr, yr, zr) = sim.simxGetObjectPosition(clientID, dummy, -1, sim.simx_opmode_buffer)
        error4, (ar, br, gr) = sim.simxGetObjectOrientation(clientID, dummy, -1, sim.simx_opmode_buffer)
        
        sim.simxAddStatusbarMessage(clientID, f'Robot position x={xr} y={yr} z={zr}', sim.simx_opmode_oneshot_wait)
        sim.simxAddStatusbarMessage(clientID, f'Robot orientation x={ar} y={br} z={gr}', sim.simx_opmode_oneshot_wait)
        sim.simxAddStatusbarMessage(clientID, f'Dummy position x={xd} y={yd} z={zd}', sim.simx_opmode_oneshot_wait)
        sim.simxAddStatusbarMessage(clientID, f'Dummy orientation x={ad} y={bd} z={gd}', sim.simx_opmode_oneshot_wait)
        
        if counter % 34 == 0:
            p1, p2, p3, p4 = args[counter // 34]
            
            jointHandles = [axis1, axis2, axis3, axis4]
            jointPositions = [p1, p2, p3, p4]
            
            move_joints(clientID, jointHandles, jointPositions)
        
        time.sleep(0.05)
    
    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)
    
    # Pausar simulação
    sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot_wait)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
