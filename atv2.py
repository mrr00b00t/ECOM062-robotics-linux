from ast import arguments
import functools
import numpy as np
import roboticstoolbox as rtb


## DEFINIÇÃO DAS CONSTANTES
l1 = 0.475
l2 = 0.4
do = 0.1

def fkine_diy(qr):
    theta1, theta2, d, theta3 = qr
    
    costheta1, sintheta1 = np.cos(theta1), np.sin(theta1)
    costheta2, sintheta2 = np.cos(theta2), np.sin(theta2)
    costheta3, sintheta3 = np.cos(theta3), np.sin(theta3)
    
    T0 = np.array([
        [1, 0, 0,   0],
        [0, 1, 0,   0],
        [0, 0, 1,  do],
        [0, 0, 0,   1],
    ])
    
    T1 = np.array([
        [costheta1, -sintheta1, 0,   l1*costheta1],
        [sintheta1,  costheta1, 0,   l1*sintheta1],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    
    T2 = np.array([
        [costheta2, sintheta2, 0, l2*costheta2],
        [sintheta2, -costheta2, 0, l2*sintheta2],
        [0, 0, -1, 0],
        [0, 0, 0, 1]
    ])
    
    T3 = np.array([
        [1,  0,  0, 0],
        [0, -1,  0, 0],
        [0,  0, -1, d],
        [0,  0,  0, 1],
    ])
    
    T4 = np.array([
        [costheta3, -sintheta3, 0, 0],
        [sintheta3, costheta3, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    
    return functools.reduce(np.matmul, [T0, T1, T2, T3, T4])

## INSTÂNCIA DO ROBÔ
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(d=do), # junta introduzida na origem para não precisar de T em y
        rtb.RevoluteDH(a=l1),
        rtb.RevoluteDH(a=l2, alpha=np.pi),
        rtb.PrismaticDH(alpha=np.pi, qlim=[0,0.1]),
        rtb.RevoluteDH()
    ],
    name="MTB - Scara ROBOT"
)

args = [
    [0, 0, 0, 0],
    [np.pi/2, -np.pi/2, 0, 0],
    [np.pi/2, -np.pi/2, 0.05, 0],
]

for arg in args:
    print('Argumentos [theta1, theta2, d, theta3] = {}'.format(arg))
    print('\nfkine (José Augusto): \n', fkine_diy(arg))
    print('\nfkine (DHRobot): \n', robot.fkine([0] + arg))

print(robot)

robot.teach()