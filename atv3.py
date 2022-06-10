import numpy as np


l1 = 0.475
l2 = 0.4
do = 0.1
d_min = 0
d_max = 0.1

def ikine(qr):
    x, y, z, phi = qr
    
    # espaço de trabalho
    flag1 = x**2 + y**2 >= (l1-l2)**2
    flag2 = (do - d_max) <= z <= (do - d_min)
    
    if not flag1 or not flag2:
        return 'Posição não alcançável!'
    
    d = do - z
    D = (x**2 + y**2 - l1**2 - l2**2) / (2*l1*l2)
    
    theta2 = np.arctan2(np.sqrt(1 - D**2), D)
    theta1 = np.arctan2(y, x) - np.arctan2(l2*np.sin(theta2), (l1 + l2*np.cos(theta2)))
    
    return [theta1, theta2, d, phi]

qr1 = [0.2, 0.1, -0.015, np.pi/4]
qr2 = [0.5, 0.1, -0.015, np.pi/4]
qr3 = [0.15, 0.15, 0, np.pi]

qrs = [qr1, qr2, qr3]

print('Saidas da ikine([x, y, z, phi]) = [theta1, theta2, d, theta3]')
for qr in qrs:
    print(ikine(qr))