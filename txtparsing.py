import os
import numpy as np

base_dir = '/home/mds/2011_09_26'

path = os.path.join(base_dir, 'calib_cam_to_cam.txt')

file = open(path)

for line in file.readlines():
    for word in line.split(' '):
        if word == 'P_rect_00:':
            P0 = line[11:-1]
        if word == 'P_rect_01:':
            P1 = line[11:-1]
        if word == 'P_rect_02:':
            P2 = line[11:-1]
        if word == 'P_rect_03:':
            P3 = line[11:-1]
        if word == 'R_rect_00:':
            R0_rect = line[11:-1]

path = os.path.join(base_dir, 'calib_velo_to_cam.txt')
cam = 'P0: ' + P0 + '\n' + 'P1: ' + P1 + '\n' + 'P2: ' + P2 + '\n' + 'P3: ' + P3 + '\n' + 'R0_rect: ' + R0_rect + '\n'

file = open(path)

for line in file.readlines():
    for word in line.split(' '):
        if word == 'R:':
            vtocR = line[3:-1]
        if word == 'T:':
            vtocT = line[3:-1]

vtoc = 'Tr_velo_to_cam: ' + vtocR + ' ' + vtocT

path = os.path.join(base_dir, 'calib_imu_to_velo.txt')

file = open(path)

for line in file.readlines():
    for word in line.split(' '):
        if word == 'R:':
            itovR = line[3:-1]
        if word == 'T:':
            itovT = line[3:-1]

itov = 'Tr_imu_to_velo: ' + itovR + ' ' + itovT

save = cam + vtoc + '\n' +  itov

path_calib = os.path.join(base_dir, 'calib/')

for i in range(0, 390):

    f = open(path_calib + '%010d' % i + '.txt', 'w')

    f.write(save)

f.close()