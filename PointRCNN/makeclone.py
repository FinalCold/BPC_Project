import os

base_dir = '/home/mds/PointRCNN/data/KITTI/object'

path = os.path.join(base_dir, 'testing/calib/')

f = open(path + '000000_bak.txt', 'r')

for i in range(390):
    with open(path + '000000_bak.txt', 'r') as in_file:
        with open(path + '%06d' % i + '.txt', 'w') as out_file:
            for line in in_file:
                out_file.write(line)