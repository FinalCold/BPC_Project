import os


base_dir = '/home/mds/PointRCNN/data/KITTI/object/training_4'

path = os.path.join(base_dir, 'calib/')


for i in range(390):
    os.rename(path + '%010d' % i + '.txt', path + '%06d' % i + '.txt')