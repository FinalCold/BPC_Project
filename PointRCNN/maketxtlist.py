import os

base_dir = '/home/mds/PointRCNN/data/KITTI/object'

path = os.path.join(base_dir, 'ImageSets/')

f = open('asd.txt', 'w')

for i in range(390):
    data = '%06d\n' % i
    f.write(data)

f.close()