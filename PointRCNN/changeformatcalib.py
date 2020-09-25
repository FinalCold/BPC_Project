import os

base_dir = '/home/mds/PointRCNN/data/KITTI/object'

path = os.path.join(base_dir, 'testing/calib/')

insert = '000000'

f = open(path + '000000.txt', 'r')
w = open(path + '000000_bak.txt', 'w')


for line in f.readlines():
    for word in line.split(' '):
        key = word
        if word.find('e') != -1:
            if word.find('+') != -1 or word.find('-') != -1:
                print(word)
                key = word.replace('e', insert + 'e')

        
        w.write(key + ' ')