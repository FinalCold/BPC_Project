import os, numpy as np, time, sys

base_dir = '/home/mds/data/'
file_list = os.listdir(base_dir)

l = len(file_list)

print('total files : ', l)

count = 0

for l in range(l+1):

    try:
        open_dir = os.path.join(base_dir, (str(l+1).zfill(6)+'.txt'))
        f = open(open_dir)

        for line in f.readlines():
                   
            line_1 = line[9:]
            lst = line_1.split()  
            lst_ = [str(l), str(2)]
            list_str_ = lst_ + lst

            tmp_ori = list_str_.pop(2)
            tmp_scr = list_str_.pop(-1)
        
            list_str_.insert(-1, tmp_scr)
            list_str_.insert(6, tmp_ori)
            rst = np.array([list_str_])
            # print(rst)

            if count == 0 :
                
                result = rst
                
            else :  
                result = np.append(result, rst, axis=0)
                
            count += 1

            print_str = 'processing %d \r' % (count)

            sys.stdout.write(print_str)
            sys.stdout.flush()

    except :
        pass

print('result shape : ', result.shape)

np.savetxt(base_dir+'result.txt', result, '%s')

print("file saved '/home/mds/data/result.txt'")

f.close()
