from xml.etree.ElementTree import Element, SubElement, ElementTree

f = open('C:/Users/dh953/OneDrive/바탕 화면/result.txt', 'r')
lines_li = []
lines = f.readlines()

for i in range(len(lines)):
    lines_li.append(lines[i])

line = []
backup = []  # backup_list of line[]
l = []
for i in range(len(lines_li)):
    l = lines_li[i].split()
    line.append(l)
    backup.append(l)
# print(line)

row = len(line)
print(row)
column = len(line[0])

for i in range(row):
    backup[i][0] = int(backup[i][0])  # convert frame_number string to int
    backup[i][1] = int(backup[i][1])  # convert track_id string to int
# print(backup)


backup.sort(key=lambda x: x[1])   # sort by track_id
print(backup)

filename = 'result'

root = Element('tracklets')     # <tracklets> root_tag
j = 0
# make tag
# for i in range(row):
i = 0
while(i<=row):
    temp=i
    i += j

    item1 = SubElement(root, 'item')     # <item> sub1_tag

    vehicle_type = backup[i][2]
    SubElement(item1, 'objectType').text = str(vehicle_type)

    count = backup[i][1]
    SubElement(item1, 'count').text = str(count)

    if i + j == row - 1:
        first_f = backup[i][0]
        frame = first_f
        first_h = backup[i][10]
        first_w = backup[i][11]
        first_l = backup[i][12]

        dimension_height = first_h
        dimension_width = first_w
        dimension_length = first_l

        SubElement(item1, 'h').text = str(dimension_height)
        SubElement(item1, 'w').text = str(dimension_width)
        SubElement(item1, 'l').text = str(dimension_length)
        SubElement(item1, 'first_frame').text = str(frame)

        poses = SubElement(item1, 'poses')

        item = SubElement(poses, 'item')
        location_x = backup[j+i][13]
        location_y = backup[j+i][14]
        location_z = backup[j+i][15]
        rotation_y = backup[j+i][16]
        SubElement(item, 'tx').text = str(location_x)
        SubElement(item, 'ty').text = str(location_y)
        SubElement(item, 'tz').text = str(location_z)
        SubElement(item, 'rz').text = str(rotation_y)

        # print("3. i: %s \tj:%s, \tj+i %s" % (i, j, j+i))
        break

    if i == 0:
        first_f = backup[i][0]
        frame = first_f
        first_h = backup[i][10]
        first_w = backup[i][11]
        first_l = backup[i][12]

    elif i != temp:
        # print("i : %s"%i)
        first_f = backup[i][0]
        frame = first_f
        first_h = backup[i][10]
        first_w = backup[i][11]
        first_l = backup[i][12]
    else:
        frame = first_f

    dimension_height = first_h
    dimension_width = first_w
    dimension_length = first_l

    SubElement(item1, 'h').text = str(dimension_height)
    SubElement(item1, 'w').text = str(dimension_width)
    SubElement(item1, 'l').text = str(dimension_length)
    SubElement(item1, 'first_frame').text = str(frame)

    poses = SubElement(item1, 'poses')

    # print("poses")
    # print("1. i: %s \tj:%s, \tj+i %s\n" % (i, j, j + i))
    j = 0
    # print("2. i: %s \tj:%s, \tj+i %s\n" % (i, j, j + i))
    while(backup[j+i][1] <= backup[j+i+1][1]):
        # print("-> item")
        item = SubElement(poses, 'item')
        location_x = backup[j+i][13]
        location_y = backup[j+i][14]
        location_z = backup[j+i][15]
        rotation_y = backup[j+i][16]
        SubElement(item, 'tx').text = str(location_x)
        SubElement(item, 'ty').text = str(location_y)
        SubElement(item, 'tz').text = str(location_z)
        SubElement(item, 'rz').text = str(rotation_y)
        # print("%s, %s"%(backup[j][1],location_x))
        # print("3. i: %s \tj:%s, \tj+i %s" % (i, j, j+i))


        if backup[j+i][1] != backup[j+i+1][1]:
            # print("break\ti+j: %s"%(i+j))
            break
        # print("j:  %s, j+1: %s" % (j,j+1))
        j += 1
    i += 1
    # print("\n\n\n")



tree = ElementTree(root)
f.close()
tree.write('C:/Users/dh953/OneDrive/바탕 화면/' + filename +'.xml')