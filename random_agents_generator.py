import random

def random_generator(mapname, my_map, rows, columns, start, goal):
    agent_num = random.randrange(10, 50)
    count = 0
    name = mapname  + '_random_' + str(agent_num) + '.txt'
    print("Random obstacle file " + name + " is created")
    with open(name, 'w') as f:
        while count < agent_num:
            result_dict = {}
            sx = random.randrange(0, rows)
            sy = random.randrange(0, columns)
            gx = random.randrange(0, rows)
            gy = random.randrange(0, columns)
            if (sx, sy) == start and (gx, gy) == goal:
                continue
            if my_map[sx][sy] or my_map[gx][gy]:
                continue
            tmp = [sx, sy, gx, gy]
            condition = tuple(tmp)
            if condition not in result_dict:
                result_dict[condition] = 1
                count+=1
                # print(sx , '\t' , sy , '\t' , gx , '\t' , gy)
                f.write(str(sx) + "\t" + str(sy) + "\t" + str(gx) + "\t" + str(gy))
                f.write("\n")
    return name