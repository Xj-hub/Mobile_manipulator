#! /usr/bin/env python
import os
import sys
import rospkg
import csv

from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
import time

def create_init_dict(filename):
    data = pd.read_csv(filename,names=['index','lat','lon'])
    data_dict = {}
    for i in range(len(data)):
        data_dict[i] = (data.iloc[i]['lat'],data.iloc[i]['lon'])
    return data_dict
 
 
def distance_matrix(coordinate_dict, size):  # generate distance matrix
    d = np.zeros((size + 2, size + 2))
    for i in range(size + 1):
        for j in range(size + 1):
            if (i == j):
                continue
            if (d[i][j] != 0):
                continue
            x1 = coordinate_dict[i][0]
            y1 = coordinate_dict[i][1]
            x2 = coordinate_dict[j][0]
            y2 = coordinate_dict[j][1]
            distance = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
            if (i == 0):
                d[i][j] = d[size + 1][j] = d[j][i] = d[j][size + 1] = distance
            else:
                d[i][j] = d[j][i] = distance
    return d

def path_length(d_matrix, path_list, size):  # calculate path len
    length = 0
    for i in range(size + 1):
        length += d_matrix[path_list[i]][path_list[i + 1]]
    return length
 
def shuffle(my_list):#start and end point not change
    temp_list=my_list[1:-1]
    np.random.shuffle(temp_list)
    shuffle_list=my_list[:1]+temp_list+my_list[-1:]
    return shuffle_list
def product_len_probability(my_list,d_matrix,size,p_num):
    len_list=[]
    pro_list=[]
    path_len_pro=[]
    for path in my_list:
        len_list.append(path_length(d_matrix,path,size))
    max_len=max(len_list)+1e-10
    gen_best_length=min(len_list)
    gen_best_length_index=len_list.index(gen_best_length)
    mask_list=np.ones(p_num)*max_len-np.array(len_list)
    sum_len=np.sum(mask_list)
    for i in range(p_num):
        if(i==0):
            pro_list.append(mask_list[i]/sum_len)
        elif(i==p_num-1):
            pro_list.append(1)
        else:
            pro_list.append(pro_list[i-1]+mask_list[i]/sum_len)
    for i in range(p_num):
        path_len_pro.append([my_list[i],len_list[i],pro_list[i]])
    return my_list[gen_best_length_index],gen_best_length,path_len_pro
 
def choose_cross(population,p_num):
    jump=np.random.random()
    if jump<population[0][2]:
        return 0
    low=1
    high=p_num
    mid=int((low+high)/2)
    #binary search
    while(low<high):
        if jump>population[mid][2]:
            low=mid
            mid=int((low+high)/2)
        elif jump<population[mid-1][2]:
            high=mid
            mid = int((low + high) / 2)
        else:
            return mid
def veriation(my_list,size):#mutation
    ver_1=np.random.randint(1,size+1)
    ver_2=np.random.randint(1,size+1)
    while ver_2==ver_1: #until ver1 and ver2 is not the same
        ver_2 = np.random.randint(1, size+1)
    my_list[ver_1],my_list[ver_2]=my_list[ver_2],my_list[ver_1]
    return my_list

def main(spots_file_name, plot = False):
    # use rospack to get package info
    rospack = rospkg.RosPack()
    # We clean up spots folder
    spots_folder_path = os.path.join(rospack.get_path('mobile_base_navigation'), "spots")

    csv_path = os.path.join(spots_folder_path, spots_file_name)
    data = create_init_dict(csv_path)
    print (data)
    start = time.time()
    # size=10
    p_num=100#how many individuals in a group 
    gen=4000#how many generations
    pm=0.1#mutate rate
    # coordinate_dict_1=coordinate_init(size)
    coordinate_dict=create_init_dict(csv_path)
    size=len(coordinate_dict)-2
    print ('--',coordinate_dict)
    print (size)
    d=distance_matrix(coordinate_dict,size)
    # print(coordinate_dict)
    path_list=list(range(size+2))
    print(path_list)#print the initial path
    population=[0]*p_num#group matrix
    #build the initial group
    for i in range(p_num):
        population[i]=shuffle(path_list)
    gen_best,gen_best_length,population=product_len_probability(population,d,size,p_num)
    # print(population)# path, len, probability
    son_list=[0]*p_num
    best_path=gen_best#initialize best path
    best_path_length=gen_best_length#initialize best path len
    every_gen_best=[]
    every_gen_best.append(gen_best_length)
    for i in range(gen):#iteration
        son_num=0
        while son_num<p_num:#produce p_num childrens
            father_index = choose_cross(population, p_num)
            mother_index = choose_cross(population, p_num)
            father=population[father_index][0]
            mother=population[mother_index][0]
            son1=father[:]
            son2=mother[:]
            prduct_set=np.random.randint(1,p_num+1)
            father_cross_set=set(father[1:prduct_set])
            mother_cross_set=set(mother[1:prduct_set])
            cross_complete=1
            for j in range(1,size+1):
                if son1[j] in mother_cross_set:
                    son1[j]=mother[cross_complete]
                    cross_complete+=1
                    if cross_complete>prduct_set:
                        break
            if np.random.random()<pm:#mutation
                son1=veriation(son1,size)
            son_list[son_num]=son1
            son_num+=1
            if son_num==p_num: break
            cross_complete=1
            for j in range(1,size+1):
                if son2[j] in father_cross_set:
                    son2[j]=father[cross_complete]
                    cross_complete+=1
                    if cross_complete>prduct_set:
                        break
            if np.random.random()<pm:#mutation
                son2=veriation(son2,size)
            son_list[son_num]=son2
            son_num+=1
        gen_best, gen_best_length,population=product_len_probability(son_list,d,size,p_num)
        if(gen_best_length<best_path_length):
            best_path=gen_best
            best_path_length=gen_best_length
        every_gen_best.append(gen_best_length)
    x=[]
    y=[]
    for point in best_path:
        x.append(coordinate_dict[point][0])
        y.append(coordinate_dict[point][1])
    print(gen_best)#last generation best path
    print(gen_best_length)#last generation best path len
    print(best_path)#best path in all history
    print(best_path_length)#best path len in all history

    
    save_csv_path = os.path.join(spots_folder_path, "reorg_" + spots_file_name)
    with open(save_csv_path, "w") as csvfile:
        writer = csv.writer(csvfile)
        for node_index in best_path:
            writer.writerow([node_index, coordinate_dict[node_index][0], coordinate_dict[node_index][1]])
    if plot:
        plt.figure(1)
        plt.subplot(211)
        plt.plot(every_gen_best)
        plt.subplot(212)
        plt.scatter(x,y)
        plt.plot(x,y)
        plt.grid()
        plt.show()

if __name__=="__main__":
    if len(sys.argv) < 2:
        print("usage: reorganize_pose.py csv_file_name")
    else:
        csv_file_name = str(sys.argv[1])+".csv"
        plot = True
        main(csv_file_name, plot = plot)


