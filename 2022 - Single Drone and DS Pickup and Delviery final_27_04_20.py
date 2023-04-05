# initialize all matrices
#p_mat, d_mat, pd_mat, p_rs, d_rs = [], [], [], [], []

#print('pd_mat-> ', pd_mat)

p_rs = [3.61,1,5.66,1.41,3.61]
d_rs = [2.24,3.16,3.61,6.4,1.41]
p_mat = [[0,2.83,2.24,4.12,7.07], [2.83,0,5,2.24,4.24], [2.24,5,0,5.83,9.22],[4.12,2.24,5.83,0,4.12]]
d_mat = [[0,4.12,5.66,8.49,3.61], [4.12,0,3,5.39,2.83], [5.66,3,0,2.83,2.24],[8.49,5.39,2.83,0,5],[3.61,2.83,2.24,5,0]]
pd_mat = [[5.83,4.12,1.41,3.16,2.24], [3.16,3.61,3.16,5.83,1], [7.81,5.1,2.24,1,4.24],[2.24,2,3.61,6.4,2],[2,6.08,7.21,10,5]]

# p_rs = [0.37,0.55,1.58]
# d_rs = [0.5,0.8,1.53]
# p_mat = [[0, 0.69, 1.7], [0.69, 0, 2.11], [1.7, 2.11, 0]]
# d_mat = [[0, 0.45, 1.84], [0.45, 0, 2.27], [1.84,2.27, 0]]
# pd_mat = [[0.16,0.6,1.68], [0.67,0.6,2.05], [1.86,2.29,0.15]]

pp_rs = p_rs[:]
# logic starts from here
path = []
route = []
starting_point_elements = []
final_distance = 0
distance = []
while len(route) < len(p_rs):
    sub_path = []
    for i in route:
        p_rs[i] = 10**10
    # identify starting point
    starting_point_elements = []
    for i in range(len(p_rs)):
        starting_point_elements.append(p_rs[i] + pd_mat[i][i] + d_rs[i])
    min_index = starting_point_elements.index(min(starting_point_elements))
    distance_travelled = starting_point_elements[min_index]
    if distance_travelled <= 20:
        sub_path.append(min_index)
        route.append(min_index)
        distance_travelled = distance_travelled - d_rs[min_index]
        distance.append(distance_travelled)
        while distance_travelled <= 20:
            next_route = []
            for i in range(len(pd_mat)):
                if i not in route:
                    next_route.append(pd_mat[i][min_index] + pd_mat[i][i] + d_rs[i])
                else:
                    next_route.append(10**10)
            min_next_route_index = next_route.index(min(next_route))
            # introduced one -d_rs[] here
            distance_travelled = distance_travelled + (next_route[min_next_route_index] - d_rs[min_next_route_index])
            if distance_travelled <= 20:
                route.append(min_next_route_index)
                sub_path.append(min_next_route_index)
                path.append(sub_path)
            else:
                path.append(sub_path)
                #print("Max distance exceeded")
path_final = []
for i in path:
    if i not in path_final:
        path_final.append(i)
if len(path_final[-1]) == 1:
    final_distance = final_distance + starting_point_elements[path_final[-1][0]]
#print(path_final, final_distance)
print("-------- Final Result ---------")
new_list = ""
for i in path_final:
    new_list = new_list + "RS - "
    for j in i:
        new_list = new_list + str(j+1) + " - "
new_list = new_list + "RS"
print("Path = ", new_list)

dist, temp = 0, 0
dist_mat = []
for i in range(len(path_final)):
    first_element, second_element = 0, 0
    for j in range(len(path_final[i])):
        if len(path_final[i]) == 1:
            dist = pp_rs[path_final[i][j]] + pd_mat[path_final[i][j]][path_final[i][j]] + d_rs[path_final[i][j]]
            temp = 1
            break

        if j == 0 and temp == 0:
            dist = pp_rs[path_final[i][j]] + pd_mat[path_final[i][j]][path_final[i][j]]

        else:
            #print('----')
            first_element = path_final[i][i]
            second_element = path_final[i][j]
            dist = dist + pd_mat[second_element][second_element - 1] + pd_mat[second_element][second_element]
    if temp == 0:
        dist = dist + d_rs[second_element]
    else:
        dist = dist
    dist_mat.append(dist)

print("Total distance travelled = ", sum(dist_mat))
