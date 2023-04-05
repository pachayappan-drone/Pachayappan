from random import uniform
import pandas as pd
from haversine import haversine, Unit

my_file = pd.read_csv("sample.csv")

distance = []

for i in range(2324):
    data_x = (my_file['X-Pickup'][i],my_file['Y-Pickup'][i])
    data_y = (my_file['X-Drop'][i], my_file['Y-Drop'][i])
    distance.append(haversine(data_x,data_y))

data = pd.DataFrame({'distance': distance})
data.to_csv('Sample_result.csv')