#!pip install haversine

#########################################################
#               Import the Libraries                    #
#########################################################
from haversine import haversine
import os
#########################################################
#          Input Configurations & Parameters            #
#########################################################
# Number of Service and each service's pick-up/Drop Points
c = 9

# Pick-up Locations Should match with number of services
p = [[28.653469, 77.251024], [28.654532, 77.246686], [28.663073, 77.25436], [28.641485, 77.24153], [28.653577, 77.232575], [28.645161, 77.248366],
     [28.666858, 77.23011], [28.660274, 77.242965], [28.652262, 77.239646]]

# Drop Locations Should match with number of services
d = [[28.657753, 77.23106], [28.670906, 77.24036], [28.658403, 77.23459], [28.656355, 77.247538], [28.654296, 77.220133], [28.66167, 77.248379],
     [28.660958, 77.223051], [28.672792, 77.254861], [28.665174, 77.255888]]

# Location of Docking Station
# TBD: Assume 1, in future Multiple Docking Stations
ds = [28.657289,77.240934]

# Drone Travel capacity in Kms
DTC = 15

# Recent Drone Travel Capacity in Kms
RDTC = 0

# Current Drone Location Coordinates
drone_loc = []

# Service Distance
srvc_dist = 0

# Service Pick-up and Docking station Distance
srvc_ds_dist = 0

# Initialization of Params before Computation
RDTC = DTC
WRDTC = 0
drone_loc = ds
srvc_cmpltd = 0
dist_trvld = 0
tot_dist_trvld = 0

# Check for the Input File
if os.path.isfile('inp.csv'):
    # Open the Input CSV File
    inp_file = open('inp.csv', 'r')
    line = inp_file.readline()

    print("Input File Found")

    # Update the Range from File
    c = int(line.strip('\n').split(',')[1])
    p.clear()
    d.clear()
    for i in range(0, c):
        line = inp_file.readline()
        data = line.strip('\n').split(',')
        p.append([])
        d.append([])
        p[i].append(float(data[1]))
        p[i].append(float(data[2]))
        d[i].append(float(data[3]))
        d[i].append(float(data[4]))

    line = inp_file.readline()
    data = line.strip('\n').split(',')
    ds[0] = float(data[1])
    ds[1] = float(data[2])

    inp_file.close()

else:
    print("No Input File")

# Service Lists
srvc_dist_list = [None] * c
srvc_complt_list = []
srvc_path_traj = []

#######################################################
#               Computation Algorithm                 #
#######################################################

while srvc_cmpltd < c:
    print("Start of Service : " + str(srvc_cmpltd + 1))

    for i in range(c):
        # Check for pending Services
        if i not in srvc_complt_list:
            srvc_dist = (haversine(drone_loc, p[i]) + haversine(p[i], d[i]) + haversine(d[i], ds))
            print("Computed Distance for Service - " + str(i + 1) + " : " + str(srvc_dist))
            srvc_dist_list[i] = srvc_dist

    # Find Minimum Service distance among list
    srvc_dist = min(srvc_dist_list)
    print("Min Service Distance : " + str(srvc_dist))

    # Get the Index of current service
    srvc_dist_idx = srvc_dist_list.index(srvc_dist)
    print("Service Distance Index : " + str(srvc_dist_idx + 1))

    # Find distance of path as Drone Location -> Pick up -> DS
    srvc_ds_dist = haversine(drone_loc, p[srvc_dist_idx]) + haversine(p[srvc_dist_idx], ds)
    print("Flexi Distance : " + str(srvc_ds_dist))

    # Service Distance is Less than the RDTC
    if (srvc_dist < RDTC):
        print("Service Accepted !! Drone Started =====> ")
        print("----------------------------------------")
        print("=====> Drone Reached Destination")

        # Calculate Actual distance Travelled
        dist_trvld = (haversine(drone_loc, p[srvc_dist_idx]) + haversine(p[srvc_dist_idx], d[srvc_dist_idx]))
        RDTC = RDTC - dist_trvld
        tot_dist_trvld = tot_dist_trvld + dist_trvld
        print("Updated RDTC : " + str(RDTC))

        # Update Drone location after reaching Drop point
        drone_loc = d[srvc_dist_idx]
        srvc_complt_list.append(srvc_dist_idx)
        srvc_path_traj.append(str(srvc_dist_idx + 1) + "P")
        srvc_path_traj.append(str(srvc_dist_idx + 1) + "D")
        print("Updated Drone location : " + str(drone_loc))

        # Update Max value so that it will be rejected during the Min function
        srvc_dist_list[srvc_dist_idx] = DTC + 1

        # Update No. of services completed
        srvc_cmpltd = srvc_cmpltd + 1
        print("End of Service : " + str(srvc_cmpltd))

    # Check for Flexible Service allocation
    elif (srvc_ds_dist < RDTC):
        print("Flexi Service Accepted !! Drone Started =====> ")
        print("------- Drone reached Pick-up Location --------")
        print("------- Drone reached Docking Station ---------")
        print("=====> Drone Reached Destination")

        # Calculate Actual distance Travelled
        dist_trvld = (haversine(drone_loc, p[srvc_dist_idx]) + haversine(p[srvc_dist_idx], ds) + haversine(ds, d[
            srvc_dist_idx]))
        tot_dist_trvld = tot_dist_trvld + dist_trvld

        # Updated RDTC
        RDTC = DTC - haversine(ds, d[srvc_dist_idx])
        print("Updated RDTC : " + str(RDTC))

        # Update Drone Location
        drone_loc = d[srvc_dist_idx]
        srvc_complt_list.append(srvc_dist_idx)
        srvc_path_traj.append(str(srvc_dist_idx + 1) + "P")
        srvc_path_traj.append("DS")
        srvc_path_traj.append(str(srvc_dist_idx + 1) + "D")
        print("Updated Drone location : " + str(drone_loc))

        # Update Max value so that it will be rejected during the Min function
        srvc_dist_list[srvc_dist_idx] = DTC + 1

        # Update No. of services completed
        srvc_cmpltd = srvc_cmpltd + 1
        print("End of Service : " + str(srvc_cmpltd))

    # Service Rejected
    else:
        print("Service Rejected, Going to Docking Station for Recharging")
        print("Wasted Distance : " + str(RDTC))

        # Backup wasted distance and recharge RDTC to Max
        WRDTC = WRDTC + RDTC
        RDTC = DTC

        # Append Travel to Docking station in the Path
        srvc_path_traj.append("DS")

        # Total distance travelled
        tot_dist_trvld = tot_dist_trvld + haversine(drone_loc, ds)
        drone_loc = ds

    print("\n\n")

# After Completion of Service Travel back to Docking station
print("After Completing services, Going to Docking Station for Recharging")
tot_dist_trvld = tot_dist_trvld + haversine(drone_loc, ds)
drone_loc = ds

print("Total Distance Traveled : " + str(tot_dist_trvld))
print("Total Wasted Distance   : " + str(WRDTC))

# Iterate over all completed services and print Path Taken
print("Path Taken : DS -> ", end="")
for i in srvc_path_traj:
    print(i + " -> ", end="")
print("DS")