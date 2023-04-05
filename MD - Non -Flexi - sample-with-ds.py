#!pip install haversine

#########################################################
#               Import the Libraries                    #
#########################################################
from haversine import haversine

#########################################################
#          Input Configurations & Parameters            #
#########################################################
# Number of Service and each service's pick-up/Drop Points
c = 5

# Pick-up Locations Should match with number of services
p = [[28.652121,77.252469], [28.656947,77.226495], [28.660599,77.238093], [28.653154,77.205982],[28.67953,77.244851]]

# Drop Locations Should match with number of services
d = [[28.659011,77.253293], [28.65534,77.244713], [28.671711,77.246385], [28.658814,77.227946],[28.650265,77.233947]]

# Location of Docking Station
# TBD: Assume 1, in future Multiple Docking Stations
ds = [28.657289,77.240934]

# Drone Travel capacity in Kms
DTC = 20

# Recent Drone Travel Capacity in Kms
RDTC = 0

# Current Drone Location Coordinates
drone_loc = []

# Service Distance
srvc_dist = 0

# Service Lists
srvc_dist_list = [0, 0, 0, 0, 0]
srvc_complt_list = []

# Initialization of Params before Computation
RDTC = DTC
drone_loc = ds
srvc_cmpltd = 0
dist_trvld = 0
tot_dist_trvld = 0

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

    # Service Distance is Less than the RDTC
    if (srvc_dist < RDTC):
        print("Service Accepted !! Drone Started =====> ")
        print("----------------------------------------")
        print("=====> Drone Reached Destination")

        # Calculate actual distance Travelled
        dist_trvld = (haversine(drone_loc, p[srvc_dist_idx]) + haversine(p[srvc_dist_idx], d[srvc_dist_idx]))
        RDTC = RDTC - dist_trvld
        tot_dist_trvld = tot_dist_trvld + dist_trvld
        print("Updated RDTC : " + str(RDTC))

        # Update Drone location after reaching Drop point
        drone_loc = d[srvc_dist_idx]
        srvc_complt_list.append(srvc_dist_idx)
        print("Updated Drone location : " + str(drone_loc))

        # Update Max value so that it will be rejected during the Min function
        srvc_dist_list[srvc_dist_idx] = DTC + 1

        # Update No. of services completed
        srvc_cmpltd = srvc_cmpltd + 1
        print("End of Service : " + str(srvc_cmpltd))

    else:
        print("Service Rejected, Going to Docking Station for Recharging")

        # After recharge RDTC will be max
        RDTC = DTC

        # Total distance travelled
        tot_dist_trvld = tot_dist_trvld + haversine(drone_loc, ds)
        drone_loc = ds

    print("\n\n")

# After Completion of Service Travel back to Docking station
print("After Completing services, Going to Docking Station for Recharging")
tot_dist_trvld = tot_dist_trvld + haversine(drone_loc, ds)
drone_loc = ds

print("Total Distance Traveled : " + str(tot_dist_trvld))

print("Path Taken : DS -> ", end="")
for i in range(c):
    print(str(srvc_complt_list[i] + 1) + " -> ", end="")
print("DS")