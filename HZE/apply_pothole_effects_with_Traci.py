import traci
import os
import json
import pandas as pd
import math
import time

# path to config-file
###HZE
NET_FILE = os.path.abspath("PotholeIntegrationInSUMO/HZE/osm.net.xml.gz")
POTHOLE_FILE = os.path.abspath("PotholeIntegrationInSUMO/HZE/potholes.data.json")
SUMO_CONFIG = os.path.abspath("PotholeIntegrationInSUMO/HZE/osm.sumocfg")
REAL_WORLD_WITH_SPEED = False  # set speed before passing the pothole to speed from pothole file mean_speed variable not necessary on fake data
ONCE_PER_POTHOLE = False
NEXT_POTHOLE = ""
start_vehicle_id = "veh10"  ##vehicle of interest (HZE car route)

# ###real world
# NET_FILE = os.path.abspath("PotholeIntegrationInSUMO/DataSet/output/net.net.xml")
# POTHOLE_FILE = os.path.abspath(
#     "PotholeIntegrationInSUMO/DataSet/output/potholesCorrection.json"
# )
# SUMO_CONFIG = os.path.abspath("PotholeIntegrationInSUMO/DataSet/output/config.sumocfg")
# REAL_WORLD_WITH_SPEED = False # set speed before passing the pothole to speed from pothole file mean_speed variable not necessary on fake data, mean speed might be part of a data set
# ONCE_PER_POTHOLE = False
# NEXT_POTHOLE = ""
# start_vehicle_id = "veh1"  ##vehicle of interest (real car route)

# ###Testing
# NET_FILE = os.path.abspath("PotholeIntegrationInSUMO/HZE/osm.net.xml.gz")
# POTHOLE_FILE = os.path.abspath(
#     "PotholeIntegrationInSUMO/HZE/Testing/potholes.data.json"
# )
# SUMO_CONFIG = os.path.abspath("PotholeIntegrationInSUMO/HZE/Testing/osm.sumocfg")
# REAL_WORLD_WITH_SPEED = False # set speed before passing the pothole to speed from pothole file mean_speed variable not necessary on fake data
# ONCE_PER_POTHOLE = False
# NEXT_POTHOLE = ""
# start_vehicle_id = "veh10"  ##vehicle of interest (HZE car route)


######### simulation variables
### evaluation
EVALUATION_PATH = os.path.abspath("PotholeIntegrationInSUMO/Evaluation")
NO_EFFECT_APPLIED = False
### test for calculations
WITH_CALCULATION = False
WITH_CALCULATION_IDM = False
WITH_CALCULATION_SPEED_REDUCE = False
### prints on/off
PRINTS_ON = False
### speed on potholes
RED_POTHOLE_SPEED_REDUCE = 25
YELLOW_POTHOLE_SPEED_REDUCE = 35
GREEN_POTHOLE_SPEED_REDUCE = -1
### depth boarders for potholes
RED_DEPTH = 0.075
YELLOW_DEPTH = 0.05
### distances
PASSED_DISTANCE = -5
LOOK_BEHIND_DISTANCE = -2.5
BRAKING_TOLERANCE_DISTANCE_LOWER_SPEED = 3.0
BRAKING_TOLERANCE_DISTANCE_HIGHER_SPEED = 8.0
######### SUMO
SUMO_BINARY = "sumo-gui"
STEP_SIZE = 0.1
traci.start(
    [
        SUMO_BINARY,
        "-c",
        SUMO_CONFIG,
        "--collision.action",
        "warn",
        "--step-length",
        str(STEP_SIZE),
    ]
)

start_vehicle_started = False
step = 0
pothole_step = 0
once = True
speedAtPothole = 0
atPothole = False

severity = ["green", "yellow", "red"]
# print(severity[0],severity[1],severity[2])
with open(POTHOLE_FILE, "r") as f:
    potholes = json.load(f)

# bufferList=[]
vehicle_data = {}
vehicle_data_all = {}
pothole_data = {}
prio_list = {}
evaluation_records = []


def depth_to_severity(depth):
    if depth > RED_DEPTH:
        return severity[2]
    return severity[1] if depth > YELLOW_DEPTH else severity[0]


def severity_to_speed(severityOfPothole, v0_speed):
    speedLimitLane = v0_speed

    # if severityOfPothole == "green":
    #     return speedLimitLane
    # if speedLimitLane < 8.4:
    #     if severityOfPothole == "yellow":
    #         return 25 / 3.6
    #     if severityOfPothole == "red":
    #         return 20 / 3.6
    # if severityOfPothole == "yellow":
    #     return 35 / 3.6
    # if severityOfPothole == "red":
    #     return 25 / 3.6
    if severityOfPothole == "green":
        return GREEN_POTHOLE_SPEED_REDUCE
    desired_speed = 0
    if severityOfPothole == "yellow":
        desired_speed = YELLOW_POTHOLE_SPEED_REDUCE / 3.6

    if severityOfPothole == "red":
        desired_speed = RED_POTHOLE_SPEED_REDUCE / 3.6
    if desired_speed < speedLimitLane:
        return desired_speed
    return -1


def pothole_speed_calculation(
    v0, h, w, alpha=3, eps=0.2
):  ##testing origin was alpha 4.5 and esp 0.2
    return v0 * math.exp(-alpha * h / (w + eps))


###pothole has nearly no influence on the speed with this calculation it impacts mainly on this term s_term = (s_star / h)^2


def pothole_speed_calculation_IDM(
    v,
    v_D,
    W,
    D,
    h,
    a_max=0.5,
    s_star=2.0,
    delta_base=4.0,
    dt=0.1,
    t_reaction=6,
    depth_factor=1,
    v_min=5.0,
    h_safe_min=2,
):

    #     Calculates the speed at the pothole based on the paper formula.

    #     Parameters:
    #     ----------
    #     v : float
    #         current vehicle speed [m/s]
    #     v_D : float
    #         normal desired speed [m/s]
    #     W : float
    #         width of the pothole [m]
    #     D : float
    #         depth of the pothole [m]
    #     h : float
    #         Distance to the pothole [m]
    #     a_max : float
    #         Maximum acceleration/deceleration [m/s^2] (default 2.0)
    #     s_star : float
    #         Desired safety distance [m] (default 2.0)
    #     delta_base : float
    #         Base exponent for IDM (default 4.0)
    #     dt : float
    #         Simulation step [s] (default 0.1)

    #     Return:
    #     ----------
    #     v_pothole : float
    #         Target speed for the pothole [m/s]
    #     “”"

    h_influence = max(v * t_reaction, 0.1)
    h_safe = max(h, h_safe_min)

    if h_safe >= h_influence:
        delta_eff = 0.0
    else:
        delta_eff = (
            -0.5
            * math.pi
            * W
            * (1 - h_safe / h_influence)
            * math.sqrt(D**2 + W**2 / 4)
            * depth_factor
        )
    delta_eff = 200
    v_term = (v / v_D) ** delta_eff if delta_eff != 0 else 0
    s_term = (s_star / h_safe) ** 2
    v_dot = a_max * (1 - v_term - s_term)

    v_next = v + v_dot * dt
    # v_next = max(v_min, min(v_next, v_D))

    return v_next


def desired_speed_pothole_last_test(
    v,  # current speed [m/s]
    v_D,  # desired speed [m/s]
    h,  # distance to pothole [m]
    W,  # pothole width [m]
    D,  # pothole depth [m]
    tau=1.0,
    kappa=0.3,
    D_ref=0.04,
):
    if h <= 0:
        return v  # pothole passed

    influence = max(0.0, 1.0 - h / (v * tau))
    # influence = min(influence, 0.7)

    geom = math.sqrt(W**2 / 4 + D**2)
    geom_ref = math.sqrt(W**2 / 4 + D_ref**2)
    # geom_factor = 1 + 0.5 * (D / D_ref - 1)

    # reduction = kappa * geom_factor * influence
    reduction = kappa * (geom / geom_ref) * influence
    v_pothole = v_D * (1 - reduction)

    return max(0.1, v_pothole)


def final_desired_speed(
    v_D, D, W, kappa=0.3, D_ref=0.04, W_ref=0.5, alpha_depth=0.7, alpha_width=0.3
):

    geom_factor = 1 + alpha_depth * (D / D_ref - 1) + alpha_width * (W_ref / W - 1)
    reduction_max = kappa * geom_factor * 1.0
    v_end = v_D * (1 - reduction_max)
    return max(0.1, v_end)


def pothole_end_speed(
    v_desired,
    depth,
    width=0.5,
    tau_r=1.0,
    kappa=0.6,
    gamma=2.0,
    D_ref=0.05,
    v_min=2.0,
):

    geom = math.sqrt((width**2) / 4 + depth**2)

    depth_factor = (depth / D_ref) ** gamma

    reduction = kappa * (math.pi * width * tau_r) / 2 * geom * depth_factor
    reduction = min(reduction, 0.9)

    v_pothole = v_desired * (1 - reduction)

    # return max(v_min, v_pothole)
    return v_pothole


### not used anymore since set speed from traci worked fine
def apply_accel_to_set_speed(desired_speed, speed, vehicle_ID):
    dtSeconds = traci.simulation.getDeltaT()  # / 1000.0
    accel = traci.vehicle.getAccel(vehicle_ID)
    if speed < desired_speed:
        speed += accel * dtSeconds
        if PRINTS_ON:
            print(f"accel applied with: {str(accel)} deltaT is: {str(dtSeconds)}")
        return speed
    return speed


### not used anymore since set speed from traci worked fine
def apply_decel_to_set_speed(desired_speed, speed, vehicle_ID):
    dtSeconds = traci.simulation.getDeltaT()  # / 1000.0
    decel = traci.vehicle.getDecel(vehicle_ID)
    if PRINTS_ON:
        print("speed:", speed)
    if speed > desired_speed:
        if speed < decel:
            speed -= speed * dtSeconds
            # speed=desired_speed
            if PRINTS_ON:
                print("speed<decel:", speed)
        else:
            speed -= decel * dtSeconds
            if PRINTS_ON:
                print("normal decel", speed)
        if PRINTS_ON:
            print(f"decel applied with: {str(decel)} deltaT is: {str(dtSeconds)}")
        return speed
    return speed


def calculate_braking_distances_based_on_desired_speed(
    startSpeed, desiredSpeed, deceleration
):
    # green pothole default value 5m
    tolerance = BRAKING_TOLERANCE_DISTANCE_LOWER_SPEED
    if startSpeed >= 69 / 3.6:
        tolerance = BRAKING_TOLERANCE_DISTANCE_HIGHER_SPEED

    if desiredSpeed == startSpeed or desiredSpeed == -1:
        return 5.0, 0
    dtSeconds = traci.simulation.getDeltaT()
    t = (desiredSpeed - startSpeed) / (-1 * deceleration)
    # if printsOn: print("time:",t)
    s = (startSpeed * t) - ((deceleration) * t * t) / 2
    # newSpeed=startSpeed
    # times=0
    # while desiredSpeed<newSpeed:
    #     newSpeed-=deceleration*dtSeconds
    #     times += 1
    # #print(times)
    # distance=s+2
    distance = s + startSpeed * dtSeconds + tolerance
    return (5.0, 0) if distance < 5.0 else (distance, t)


def add_current_data_to_list(
    pothole_name,
    vehicle_ID,
    current_pos_vehicle,
    current_speed_vehicle,
    pos_on_map,
    distance_vehicle_to_pothole,
):
    if vehicle_ID not in vehicle_data:
        vehicle_data_all[vehicle_ID] = {
            "Pothole name": pothole_name,
            "Pos on lane": current_pos_vehicle,
            "Pos on map": pos_on_map,
            "Current speed": current_speed_vehicle,
            "Distance": distance_vehicle_to_pothole,
        }


def euclidDist(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def distance_to_junctions(
    junction,
    route,
    veh_edge,
    veh_id,
    veh_pos,
    veh_x,
    veh_y,
    obj_pos,
    obj_x,
    obj_y,
    obj_edge,
):
    junctionComplete = junction.lstrip(":").rsplit("_", 1)[0]
    v_idx = 0
    o_idx = 0
    dist = 0
    junctionEdges = [
        (
            traci.junction.getIncomingEdges(junctionComplete),
            traci.junction.getOutgoingEdges(junctionComplete),
        )
    ]
    for j in junctionEdges:
        flat = [e for tuple in j for e in tuple]
        for f in flat:
            if f in route and veh_edge in route:
                v_idx = route.index(veh_edge)
                o_idx = route.index(f)
                if o_idx < v_idx:
                    dist = euclidDist((obj_x, obj_y), (veh_x, veh_y))
                    return -1 * dist
                dist = euclidDist((obj_x, obj_y), (veh_x, veh_y))
                return dist
            if f in route and veh_edge == junction and obj_edge in route:
                v_idx = route.index(f)
                o_idx = route.index(obj_edge)
                # print("v_idx:", v_idx, "o_idx:", o_idx)
                if o_idx <= v_idx:
                    dist = euclidDist((obj_x, obj_y), (veh_x, veh_y))
                    return -1 * dist
                dist = euclidDist((obj_x, obj_y), (veh_x, veh_y))
                return dist


def signed_route_distance(
    veh_id, veh_edge, veh_pos, veh_x, veh_y, obj_edge, obj_pos, obj_x, obj_y, route
):
    if obj_edge == veh_edge:
        return obj_pos - veh_pos

    if obj_edge.startswith(":") and veh_edge == obj_edge:
        return traci.lane.getLength(traci.vehicle.getLaneID(veh_id)) - veh_pos + obj_pos

    ### filter for parallel lanes
    if veh_edge == "-" + obj_edge:
        return None
    if veh_edge.startswith("-"):
        veh_edge_split = veh_edge.lstrip("-")
        if veh_edge_split == obj_edge:
            return None
    ### junctions
    if veh_edge.startswith(":"):
        # print("junction is next:", veh_edge)
        rd_on_junction = distance_to_junctions(
            veh_edge,
            route,
            veh_edge,
            veh_id,
            veh_pos,
            veh_x,
            veh_y,
            obj_pos,
            obj_x,
            obj_y,
            obj_edge,
        )
        # print("rd_on_junction", rd_on_junction)
        return rd_on_junction

    if obj_edge.startswith(":") and veh_edge != obj_edge:
        obj_edge = obj_edge.lstrip(":").rsplit("_", 1)[0]
        return distance_to_junctions(
            obj_edge,
            route,
            veh_edge,
            veh_id,
            veh_pos,
            veh_x,
            veh_y,
            obj_pos,
            obj_x,
            obj_y,
            obj_edge,
        )

    if obj_edge in route and veh_edge in route:
        v_idx = route.index(veh_edge)
        o_idx = route.index(obj_edge)

        if o_idx < v_idx:
            dist = euclidDist((obj_x, obj_y), (veh_x, veh_y))
            return -1 * dist
        dist = euclidDist((obj_x, obj_y), (veh_x, veh_y))
        return dist
    return None


def apply_pothole_effect_to_pothole(
    pothole_name,
    pothole_depth,
    pothole_length,
    rd,
    pothole_lane,
    vehicle_ID,
    v0_speed,
    veh_current_speed,
):
    distance = rd
    current_speed_vehicle = veh_current_speed
    data = pothole_data[pothole_name]
    severity_of_pothole = depth_to_severity(pothole_depth)
    desired_speed_pothole = severity_to_speed(severity_of_pothole, v0_speed)

    if WITH_CALCULATION:
        desired_speed_pothole = pothole_speed_calculation(
            v0_speed, pothole_depth, pothole_length
        )
    if WITH_CALCULATION_IDM:
        # print(traci.vehicle.getAccel(vehicle))
        desired_speed_pothole = pothole_speed_calculation_IDM(
            current_speed_vehicle,
            v0_speed,
            pothole_length,
            pothole_depth,
            distance,
            traci.vehicle.getAccel(vehicle_ID),
        )  # accel is 2.6 for common vehicle

    if WITH_CALCULATION_SPEED_REDUCE:
        # desired_speed_pothole = desired_speed_pothole_last_test(
        #     current_speed_vehicle, v0_speed, distance, pothole_length, pothole_depth
        # )

        # desired_speed_pothole = final_desired_speed(
        #     v0_speed, pothole_depth, pothole_length
        # )
        desired_speed_pothole = pothole_end_speed(
            v0_speed, pothole_depth, pothole_length
        )

    if LOOK_BEHIND_DISTANCE < distance < data["distanceForSaveBraking"][0]:
        traci.vehicle.setSpeedMode(vehicle_ID, 0b0011111)
        traci.vehicle.setSpeed(vehicle_ID, desired_speed_pothole)
        if PRINTS_ON:
            print("desired speed", desired_speed_pothole)
        if PRINTS_ON:
            print("vehicle speed", current_speed_vehicle)


last_real_edge = {}


def get_logical_vehicle_edge(veh_id):
    edge = traci.vehicle.getRoadID(veh_id)

    if edge.startswith(":"):
        return last_real_edge.get(veh_id)
    last_real_edge[veh_id] = edge
    return edge


# def get_prio_index(data):
#     prio_index = 0
#     for i, p in enumerate(data):
#         if p["rd"] >= 0:
#             prio_index = i
#             # print("prior_index in first if:", prio_index)
#             break
#     if prio_index is None:
#         prio_index = len(data) - 1

#     # print("prior_index:", prio_index)
#     for i in range(1, len(data)):
#         # for i in range(prio_index + 1, len(data)):
#         prev = data[i - 1]
#         curr = data[i]

#         delta_rd = curr["rd"] - prev["rd"]
#         # print("delta_rd:", delta_rd)
#         if delta_rd <= prev["safe_barking_distance"]:
#             if curr["safe_barking_distance"] > prev["safe_barking_distance"]:
#                 prio_index = i
#         else:
#             # print("break")
#             break
#     if PRINTS_ON:
#         print(
#             f"Next pothole to apply pothole effect with index {prio_index}:",
#             data[prio_index]["pothole"],
#         )
#     return prio_index


def get_prio_index(data):

    active = [
        (i, p) for i, p in enumerate(data) if 0 <= p["rd"] <= p["safe_barking_distance"]
    ]

    if not active:

        candidates = [(i, p) for i, p in enumerate(data) if p["rd"] >= 0]
        return min(candidates, key=lambda x: x[1]["rd"])[0] if candidates else -1

    return max(active, key=lambda x: x[1]["safe_barking_distance"])[0]


##### get barking distance of default vehicle for all potholes
for pothole in potholes:
    pothole_name = pothole["name"]
    lane_ID_pothole = pothole["edge"]
    depth_pothole = pothole["depth"]
    length_pothole = pothole["length"]
    speed_limit_lane_pothole = traci.lane.getMaxSpeed(lane_ID_pothole)

    severity_of_pothole = depth_to_severity(depth_pothole)
    desired_speed_pothole = severity_to_speed(
        severity_of_pothole, speed_limit_lane_pothole
    )

    v0_speed = traci.lane.getMaxSpeed(lane_ID_pothole)
    if WITH_CALCULATION:
        desired_speed_pothole = pothole_speed_calculation(
            v0_speed, depth_pothole, length_pothole
        )
    if WITH_CALCULATION_SPEED_REDUCE:
        # desired_speed_pothole = final_desired_speed(
        #     v0_speed, depthPothole, lengthPothole
        # )
        desired_speed_pothole = pothole_end_speed(
            v0_speed, depth_pothole, length_pothole
        )
    #     desired_speed_pothole = desired_speed_pothole_last_test(v0_speed)
    ####only works if vehicles always have the same properties (decel) otherwise update on first seen (new vehicle), new list with vehicle data and potholes combined###
    deceleration = 4.5  # comfort barking not hard barking would be 9.0, 4.5 value for default vehicle need to be changed if not    decel=traci.vehicle.getDecel(vehicle_ID)

    if pothole_name not in pothole_data:
        pothole_data[pothole_name] = {
            "distanceForSaveBraking": calculate_braking_distances_based_on_desired_speed(
                speed_limit_lane_pothole, desired_speed_pothole, deceleration
            )
        }
# if printsOn: print(pothole_data)
print(pothole_data)

MAX_BRAKING_DISTANCE = max(
    v["distanceForSaveBraking"][0] for v in pothole_data.values()
)
print(MAX_BRAKING_DISTANCE)


step_times = []
#### main loop
while traci.simulation.getMinExpectedNumber() > 0:
    t0 = time.perf_counter()
    traci.simulationStep()

    # if vehicle_id != 0:
    if not start_vehicle_started and start_vehicle_id in traci.vehicle.getIDList():
        start_vehicle_started = True
        print(f"vehicle {start_vehicle_id} started.")

    if start_vehicle_id not in traci.vehicle.getIDList() and start_vehicle_started:
        print(f"vehicle {start_vehicle_id} reached destination. Simulation closed.")
        # traci.close()
        break

    for vehicle_ID in traci.vehicle.getIDList():

        # edge_ID_vehicle = get_logical_vehicle_edge(vehicle_ID)
        edge_ID_Vehicle = traci.vehicle.getRoadID(vehicle_ID)
        lane_ID_vehicle = traci.vehicle.getLaneID(vehicle_ID)
        lane_pos = traci.vehicle.getLanePosition(vehicle_ID)
        veh_route = traci.vehicle.getRoute(vehicle_ID)
        x_pos_vehicle, y_pos_vehicle = traci.vehicle.getPosition(vehicle_ID)
        veh_current_speed = traci.vehicle.getSpeed(vehicle_ID)
        ##### get simulation time per step for evaluation
        sim_time = traci.simulation.getTime()
        current_pothole_for_evaluation = "-"

        potholes_in_range = []
        # print(veh_route)

        # if lane_ID_vehicle ==":cluster_1967351583_1967351598_1967351610_1967351650_#1more_4_0": ### end of flanderstraße at HZE_OSM
        #     if(once):print(str(traci.simulation.getTime()))
        #     once=False

        ### get infos, distances and select next potholes if in range
        for pothole in potholes:
            pos_pothole_on_lane = pothole["pos"]
            distance_on_lane = pos_pothole_on_lane - lane_pos
            pothole_name = pothole["name"]
            lane_ID_pothole = pothole["edge"]
            depth_pothole = pothole["depth"]
            length_pothole = pothole["length"]
            xPosPothole = pothole["x,y"][0]
            yPosPothole = pothole["x,y"][1]
            speed_limit_lane_pothole = traci.lane.getMaxSpeed(lane_ID_pothole)

            edge_ID_pothole = traci.lane.getEdgeID(lane_ID_pothole)

            rd = signed_route_distance(
                vehicle_ID,
                edge_ID_Vehicle,
                lane_pos,
                x_pos_vehicle,
                y_pos_vehicle,
                edge_ID_pothole,
                pos_pothole_on_lane,
                xPosPothole,
                yPosPothole,
                veh_route,
            )
            # print("rd", rd)

            if rd is not None and rd <= MAX_BRAKING_DISTANCE and rd > PASSED_DISTANCE:
                if PRINTS_ON:
                    print(
                        "Vehicle pos: ",
                        x_pos_vehicle,
                        y_pos_vehicle,
                        "lane:",
                        lane_ID_vehicle,
                    )
                if PRINTS_ON:
                    print("distance to pothole (on same lane):", distance_on_lane)
                pothole_step += 1
                if PRINTS_ON:
                    print(
                        (f"Step: {pothole_step} Pothole: " + pothole_name)
                        + " edge: "
                        + pothole["edge"]
                        + " position: "
                        + str(pos_pothole_on_lane)
                    )
                # append potholes in range to sort with prio later on
                if not REAL_WORLD_WITH_SPEED:
                    potholes_in_range.append(
                        {
                            "pothole": pothole_name,
                            "depth": depth_pothole,
                            "length": length_pothole,
                            "rd": rd,
                            "lane": lane_ID_pothole,
                            "safe_barking_distance": pothole_data[pothole_name][
                                "distanceForSaveBraking"
                            ][0],
                        }
                    )
                if REAL_WORLD_WITH_SPEED:
                    potholes_in_range.append(
                        {
                            "pothole": pothole_name,
                            "depth": depth_pothole,
                            "length": length_pothole,
                            "rd": rd,
                            "lane": lane_ID_pothole,
                            "real_wold_speed": pothole["speed"],
                            "mean_speed": pothole["mean speed"],
                            "safe_barking_distance": pothole_data[pothole_name][
                                "distanceForSaveBraking"
                            ][0],
                        }
                    )

        ### select pothole to apply pothole effect
        if len(potholes_in_range) > 0:

            # if real wold data is used start speed at pothole with real wold start speed at pothole
            if REAL_WORLD_WITH_SPEED:
                # print("unsorted:",pot)

                potholes_in_range.sort(key=lambda x: x["rd"])
                if NEXT_POTHOLE != potholes_in_range[0]["pothole"]:
                    ONCE_PER_POTHOLE = True
                if ONCE_PER_POTHOLE == True:
                    ONCE_PER_POTHOLE = False
                    # print("next potholes:", potholes_in_range)
                    speed_from_real_wold = potholes_in_range[0]["real_wold_speed"][0]
                    NEXT_POTHOLE = potholes_in_range[0]["pothole"]
                    if PRINTS_ON:
                        print("speed from pothole list:", speed_from_real_wold)
                    traci.vehicle.setSpeedMode(vehicle_ID, 0)
                    # traci.vehicle.setSpeedMode(vehicle_ID, 0b0011001)
                    traci.vehicle.setSpeed(vehicle_ID, speed_from_real_wold)

                    if PRINTS_ON:
                        print("speed vehicle:", traci.vehicle.getSpeed(vehicle_ID))
                    vehicle_data[vehicle_ID] = {"next pothole:", NEXT_POTHOLE}
                    break

            # sort potholes in range based on relative distance
            potholes_in_range.sort(key=lambda x: x["rd"])
            # print("potholes in range:", potholes_in_range)
            vehicle_data[vehicle_ID] = {"data": potholes_in_range}
            # print("vehicle_data:", vehicle_data)
            # print(len(vehicle_data[vehicle_ID]["data"]))
            data = vehicle_data[vehicle_ID]["data"]
            # print("data", data, "len:", len(data))
            prio_index = get_prio_index(data)

            # use prior index to select next pothole to apply pothole effect
            worst_pothole_name = data[prio_index]["pothole"]
            worst_pothole_depth = data[prio_index]["depth"]
            worst_pothole_length = data[prio_index]["length"]
            worst_pothole_rd = data[prio_index]["rd"]
            worst_pothole_lane = data[prio_index]["lane"]
            if REAL_WORLD_WITH_SPEED:
                worst_pothole_speed = data[prio_index]["real_wold_speed"]
            if REAL_WORLD_WITH_SPEED:
                worst_pothole_mean_speed = data[prio_index]["mean_speed"]
            if PRINTS_ON:
                print(
                    "worst pothole(name,depth,length,rd,lane):",
                    worst_pothole_name,
                    worst_pothole_depth,
                    worst_pothole_length,
                    worst_pothole_rd,
                    worst_pothole_lane,
                )
            v0_speed = traci.lane.getMaxSpeed(worst_pothole_lane)
            if REAL_WORLD_WITH_SPEED:
                v0_speed = worst_pothole_mean_speed
            if NO_EFFECT_APPLIED == False:
                apply_pothole_effect_to_pothole(
                    worst_pothole_name,
                    worst_pothole_depth,
                    worst_pothole_length,
                    worst_pothole_rd,
                    worst_pothole_lane,
                    vehicle_ID,
                    v0_speed,
                    veh_current_speed,
                )

            # applyPotholeEffects(pothole["depth"],vehicle_ID,rd,pothole,speedLimitLanePothole) and worst_pothole_lane == lane_ID_vehicle
            # print(
            #     "worst_pothole_rd:",
            #     worst_pothole_rd,
            #     "worst_pothole_length:",
            #     worst_pothole_length,
            # )
            # change name to potholes to get current pothole if passed
            current_pothole_for_evaluation = (
                worst_pothole_name + "_" + depth_to_severity(worst_pothole_depth)
            )

            if worst_pothole_rd < -1 * worst_pothole_length:
                traci.vehicle.setSpeed(vehicle_ID, -1)
                current_pothole_for_evaluation = "-"
            if (
                pothole_data[worst_pothole_name]["distanceForSaveBraking"][0]
                < worst_pothole_rd
            ):
                traci.vehicle.setSpeed(vehicle_ID, -1)
                current_pothole_for_evaluation = "-"

            # ##### get simulation time per step for evaluation
            # sim_time = traci.simulation.getTime()

            ##### get info over speeds on potholes for evaluation
        evaluation_records.append(
            {
                "time": sim_time,
                "vehicle_id": vehicle_ID,
                "pothole": current_pothole_for_evaluation,
                "speed_mps": veh_current_speed,
            }
        )
        # traci.vehicle.setSpeed(vehicle_ID, -1)
    step += 1
    t1 = time.perf_counter()
    step_times.append(t1 - t0)

# save evaluation data in data frame
df = pd.DataFrame(evaluation_records)
df.to_csv(EVALUATION_PATH + "/vehicle_pothole_speeds.csv", index=False)

df_performance_test = pd.DataFrame(step_times)
print("Average time per step:", sum(step_times) / len(step_times), "secs")
print("Max:", max(step_times))
print("Min:", min(step_times))
df_performance_test.to_csv(EVALUATION_PATH + "/performance_test.csv", index=False)

traci.close()
print("Simulation finished.")
