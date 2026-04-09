import os
import json
import xml.etree.ElementTree as ET
from sumolib import net
import math
import random

### INPUT
NET_PATH = "PotholeIntegrationInSUMO/HZE/osm.net.xml.gz"  # HZE example from Esslingen Flanderstraße
# NET_PATH="PotholeIntegrationInSUMO/DataSet/output/net.net.xml" # dataset input, real world data
NET_FILE = os.path.abspath(NET_PATH)
# NET_FILE = os.path.abspath("PotholeIntegrationInSUMO/HZE/osm.net.xml.gz")

FROM_JSON_FILE = False  # flag for generating from json file or not
if FROM_JSON_FILE and os.path.exists("PotholeIntegrationInSUMO/HZE/potholes.json"):
    POTHOLE_FILE = os.path.abspath("PotholeIntegrationInSUMO/HZE/potholes.json")
# if a==True and os.path.exists("PotholeIntegrationInSUMO/DataSet/output/potholesCorrection.json"):
#     POTHOLE_FILE = os.path.abspath("PotholeIntegrationInSUMO/DataSet/output/potholesCorrection.json")

### OUTPUT
POTHOLE_DATA = os.path.abspath(
    "PotholeIntegrationInSUMO/HZE/make_pothole_output/potholes.data.json"
)
POLY_FILE = os.path.abspath(
    "PotholeIntegrationInSUMO/HZE/make_pothole_output/potholes.poly.xml"
)
# OBSTACLE_FILE = os.path.abspath(
#     "PotholeIntegrationInSUMO/HZE/make_pothole_output/obstacle.obj.xml"
# )  ###was for testing not needed anymore
ROUTE_FILE = os.path.abspath(
    "PotholeIntegrationInSUMO/HZE/make_pothole_output/pseudoVehicle.rou.xml"
)

### Settings for random generation: seed and count of potholes per lane
random.seed(42)  # getting always the same uniform distribution
RANDOM_POTHOLES_PER_LANE = (
    1  # setting for the amount of pothole randomly created at lane
)


### SUMO road net read in and roots for the XML data
ROAD_NET = net.readNet(NET_FILE)
ROOT = ET.Element("additional")
ROOT_2 = ET.Element("routes")
# root3 = ET.Element("additional")
DATA = []  # list for  save between


def depth_to_color(depth):
    if depth > 0.075:
        return "#FF0000"  # red
    elif depth > 0.05:
        return "#FFA600"  # orange
    else:
        return "#48FF00"  # green


def rotate_point(px, py, cx, cy, theta):
    """rotate a point based on the center (cx,cy) and theta"""
    dx, dy = px - cx, py - cy
    qx = cx + math.cos(theta) * dx - math.sin(theta) * dy
    qy = cy + math.sin(theta) * dx + math.cos(theta) * dy
    return qx, qy


def get_xy_on_lane(lane, pos):
    shape = lane.getShape()
    if pos <= 0:
        return shape[0]

    def distance(p1, p2):
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    traveled = 0
    for i in range(len(shape) - 1):
        seg_len = distance(shape[i], shape[i + 1])
        if traveled + seg_len >= pos:
            ratio = (pos - traveled) / seg_len
            x = shape[i][0] + ratio * (shape[i + 1][0] - shape[i][0])
            y = shape[i][1] + ratio * (shape[i + 1][1] - shape[i][1])
            return x, y
        traveled += seg_len
    return shape[-1]


def get_edges_by_name(name):
    target_name = name
    target_edges = [e for e in ROAD_NET.getEdges() if e.getName() == target_name]
    return target_edges


def random_pos(lane):
    random_pos = random.uniform(1.0, lane.getLength() - 1.0)
    return random_pos


def random_width(lane):
    random_width = random.uniform(0.1, lane.getWidth() - 0.1)
    return random_width


def random_depth():
    random_depth = random.uniform(0.01, 0.15)
    return random_depth


def random_pothole_on_edge_with_name(name):
    target_name = name
    target_edges = get_edges_by_name(target_name)
    counter = 0
    for edge in target_edges:
        lane = edge.getLanes()[0]
        if edge.getLaneNumber() > 1:
            lane = edge.getLanes()[1]
        # print(lane)
        for i in range(RANDOM_POTHOLES_PER_LANE):
            pos = random_pos(lane)
            width = random_width(lane)
            depth = random_depth()
            pothole_name = f"pothole{counter}" + f"_{i}"
            length = width / 2
            x, y = get_xy_on_lane(lane, pos)
            create_polygon_on_lane(lane, pos, width, length, depth, pothole_name)
            create_pseudo_vehicle_on_lane_in_route_file(
                lane, pos, width, length, depth, pothole_name
            )
            save_pothole_data_to_json_file(
                lane, pos, width, depth, length, pothole_name, x, y
            )

        # pos = randomPos(lane)
        # width = randomWidth(lane)
        # depth = randomDepth()
        # pothole_name = f"pothole{counter}"
        # #width=width/2
        # length=width/2
        # x,y = getXyOnLane(lane,pos)

        # createPolygonOnLane(lane,pos,width,length,depth,pothole_name)
        # createPseudoVehicleOnLaneInRouteFile(lane,pos,width,length,depth,pothole_name)
        # savePotholeDataToJsonFile(lane,pos,width,depth,length,pothole_name,x,y)
        counter += 1


def create_polygon_on_lane(lane, pos, width, length, depth, pothole_name):
    x, y = get_xy_on_lane(lane, pos)  # center
    dx, dy = (
        lane.getShape()[-1][0] - lane.getShape()[0][0],
        lane.getShape()[-1][1] - lane.getShape()[0][1],
    )  # tangent of lane
    theta = math.atan2(dy, dx)  # rotation in rad
    half_l, half_w = length / 2, width / 2
    rect = [
        (x - half_l, y - half_w),
        (x + half_l, y - half_w),
        (x + half_l, y + half_w),
        (x - half_l, y + half_w),
    ]
    rotated = [rotate_point(px, py, x, y, theta) for px, py in rect]
    shape_str_rotated = " ".join(
        f"{x:.2f},{y:.2f}" for x, y in rotated
    )  # x1,y1 x2,y2 x3,y3 x4,y4 and 2 decimal places
    # shape_str = f"{x-w/2},{y-w/2} {x+w/2},{y-w/2} {x+w/2},{y+w/2} {x-w/2},{y+w/2}"  # Polygon (square)
    color = depth_to_color(depth)
    ET.SubElement(
        ROOT,
        "poly",
        {  # poly
            "id": pothole_name,
            "type": "pothole",  # type "pothole" is generic
            "color": color,
            "fill": "1",
            "layer": "5",
            # "shape": shape_str
            "shape": shape_str_rotated,
            # "shape": shape_str_rectangle
        },
    )


def create_polygon_on_xy_pos(x, y, width, length, depth, pothole_name):
    # x, y = getXyOnLane(lane, pos) #center

    half_l, half_w = length / 2, width / 2
    rect = [
        (x - half_l, y - half_w),
        (x + half_l, y - half_w),
        (x + half_l, y + half_w),
        (x - half_l, y + half_w),
    ]
    shape_str = " ".join(
        f"{x:.2f},{y:.2f}" for x, y in rect
    )  # x1,y1 x2,y2 x3,y3 x4,y4 and 2 decimal places
    # shape_str = f"{x-w/2},{y-w/2} {x+w/2},{y-w/2} {x+w/2},{y+w/2} {x-w/2},{y+w/2}"  # Polygon (square)
    color = depth_to_color(depth)
    ET.SubElement(
        ROOT,
        "poly",
        {  # poly
            "id": pothole_name,
            "type": "pothole",  # type "pothole" is generic
            "color": color,
            "fill": "1",
            "layer": "5",
            # "shape": shape_str
            "shape": shape_str,
            # "shape": shape_str_rectangle
        },
    )


def create_pseudo_vehicle_on_lane_in_route_file(
    lane, pos, width, length, depth, pothole_name
):
    pos = pos + length / 2
    pothole_name = f"{pothole_name}_"
    ET.SubElement(
        ROOT_2,
        "vType",
        {  # vType
            "id": pothole_name,
            "length": str(length),
            "width": str(width),
            "height": str(depth),
            # "carFollowModel":"none",
            # "laneChangeModel":"none",
            # "accel": "0.1",
            # "decel": "0.1",
            # "minGap": "0.0", #for test if it even get called at model ("Krauss",LCM)
            # "minGapFactor": "0.0",
            # "maxSpeed": "0.1",
            # "collision.mingap-factor":"0",
            # "collision.action":"none",
            # "scale": "0" #0 makes them invisible and unused...
            # "vClass": "ignoring",
            # "lcStrategic": "0",
            # "lcCooperative": "0",
            # "lcSpeedGain": "0",
            # "lcKeepRight": "0",
            # "lcPushy": "0",
            # "lcAssertive": "0"
        },
    )

    color = depth_to_color(depth)
    if type(lane) == str:
        laneID = lane
    if type(lane) is not str:
        laneID = lane.getID()
    edgeID = laneID
    edgeID = laneID[: laneID.rfind("_")]
    trip = ET.SubElement(
        ROOT_2,
        "trip",
        {
            "id": pothole_name,
            "type": pothole_name,
            "depart": "0.00",
            "departPos": "stop",
            "color": color,
            "from": edgeID,
            "to": edgeID,
        },
    )

    ET.SubElement(
        trip,
        "stop",
        {
            "lane": laneID,
            "startPos": str(pos),
            "endPos": str(pos),
            "duration": "3600.00",  # may change to simulation time 3600s=60mins
        },
    )


def save_pothole_data_to_json_file(edge, pos, width, depth, length, pothole_name, x, y):
    if type(edge) == str:
        edge = edge
    if type(edge) is not str:
        edge = edge.getID()
    DATA.append(
        {
            "name": pothole_name,
            "edge": edge,
            "pos": pos,
            "width": width,
            "depth": depth,
            "length": length,
            "x,y": (x, y),
        }
    )


# def createObstacleFile(lane,pos,width,length,depth,pothole_name,shape):
#     color = depthToColor(depth)
#     laneID=lane.getID() #documentation says getEdgeID(), but dose not work
#     edgeID=laneID[: laneID.rfind('_')]

#     ET.SubElement(root3, "type", {
#         "id":pothole_name+"_",
#         "shape":shape,
#         "color":color,
#         "layer":"3"
#         })
#     ET.SubElement(root3, "obstacle", { #poly
#         "id": pothole_name,
#         "type": pothole_name+"_", #type "pothole" is generic
#         "color": color,
#         "lane":edgeID,
#         "pos":str(pos),
#         "length":str(length),
#         "width":str(width),
#         "fill": "1",
#         "layer": "3",
#     })

# --- potholes with potholes from json file
if FROM_JSON_FILE == True and os.path.exists(NET_PATH):

    with open(POTHOLE_FILE, "r") as f:
        potholes = json.load(f)

    for i, p in enumerate(potholes):
        # junction = ROAD_NET.getJunction()
        # edge = ROAD_NET.getEdge(p["edge"])
        edge_name = p["edge"]
        x, y = p["x,y"][0], p["x,y"][1]
        # if "_" in edge_name:
        #     edge = ROAD_NET.getEdge(p["edge"])
        #     #print(edge)
        #     lane = edge.getLanes()[0]   # first lane

        pos = p["pos"]
        width = p["width"]
        depth = p["depth"]
        length = p["length"]
        if "#" in edge_name:
            lane = ROAD_NET.getLane(p["edge"])
            pothole_name = f"pothole{i}"
            print("Pothole:", pothole_name)
            create_polygon_on_lane(lane, pos, width, length, depth, pothole_name)
            # createObstacleFile(lane,pos,width,length,depth,pothole_name)
            create_pseudo_vehicle_on_lane_in_route_file(
                lane, pos, width, length, depth, pothole_name
            )
            # save_pothole_data_to_json_file(lane,pos,width,depth,length,pothole_name)

        if ":" in edge_name:
            lane = p["edge"]
            pothole_name = f"potholeAtJunction{i}"
            print("Pothole at junction:", pothole_name)
            create_polygon_on_xy_pos(x, y, width, length, depth, pothole_name)
        # create_pseudo_vehicle_on_lane_in_route_file(lane,pos,width,length,depth,pothole_name)
        save_pothole_data_to_json_file(
            lane, pos, width, depth, length, pothole_name, x, y
        )


# --- random potholes based on street name
if FROM_JSON_FILE == False:
    random_pothole_on_edge_with_name("Flandernstraße")

# # --- generation of pothole with values
# lane = ROAD_NET.getLane("9514888#0_1")
# x, y = get_xy_on_lane(lane, 30.0)
# pos = 30.0
# width = 0.5
# length = 0.5
# depth = 0.0763
# create_polygon_on_lane(lane, pos, width, length, depth, "pothole_test")
# save_pothole_data_to_json_file(lane, pos, width, depth, length, "pothole_test", x, y)
# ### prio test
# lane = ROAD_NET.getLane("9514888#1_1")
# x, y = get_xy_on_lane(lane, 5.0)
# pos = 5.0
# width = 0.5
# length = 0.5
# depth = 0.04
# create_polygon_on_lane(lane, pos, width, length, depth, "pothole_green")
# save_pothole_data_to_json_file(lane, pos, width, depth, length, "pothole_green", x, y)
# lane = ROAD_NET.getLane("9514888#1_1")
# x, y = get_xy_on_lane(lane, 15.0)
# pos = 15.0
# width = 0.5
# length = 0.5
# depth = 0.065
# create_polygon_on_lane(lane, pos, width, length, depth, "pothole_yellow")
# save_pothole_data_to_json_file(lane, pos, width, depth, length, "pothole_yellow", x, y)
# lane = ROAD_NET.getLane("9514888#1_1")
# x, y = get_xy_on_lane(lane, 25.0)
# pos = 25.0
# width = 0.5
# length = 0.5
# depth = 0.08
# create_polygon_on_lane(lane, pos, width, length, depth, "pothole_red")
# save_pothole_data_to_json_file(lane, pos, width, depth, length, "pothole_red", x, y)

# # # --- generation of pothole with values for evaluation brake distances

# lane = ROAD_NET.getLane("9514888#0_1")
# x, y = get_xy_on_lane(lane, 50.0)
# pos = 50.0
# width = 0.5
# length = 0.5
# depth = 0.0763
# create_polygon_on_lane(lane, pos, width, length, depth, "pothole_red_on_70_road")
# save_pothole_data_to_json_file(
#     lane, pos, width, depth, length, "pothole_red_on_70_road", x, y
# )

# lane = ROAD_NET.getLane("9514888#1_1")
# x, y = get_xy_on_lane(lane, 30.0)
# pos = 30.0
# width = 0.5
# length = 0.5
# depth = 0.0763
# create_polygon_on_lane(lane, pos, width, length, depth, "pothole_red_on_50_road")
# save_pothole_data_to_json_file(
#     lane, pos, width, depth, length, "pothole_red_on_50_road", x, y
# )

# lane = ROAD_NET.getLane("9514888#2_1")
# x, y = get_xy_on_lane(lane, 20.0)
# pos = 20.0
# width = 0.5
# length = 0.5
# depth = 0.0763
# create_polygon_on_lane(lane, pos, width, length, depth, "pothole_red_on_30_road")
# save_pothole_data_to_json_file(
#     lane, pos, width, depth, length, "pothole_red_on_30_road", x, y
# )

# # --- generation of pothole with values for evaluation prior test

# ## first road
# lane = ROAD_NET.getLane("9514888#0_1")
# x, y = get_xy_on_lane(lane, 15.0)
# pos = 15.0
# width = 0.5
# length = 0.5
# depth = 0.08
# create_polygon_on_lane(lane, pos, width, length, depth, "pothole_red_on_1_road")
# save_pothole_data_to_json_file(
#     lane, pos, width, depth, length, "pothole_red_on_1_road", x, y
# )
# lane = ROAD_NET.getLane("9514888#0_1")
# x, y = get_xy_on_lane(lane, 20.0)
# pos = 20.0
# width = 0.5
# length = 0.5
# depth = 0.065
# create_polygon_on_lane(lane, pos, width, length, depth, "pothole_yellow_on_1_road")
# save_pothole_data_to_json_file(
#     lane, pos, width, depth, length, "pothole_yellow_on_1_road", x, y
# )
# lane = ROAD_NET.getLane("9514888#0_1")
# x, y = get_xy_on_lane(lane, 25.0)
# pos = 25.0
# width = 0.5
# length = 0.5
# depth = 0.04
# create_polygon_on_lane(lane, pos, width, length, depth, "pothole_green_on_1_road")
# save_pothole_data_to_json_file(
#     lane, pos, width, depth, length, "pothole_green_on_1_road", x, y
# )

# ## second road
# lane = ROAD_NET.getLane("9514888#1_1")
# x, y = get_xy_on_lane(lane, 5.0)
# pos = 5.0
# width = 0.5
# length = 0.5
# depth = 0.04
# create_polygon_on_lane(lane, pos, width, length, depth, "pothole_green_on_2_road")
# save_pothole_data_to_json_file(
#     lane, pos, width, depth, length, "pothole_green_on_2_road", x, y
# )

# lane = ROAD_NET.getLane("9514888#1_1")
# x, y = get_xy_on_lane(lane, 10.0)
# pos = 10.0
# width = 0.5
# length = 0.5
# depth = 0.065
# create_polygon_on_lane(lane, pos, width, length, depth, "pothole_yellow_on_2_road")
# save_pothole_data_to_json_file(
#     lane, pos, width, depth, length, "pothole_yellow_on_2_road", x, y
# )

# lane = ROAD_NET.getLane("9514888#1_1")
# x, y = get_xy_on_lane(lane, 15.0)
# pos = 15.0
# width = 0.5
# length = 0.5
# depth = 0.08
# create_polygon_on_lane(lane, pos, width, length, depth, "pothole_red_on_2_road")
# save_pothole_data_to_json_file(
#     lane, pos, width, depth, length, "pothole_red_on_2_road", x, y
# )

# ## third road
# lane = ROAD_NET.getLane("9514888#2_1")
# x, y = get_xy_on_lane(lane, 5.0)
# pos = 5.0
# width = 0.5
# length = 0.5
# depth = 0.065
# create_polygon_on_lane(lane, pos, width, length, depth, "pothole_yellow_on_3_road")
# save_pothole_data_to_json_file(
#     lane, pos, width, depth, length, "pothole_yellow_on_3_road", x, y
# )
# lane = ROAD_NET.getLane("9514888#2_1")
# x, y = get_xy_on_lane(lane, 10.0)
# pos = 10.0
# width = 0.5
# length = 0.5
# depth = 0.04
# create_polygon_on_lane(lane, pos, width, length, depth, "pothole_green_on_3_road")
# save_pothole_data_to_json_file(
#     lane, pos, width, depth, length, "pothole_green_on_3_road", x, y
# )
# lane = ROAD_NET.getLane("9514888#2_1")
# x, y = get_xy_on_lane(lane, 15.0)
# pos = 15.0
# width = 0.5
# length = 0.5
# depth = 0.08
# create_polygon_on_lane(lane, pos, width, length, depth, "pothole_red_on_3_road")
# save_pothole_data_to_json_file(
#     lane, pos, width, depth, length, "pothole_red_on_3_road", x, y
# )

# --- write poly-file ---
tree = ET.ElementTree(ROOT)
ET.indent(tree, space="  ", level=0)
tree.write(POLY_FILE, encoding="UTF-8", xml_declaration=True)
print(f"{POLY_FILE} created")

# # --- write obstacle-file ---
# tree = ET.ElementTree(root3)
# ET.indent(tree, space="  ", level=0)
# tree.write(OBSTACLE_FILE, encoding="UTF-8", xml_declaration=True)
# print(f"{OBSTACLE_FILE} created")

# --- write route-file ---
tree = ET.ElementTree(ROOT_2)
ET.indent(tree, space="  ", level=0)
tree.write(ROUTE_FILE, encoding="UTF-8", xml_declaration=True)
print(f"{ROUTE_FILE} created")

# --- write json-file
with open(POTHOLE_DATA, "w") as f:
    json.dump(DATA, f, indent=6)
print(f"{POTHOLE_DATA} created")
