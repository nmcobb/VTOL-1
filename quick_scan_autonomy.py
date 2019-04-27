import json
import math
import time
from conversions import *
from threading import Thread
from pymavlink import mavutil
from dronekit import connect, Command, VehicleMode, LocationGlobalRelative
from detailed_search import detailed_search

# first import gives access to global variables in "autonomy" namespace
# second import is for functions
import autonomy
from autonomy import comm_simulation, acknowledge, bad_msg, takeoff, land, update_thread, \
    change_status, setup_xbee, start_auto_mission

search_area = None  # search area object, populated by callback on start


# Represents a search area for quick scan aerial vehicles
class SearchArea:
    def __init__(self, tl, tr, bl, br):
	# tl-br are tuples containing the coordinates of rectangle corners
	self.tl = (bytes.fromhex(tl[0]), bytes.fromhex(tl[1]))
	self.tr = (bytes.fromhex(tr[0]), bytes.fromhex(tr[1]))
	self.bl = (bytes.fromhex(bl[0]), bytes.fromhex(bl[1]))
	self.br = (bytes.fromhex(br[0]), bytes.fromhex(br[1]))

    def __str__(self):
        return "SearchArea(" + \
               str(self.tl) + ", " + \
               str(self.tr) + ", " + \
               str(self.bl) + ", " + \
               str(self.br) + ")"


# Callback function for messages from GCS, parses JSON message and sets globals
def xbee_callback(message):
    global search_area

    address = message.remote_device.get_64bit_addr()
    msg = json.loads(message.data)
    print("Received data from %s: %s" % (address, msg))

    try:
        msg_type = msg["type"]

        if msg_type == "addMission":
            area = msg["missionInfo"]["searchArea"]
            search_area = SearchArea(area["topLeft"], area["topRight"],
                area["bottomLeft"], area["bottomRight"])
            autonomy.start_mission = True
            acknowledge(address, msg["id"])

        elif msg_type == "pause":
            autonomy.pause_mission = True
            acknowledge(address, msg["id"])

        elif msg_type == "resume":
            autonomy.pause_mission = False
            acknowledge(address, msg["id"])

        elif msg_type == "stop":
            autonomy.stop_mission = True
            acknowledge(address, msg["id"])

        elif msg_type == "acknowledge":
            # TODO check the ID
            pass

        else:
            bad_msg(address, "Unknown message type: \'" + msg_type + "\'")

    # KeyError if message was missing an expected key
    except KeyError as e:
        bad_msg(address, "Missing \'" + e.args[0] + "\' key")


# Generate NED and LLA waypoints in spiral pattern
def generate_waypoints(configs, search_area):
    print("Begin generating waypoints")

    waypointsNED = []
    waypointsLLA = []

    # start at the bottom left
    start = search_area.bl
    # end at the top right
    end = search_area.tr

    # pre-defined in configs file
    altitude = configs["altitude"]
    lat = start[0]
    lon = start[1]
    startX, startY, startZ = geodetic2ecef(lat, lon, altitude)
    n, e, d = ecef2ned(startX, startY, startZ, lat, lon, altitude)
    waypointsLLA.append(LocationGlobalRelative(lat, lon, altitude))
    waypointsNED.append([n, e, d])

    endX, endY, endZ = geodetic2ecef(end[0], end[1], altitude)
    n2, e2, d2 = ecef2ned(endX, endY, endZ, end[0], end[1], altitude)

    fovH = math.radians(62.2)  # raspicam horizontal FOV
    boxH = 2 * altitude / math.tan(fovH / 2)  # height of bounding box
    overlap = (0.5 * boxH) / (2 * math.pi)  # distance between zags with 50% overlap

    temp_n = n
    while tempn <= n2:
        temp_e = e
        # convert NED to LLA
        newLat, newLon, newAlt = ned2geodetic(temp_n, temp_e, d, lat, lon, altitude)
        waypointsNED.append([temp_n, temp_e, d])
        waypointsLLA.append(LocationGlobalRelative(newLat, newLon, altitude))

        temp_e = e2
        # convert ECEF to NED and LLA
        newLat, newLon, newAlt = ned2geodetic(temp_n, temp_e, d, lat, lon, altitude)
        waypointsNED.append([temp_n, temp_e, d])
        waypointsLLA.append(LocationGlobalRelative(newLat, newLon, altitude))

        temp_n += overlap

        #                     |
        # --------------------|

        temp_e = e2
        # convert ECEF to NED and LLA
        newLat, newLon, newAlt = ned2geodetic(temp_n, temp_e, d, lat, lon, altitude)
        waypointsNED.append([temp_n, temp_e, d])
        waypointsLLA.append(LocationGlobalRelative(newLat, newLon, altitude))

        temp_e = e
        # convert ECEF to NED and LLA
        newLat, newLon, newAlt = ned2geodetic(temp_n, temp_e, d, lat, lon, altitude)
        waypointsNED.append([temp_n, temp_e, d])
        waypointsLLA.append(LocationGlobalRelative(newLat, newLon, altitude))

        temp_n += overlap

        # |
        # |-------------------|
        # --------------------|


    return (waypointsNED, waypointsLLA)


def quick_scan_adds_mission(vehicle, lla_waypoint_list, altitude):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """
    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear()

    # Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    # This command is there when vehicle is in AUTO mode, where it takes off through command list
    # In guided mode, the actual takeoff function is needed, in which case this command is ignored
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, altitude))

    # Add point directly above vehicle for takeoff
    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0,
                0, 0, 0, vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, altitude))

    print(" Define/add new commands.")
    # Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    for point in lla_waypoint_list:
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point.lat, point.lon, point.alt))

    # Adds dummy end point - this endpoint is the same as the last waypoint and lets us know we have reached destination.
    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0,
                0, 0, 0, lla_waypoint_list[-1].lat, lla_waypoint_list[-1].lon, lla_waypoint_list[-1].alt))
    print("Upload new commands to vehicle")
    cmds.upload()


# Main autonomous flight thread
# :param configs: dict from configs file
def quick_scan_autonomy(configs, autonomyToCV, gcs_timestamp, connection_timestamp):
    print("\n######################## STARTING QUICK SCAN AUTONOMY ########################")
    autonomy.configs = configs

    # If comms is simulated, start comm simulation thread
    if configs["quick_scan_specific"]["comms_simulated"]["toggled_on"]:
        comm_sim = Thread(target=comm_simulation, args=(configs["quick_scan_specific"]["comms_simulated"]["comm_sim_file"], xbee_callback,))
        comm_sim.start()
    # Otherwise, set up XBee device and add callback
    else:
        comm_sim = None
        autonomy.xbee = setup_xbee()
        autonomy.gcs_timestamp = gcs_timestamp
        autonomy.connection_timestamp = connection_timestamp
        autonomy.xbee.add_data_received_callback(xbee_callback)

    # Generate waypoints after start_mission = True
    while not autonomy.start_mission:
        pass

    # Generate waypoints
    waypoints = generate_waypoints(configs, search_area)

    # Start SITL if vehicle is being simulated
    if (configs["vehicle_simulated"]):
        import dronekit_sitl
        sitl = dronekit_sitl.start_default(lat=35.328423, lon=-120.752505)
        connection_string = sitl.connection_string()
    else:
        if (configs["3dr_solo"]):
            connection_string = "udpin:0.0.0.0:14550"
        else:
            connection_string = "/dev/serial0"

    # Connect to vehicle
    vehicle = connect(connection_string, wait_ready=True)

    # Starts the update thread
    update = Thread(target=update_thread, args=(vehicle, configs["mission_control_MAC"]))
    update.start()

    # Send mission to vehicle
    quick_scan_adds_mission(vehicle, waypoints[1], configs["altitude"])

    # Start the mission
    start_auto_mission(vehicle)

    # Change vehicle status to running
    change_status("running")

    # Fly about spiral pattern
    while vehicle.commands.next != vehicle.commands.count:
        print(vehicle.location.global_frame)
        time.sleep(1)
        # Holds the copter in place if receives pause
        if autonomy.pause_mission:
            vehicle.mode = VehicleMode("ALT_HOLD")
        # Lands the vehicle if receives stop mission
        elif autonomy.stop_mission:
            land(vehicle)
            return
        # Continues path
        else:
            vehicle.mode = VehicleMode("AUTO")

    # Switch to detailed search if role switching is enabled
    if configs["quick_scan_specific"]["role_switching"]:
        autonomy.mission_completed = True
        update.join()
        detailed_search(vehicle)
    else:
        land(vehicle)

        # Ready for a new mission
        autonomy.mission_completed = True

        # Wait for update thread to end
        update.join()

    # Wait for comm simulation thread to end
    if comm_sim:
        comm_sim.join()
    else:
        autonomy.xbee.close()
