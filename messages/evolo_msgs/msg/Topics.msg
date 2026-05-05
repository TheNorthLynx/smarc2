# This file defines all the topics used on Evolo
# The goal is to have a single location that all cpp/py scripts can easily access


#communication to and from captain
string EVOLO_CAPTAIN_TO = captain/to
string EVOLO_CAPTAIN_FROM= captain/from
string EVOLO_CAPTAIN_STATE= captain/state

string EVOLO_MQTT_RECEIVE = mqtt/fromevolo
string EVOLO_MQTT_SEND = mqtt/toevolo

string EVOLO_REQUESTED_CTRL = ctrl/requested
string EVOLO_SAFE_CTRL = ctrl/twist_setpoint

# LIDAR
int32 EVOLO_OCCUPANCY_GRID_SIZE = 100
string EVOLO_OCCUPANCY_GRID = occupancyMap
string EVOLO_CBF_OBSTACLES = obstacles

#TDODO CAMERA & GIMBAL

#TODO SIDESCAN