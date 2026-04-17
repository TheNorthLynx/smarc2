# This file defines all the topics used on Evolo
# The goal is to have a single location that all cpp/py scripts can easily access


#communication to and from captain
string EVOLO_CAPTAIN_TO = captain/to
string EVOLO_CAPTAIN_FROM= captain/from
string EVOLO_CAPTAIN_STATE= captain/state

string EVOLO_MQTT_RECEIVE = mqtt/fromevolo
string EVOLO_MQTT_SEND = mqtt/toevolo

string EVOLO_SIM_CTRL_TO = ctrl/twist_setpoint

#TODO LIDAR
string EVOLO_OCCUPANCY_GRID = occupancyMap
int32 EVOLO_OCCUPANCY_GRID_SIZE = 100

#TDODO CAMERA & GIMBAL

#TODO SIDESCAN