from dronekit import connect,VehicleMode,LocationGlobalRelative,LocationGlobal
import time
import json
import math


def haversine_distance(lat1, lon1, lat2, lon2):
    r = 6371000  # radius of the earth in meters
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = math.sin(dLat / 2) * math.sin(dLat / 2) + \
        math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * \
        math.sin(dLon / 2) * math.sin(dLon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = r * c
    return distance

def reverse_haversine(point,distance,bearing):
    r=6371000
    la1=math.radians(point.lat)
    lo1=math.radians(point.lon)  
    ad=distance/r
    theta=math.radians(bearing)
    lat2=math.degrees(math.asin(math.sin(la1)*math.cos(ad)+math.cos(la1)*math.cos(theta)*math.sin(ad)))
    lon2=math.degrees(lo1+math.atan2(math.sin(theta)*math.sin(ad)*math.cos(la1),math.cos(ad)-math.sin(la1)*math.sin(lat2)))
    return LocationGlobal(lat2,lon2,20)

def goto_waypoint(point):
    vehicle.simple_goto(point)

    while True:
        a=vehicle.location.global_frame.lat
        b=vehicle.location.global_frame.lon
        c=vehicle.location.global_relative_frame.alt
        if (haversine_distance(a,b,point.lat,point.lon)<15):
            print("Waypoint reached")
            break
        print("Current location: ",a,b,c)
        time.sleep(0.1)



with open('waypoint.json') as waypoint_json:
    data=json.load(waypoint_json)
    waypoint1=LocationGlobal(data['waypoints'][0]['lat1'],data['waypoints'][0]['lon1'],data['waypoints'][0]['alt1'])
    waypoint2=LocationGlobal(data['waypoints'][1]['lat2'],data['waypoints'][1]['lon2'],data['waypoints'][1]['alt2'])
    waypoint3=LocationGlobal(data['waypoints'][2]['lat3'],data['waypoints'][2]['lon3'],data['waypoints'][2]['alt3'])
    waypoint4=LocationGlobal(data['waypoints'][3]['lat4'],data['waypoints'][3]['lon4'],data['waypoints'][3]['alt4'])

vehicle=connect("udp:127.0.0.1:14550",wait_ready=False)
vehicle.mode=VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed:
    print("waiting to Arm")
    time.sleep(1)
else:
    print("Vehicle armed")

altitude=20
vehicle.simple_takeoff(altitude)
while vehicle.location.global_relative_frame.alt<0.95*altitude:
    print("Gaining altitude. Current Altitude is: ",vehicle.location.global_relative_frame.alt)
    time.sleep(1)

print("Moving towards waypoint 1")
goto_waypoint(reverse_haversine(waypoint1,10,180))

print("Moving towards waypoint 2")
goto_waypoint(reverse_haversine(waypoint2,10,180))

print("Moving towards waypoint 3")
goto_waypoint(reverse_haversine(waypoint3,10,180))

print("Moving towards waypoint 4")
goto_waypoint(reverse_haversine(waypoint4,10,180))

vehicle.mode=VehicleMode("LAND")
while True:
    current_altitude = vehicle.location.global_relative_frame.alt
    if current_altitude < 0.3:
        print("Drone has landed")
        break
    print("Landing. Altitude is: ",current_altitude)
    time.sleep(1)

vehicle.armed=False
vehicle.close()
