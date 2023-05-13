#!/usr/bin/env python3
from aiohttp import web
from rtcbot import RTCConnection, CVCamera, Microphone, getRTCBotJS

import rospy
from geometry_msgs.msg import Twist

import threading

#rospy init
rospy.init_node("velocity_publisher")
velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
rospy.sleep(2)

#Route table definition
routes = web.RouteTableDef()

# Create cameras
cam = CVCamera(cameranumber=0)
#cam2 = CVCamera(cameranumber=2)
mic = Microphone()

# 1 global connection
conn = RTCConnection()
# Initial subscriptions
conn.video.putSubscription(cam)
conn.audio.putSubscription(mic)

keystates = {"w": False, "a": False, "s": False, "d": False, "j": False, "k": False}
robot_speed=0.1

#Publisher for speed commands
def publisher():
    global keystates
    global robot_speed
    loop_rate = rospy.Rate(10)
    robot_vel = Twist()
    
    #Slow infinite loop to set robot_vel
    while True:
        #Forward/back
        if keystates["w"]:
            robot_vel.linear.x=robot_speed
        elif keystates["s"]:
            robot_vel.linear.x=-robot_speed
        else:
            robot_vel.linear.x=0
        #Left/right
        if keystates["a"]:
            robot_vel.linear.y=robot_speed
        elif keystates["d"]:
            robot_vel.linear.y=-robot_speed
        else:
            robot_vel.linear.y=0
        #Turn
        if keystates["j"]:
            robot_vel.angular.z=robot_speed
        elif keystates["k"]:
            robot_vel.angular.z=-robot_speed
        else:
            robot_vel.angular.z=0

        velocity_pub.publish(robot_vel)
        loop_rate.sleep()

@conn.subscribe
def onMessage(m):
    global keystates
    global robot_speed
    #Reading keycodes
    if m["keyCode"] == 87:  # W
        keystates["w"] = m["type"] == "keydown"
    elif m["keyCode"] == 83:  # S
        keystates["s"] = m["type"] == "keydown"
    elif m["keyCode"] == 65:  # A
        keystates["a"] = m["type"] == "keydown"
    elif m["keyCode"] == 68:  # D
        keystates["d"] = m["type"] == "keydown"
    elif m["keyCode"] == 74:  # J
        keystates["j"] = m["type"] == "keydown"
    elif m["keyCode"] == 75:  # K
        keystates["k"] = m["type"] == "keydown"
    
    #Change speed
    elif m["keyCode"] == 38 and m["type"] == "keydown":  # Up arrow
        robot_speed+=0.1
    elif m["keyCode"] == 40 and m["type"] == "keydown" and robot_speed>0.1:  # Down arrow
        robot_speed-=0.1
    
    #Switch cameras
    #elif m["keyCode"] == 49 and m["type"] == "keydown":  # 1
        #conn.video.putSubscription(cam)
    #elif m["keyCode"] == 50 and m["type"] == "keydown":  # 2
        #conn.video.putSubscription(cam2)
    

# Serve the RTCBot javascript library at /rtcbot.js
@routes.get("/rtcbot.js")
async def rtcbotjs(request):
    return web.Response(content_type="application/javascript", text=getRTCBotJS())

# This sets up the connection
@routes.post("/connect")
async def connect(request):
    clientOffer = await request.json()
    serverResponse = await conn.getLocalDescription(clientOffer)
    return web.json_response(serverResponse)

@routes.get("/")
async def index(request):
    return web.Response(
        content_type="text/html",
        text=open("index.html", "r").read()
    )

async def cleanup(app=None):
    await conn.close()

#separate thread for publisher function
pub_thread = threading.Thread(target=publisher)
pub_thread.start()

#webapp
app = web.Application()
app.add_routes(routes)
app.on_shutdown.append(cleanup)
web.run_app(app)
