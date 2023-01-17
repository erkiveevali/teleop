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

#Publisher for speed commands
def publisher():
    #Global variable for keystates
    global keystates
    
    loop_rate = rospy.Rate(10)
    robot_vel = Twist()
    
    #Slow infinite loop to set robot_vel
    while True:
        #Forward/back
        if keystates["w"]:
            robot_vel.linear.x=0.1
        elif keystates["s"]:
            robot_vel.linear.x=-0.1
        else:
            robot_vel.linear.x=0
        #Left/right
        if keystates["a"]:
            robot_vel.linear.y=0.1
        elif keystates["d"]:
            robot_vel.linear.y=-0.1
        else:
            robot_vel.linear.y=0
        #Turn
        if keystates["j"]:
            robot_vel.linear.z=0.1
        elif keystates["k"]:
            robot_vel.linear.z=-0.1
        else:
            robot_vel.linear.z=0

        velocity_pub.publish(robot_vel)
        loop_rate.sleep()

@conn.subscribe
def onMessage(m):
    global keystates
    #Reading keycodes
    if m["keyCode"] == 87:  # W
        keystates["w"] = m["type"] == "keydown"
    elif m["keyCode"] == 83:  # S
        keystates["s"] = m["type"] == "keydown"
    elif m["keyCode"] == 65:  # A
        keystates["a"] = m["type"] == "keydown"
    elif m["keyCode"] == 68:  # D
        keystates["d"] = m["type"] == "keydown"
    elif m["keyCode"] == 68:  # J
        keystates["j"] = m["type"] == "keydown"
    elif m["keyCode"] == 68:  # K
        keystates["k"] = m["type"] == "keydown"
    
    #Switch cameras
    #elif m["keyCode"] == 49:  # 1
        #conn.video.putSubscription(cam)
    #elif m["keyCode"] == 50:  # 2
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

pub_thread = threading.Thread(target=publisher)
pub_thread.start()

app = web.Application()
app.add_routes(routes)
app.on_shutdown.append(cleanup)
web.run_app(app)
