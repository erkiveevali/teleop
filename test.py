#!/usr/bin/env python3
from aiohttp import web
routes = web.RouteTableDef()

from rtcbot import RTCConnection, CVCamera, Microphone, getRTCBotJS

# Create cameras
cam = CVCamera(cameranumber=0)
cam2 = CVCamera(cameranumber=2)
mic = Microphone()

# 1 global connection
conn = RTCConnection()
# Initial subscriptions
conn.video.putSubscription(cam)
conn.audio.putSubscription(mic)

keystates = {"w": False, "a": False, "s": False, "d": False}

@conn.subscribe
def onMessage(m):
    global keystates
    
    #WASD control
    if m["keyCode"] == 87:  # W
        keystates["w"] = m["type"] == "keydown"
    elif m["keyCode"] == 83:  # S
        keystates["s"] = m["type"] == "keydown"
    elif m["keyCode"] == 65:  # A
        keystates["a"] = m["type"] == "keydown"
    elif m["keyCode"] == 68:  # D
        keystates["d"] = m["type"] == "keydown"
    
    #Switch cameras
    elif m["keyCode"] == 49:  # 1
        conn.video.putSubscription(cam)
    elif m["keyCode"] == 50:  # 2
        conn.video.putSubscription(cam2)
    
    print({
            "forward": keystates["w"] * 1 - keystates["s"] * 1,
            "leftright": keystates["d"] * 1 - keystates["a"] * 1,
        })

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

app = web.Application()
app.add_routes(routes)
app.on_shutdown.append(cleanup)
web.run_app(app)
