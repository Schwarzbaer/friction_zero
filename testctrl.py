from direct.showbase.ShowBase import ShowBase
from direct.actor.Actor import Actor

from panda3d.core import DirectionalLight
from panda3d.core import KeyboardButton


def approach(a, b, i):
	if a < b:
		a += i
		if a > b:
			a = b
	if a > b:
		a -= i
		if a < b:
			a = b

def approachVec(vecA, vecB, i):
	for v, vec in enumerate(vecA):
		vecA[v] = approach(vec, vecB[v], i)



s = ShowBase()
s.cam.set_pos(0, -5, 5)
s.cam.look_at(0, 0, 0)

a = s.loader.loadModel('assets/cars/Ricardeaut_Magnesium.bam')
a.reparent_to(s.render)

repulsors = a.findAllMatches("**/fz_repulsor*")
x_tags = "left", "right", "hover"
y_tags = "forward", "backward"

def control(action):
    s = 4
    for repulsor in repulsors:
        hpr = repulsor.getHpr()
        dst = repulsor.getHpr()
        if action in x_tags:
            dst.z = float(repulsor.getTag("ctrl_"+action))
        elif action in y_tags:
            dst.y = float(repulsor.getTag("ctrl_"+action))
        else:
            dst.x = dst.y = dst.z = 0
            s = 2

        repulsor.setHpr(hpr)

def update(task):
    control("none")
    if s.mouseWatcherNode.is_button_down(KeyboardButton.left()):
        control("left")
    elif s.mouseWatcherNode.is_button_down(KeyboardButton.right()):
        control("right")
    if s.mouseWatcherNode.is_button_down(KeyboardButton.up()):
        control("forward")
    elif s.mouseWatcherNode.is_button_down(KeyboardButton.down()):
        control("backward")
    if s.mouseWatcherNode.is_button_down(KeyboardButton.space()):
        control("hover")
    return task.cont

s.taskMgr.add(update)

l = DirectionalLight("light")
ln = render.attachNewNode(l)
render.setLight(ln)

s.run()
