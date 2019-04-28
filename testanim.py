from direct.showbase.ShowBase import ShowBase
from direct.actor.Actor import Actor

from panda3d.core import DirectionalLight, KeyboardButton

s = ShowBase()
s.cam.setPos(0, -5, 5)
s.cam.lookAt(0, 0, 0)

a = Actor('assets/cars/Ricardeaut_Magnesium.bam')
a.reparentTo(s.render)
puppet = s.loader.loadModel('assets/cars/Ricardeaut_Magnesium.bam')
puppet.reparentTo(a)

repulsors = a.findAllMatches('**/fz_repulsor*')
print(repulsors)
for r, repulsor in enumerate(repulsors):
    repulsor.setPos(0,0,0)
    repulsor.setP(-90)
    joint = a.exposeJoint(None,"modelRoot","repulsor_bone:"+str(r))
    repulsor.reparentTo(joint)

a.enableBlend()
animations = ["gems", "accelerate", "turn", "strafe", "hover"]
for animation in animations:
    a.setControlEffect(animation, 1)
    a.play(animation)

def pingPong(animation, d, min=0, mid=10, max=20):
    frame = a.getCurrentFrame(animation)
    if frame == None: frame = 0
    rate = a.getPlayRate(animation)
    if d == 1:
        if frame < max:   a.setPlayRate( 1, animation)
        else:             a.setPlayRate( 0, animation)
    elif d == -1:
        if frame > min:   a.setPlayRate(-1, animation)
        else:             a.setPlayRate( 0, animation)
    else:
        if frame > mid:   a.setPlayRate(-1, animation)
        elif frame < mid: a.setPlayRate( 1, animation)
        else:             a.setPlayRate( 0, animation)

def update(task):
    if s.mouseWatcherNode.is_button_down(KeyboardButton.up()):
        pingPong("accelerate", -1)
    elif s.mouseWatcherNode.is_button_down(KeyboardButton.down()):
        pingPong("accelerate", 1)
    else:
        pingPong("accelerate", 0)

    if s.mouseWatcherNode.is_button_down(KeyboardButton.right()):
        pingPong("turn", -1)
    elif s.mouseWatcherNode.is_button_down(KeyboardButton.left()):
        pingPong("turn", 1)
    else:
        pingPong("turn", 0)

    if s.mouseWatcherNode.is_button_down(KeyboardButton.ascii_key(b'd')):
        pingPong("strafe", -1)
    elif s.mouseWatcherNode.is_button_down(KeyboardButton.ascii_key(b'a')):
        pingPong("strafe", 1)
    else:
        pingPong("strafe", 0)

    if s.mouseWatcherNode.is_button_down(KeyboardButton.ascii_key(b's')):
        pingPong("hover", 1, 0,0,20)
    else:
        pingPong("hover", 0, 0,0,20)

    return task.cont

s.taskMgr.add(update)

l = DirectionalLight("light")
ln = render.attachNewNode(l)
render.setLight(ln)

s.run()
