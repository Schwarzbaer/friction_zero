import sys

from direct.showbase.ShowBase import ShowBase
from direct.actor.Actor import Actor

from panda3d.core import DirectionalLight, KeyboardButton, SequenceNode


s = ShowBase()
s.accept('escape', sys.exit)
s.cam.setPos(0, -5, 5)
s.cam.lookAt(0, 0, 0)

a = Actor('assets/cars/Ricardeaut_Magnesium.bam')
a.reparentTo(s.render)
puppet = s.loader.loadModel('assets/cars/Ricardeaut_Magnesium.bam')
puppet.find("armature").hide()
puppet.reparentTo(a)

beam = s.loader.loadModel('assets/effect/beam.bam')

# convert all nodes called "sequence_node:N" into a SequenceNode
sequence_nodes = beam.findAllMatches("**/sequence_node:*")
for sequence_node in sequence_nodes:
    framerate = sequence_node.get_tag("sequence_fps")
    SequenceNode("sequence").replace_node(sequence_node.node())
    sequence_node.node().setFrameRate(int(framerate))
    sequence_node.node().loop(1)

# instance the animated beam to all fz_beam nodes (just one for magnesium)
beam_nodes = a.findAllMatches("**/*fz_beam*")
beams = []
for beam_node in beam_nodes:
    beams.append(beam.instanceTo(beam_node))

thrust = 0

def update(task):
    global thrust
    if thrust > 0:
        thrust -= 1
    if s.mouseWatcherNode.is_button_down(KeyboardButton.space()):
        if thrust < 15:
            thrust += 2

    for beam in beams:
        beam.setScale(thrust/8)

    return task.cont

s.taskMgr.add(update)

l = DirectionalLight("light")
ln = render.attachNewNode(l)
render.setLight(ln)

s.run()
