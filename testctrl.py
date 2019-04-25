import sys

from direct.showbase.ShowBase import ShowBase
from direct.actor.Actor import Actor

from panda3d.core import DirectionalLight
from panda3d.core import KeyboardButton
from panda3d.core import VBase3


s = ShowBase()
s.disable_mouse()
s.accept('escape', sys.exit)


s.cam.set_pos(0, -5, 5)
s.cam.look_at(0, 0, 0)


a = s.loader.loadModel('assets/cars/Ricardeaut_Magnesium.bam')
a.reparent_to(s.render)


repulsors = a.findAllMatches("**/fz_repulsor*")
ACCELERATE = 'accelerate'
TURN_LEFT = 'turn_left'
TURN_RIGHT = 'turn_right'
STRAFE = 'strafe'
HOVER = 'hover'
tags = [ACCELERATE, TURN_LEFT, TURN_RIGHT, STRAFE, HOVER]
X = '_x'
Y = '_y'
for repulsor in repulsors:
    for tag in tags:
        tag_x = repulsor.get_tag(tag+X)
        if tag_x == '':
            tag_x = 0.0
        else:
            tag_x = float(tag_x)
        tag_y = repulsor.get_tag(tag+Y)
        if tag_y == '':
            tag_y = 0.0
        else:
            tag_y = float(tag_y)
        angle = VBase3(tag_x, tag_y, 0)
        repulsor.set_python_tag(tag, angle)


def animate(forward, turn, strafe, hover):
    # If too many commands are given, normalize their strength
    length = sum([abs(forward), abs(turn), abs(strafe), hover])
    if length > 1:
        forward /= length
        turn /= length
        strafe /= length
        hover /= length
    # Split the turn signal into animation blend factors
    if turn > 0.0:
        turn_left = 0.0
        turn_right = turn
    else:
        turn_left = -turn
        turn_right = 0.0

    for repulsor in repulsors:
        #print(repulsor.get_python_tag(ACCELERATE))
        # FIXME: Negating because wrong tag value
        acceleration_angle = -(repulsor.get_python_tag(ACCELERATE) * forward)
        turning_left_angle = repulsor.get_python_tag(TURN_LEFT) * turn_left
        turning_right_angle = repulsor.get_python_tag(TURN_RIGHT) * turn_right
        strafing_angle = repulsor.get_python_tag(STRAFE) * strafe
        hover_angle = repulsor.get_python_tag(HOVER) * hover
        angle = acceleration_angle + turning_left_angle + \
                turning_right_angle + strafing_angle + hover_angle
        print(turn_left, turn_right)
        repulsor.set_hpr(
            angle.z,
            angle.x,
            angle.y,
        )

def update(task):
    forward, turn, strafe, hover = 0, 0, 0, 0
    if s.mouseWatcherNode.is_button_down(KeyboardButton.up()):
        forward += 1
    if s.mouseWatcherNode.is_button_down(KeyboardButton.down()):
        forward -= 1
    if s.mouseWatcherNode.is_button_down(KeyboardButton.left()):
        turn -= 1
    if s.mouseWatcherNode.is_button_down(KeyboardButton.right()):
        turn += 1
    if s.mouseWatcherNode.is_button_down(KeyboardButton.ascii_key(b'a')):
        strafe -= 1
    if s.mouseWatcherNode.is_button_down(KeyboardButton.ascii_key(b'd')):
        strafe += 1
    if s.mouseWatcherNode.is_button_down(KeyboardButton.ascii_key(b's')):
        hover += 1
    animate(forward, turn, strafe, hover)
    #if s.mouseWatcherNode.is_button_down(KeyboardButton.space()):
    #    control("hover")
    return task.cont

s.taskMgr.add(update)

l = DirectionalLight("light")
ln = render.attachNewNode(l)
render.setLight(ln)

s.run()
