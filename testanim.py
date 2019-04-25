from direct.showbase.ShowBase import ShowBase
from direct.actor.Actor import Actor

from panda3d.core import DirectionalLight

s = ShowBase()
s.cam.setPos(0, -5, 5)
s.cam.lookAt(0, 0, 0)

a = Actor('assets/cars/Ricardeaut_Magnesium.bam')
a.reparentTo(s.render)
puppet = s.loader.loadModel('assets/cars/Ricardeaut_Magnesium.bam')
puppet.reparentTo(a)

repulsors = a.findAllMatches('**/fz_repulsor*')
for r, repulsor in enumerate(repulsors):
    repulsor.setPos(0,0,0)
    repulsor.setP(-90)
    joint = a.exposeJoint(None,"modelRoot","repulsor_bone:"+str(r))
    repulsor.reparentTo(joint)
a.loop('gems')

l = DirectionalLight("light")
ln = render.attachNewNode(l)
render.setLight(ln)

s.run()
