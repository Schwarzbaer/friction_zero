#!/usr/bin/env python

import sys

from direct.gui.DirectGui import DirectWaitBar

from direct.showbase.ShowBase import ShowBase
from direct.showbase.Audio3DManager import Audio3DManager

from panda3d.core import NodePath


class MyApp(ShowBase):
    def __init__(self):
        # The basics
        ShowBase.__init__(self)
        base.disableMouse()
        self.accept("escape", sys.exit)

        # Audio listener gets attached to the camera
        self.audio3d = Audio3DManager(
            base.sfxManagerList[0],
            self.camera,
        )

        # One repulsor emitter with sound
        self.repulsor_emitter = base.render.attach_new_node("repulsor_0")
        self.repulsor_sound = self.audio3d.load_sfx(
            'assets/audio/sound/repulsor.wav',
        )
        self.audio3d.attachSoundToObject(
            self.repulsor_sound,
            self.repulsor_emitter,
        )
        self.repulsor_sound.set_loop(True)
        self.repulsor_sound.play()

        self.repulsor_power = 0.0
        self.repulsor_power_bar = DirectWaitBar(
            text = "",
            value = 0,
            pos = (0, 0, 0),
            scale = 1,
        )

        self.accept("q", self.change_repulsor_power, [0.1])
        self.accept("a", self.change_repulsor_power, [-0.1])
        base.task_mgr.add(self.update_repulsor)

    def change_repulsor_power(self, change):
        self.repulsor_power += change
        if self.repulsor_power > 1.0:
            self.repulsor_power = 1.0
        if self.repulsor_power < 0.0:
            self.repulsor_power = 0.0

    def update_repulsor(self, task):
        self.repulsor_power_bar['value'] = self.repulsor_power * 100
        self.repulsor_sound.set_volume(self.repulsor_power)
        self.repulsor_sound.set_play_rate(1 + self.repulsor_power * 2)
        return task.cont


app = MyApp()
app.run()
