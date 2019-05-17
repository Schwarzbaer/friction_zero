#!/usr/bin/env python

import sys
from random import random

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

        # One gyro as well
        self.gyro_power = 0.0
        self.gyro_power_bar = DirectWaitBar(
            text = "",
            value = 0,
            pos = (0, 0, -0.2),
            scale = 1,
        )

        self.accept("q", self.change_repulsor_power, [0.1])
        self.accept("a", self.change_repulsor_power, [-0.1])
        base.task_mgr.add(self.update_repulsor)
        self.accept("w", self.max_out_gyro)
        self.accept("w-repeat", self.max_out_gyro)
        base.task_mgr.add(self.update_gyro)

    def change_repulsor_power(self, change):
        self.repulsor_power += change
        if self.repulsor_power > 0.9:
            self.repulsor_power = 0.9
        if self.repulsor_power < 0.0:
            self.repulsor_power = 0.0

    def update_repulsor(self, task):
        randomized_power = self.repulsor_power + random() * 0.1
        self.repulsor_power_bar['value'] = randomized_power * 100
        self.repulsor_sound.set_volume(randomized_power)
        self.repulsor_sound.set_play_rate(1 + randomized_power * 2)
        return task.cont

    def max_out_gyro(self):
        self.gyro_power = 1.0

    def update_gyro(self, task):
        self.gyro_power_bar['value'] = self.gyro_power * 100
        # FIXME: Adjust gyro whining sound here
        self.gyro_power -= globalClock.dt
        if self.gyro_power < 0.0:
            self.gyro_power = 0.0
        return task.cont


app = MyApp()
app.run()
