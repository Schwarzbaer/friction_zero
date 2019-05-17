#!/usr/bin/env python

import sys
from random import random

from direct.gui.DirectGui import DirectWaitBar

from direct.showbase.ShowBase import ShowBase
from direct.showbase.Audio3DManager import Audio3DManager

from panda3d.core import NodePath
from panda3d.core import KeyboardButton


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
        self.emitter = base.render.attach_new_node("repulsor_0")

        # One repulsor sound
        self.repulsor_sound = self.audio3d.load_sfx(
            'assets/audio/sound/repulsor.wav',
        )
        self.audio3d.attachSoundToObject(
            self.repulsor_sound,
            self.emitter,
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

        # One gyro as well
        self.gyro_sound = self.audio3d.load_sfx(
            'assets/audio/sound/gyro.wav',
        )
        self.audio3d.attachSoundToObject(
            self.gyro_sound,
            self.emitter,
        )
        self.gyro_sound.set_loop(True)
        self.gyro_sound.play()

        self.gyro_power = 0.0
        self.gyro_power_bar = DirectWaitBar(
            text = "",
            value = 0,
            pos = (0, 0, -0.2),
            scale = 1,
        )
        base.task_mgr.add(self.update_gyro)

        # And one thruster.
        self.thruster_sound = self.audio3d.load_sfx(
            'assets/audio/sound/hairdryer.wav',
        )
        self.audio3d.attachSoundToObject(
            self.thruster_sound,
            self.emitter,
        )
        self.thruster_overheat_sound = self.audio3d.load_sfx(
            'assets/audio/sound/pop.wav',
        )
        self.audio3d.attachSoundToObject(
            self.thruster_overheat_sound,
            self.emitter,
        )
        self.thruster_sound.set_loop(True)
        self.thruster_sound.play()
        self.thruster_power = 0.0
        self.thruster_overheated = False
        self.thruster_heat = 0.0
        self.thruster_heat_bar = DirectWaitBar(
            text = "",
            value = 0,
            pos = (0, 0, -0.4),
            scale = 1,
        )
        base.task_mgr.add(self.update_thruster)

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

    def update_gyro(self, task):
        if base.mouseWatcherNode.is_button_down(KeyboardButton.ascii_key(b'w')):
            self.gyro_power = 1.0
        self.gyro_power_bar['value'] = self.gyro_power * 100
        self.gyro_sound.set_volume(self.gyro_power)
        self.gyro_sound.set_play_rate(0.2 + self.gyro_power * 0.8)

        self.gyro_power -= globalClock.dt
        if self.gyro_power < 0.0:
            self.gyro_power = 0.0
        return task.cont

    def update_thruster(self, task):
        heating = 0.33
        cooling = 0.04
        thruster_ramp_time = 0.4
        thrusting = base.mouseWatcherNode.is_button_down(
            KeyboardButton.ascii_key(b's'),
        )
        overheated = self.thruster_heat > 1.0
        if overheated and not self.overheated:
            self.thruster_overheat_sound.play()
        self.overheated = overheated

        if thrusting and not overheated:
            self.thruster_power += (1 / thruster_ramp_time) * globalClock.dt
        else:
            self.thruster_power -= (1 / thruster_ramp_time) * globalClock.dt
        if self.thruster_power < 0.0:
            self.thruster_power = 0.0
        if self.thruster_power > 1.0:
            self.thruster_power = 1.0
        power = self.thruster_power
        self.thruster_heat += (-cooling * (1 - power) + heating * power) * globalClock.dt
        if self.thruster_heat < 0.0:
            self.thruster_heat = 0.0
        self.thruster_sound.set_volume(self.thruster_power)
        self.thruster_sound.set_play_rate(self.thruster_power)
        self.thruster_heat_bar['value'] = self.thruster_heat * 100
        self.thruster_heat_bar['text'] = "{:3.0f}%".format(
            self.thruster_power * 100,
        )
        if self.thruster_heat > 1.0:
            self.thruster_heat_bar['barColor'] = (1, 1, 1, 1)
        else:
            self.thruster_heat_bar['barColor'] = (1, 0, 0, 1)
        return task.cont


app = MyApp()
app.run()
