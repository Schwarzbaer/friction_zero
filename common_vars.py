from panda3d.core import BitMask32


FRICTION = 'friction'
DEFAULT_FRICTION_VALUE = 1.25
CM_TERRAIN = BitMask32.bit(1)
CM_VEHICLE = BitMask32.bit(2)
CM_COLLIDE = BitMask32.bit(3)
