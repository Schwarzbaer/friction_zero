Friction: Zero
==============

A racing game about hovering vehicles.


TODO
----

### Code

* Shadows
* Avionics
  * Gyroscopic stabilization
    * Determine local "up" from repulsor ray fractions
  * Basic bone control of repulsors
* Animations
  * Repulsor animation
* Gamepad support
  * Left stick controls bone animation.
    * Neutral is straight down
    * Up/Down is backward/forward tilt
    * left/right for spiral setup
      * If <button> is pressed, parallel sideways strafe tilt
  * Right stick for camera offset
  * L1 for thrust
  * D-pad for gyro?
* Air drag and aerodynamics
  * Drag scales with the square of the speed
  * Artist-defined air density
  * Air brakes
* Course gates
  * Check which gate has been passed this frame
  * Race rules accounting
  * Timekeeping
* GUI
  * Speed
* Camera controls and automatics
  * Camera distance should be related to vehicle's linear speed.
  * Camera should respect the ground / objects.
* Repulsoring other vehicles
* Clamp timestep in physics simulation
* CCD
* Countdown to start


### Art

* Design two other vehicles
* When movement is mostly implemented, make racetrack
  * Start with a looping road.
  * Add start/finish, gates, spawn points
  * Add surroundings
* Sounds:
  * engine (playbackrate to speed?)
  * vehicleXvehicle impact
  * vehicleXenvironment impact
* Music:
  * menu
  * track


### Tools

* Map / vehicle verification
* File for map / vehicle values: TOML? YAML?
  * Write file from model data
  * Read file during construction


### Other

* Add license: BSD and CC0?


### Post-Prototype

* Menu
  * Main menu
  * Options
  * Track selection screen
  * Vehicle selection screen
    * Vehicle material color-picker
    * Choose repulsor model
* AI


Models checklist
----------------

* Vehicles
  * Scene
    * friction			(1)
    * mass			(1000)
  * fz_repulsor:N
    * activation_distance	(5)
    * force			(3000)
  * fz_thruster:N
    * force			(20000)
  * fz_spawn_point_connector

* Map
  * Scene
    * gravity (temp)		(9.810)
  * fz_collision
    * named_collision
      * friction		(1)
  * fz_spawn_point:N
