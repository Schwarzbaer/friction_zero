Friction: Zero
==============

A racing game about hovering vehicles.


TODO
----

### Code

* Air drag and aerodynamics
  * Drag scales with the square of the speed
  * Artist-defined air density
  * Air brakes
* Course gates
* Friction
* Refactoring: Move vehicle control logic into an ECU method
* Avionics (should come after ECU)
  * Gyroscopic stabilization
    * Angular damping works just fine, now proper logic for control and
      environment-sensitive stabilization has to be implemented.
  * Replacement of linear damping on repulsors
* Camera controls and automatics
* GUI
* Menu
  * Main menu
  * Options
  * Track selection screen
  * Vehicle selection screen
    * Vehicle material color-picker
    * Choose repulsor model
* Countdown to start
* Animations
* Gamepad support
  * D-pad for gyro?


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

### Other

* Add license


### Models checklist

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
  

