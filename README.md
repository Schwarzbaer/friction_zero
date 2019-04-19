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
* Reparent vehicles to environment instead of render, then remove dlight
* Course gates
* Friction
* Refactoring: Move vehicle control logic into an ECU method
* Avionics (should come after ECU)
  * Gyroscopic stabilization
    * Angular damping works just fine, now proper logic for control and
      environment-sensitive stabilization has to be implemented.
  * Replacement of linear damping on repulsors
* Artist-defined repulsors
  * Find a better name than repulsor:*
    * Maybe fz_repulsor*?
* Artist-defined gravity
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
