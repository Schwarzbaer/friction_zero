Friction: Zero
==============

A racing game about hovering vehicles.


TODO
----

### Code

* Bug
  * Add a HUD element for repulsor activation
  * Why are there repulsor contacts reported during flight?
* Animations
  * Air brake
* ECU
  * Gyroscopic stabilization
    * Determine local "up" from repulsor ray fractions
  * Repulsors should act like dampening springs
* Air drag and aerodynamics
  * Drag scales with the square of the speed
  * Artist-defined air density
  * Air brakes
* Course gates
  * Check which gate has been passed this frame
  * Race rules accounting
  * Timekeeping
* Sound
  * Repulsor activity
  * Background music
  * Crashes
* GUI
  * Basic setup: In a separate render bin
  * Speed
  * Elevation: Fighter jet like height-over-ground indication
  * Linear speed
  * Repulsor activation levels
  * Thruster activation
* Limits on thruster
  * Fuel
  * Overheating
* Camera controls and automatics
  * Camera distance should be related to vehicle's linear speed.
  * Camera should respect the ground / objects.
  * Cinematics
* Shadows
* Repulsoring other vehicles
* Clamp timestep in physics simulation
  * Move value min_frame_rate into settings file
* CCD
* Countdown to start
* Recording races
  * Racing against ghosts


### Art

* Design two other vehicles
* Airbrake animations
* Effect for the point where the repulsor ray hits the ground
* When movement is mostly implemented, make racetrack
  * Start with a looping road.
  * Add start/finish, gates, spawn points
  * Add surroundings
* Lab map: Like plane, but with stunt elements to the right, left, and back of
  starting position.
* Tricks map: Like the lab map, but prettier. A skatepark for hovercars.
* Driving school: A tutorial map.
* Sounds:
  * engine (playbackrate to speed?)
  * vehicleXvehicle impact
  * vehicleXenvironment impact
* Music:
  * menu
  * track


### Tools

* Normalize pman build / asset workflow
* Map / vehicle verification
* File for map / vehicle values: TOML? YAML?
  * Write file from model data
  * Read file during construction


### Other

* Add license: BSD and CC0?
* Add CREDITS


### Post-Prototype

* Keybindings
  * Allow for multiple players
  * Allow for multiple devices (per player?)
* Menu
  * Main menu
  * Options
  * Track selection screen
  * Vehicle selection screen
    * Vehicle material color-picker
    * Choose repulsor model
* Ingame screens from trackside AI cameras
* Surfaces: Different materials can be repulsed differently well
* Points (bonus time?) for tricks
* Quantum lock: Become locked to a track like a superconductor in a magnetic
  field
* Minimap
* Repulsor pads on maps
* Optional gates
* AI
  * Cruise mode / Autopilot
    * Follow a given direction or given path
    * ...and correct if pushed from it
  * Racing
  * Civilian
  * Escaper
    * Try to stay away from a given set of points
    * Try to get away from a given set of vehicles
  * Pursuer
    * Stay close to a given vehicle
* Game mode: Series (requires racing AI)
  * Formula R: Rally-style tracks
  * Formula 0: Race and city tracks
  * Formula C: Cross-country longhaul drive
  * Formula Q: Quantum lock tracks (requires quantum lock)
  * Formula X: One of each, with or without changing cars between races
* Game mode: Taxi driver / Transporter (requires civilian AI)
  * Pick up and drop off guests / cargo
  * Optimize for time and / or passenger comfort
* Game mode: Escape and Pursuit (requires escaper and pursuer AI)
  * The escaper tries to stay a distance away from the pursuer
  * The pursuer tries to not let that happen until a timer runs out
* Multiplayer
  * Split-screen
  * Networked
* Damage model
  * Artwork for aesthetic damage
  * Systems (repulsors, gyros, thrusters) suffer performance loss (low-powered
    or offline) for a time (or permanently)
* Game mode: Demolition Derby (requires damage model. May require weapons.)
  * Last one standing


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
