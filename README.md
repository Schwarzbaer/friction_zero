Friction: Zero
==============

A racing game about hovering vehicles.


TODO
----

### Code

* Animations
  * See branch magnesium-animation-bug. Using subParts in an Actor causes an
    exception when .pose is used, as Actor seems to look for the file with the
    animation, despite it being present in the model file.
    * Write minimalized example
    * Probably bugfix Actor
* Controls
  * rstick should disable to-horizon stabilization.
  * Space mouse bindings
* ECU
  * Gyroscopic stabilization
    * Replace numpy.linalg.eig() with a PCA-based approach
  * Repulsor damping: Why is there a droop in height relative to target height
    at low repulsor powers, even though sufficient capacity is present? It can
    be mitigated by upping the repulsor power, but that leads to absurd jumping
    potential.
    * At an activation_distance of 8m and a force of 10kN per repulsor, a 1t
      Magnesium jumps into the air at 30km/h, reaching maybe 30-50 meters of
      height from an on-the-ground start.
    * A target flight height of 6m leads to an actual flight height of 4m. At
      4m, acceleration is near optimum, activation still staying below 100%
      constantly. One step higher, it going over 100% intermittendly is normal.
    * Reducing the repulsor power to 9kN leads to a drastic performance loss in
      the algorithm. A target height of ~11m has to be set to achieve lift that
      is slowly penduluming between 1m and 4m. Jumping abilities feel good
      though; on lab, a Magnesium lying on the floor, 8m activation distance,
      can easily clear the height of the largest block in the raised platform,
      but not about twice as much. At 1m, the block can barely cleared, at 3m
      about 2m are missing at the peak of the jump.
  * (low importance) Make angular stabilization deactivateable
* Gyro power: 800Nm units is a bit high as a cap. 400Nm still helps a lot, but
  has noticeable wobbles at collisions or big ground normal changes. 300Nm is
  still very playable, but gyro weakness i noticeable, and one instability
  occurred. 200Nm still lets the car (sometimes? Frame rate dependent?) roll on
  the ground, 175Nm is too small for that.
* File for map / vehicle values: TOML? YAML?
  * Write file from model data
  * Read file during construction
* Air drag and aerodynamics (https://www.gamedev.net/forums/topic/457235-flight-sim-physics/)
  * Drag scales with the square of the speed
  * Artist-defined air density
  * Air brakes
  * Sail effect (downforce, vector turning)
  * Stabilizers effect (increasing sail effect)
* Course gates
  * Check which gate has been passed this frame
  * Race rules accounting
  * Timekeeping
* Timing bug: Physics should happen after rendering, camera adjustments before
  it. Currently the most visible effect is that last frame's contact points are
  shown.
* Sound
  * Repulsor activity
  * Background music
  * Crashes
* GUI
  * (SMALL) Basic setup: In a separate render bin
  * (SMALL) Elevation: Fighter jet like height-over-ground indication
  * (SMALL) Speed direction
  * (SMALL) Repulsor activation levels
  * (SMALL) Thruster activation
* Limits on thruster
  * Fuel
  * Overheating
* Camera controls and automatics
  * (SMALL) HUD element for camera mode
  * Camera distance and FOV should be related to vehicle's linear speed.
  * Camera should respect the ground / objects.
  * Cinematics
* Shadows
* Repulsoring other vehicles
* Clamp timestep in physics simulation
  * (SMALL) Move value min_frame_rate into settings file
* Game Mode: Free Driving
* Countdown to start
* Game Mode: Timed solo race
* Recording races
* Game Mode: Racing against ghosts (recorded races)


### Art

* Magnesium
  * Add one or more thrusters with an animation
* All maps
  * Some ambient lighting, please!
* Lab map
  * Add a tube section into the large looping, so that infinite loopings become
    possible.
* hilltrack
  * Align the track's texture with the track's direction
* Design two other vehicles
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


### Other

* Add license: BSD and CC0?
* Add CREDITS


### Post-Prototype

* ECSify everything
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
* Game mode Tricks: Get as many tricks points as possible in a limited time
* Game mode Rally: Timed race with time deducted for tricks
* Game mode Treasure Hunt: Find items hidden on a map
  * Radar may indicate distance and/or direction when close
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
* City map: Start with a few blocks and expand.


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
