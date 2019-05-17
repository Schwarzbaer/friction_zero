Friction: Zero
==============

A racing game about hovering vehicles.


TODO
----

### Bugs

* Move the `* dt` from the gyro's ECU into `apply_gyroscope()`.
* Animations
  * See branch magnesium-animation-bug. Using subParts in an Actor causes an
    exception when .pose is used, as Actor seems to look for the file with the
    animation, despite it being present in the model file.
    * Revert to last commit before the merge; animations are now in separate
      files. Whoops...
    * Monitor issue or fix it yourself: https://github.com/panda3d/panda3d/issues/647
* All physics-related things that use `globalClock.dt` should pull the time step
  size from the Environment instead, so that they too are capped by the physics
  minimum framerate.
* When a vehicle flips on its back, it gets repulsor contacts above (globally)
  the repulsors.


### Features

* (SMALL STUFF) Default controller for NPC vehicles
* Controls
  * Space mouse bindings
* ECU
  * Gyroscopic stabilization
    * Replace numpy.linalg.eig() with a PCA-based approach
  * (low importance) Make angular stabilization deactivateable
* Aerodynamics
  * Airfoil effect
* Course gates
  * Check which gate has been passed this frame
  * Race rules accounting
  * Timekeeping
* Particle systems
  * Repulsor contact points
  * Vehicle-terrain crashes
  * Vehicle-vehicle crashes
* Sound
  * Repulsor activity
  * Background music
  * Crashes
* GUI
  * (SMALL) Basic setup: In a separate render bin
  * (SMALL) Elevation: Fighter jet like height-over-ground indication
  * (SMALL) Speed direction
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
* racetrack
  * Add lane demarcations (ca. 2.75m apart)
  * Align starting positions with track
  * Add a gentle hilliness to the terrain
  * Add a stretch gently curving back and forth
  * Make hard curves gentler and/or add a bowl shape to their outsides
  * Move trees a few meters away from the track
    * More trees!
  * There should be something big in the center of the map that makes it easier
    to estimate one's position on the map.
* Lab map
  * More visual cues to determine up and down in tunnels
  * Plug up holes, basement_pool sticks out in east-tunnel.
* hilltrack
  * Make the texture direction align with the track, so that you can see where
    in the pipe you are on the left/right direction.
  * Make the bends at the edge higher and more overbearing, to reduce the risk
    of slipping off the track.
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
* Driver g-meter
* Exchangeable vehicle parts
* Menu
  * Main menu
  * Options
  * Track selection screen
  * Vehicle selection screen
    * Vehicle material color-picker
    * Choose repulsor model
* Keybindings
  * Allow for multiple players
  * Allow for multiple devices (per player?)
* Limits on thruster
  * Fuel, replenishable by pit stop
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
  * fz_cockpit
    * fz_cockpit_camera

* Map
  * Scene
    * gravity (temp)		(9.810)
  * fz_collision
    * named_collision
      * friction		(1)
  * fz_spawn_point:N
