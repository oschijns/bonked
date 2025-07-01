# Bonked!
A simple collision engine for retro console development.

Contrary to fully fledge physics engine such as Rapier or Avian, Bonked! only
supports basic features such as preventing solid objects to pass through each
other. Physics objects are filtered into three categories:
- Kinematic bodies which are intended to move over time.
- Static bodies which should not move at all.
- Trigger areas which can trigger events when kinematic bodies pass through them.

The library also support a few queries:
- Raycasts against solid bodies.
- Shape queries against solid bodies.
- Point queries against trigger areas.
