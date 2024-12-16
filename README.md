# bonked
Simple ECS physics engine for embedded systems

## Status
This library does not work in its current state.
I initially planed for the library user to implement their own collision accumulator.
But this actually cause a lot of issues.

This library was intended for providing a basic physics engine for old consoles.
But it doesn't provide proper solution for neither 2D nor 3D games.
For old 2D game consoles, which usually only support 2D tiles, you would prefer 
to base your collision system on tiles instead of arbitrary rotated 2D shapes.
For 3D games, primitives are to restrictive, and most games were instead based on mesh collisions.

This library could be reworked for making space shooters,
which would be a type of game better suited for the feature offered byt this library.
