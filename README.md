# TrueNgine
## A N-Dimensional Renderer

This is an attempt to implement an N-Dimensional Renderer, to allow the visualization of N-Dimensonal shapes.

The rendering is divided in two parts: the cut of the space and the perspective rendering.


## Cutting the Space
This part of the rendering takes the ND model and, given the camera current position in ND-space, slices it so that only the part that intersects the current viewed space is seen.

This has not yet been implemented, but will be added in the future.

## Perspective Rendering
This part of the rendering utilizes perspective projection to show the user the N dimensions above 3D look like.

The perspective projection was made following this incredible article by Magnus Deininger:

https://ef.gy/linear-algebra:perspective-projections

Some adjustments have been made, given the different libraries used and other factors, but it remains largely the same.

This part is fully functional.


