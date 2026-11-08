---
name: vtk-rendering-checks
description: "How to measure 3D view rendering (flicker, antialiasing) in the app, and the OIT-vs-MSAA pitfall"
metadata:
  type: reference
---

- Pixel captures of the 3D views need a real on-screen window: with QT_QPA_PLATFORM=offscreen (the tests' setting) the VTK window renders black, and SetOffScreenRendering does not help. Show the SimView, process events, then use vtkWindowToImageFilter.
- Translucent actors (opacity < 1) go through VTK's order-independent translucency (OIT, on by default), which bypasses the window's MSAA ×8: lines come out with 1 px aliased steps that crawl while the view turns. Fix used for the orbits (8 Nov 2026): draw opaque, with the colour pre-blended over the black background. The trajectories (opacity 0.8 / 0.7) still have this.
- pyvista's default theme background is white: a white test line on it shows nothing.

Related: [[mouse-rotation-glide]]
