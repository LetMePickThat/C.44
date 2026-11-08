---
name: vtk-rendering-checks
description: "How to measure 3D view rendering (flicker, antialiasing) in the app, and the OIT-vs-MSAA pitfall"
metadata:
  node_type: memory
  type: reference
  modified: 2026-11-08T17:11:08.915Z
---

- Pixel captures of the 3D views need a real on-screen window: with QT_QPA_PLATFORM=offscreen (the tests' setting) the VTK window renders black, and SetOffScreenRendering does not help. Show the SimView, process events, then use vtkWindowToImageFilter.
- Translucent actors (opacity < 1) go through VTK's order-independent translucency (OIT, on by default), which bypasses the window's MSAA ×8: lines come out with 1 px aliased steps that crawl while the view turns. Fix used for the orbits (8 Nov 2026): draw opaque, with the colour pre-blended over the black background. The trajectories (opacity 0.8 / 0.7) still have this.
- pyvista's default theme background is white: a white test line on it shows nothing.
- GPU float32 precision (8 Nov 2026, user saw scrambled path segments and crushed symbols zoomed in close): VTK shifts
  a vertex buffer to its data's centre once, so km-scale geometry is rounded to ~0.1 m far from that centre. Any NEW
  dynamic actor made with add_mesh outside SimView._poly must get gui/sim_view._near_camera_precision(actor, mesh)
  (vtkPolyDataMapper with VBO shift NEAR_PLANE = 5; the enum is not wrapped). pyvista's DataSetMapper has no
  SetVBOShiftScaleMethod. Bodies / ship model placed by an actor matrix are not affected.
- A tiny test object near the camera needs cam.SetClippingRange set by hand (ResetCameraClippingRange over a scene
  of hundreds of km puts the near plane beyond it: nothing drawn).

Related: [[mouse-rotation-glide]]
