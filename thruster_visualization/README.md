## Joystick interface
The `thruster_visualization` package allows users to visualize the thruster forces outputs from the thruster allocation in Foxglove studios.
It subscribes on the thrust/thruster_forces topic and publishes the according arrows as a visualization msg MarkerArray on the topic visualization_marker that you can show in Foxglove.

The thruster placement and orientation is based on the one shown in freya.yaml.

    /\          /\
   /  \  front /  \
  /    \      /    \
  |=3↗=|      |=0↖=|
  |    |      |    |
  |    |      |    |
  |    |======|    |
  |    |      |    |
  |    |      |    |
  |    |      |    |
  |=2↖=|==||==|=1↗=|
