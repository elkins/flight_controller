# Flight Demonstration Videos

This directory contains recorded flight demonstrations from the PyBullet physics simulation.

## safe_landing_demo.mp4

**Duration**: 5.8 seconds  
**Resolution**: 1024x768  
**Mode**: Hover with safe landing sequence

Demonstrates the advanced flight controller's safe landing capability:
- Stable hover at ~2m altitude
- Gradual descent during landing approach phase
- Controlled final landing with reduced throttle
- Smooth touchdown without crashing

This video showcases the 5-phase flight sequence implementation with velocity damping and smart throttle control that enables safe landings in the physics simulation.

### To record your own videos:

```bash
python examples/pybullet_qgroundcontrol.py --duration 35 --mode hover --gui --record videos/my_flight.mp4
```

Available modes: `hover`, `circle`, `figure8`, `square`, `waypoint`
