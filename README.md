# 🚁 Drone Classroom Tour Simulation

A PyBullet-based drone simulation that autonomously tours a realistic classroom environment with computer workstations and seating areas.

## 🎯 Features

- **Realistic Environment**: L-shaped classroom with 20 computer workstations and lounge area
- **Autonomous Navigation**: 36-waypoint tour covering every area systematically  
- **Physics Simulation**: Real drone dynamics with PyBullet physics engine
- **Complete Coverage**: Guaranteed tour completion with timeout protection
- **Visual Feedback**: Real-time progress tracking and waypoint visualization

## 🏢 Environment Layout

**Main Classroom:**
- 16 computer workstations (4x4 grid)
- 32 chairs with realistic spacing
- Professional classroom lighting

**Extension Area:**
- 4 additional workstations
- L-shaped connection to main room

**Lounge Area:**
- L-shaped red sofa arrangement
- Coffee table centerpiece
- Relaxation zone for students

## 🚀 Quick Start

### Prerequisites
```bash
pip install pybullet numpy
```

### Running the Simulation
```bash
python lab_simulation.py
```

### Controls
- **Enter**: Start simulation
- **Space**: Pause/Resume
- **R**: Reset drone position
- **Q**: Quit simulation

## 📊 Technical Details

- **Physics Engine**: PyBullet
- **Drone Model**: Quadcopter with realistic dynamics
- **Tour Duration**: 8-10 minutes
- **Waypoints**: 36 strategic positions
- **Movement System**: Force-based propulsion with PID control

## 🎥 Demo

![Drone Tour Demo](demo/tour_preview.gif)

*Complete autonomous tour of the classroom environment*

## 📁 Project Structure

```
drone-classroom-tour/
├── lab_simulation.py      # Main simulation code
├── README.md             # Project documentation
├── requirements.txt      # Python dependencies
├── demo/                # Demo videos and screenshots
│   ├── tour_preview.gif
│   ├── classroom_layout.png
│   └── full_tour.mp4
└── .gitignore           # Git ignore rules
```

## 🛠️ Customization

The simulation can be easily modified:
- **Waypoints**: Edit the `waypoints` list in `lab_simulation.py`
- **Environment**: Modify furniture positions and colors
- **Drone Behavior**: Adjust force multipliers and movement parameters
- **Camera Settings**: Change viewing angles and follow modes

## 📈 Performance

- **Frame Rate**: 60 FPS (real-time simulation)
- **Physics Steps**: 240 Hz for accurate dynamics  
- **Memory Usage**: ~200MB typical
- **CPU Usage**: Moderate (optimized for smooth playback)

## 🤝 Contributing

Feel free to submit issues and enhancement requests!

## 📄 License

MIT License - feel free to use and modify for your projects. 
