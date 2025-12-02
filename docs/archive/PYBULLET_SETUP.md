# PyBullet Simulator Setup

## Python Version Requirement

⚠️ **IMPORTANT**: PyBullet currently only supports Python ≤ 3.12.x

Your current Python version is **3.13.3**, which is **not compatible** with PyBullet.

## Installation Options

### Option 1: Use Python 3.12 (Recommended)

Install Python 3.12 using pyenv:

```bash
# Install Python 3.12
pyenv install 3.12.8

# Create virtual environment
cd /Users/georgeelkins/aviation/flight_controller
pyenv local 3.12.8
python -m venv venv_pybullet
source venv_pybullet/bin/activate

# Install dependencies
pip install pybullet numpy

# Run tests
python test_pybullet.py
```

### Option 2: Use Docker

Run PyBullet in an isolated container:

```bash
# Create Dockerfile
cat > Dockerfile <<'EOF'
FROM python:3.12-slim

WORKDIR /app
COPY . .

RUN pip install pybullet numpy

CMD ["python", "test_pybullet.py"]
EOF

# Build and run
docker build -t flight-sim .
docker run flight-sim
```

### Option 3: Use Conda

Create isolated conda environment:

```bash
# Create environment with Python 3.12
conda create -n flight_sim python=3.12
conda activate flight_sim

# Install PyBullet
pip install pybullet numpy

# Run tests
python test_pybullet.py
```

## Verifying Installation

After installing with compatible Python version:

```python
python -c "import pybullet as p; print(f'PyBullet {p.getVersionString()} installed successfully')"
```

## Without PyBullet

The flight controller works fine without PyBullet:

- **Hardware Testing**: All HAL tests pass with mock hardware
- **PyBoard Deployment**: No PyBullet needed on embedded hardware  
- **Core Functionality**: PID, RC, ESC, IMU all work independently

PyBullet is **only needed for**:
- Physics-based simulation testing
- Flight dynamics validation
- PID tuning in safe environment
- Multi-rotor dynamics research

## Current Test Status

```bash
# These tests work without PyBullet (27/27 passing):
python test_hal.py                    # ✅ Interface tests
python test_hal_robustness.py         # ✅ Robustness tests
python test_core.py                   # ✅ Core flight controller
python test_flight_sim.py             # ✅ Mock flight simulation

# Requires PyBullet + Python ≤3.12:
python test_pybullet.py               # ⚠️  Needs compatible Python
```

## Alternative Simulators

If PyBullet installation is problematic, see `SIMULATOR_INTEGRATION.md` for:

1. **Gazebo** - Industry-standard robotics simulator
2. **JSBSim** - High-fidelity flight dynamics
3. **AirSim** - Microsoft's drone simulator with visuals
4. **Webots** - Open-source robot simulator

## Support

PyBullet tracking Python 3.13 support:
- Issue: https://github.com/bulletphysics/bullet3/issues/4434
- Status: In progress, no release date yet

Until then, use Python 3.12 or alternative simulators.
