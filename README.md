# Simulation development

## Setup

1. Clone mujoco-wasm and this repo into same folder

```bash
cd /some/root/dir/
git clone https://github.com/menloresearch/mujoco-wasm.git
git clone https://github.com/menloresearch/robotics-sim.git
```

2. Follow the instruction [here](https://github.com/menloresearch/mujoco-wasm?tab=readme-ov-file#for-webassembly) to build the shared lib mucojo-wasm need.

3. Build the mucojo-wasm

```bash
cd /some/root/dir/robotics-sim
emcmake cmake .
make
```

4. (optional) Download the `.STL` Unitree G1 3D models
```bash
cd /some/root/dir/
git clone https://github.com/unitreerobotics/unitree_ros.git
cp -r unitree_ros/robots/g1_description/meshes robotics-sim/examples/scenes
``` 

5. Build TWIST2 env:

**1**. Create conda environment:
```bash
conda env remove -n twist2
conda create -n twist2 python=3.8
conda activate twist2
```

**2**. Install isaacgym. Download from [official link](https://developer.nvidia.com/isaac-gym) and then install it:
```bash
cd isaacgym/python && pip install -e .
```

**3**. Install packages:
```bash
cd rsl_rl && pip install -e . && cd ..
cd legged_gym && pip install -e . && cd ..
cd pose && pip install -e . && cd ..
pip install "numpy==1.23.0" pydelatin wandb tqdm opencv-python ipdb pyfqmr flask dill gdown hydra-core imageio[ffmpeg] mujoco mujoco-python-viewer isaacgym-stubs pytorch-kinematics rich termcolor zmq
pip install redis[hiredis] # for redis communication
pip install pyttsx3 # for voice control
pip install onnx onnxruntime-gpu # for onnx model inference
pip install customtkinter # for gui
```

if this is your first time to use redis, install and start redis server:
```bash
# sudo apt install redis-server
# redis-server --daemonize yes

sudo apt update
sudo apt install -y redis-server

sudo systemctl enable redis-server
sudo systemctl start redis-server
```

edit `/etc/redis/redis.conf`:
```bash
sudo nano /etc/redis/redis.conf
```

modify to the following lines:
```bash
bind 0.0.0.0
protected-mode no
```

then restart redis-server:
```bash
sudo systemctl restart redis-server
```

6. Build GMR & GVHMR env refering to https://github.com/YanjieZe/GMR.git and https://github.com/zju3dv/GVHMR.git 
  # Install GMR in editable mode
  cd ~/GMR
  pip install -e .

  # Install GVHMR in editable mode  
  cd ~/GVHMR
  pip install -e .


Install isaacgym. Download from official link and then install it:

cd isaacgym/python && pip install -e .

## Start


**Terminal 1**. Redis:
```bash
 redis-server
```


**Terminal 2**. WebSocket Bridge:

```bash
cd /robotics-sim/TWIST2
./start_wasm_bridge.sh

```

**Terminal 3**. HTTP Server:


```bash
cd robotics-sim
python3 -m http.server 8000

```
 
**Terminal 4**. Sim2Sim:


```bash
cd /robotics-sim/TWIST2
conda activate twist2
./sim2sim.sh
```

**Terminal 5**. Video Processing and motion retargeting:
```bash
  cd /home/user/mj_teleop/TWIST2
  ./process_latest_video.sh

```



## overview

Repo structure:
  ├── mj_teleop/           # Main project
  │   ├── robotics-sim/
  │   ├── mujoco-wasm/
  │   └── TWIST2/
  ├── GMR/                 # Installed with pip install -e
  └── GVHMR/               # Installed with pip install -e

  1. Browser → upload video
  2. GVHMR pipeline → processes video → generates .pkl motion file
  3. run_motion_server.sh → loads .pkl → publishes motion reference to Redis
  4. sim2sim.sh → runs TWIST2 policy (headless) tracking the motion reference
  5. WebSocket bridge → streams sim state from Redis to browser
  6. MuJoCo WASM in browser → visualizes what sim2sim.sh is doingi 