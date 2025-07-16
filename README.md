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

5. Install rest of the JS depenecy and launch the dev-server

```bash
bun install  # or npm install
npx live-server .
```