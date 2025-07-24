# g1_motion-visualization-by-Meshcat

## Notice!!!
this repo is only based on g1 robot's urdf,not consider the physical properties at all.
To get realer simulation,you'd better use mujoco 

## Depency

To use this repo,you'd better create conda env

```
conda create -n myenv python=3.8
conda activate myenv
pip install numpy transforms3d matplotlib pygame meshcat
conda install -c conda-forge pyyaml rospkg casadi
conda install pinocchio=3.1.0 -c conda-forge
```

## Usage

you need to prepare the motion data whose format as same as the motion_guzhang.seq
and then replace the code path in test_v1.py line 46
### example 

```
conda activate myenv
python test_v1.py
```
