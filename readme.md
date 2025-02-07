# Task Allocation without Direct Communication: Graphical Game-based Swarm Interception Allocation

<!-- [![Rfly](https://rfly.buaa.edu.cn/img/logo-banner.af7a6ec6.png)](https://rfly.buaa.edu.cn/) -->
<a href="https://rfly.buaa.edu.cn/">
 <img src="https://rfly.buaa.edu.cn/img/logo-banner.af7a6ec6.png" width = "300" alt="Reliable Flight Control Group" />
</a>

## I. Environment
* [`ROS Melodic`](http://wiki.ros.org/melodic)
* [`RflySim`](https://rflysim.com/doc/en/) simulation environment
* `Python` environment (recommended: Anaconda) with [`pymavlink`](https://pypi.org/project/pymavlink/) installed

## II. Simulation
### 1. Launch the `RflySim` Scene
Double-click `RflySim\GTASITL.bat`

### 2. Launch the Allocation Algorithm
```
python main.py
```
After the allocation results are calculated, statistics will be printed in the terminal, and a diagram will be displayed.

Close the diagram, and the corresponding aircraft and direct relationships will be created in `RflySim`. Drag to an appropriate viewpoint, `Ctrl + scroll to zoom in` on the aircraft, press `S` to display the numbers, and you will achieve the effect shown in the paper.

Press any key in the terminal to start the movement effect.

## III. Real Flight Experiment
Compile the `RealFlight` workspace, and run with one command in the terminal:

```
./RealFlight/_scripts/zzfly_startup.sh
```