# Distributed Consensus based Merging and Spacing

This repository contains an implementation of a research prototype system to enable several UAS vehicles approaching an intersection to coordinate merging and spacing to ensure safe passage through the intersection. Details of this research can be found in this [jupyter notebook](). This implementation combines a scheduling algorithm and a distributed consensus (RAFT) algorithm to enable merging and spacing.

Clone the repository and update submodules to get all dependent repositories.

Requires the [LCM library](https://lcm-proj.github.io/) to enable interprocess communication.

### Launching vehicles

The `SetupAgent.py` script initializes a UAS vehicle with the `UAVAgent` class. Currently, only simplethe kinematic motion of the vehicle is modeled. This script also initializes a RAFT communication server with the help of the LCM library to send/receive data from other vehicles. `SetupAgent.py` takes the following inputs as arguments:

```
python SetupAgent.py <ID> <x> <y> <speed> <log>
```

ID is an integer. x,y represent the position. Speed of the vehicle is in m/s. log (True/False) enables data logging for analysis. Running this script instantiates a single vehicle at the specified position and speed moving towards an intersection located at (0,0). The position of the intersection itself can be changed within the script. Launch several vehicles (as separate processes) using this script to simulate multiple vehicles approaching the intersection. The vehicles can communicate with each other and share information using the LCM framework. Consequently, they exchange the necessary information to achieve consensus on the order in which the vehicles must pass through the intersection. Note that it is sufficient to enable logging in only one vehicle.

### Analying results

The `SetupAgent.py` script stores the necessary data to analyze the results for all the vehicles in separate `.csv` files. These files are accessed by the `AnalyzeData.py` script to plot outputs and generate animations of the final results. 

```
python AnalyzeData.py <num vehicles in simulation>
```
