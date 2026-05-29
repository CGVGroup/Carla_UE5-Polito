The main goal of the CARLA Autonomous Driving Leaderboard is to evaluate the driving proficiency of autonomous agents in realistic traffic situations. The leaderboard serves as an open platform for the community to perform fair and reproducible evaluations, simplifying the comparison between different approaches.

Autonomous agents have to drive through a set of predefined routes. For each route, agents are initialized at a starting point and have to drive to a destination point. The agents will be provided with a description of the route. Routes will happen in a variety of areas, including freeways, urban scenes, and residential districts.

Agents will face multiple traffic situations based in the NHTSA typology, such as:

* Lane merging
* Lane changing
* Negotiations at traffic intersections
* Negotiations at roundabouts
* Handling traffic lights and traffic signs
* Coping with pedestrians, cyclists and other elements

The user can change the weather of the simulation, allowing the evaluation of the agent in a variety of weather conditions, including daylight scenes, sunset, rain, fog, and night, among others.

More information can be found [here](https://leaderboard.carla.org/)


# CARLA Leaderboard Setup & Execution Guide

This guide provides step-by-step instructions to set up the CARLA Leaderboard benchmark using Python 3.7 and run it with the required components.

## 1. Environment Setup

Create a Python 3.7 Conda environment:

```bash
conda create -n py37 python=3.7
conda activate py37
```

## 2. Install CARLA Python API Requirements

Navigate to your CARLA root folder and install the Python API dependencies:

```bash
cd ${CARLA_ROOT}  # Replace ${CARLA_ROOT} with the path to your CARLA root directory

pip3 install -r PythonAPI/carla/requirements.txt
```

## 3. Install Leaderboard Requirements

Navigate to the Leaderboard root folder and install the dependencies:

```bash
cd ${LEADERBOARD_ROOT}  # Replace ${LEADERBOARD_ROOT} with the path to your Leaderboard root directory

pip3 install -r requirements.txt
```

## 4. Install Scenario Runner Requirements

Navigate to the Scenario Runner root folder and install the dependencies:

```bash
cd ${SCENARIO_RUNNER_ROOT}  # Replace ${SCENARIO_RUNNER_ROOT} with the path to your Scenario Runner root directory

pip3 install -r requirements.txt
```

## 5. Set Environment Variables

Set the necessary environment variables before running CARLA or the Leaderboard:

```bash
export CARLA_ROOT=PATH_TO_CARLA_ROOT
export SCENARIO_RUNNER_ROOT=PATH_TO_SCENARIO_RUNNER
export LEADERBOARD_ROOT=PATH_TO_LEADERBOARD

export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":"${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg":${PYTHONPATH}
```

Make sure to replace `PATH_TO_CARLA_ROOT`, `PATH_TO_SCENARIO_RUNNER`, and `PATH_TO_LEADERBOARD` with the correct absolute paths.

## 6. Launch CARLA Simulator

In the CARLA root folder, launch the CARLA simulator with the following command:

```bash
cd ${CARLA_ROOT}
./CarlaUE4.sh -quality-level=Epic -world-port=2000 -resx=800 -resy=600
```

## 7. Run the Leaderboard

Once CARLA is running, execute the Leaderboard benchmark:

```bash
./run_leaderboard_xai.sh
```
