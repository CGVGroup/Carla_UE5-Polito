#!/bin/bash
export CARLA_ROOT=/home/tda/Desktop/CARLA_0.9.15
export PYTHONPATH=${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/
export LEADERBOARD_ROOT=$(pwd)
export TEAM_AGENT=$LEADERBOARD_ROOT/leaderboard/autoagents/npc_agent.py
export SCENARIO_RUNNER_ROOT=/home/tda/Desktop/phd/code/scenario_runner
export PYTHONPATH=$PYTHONPATH:${SCENARIO_RUNNER_ROOT}:${LEADERBOARD_ROOT}
export ROUTES=$LEADERBOARD_ROOT/data/routes_Town15_long3.xml
export ROUTES_SUBSET=0
export REPETITIONS=1
echo $PYTHONPATH

export DEBUG_CHALLENGE=1
export CHALLENGE_TRACK_CODENAME=SENSORS
export CHECKPOINT_ENDPOINT="${LEADERBOARD_ROOT}/results.json"
export RECORD_PATH=
export RESUME=

#!/bin/bash

python3 ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator.py \
--routes=${ROUTES} \
--routes-subset=${ROUTES_SUBSET} \
--repetitions=${REPETITIONS} \
--track=${CHALLENGE_TRACK_CODENAME} \
--checkpoint=${CHECKPOINT_ENDPOINT} \
--debug-checkpoint=${DEBUG_CHECKPOINT_ENDPOINT} \
--agent=${TEAM_AGENT} \
--agent-config=${TEAM_CONFIG} \
--debug=${DEBUG_CHALLENGE} \
--record=${RECORD_PATH} \
# --host=158.109.9.49 \
# --resume=${RESUME}