#!/bin/bash

RUNS=50
NUM_OF_AGENTS=(50)
COMMUNICATIONS_RANGE=(-1)
LOCALPATH=$(pwd)/
for x in "${NUM_OF_AGENTS[@]}"
do
  for y in "${COMMUNICATIONS_RANGE[@]}"
  do
    for (( c=1; c<=$RUNS; c++ ))
    do  
        if test -f "${LOCALPATH}results/results_status_greedy_${x}_${y}_1_1.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_status_greedy_${x}_${y}_1_1.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_status_greedy_${x}_${y}_1_1.yaml"
        fi
        if test -f "${LOCALPATH}results/results_randomChoice_greedy_${x}_${y}_1_1.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_randomChoice_greedy_${x}_${y}_1_1.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_randomChoice_greedy_${x}_${y}_1_1.yaml"
        fi
        if test -f "${LOCALPATH}results/results_timing_greedy_${x}_${y}_1_1.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_timing_greedy_${x}_${y}_1_1.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_timing_greedy_${x}_${y}_1_1.yaml"
        fi
        if test -f "${LOCALPATH}results/results_visitedCells_greedy_${x}_${y}_1_1.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_visitedCells_greedy_${x}_${y}_1_1.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_visitedCells_greedy_${x}_${y}_1_1.yaml"
        fi
        echo "Run #$c" 

        cp IGinput_param_greedy_1_1.yaml IGparam_found_script_greedy_1_1.yaml
        sed -i -e "s/_SEED_/$c/g" IGparam_found_script_greedy_1_1.yaml
        sed -i -e "s/_NUM_OF_AGENTS_/$x/g" IGparam_found_script_greedy_1_1.yaml
        sed -i -e "s/_COMMUNICATIONS_RANGE_/$y/g" IGparam_found_script_greedy_1_1.yaml
        sed -Ei "s|_PATH_|$LOCALPATH|g" IGparam_found_script_greedy_1_1.yaml
        /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build/MACPP -i IGparam_found_script_greedy_1_1.yaml
    done	
  done
done

: <<'END'
END
