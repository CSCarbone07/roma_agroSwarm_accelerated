#!/bin/bash

RUNS=100
NUM_OF_AGENTS=(50)
COMMUNICATIONS_RANGE=(-1)
LOCALPATH=$(pwd)/
for x in "${NUM_OF_AGENTS[@]}"
do
  for y in "${COMMUNICATIONS_RANGE[@]}"
  do
    for (( c=1; c<=$RUNS; c++ ))
    do  
        if test -f "${LOCALPATH}results/results_status_${x}_${y}_greedy_1_0.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_status_${x}_${y}_greedy_1_0.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_status_${x}_${y}_greedy_1_0.yaml"
        fi
        if test -f "${LOCALPATH}results/results_randomChoice_${x}_${y}_greedy_1_0.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_randomChoice_${x}_${y}_greedy_1_0.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_randomChoice_${x}_${y}_greedy_1_0.yaml"
        fi
        if test -f "${LOCALPATH}results/results_timing_${x}_${y}_greedy_1_0.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_timing_${x}_${y}_greedy_1_0.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_timing_${x}_${y}_greedy_1_0.yaml"
        fi
        if test -f "${LOCALPATH}results/results_visitedCells_${x}_${y}_greedy_1_0.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_visitedCells_${x}_${y}_greedy_1_0.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_visitedCells_${x}_${y}_greedy_1_0.yaml"
        fi
        echo "Run #$c with $i ATTRACTION and $j REPULSION" 

        cp RWinput_param_1_0_10.yaml IGinput_param_greedy_1_0.yaml
        sed -i -e "s/_SEED_/$c/g" IGinput_param_greedy_1_0.yaml
        sed -i -e "s/_NUM_OF_AGENTS_/$x/g" IGinput_param_greedy_1_0.yaml
        sed -i -e "s/_COMMUNICATIONS_RANGE_/$y/g" IGinput_param_greedy_1_0.yaml
        sed -Ei "s|_PATH_|$LOCALPATH|g" IGinput_param_greedy_1_0.yaml
        /home/rococoadmin/CarlosCarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build/MACPP -i IGinput_param_greedy_1_0.yaml
    done	
  done
done

: <<'END'
END
