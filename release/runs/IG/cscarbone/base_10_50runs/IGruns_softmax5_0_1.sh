#!/bin/bash

RUNS=50
NUM_OF_AGENTS=(50)
COMMUNICATIONS_RANGE=(10)
LOCALPATH=$(pwd)/
for x in "${NUM_OF_AGENTS[@]}"
do
  for y in "${COMMUNICATIONS_RANGE[@]}"
  do
    for (( c=1; c<=$RUNS; c++ ))
    do  
        if test -f "${LOCALPATH}results/results_status_softmax5_${x}_${y}_0_1.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_status_softmax5_${x}_${y}_0_1.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_status_softmax5_${x}_${y}_0_1.yaml"
        fi
        if test -f "${LOCALPATH}results/results_randomChoice_softmax5_${x}_${y}_0_1.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_randomChoice_softmax5_${x}_${y}_0_1.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_randomChoice_softmax5_${x}_${y}_0_1.yaml"
        fi
        if test -f "${LOCALPATH}results/results_timing_softmax5_${x}_${y}_0_1.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_timing_softmax5_${x}_${y}_0_1.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_timing_softmax5_${x}_${y}_0_1.yaml"
        fi
        if test -f "${LOCALPATH}results/results_visitedCells_softmax5_${x}_${y}_0_1.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_visitedCells_softmax5_${x}_${y}_0_1.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_visitedCells_softmax5_${x}_${y}_0_1.yaml"
        fi
        echo "Run #$c" 

        cp IGinput_param_softmax5_0_1.yaml IGparam_found_script_softmax5_0_1.yaml
        sed -i -e "s/_SEED_/$c/g" IGparam_found_script_softmax5_0_1.yaml
        sed -i -e "s/_NUM_OF_AGENTS_/$x/g" IGparam_found_script_softmax5_0_1.yaml
        sed -i -e "s/_COMMUNICATIONS_RANGE_/$y/g" IGparam_found_script_softmax5_0_1.yaml
        sed -Ei "s|_PATH_|$LOCALPATH|g" IGparam_found_script_softmax5_0_1.yaml
        /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build/MACPP -i IGparam_found_script_softmax5_0_1.yaml
    done	
  done
done

: <<'END'
END
