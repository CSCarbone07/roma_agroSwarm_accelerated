#!/bin/bash

RUNS=100
NUM_OF_AGENTS=(50)
COMMUNICATIONS_RANGE=(-1)
ATTRACTION=(0 4 8 16 32 64)
REPULSION=(0)
LOCALPATH=$(pwd)/
for x in "${NUM_OF_AGENTS[@]}"
do
  for y in "${COMMUNICATIONS_RANGE[@]}"
  do
    for i in "${ATTRACTION[@]}"
    do
      for j in "${REPULSION[@]}"
      do
        for (( c=1; c<=$RUNS; c++ ))
        do  
            if test -f "${LOCALPATH}results/results_status_${x}_${y}_${i}_${j}.yaml"
            then
                echo "$c:" >> "${LOCALPATH}results/results_status_${x}_${y}_${i}_${j}.yaml"
            else
                echo "$c:" > "${LOCALPATH}results/results_status_${x}_${y}_${i}_${j}.yaml"
            fi
            if test -f "${LOCALPATH}results/results_randomChoice_${x}_${y}_${i}_${j}.yaml"
            then
                echo "$c:" >> "${LOCALPATH}results/results_randomChoice_${x}_${y}_${i}_${j}.yaml"
            else
                echo "$c:" > "${LOCALPATH}results/results_randomChoice_${x}_${y}_${i}_${j}.yaml"
            fi
            if test -f "${LOCALPATH}results/results_timing_${x}_${y}_${i}_${j}.yaml"
            then
                echo "$c:" >> "${LOCALPATH}results/results_timing_${x}_${y}_${i}_${j}.yaml"
            else
                echo "$c:" > "${LOCALPATH}results/results_timing_${x}_${y}_${i}_${j}.yaml"
            fi
            if test -f "${LOCALPATH}results/results_visitedCells_${x}_${y}_${i}_${j}.yaml"
            then
                echo "$c:" >> "${LOCALPATH}results/results_visitedCells_${x}_${y}_${i}_${j}.yaml"
            else
                echo "$c:" > "${LOCALPATH}results/results_visitedCells_${x}_${y}_${i}_${j}.yaml"
            fi
            echo "Run #$c with $i ATTRACTION and $j REPULSION" 

            cp RWinput_param_0_0.yaml RWparam_found_script_0_0.yaml
            sed -i -e "s/_SEED_/$c/g" RWparam_found_script_0_0.yaml
            sed -i -e "s/_NUM_OF_AGENTS_/$x/g" RWparam_found_script_0_0.yaml
            sed -i -e "s/_COMMUNICATIONS_RANGE_/$y/g" RWparam_found_script_0_0.yaml
            sed -i -e "s/_ATTRACTION_/$i/g" RWparam_found_script_0_0.yaml
            sed -i -e "s/_REPULSION_/$j/g" RWparam_found_script_0_0.yaml
            sed -Ei "s|_PATH_|$LOCALPATH|g" RWparam_found_script_0_0.yaml
            /home/rococoadmin/CarlosCarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build/MACPP -i RWparam_found_script_0_0.yaml
        done
      done
    done	
  done
done

: <<'END'
END
