#!/bin/bash

RUNS=100
ATTRACTION=(0)
REPULSION=(0 4 8 16 32 64)
LOCALPATH=$(pwd)/
for i in "${ATTRACTION[@]}"
do
  for j in "${REPULSION[@]}"
  do
    for (( c=1; c<=$RUNS; c++ ))
    do  
        if test -f "${LOCALPATH}results/results_status_${i}_${j}.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_status_${i}_${j}.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_status_${i}_${j}.yaml"
        fi
        if test -f "${LOCALPATH}results/results_randomChoice_${i}_${j}.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_randomChoice_${i}_${j}.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_randomChoice_${i}_${j}.yaml"
        fi
        if test -f "${LOCALPATH}results/results_timing_${i}_${j}.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_timing_${i}_${j}.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_timing_${i}_${j}.yaml"
        fi
        if test -f "${LOCALPATH}results/results_visitedCells_${i}_${j}.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_visitedCells_${i}_${j}.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_visitedCells_${i}_${j}.yaml"
        fi
        echo "Run #$c with $i ATTRACTION and $j REPULSION" 

        cp RWinput_param.yaml RWparam_found_script.yaml
        sed -i -e "s/_SEED_/$c/g" RWparam_found_script.yaml
        sed -i -e "s/_ATTRACTION_/$i/g" RWparam_found_script.yaml
        sed -i -e "s/_REPULSION_/$j/g" RWparam_found_script.yaml
        sed -Ei "s|_PATH_|$LOCALPATH|g" RWparam_found_script.yaml
        /home/cscarbone/release/build/MACPP -i RWparam_found_script.yaml
    done	
  done
done

: <<'END'
END
