#!/bin/bash

RUNS=50
LOCALPATH=$(pwd)/

    for (( c=1; c<=$RUNS; c++ ))
    do  
        if test -f "${LOCALPATH}results/results_status_IG.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_status_IG.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_status_IG.yaml"
        fi
        if test -f "${LOCALPATH}results/results_randomChoice_IG.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_randomChoice_IG.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_randomChoice_IG.yaml"
        fi
        if test -f "${LOCALPATH}results/results_timing_IG.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_timing_IG.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_timing_IG.yaml"
        fi
        if test -f "${LOCALPATH}results/results_visitedCells_IG.yaml"
        then
            echo "$c:" >> "${LOCALPATH}results/results_visitedCells_IG.yaml"
        else
            echo "$c:" > "${LOCALPATH}results/results_visitedCells_IG.yaml"
        fi
        echo "Run #$c "

        cp input_param.yaml param_found_script.yaml
        sed -i -e "s/_SEED_/$c/g" param_found_script.yaml
        sed -Ei "s|_PATH_|$LOCALPATH|g" param_found_script.yaml
        /home/cscarbone/release/build/MACPP -i param_found_script.yaml
    done	

: <<'END'
END
