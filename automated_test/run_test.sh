#!/bin/bash

#---------------------------------------------------------
# Example usage:./run_test.sh test_name
#---------------------------------------------------------

# Create a folder to copy log files from docker
if [ ! -d /$PWD/logs/$teamName ]; then
  mkdir -p /$PWD/logs/$teamName/;
fi

function combine_logs() {
    local dirname="$1"
    local j=1
    local arr=()

    for file in $dirname/*
    do
        log_file="${file##*/}"
        echo "Run $j" >> $dirname/combined_logs.txt
        
        while read -r LINE
        do
            echo "$LINE" >> $dirname/combined_logs.txt
        done < "$file"
        echo "---" >> $dirname/combined_logs.txt
        echo " " >> $dirname/combined_logs.txt

        arr[$j]="$file"

        let j++
    done

    # echo ${arr[@]}

    for value in "${arr[@]}"; 
    do
        rm $value
    done

}

function run_test() {
    local teamname="ariac_test"
    local testname="$1"
    local iterations=$2
    local j=1
    local k=1
    
    if [ ! -d /$PWD/logs/$teamname/$testname\_$j ]; then
        mkdir -p /$PWD/logs/$teamname/$testname\_$j/;
    else
        while [ -d /$PWD/logs/$teamname/$testname\_$j ]; do
            let j++  
        done
        mkdir -p /$PWD/logs/$teamname/$testname\_$j/;
    fi

    for ((i=1;i<=iterations;i++)); 
    do
        docker exec -it $teamname bash -c ". /container_scripts/run_test.sh $testname"
        
        echo "==== Copying logs to"

        docker cp $teamname:/tmp/test_log.txt $PWD/logs/$teamname/$testname\_$j/test_log$i.txt;
    done

    local dir_name=/$PWD/logs/$teamname/$testname\_$j

    combine_logs $dir_name
}


if [[ "$1" != "run-all" ]] ; then
    if [[ ! $2 ]] ; then
        echo "==== Running test $1 1 time"
        run_test $1 1
    else
        echo "==== Running test $1 $2 times"
        run_test $1 $2
    fi

else
    if [[ ! $2 ]] ; then
        iterations=1
    else
        iterations=$2
    fi
    
    echo "==== Running all tests from the tests directory ${iterations} times"
    # absolute path of the current script
    tests_dir="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )" # https://stackoverflow.com/a/4774063/99379
    # get each file in the tests folder
    for entry in "$tests_dir"/tests/*
    do
        # e.g., test1.launch.py
        test_file="${entry##*/}"
        # e.g., test1
        test_name=${test_file::-10}

        run_test $test_name $iterations

    done
fi
