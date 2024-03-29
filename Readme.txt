How to use this package
    - To build the docker container and workspace within (usually first step)
        ./build_container.sh nvidia
    
    - To run the test from outside the container
        ./run_trial.sh ($1:trial name or "run-all", ($2:number for how many times to run the trial))
        