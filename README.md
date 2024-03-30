# ARIAC_testing
This Repository contains the source code for the Ariac Testing for the ARIAC competition.


    - Clone the repository
        git clone https://github.com/xtan99/ARIAC_testing.git
    
    - Navigate to the testing folder
        cd ARIAC_testing/automated_test

    - To build the docker container
        ./build_container.sh nvidia
    
    - To run the test 
        ./run_test.sh ($1:test name or "run-all", $2:number of times to run the test)