# ARIAC_testing
This Repository contains the source code for Ariac Testing for the ARIAC competition.

Install and use directions:

    - Clone the repository
        git clone https://github.com/xtan99/ARIAC_testing.git
    
    - Navigate to the testing folder
        cd ARIAC_testing/automated_test

    - To build the docker container
        ./build_container.sh nvidia
    
    - To run the test 
        ./run_test.sh ($1:test name or "run-all", $2:number of times to run the test)

        Test Names:
            1. test_pickparts (Test to pick bin parts)
            2. test_picktrays (Test to pick and place trays)
            3. test_assembly  (Test to perform assembly)
    
About the tests:

    - The results for each test run can be found in ARIAC_test/automated_test/logs/ in a folder 
      with the same name as the test, followed by a number based on the number of previously 
      existing folders for the same test
    
    - Each time the test is run, a new log file is generated, and the files are combined at the 
      end of all the test runs
    
    - Gazebo and MOVEit failures will also be logged

    1. Pick Parts from Bins:
        - This test aims to confirm that the Floor Robot is able to successfully pick all 4 types 
          of parts from the bins
        - The robot will attempt to pick all the detected parts once, first from the right bin 
          and then the left bins
        - This test shares a trial file along with the pick tray test named testing.yaml
        - The number and types of parts to be picked can be modified in the trial file
        - Implementation details:
            - The test is run based on the BinTest function in the ariac_test.cpp file
            - The run_test.py script will run the corresponding launch file based on the test name
            - Failure check: The robot will check if a part is attached to the end effector everytime 
              after attempting to pick a part
            - The log file will contain details regarding the type, color and bin side of the part 
              incase there is a failure
    
    2. Pick Trays and place on AGVs:
        - This test aims to confirm that the Floor Robot can pick 2 trays from each kitting stations 
          and place them on the 4 AGVs
        - The robot will attempt to pick all the detected trays once from each kitting station
        - This test shares a trial file along with the pick parts test named testing.yaml
        - Implementation details:
            - The test is run based on the TrayTest function in the ariac_test.cpp file
            - The run_test.py script will run the corresponding launch file based on the test name
            - Failure check: The robot will check if the tray is attached to the end effector everytime 
              after attempting to pick a tray
            - The log file will contain details regarding the tray-number and kit station side incase 
              there is a failure
    
    3. Assembly:
        - This test aims to confirm that Assembly is working as intended 
        - Parts will be spawned on AGV-1, and the competitor system will proceed with the assembly task
        - Implementation details:
            - The test is run based on the AssemblyTest function in the ariac_test.cpp file
            - The run_test.py script will run the corresponding launch file based on the test name
            - The assembly test has a separate trial file named assembly_test.yaml
            - It completes the assembly task based on the order
            - Failure check: The system will check if all the parts are attached to the assembly kit after 
              finishing assembly
            - The log file will contain details regarding the type of part that was not attached incase 
              there is a failure
            
            
