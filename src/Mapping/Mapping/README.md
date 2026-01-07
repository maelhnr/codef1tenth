# Occupancy Grid

This folder contains the code for an Occupancy Grid. This algorithm discretizes the world into a 2D map made of squared cells. Each cells stores its probability of being occupied (0 is free and 1 is occupied). Because we encode probabilities, the cell is never thought of as completely free or completely empty. To combat this the probabilities are cast to 0 or 1 when publishing to the map topic.

## How to launch
Like all of the algorithms, it can be launched using the same command and selecting the corresponding launch file.

    roslaunch autonomous_car map_og.launch
    
Don't forget about the previous command _source devel/setup.bash_

## Parameters 
This algorithm can be modified using the relevant yaml file, _Map_params.yaml_. In this file you can specify:
- Map_bounds : A tuple representing the bounds of the map [xmin, xmax, ymin, ymax]
- Map_resolution : A double representing the length of each side of the cells

- Map_Log_Probability_Clip : A double representing the maximum log-probability. For more informations see the documentation 

- Map_OG_Threshold_Occupied : A double representing the threshold probability between free and occupied
- Map_OG_Probability_Occupied : A double representing the probability that a cell recorded as occupied is indeed occupied
- Map_OG_Probability_Free : A double representing the probability that a cell recorded as free is indeed free
- Map_OG_Probability_Prior : A double representing the probability that a cell has with no measurement 

- Map_OG_throttle_scans : An int used to indicate the period of scans which are used. 1 is all, 2 is 1 out of 2, 3 is 1 out of 3, ...

## State of the algorithm
The algorithm works as is. It has been optimized to run faster but at the moment is only suitable to be run on a personnal computer. Being able to parralellize the computation using the GPU would be a massive boost.

We also tried to implement a change of reference frame of the measurement (from LIDAR to base_frame) because otherwise it cannot be used correctly with the odometry (which is expressed in base_frame). The code is commented at the moment, it should work but we did not have the time to test it.
