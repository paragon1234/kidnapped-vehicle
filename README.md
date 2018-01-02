# Localization with Particle Filters - Finding a Kidnapped Vehicle

## Project Basics
A robot(car) has been kidnapped and stationed at an unknown location. It has to determine its position (with respect to an input map) with reasonable accuracy, using a C++ 2D particle filter. Luckily it has (input to particle filter):

* a map of this location, 
* a (noisy) GPS estimate of its initial location, and 
* lots of (noisy) sensor and control data, at each timestamp. 
* a list of landmarks in the location.

### Particle Filter Input
#### The Map
`map_data.txt` present in the `data` directory, includes the position of landmarks (in meters). Each row has three columns
1. x position
2. y position
3. landmark id

#### Control Data
control_data contains 
1. vehicle speed (in meters per second)
2. vehicle yaw rate (in radians per second)

#### Observation Data
At each timestep (as vehicle move), observation data for all landmarks that are sufficiently close to the vehicle. Each row in these files corresponds to a single landmark. The two columns represent:
1. x distance to the landmark in meters (right is positive) RELATIVE TO THE VEHICLE.
2. y distance to the landmark in meters (forward is positive) RELATIVE TO THE VEHICLE.

#### Initial GPS coordinates
These include:
1. x coordinate
2. y coordinate
3. angle (driving direction)

#### Landmark Positions
These include:
1. x position
2. y position
3. landmark id


## Project Design and Implementation
In this C++ project, a two-dimensional particle filter is used to help localize a car placed in an somewhat unknown location. Along with sufficient accuracy to have localized the vehicle within a small amount of space, the project also required having an efficient algorithm, as there was a time limit to how long it could run for without failing the given parameters.

### Project Steps
1. A less accurate map data (similar to GPS) is used to initialize the car's location, 
2. predict the new location based on velocity and yaw (turn) rate, 
3. transform sensor observation points into map coordinates, associated these observations with landmarks on the map, 
4. calculate the likelihood that a given particle made those observation based off of the landmark positions in the map. 
5. The particles were then re-sampled based on how likely a given particle was to have made the observations of the landmarks, which helps to more accurately localize the vehicle.

### Project Implementation
All steps within 'particle_filter.cpp' in the 'src' folder
* init function- estimate position from GPS data using particle filters, add random noise
* prediction function- predict position based on adding velocity and yaw rate to particle filters, add random noise
* updateWeights function step1- Transformation of observation points to map coordinates (given in vehicle coordinates)
* updateWeights function step2- Find map landmarks within the sensor range
* updateWeights function step3- Association of landmarks to the transformed observation points
        This step calls dataAssociation function
* updateWeights function step4- Calculation of multi-variate Gaussian distribution
* resample function - Resamples particles, with replacement occurring based on weighting distributions

## Running the Code
Once you have this repository on your machine, cd into the repository's root directory and run the following commands from the command line:
```
> ./clean.sh
> ./build.sh
> ./run.sh
```

If everything worked you should see something like the following output:

```
Time step: 2443
Cumulative mean weighted error: x .126 y .123 yaw .004
Runtime (sec): 49.04
Success! Your particle filter passed!
```
Please note that due to the random numbers generated in certain portions of my approach (for the Gaussian distributions), results may vary slightly.

## Results
### Performance/Accuracy
The particle filter met the requirements, which were 1 meter in error for x and y translations, 0.05 rad in error for yaw, and 45 seconds of runtime for the particle filter. 
Also, all the calculations were performed in run-time.



