## Particle Filter Kidnapped Vehicle Project

The goal of this project if to implement a particle filter to solve the kidnapped vehicle problem.

[//]: # (Image References)
[image1]: ./images/particle_filter.png
[image2]: ./images/pseudocode.png
[image3]: ./images/prediction_eq.png
[image4]: ./images/homogenous.png
[image5]: ./images/multivariate_gauss.png
[image6]: ./images/wheel.png
[image7]: ./images/result.png

![alt text][image7]

### Setup
* Download Simulator [here](https://github.com/udacity/self-driving-car-sim/releases)
* Install uWebSocketIO [here](https://github.com/uWebSockets/uWebSockets)

### Basic Build Instructions
Run following commands in terminal to streamline the build and run process:

```
> ./clean.sh
> ./build.sh
> ./run.sh
```

Run from this project's directory.

### Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The file `particle_filter.cpp` in the `src` directory contains the implememntation of a `ParticleFilter` class and some associated methods.

#### Particle Filter Algorithm Steps and Inputs

![alt text][image1]

#### Psuedo Code

![alt text][image2]

#### 1. Initialization
The most practical way to initialize our particles and generate real time output, is to make an initial estimate using GPS input. I chose to sample 100 particles around the GPS measurements. 

As with all sensor based operations, this step is impacted by noise. I took into account Gaussian sensor noise around the initial GPS position estimate and the intial heading estimate.

I did this in lines 22 to 64 of the function `init()` in `particle_filter.cpp`.

#### 2. Prediction
In this step, I predicted where the car will be at the next time step by updating each particle's location based on velocity and yaw rate measurements. To account for the uncertainty in the control input, I also added Gaussian noise to the velocity and yaw rate.

Here are the equations for updating x, y, and the yaw angle when the yaw rate is not equal to zero:

![alt text][image3]

I did this in lines 66 to 97 of the function `prediction()` in `particle_filter.cpp`.


#### 3. Update
The goal is to find a weight parameter for each particle that represents how well the particle fits to being the same location as the actual car.

##### Landmark within Sensor Range
I first found all landmarks within the sensor range. I did this in lines 137 to 157 of `updateWeights()` in `particle_filter.cpp`.

##### Transform
I then transformed the car's measurements from its local car coordinate system to the map's coordinate sysem.

Observations in the car coordinate system can be transformed into map coordinates (x_m and y_m) by passing car observation coordinates (x_c and y_c), map particle coordinates (x_p and y_p), and our rotation angle (-90 degrees) through a homogenous transformation matrix. This homogenous transformation matrix, shown below, performs rotation and translation.

![alt text][image4]

I did this in lines 160 to 171 of the function `updateWeights()` in `particle_filter.cpp`.

##### Association
Next, each measurement needs to be associate with a landmark identifies, for this I took the closest landmark to each transformed observation.

I used Nearest Neighbor to match landmark measurements to objects in the real world, which I simply took the the closest measurement as the correct correspondents.

I did this in lines 99 to 123 of the function `dataAssociation()` in`particle_filter.cpp`.

#### 4. Update Weights
Finally, I used information obtained from previous steps to calculate the weight value of the particle. The particle's final weight is the product of each measurement's Multivariate-Gaussian probability density:

![alt text][image5]

Where the mean of the Multivariate-Gaussian is the measurment's associated landmark position and the Multivariate-Gaussian's standard deviation is described by our initial uncertainty in the x and y ranges. The Multivariate-Gaussian is evaluated at the point of the transformed measurement's position.

I did this in lines 177 to 205 of the function`updateWeights()` in `particle_filter.cpp`.

#### 5. Resampling
In this step, I randomly drew new particles from old ones with replacement in porportion to their importance weights. After resampling, particles with higher weights are likely to stay and particles with lower weights may die out.

I used Resampling Wheel technique to achieve this:

![alt text][image6]

Here's the pseudo code for resampling:
```
p3 = []
index = int(random.random()*N)
beta = 0.0
mw = max(w)
for i in range(N):
  beta += random.random()*2*mw
  while w[index] < beta:
     beta = beta - w[index]
     index = index + 1

  p3.append(p[index])
 ```
I did this in lines 211 to 244 of the function`resample()` in `particle_filter.cpp`.

### Results

**Accuracy**: my particle filter localizes vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

**Performance**: my particle filter completes execution within the time of 100 seconds.

