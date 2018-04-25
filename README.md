# Kidnapped Vehicle Project

## 0. Overview

### 0.1 Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

This project implements a 2 dimensional particle filter in C++. The particle filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter gets an observation along with control data. 

### 0.2 Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

    $  mkdir build
    $  cd build
    $  cmake ..
    $  make
    $  ./particle_filter

## 1. Particle Filter Class

### 1.1 Initialize
The particle filter class uses a list of particles to track changes in position over time that have to be instantiated before the filter can begin. Using a large number of particles for the filter will result in a more accurate estimation of the world position however will ultimately slow down the process with large amounts of computational steps. Similarily, too few particles will result in faster computation time but less references to assist in localization. 

It is good practise to initialize all the particles with values from a random Gaussian sample based on the initalize position of the car and the standard deviation associated with the data value (x/y/theta). Now populate the list of particles in the particle filter with the number of desired number of particles.

    $  for (int i = 0; i < num_particles; i++) {
    $    // Create new particle.
    $    Particle particle;
    $    particle.id = i;
    $    particle.x = dist_x(gen);
    $    particle.y = dist_y(gen);
    $    particle.theta = dist_theta(gen);
    $    particle.weight = 1.0;
    $
    $    // Add particle to list.
    $    particles.push_back(particle);
    $
    $  }

In order to prevent the number of particles growing uncontrollabley and from instantiating multiple particles with the same id, only perform this step once and add a gaurd using `is_initialized` to catch such cases. 

### 1.2 Prediction
This step updates our belief of where all the particles are in the world coordinate frame by regenerating random Gaussian values for x, y, and theta based on the current velocity and yaw rate of the car. Using these estimates as the mean a Gaussian distribution is generated and a sample from each is set as the new particle postion estimate.

<p align="center">
 <img src="./res/prediction_formula.png" width="350">
</p>

### 1.3 Data Association
In order to use the observed particle measurements from the lidar on the car it is essential to know which measurement value most likely corresponds to which particle in the filter. To determine this the nearest neighbour for each measurement is determined so an observed measurement does not interfere with an unassociated particle.

<p align="center">
 <img src="./res/association.png" width="350">
</p>

### 1.4 Update Weights
After predicting the new particle positions and associating the measured data with their mostly likely matches, we need to update the particle weights using a multi-variate Gaussian distribution. 

#### 1.4.1 Transform Coordinates
For each particle find all possible lardmarks in range of the sensor and then transform the landmarks from car coordinates to map coordinates.

<p align="center">
 <img src="./res/transform_matrix.png" width="350">
</p>

#### 1.4.2 Associate Data
With all the observation transformed into the map coordinates we can now associated the measurements with the most likely particles using the function discussed in Section 1.3.

#### 1.4.3 Determine Measurement Probability
Find the associated particle and recalculate the probability using the equation provided below.

<p align="center">
 <img src="./res/particle_weight.png" width="350">
</p>

### 1.5 Resample
Resample particles with replacement using probability proportional to their weight...

## 2.0 Tuning & Optimization
- number of particles and how it affects time/accuracy
- optimize by generating gaussian noise outside loop
- reduce unnecessary calculations

## 3.0 Running the Simulator
[insert gif]



