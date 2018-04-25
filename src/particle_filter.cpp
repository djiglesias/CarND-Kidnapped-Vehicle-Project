/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {

  // Catch redundant initialization.
  if (is_initialized) return;

  // Set number of particles.
  num_particles = 100;

  // Extract values for better understanding.
  default_random_engine gen;
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  // Generate a normal distribution model for initializing.
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);

  // Populate filter with particles.
  for (int i = 0; i < num_particles; i++) {

    // Set weights.
    weights.push_back(1.0);

    // Create new particle.
    Particle particle;
    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1.0;

    // Add particle to list.
    particles.push_back(particle);

  }

  // Set initialization flag.
  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

  // Catch initialization.
  if (!is_initialized) return;

  // Gaussian distribution generator.
  default_random_engine gen;

  // Update particle states.
  for (int i = 0; i < num_particles; i++) {

    // Update estimated position.
    double p_x = particles[i].x;
    double p_y = particles[i].y;
    double p_theta = particles[i].theta;

    double x, y, theta;
    x = p_x + velocity / yaw_rate * (sin(p_theta + yaw_rate * delta_t) - sin(theta));
    y = p_y + velocity / yaw_rate * (cos(theta) - cos(p_theta + yaw_rate * delta_t));
    theta = p_theta + yaw_rate * delta_t;

    // Generate a normal distribution model for initializing.
    normal_distribution<double> dist_x(x, std_pos[0]);          // std x
    normal_distribution<double> dist_y(y, std_pos[1]);          // std y
    normal_distribution<double> dist_theta(theta, std_pos[2]);  // std theta

    // Set new particle position.
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);

  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

  // Determine nearest neighbour for observed measurements.
  for (int i = 0; i < observations.size(); i++) {

    // Set large distance.
    int id = -1;
    double min_distance = 10000.0;

    // Compare with all predicted measurements.
    for (int j = 0; j < predicted.size(); j++) {

      // Determine absolute distance to particle.
      double delta_x = observations[i].x - predicted[j].x;
      double delta_y = observations[i].y - predicted[j].y;
      double delta_d = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

      // Check if this point is nearer than the previous.
      if (delta_d < min_distance) {
        min_distance = delta_d;
        id = predicted[j].id;
      }
    }

    // Update observation with nearest particle id.
    observations[i].id = id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
    const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

  // Loop through each particle.
  for (int i = 0; i < num_particles; i++) {

    // Extract particle values for better understanding.
    double x_p = particles[i].x;
    double y_p = particles[i].y;
    double theta_p = particles[i].theta;

    // Predict measurements to all the map landmarks in range of particle.
    std::vector<LandmarkObs> particles_in_range;
    for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {

      // Extract car observation values for better understanding.
      int id = map_landmarks.landmark_list[j].id_i;
      double x = map_landmarks.landmark_list[j].x_f;
      double y = map_landmarks.landmark_list[j].y_f;

      // Distance of landmark to particle.
      double delta_x = x - x_p;
      double delta_y = y - y_p;
      double delta_d = sqrt(pow(delta_x, 2) + pow(delta_y,2));

      // Check if in range.
      if (delta_d <= sensor_range)
        particles_in_range.push_back(LandmarkObs{id,x,y});

    }

    // Transform from car coords to map coords.
    std::vector<LandmarkObs> transformed_observations;
    for (int j = 0; j < observations.size(); j++) {

      // Extract car observation values for better understanding.
      int id_c = observations[j].id;
      double x_c = observations[j].x;
      double y_c = observations[j].y;

      // Transform coordinates.
      double x_t = x_p + (cos(theta_p) * x_c) - (sin(theta_p * y_c));
      double y_t = y_p + (sin(theta_p) * x_c) + (cos(theta_p * y_c));
      transformed_observations.push_back(LandmarkObs{id_c, x_t, y_t});
    }

    // Associate observations to map landmarks.
    dataAssociation(particles_in_range, transformed_observations);

    // Calculate the new weight of each particle.
    particles[i].weight = 1.0;
    for (int j = 0; j < transformed_observations.size(); j++) {

      // Define inputs.
      double sig_x = std_landmark[0];
      double sig_y = std_landmark[1];
      double x_obs = transformed_observations[j].x;
      double y_obs = transformed_observations[j].y;
      double mu_x = particles[i].x;
      double mu_y = particles[i].y;
      int id = transformed_observations[j].id;

      // Search vector for id.
      double x_p, y_p;
      int k = 0;
      bool search = true;

      while (search && k < particles_in_range.size()) {
        // Check particle id.
        if (particles_in_range[k].id == id) {
          search = false;
          x_p = particles_in_range[k].x;
          y_p = particles_in_range[k].y;
        }
        k++;
      }

      // Calculating & normalize weight.
      double dx = x_obs - x_p;
      double dy = y_obs - y_p;
      double dx_2 = dx*dx;
      double dy_2 = dy*dy;
      double sig_x_2 = sig_x*sig_x;
      double sig_y_2 = sig_y*sig_y;

      double weight = (1.0/(2.0*M_PI*sig_x*sig_y)) * exp(-(dx_2/(2.0*sig_x_2) + (dy_2/(2.0*sig_y_2))));
      if (weight == 0) {
        particles[i].weight *= 0.00001;
      } else {
        particles[i].weight *= weight;
      }
    }
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution



}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseX(Particle best)
{
  vector<double> v = best.sense_x;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseY(Particle best)
{
  vector<double> v = best.sense_y;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
