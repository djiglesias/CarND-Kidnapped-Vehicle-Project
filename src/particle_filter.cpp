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
  this->num_particles = 5;

  // Extract values for better understanding.
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  // Generate a normal distribution model for initializing.
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);

  // Populate filter with particles.
  for (int i = 0; i < this->num_particles; i++) {

    // Set weights.
    this->weights.push_back(1.0);

    // Create new particle.
    Particle particle;
    particle.id = i;
    particle.x = dist_x(this->gen);
    particle.y = dist_y(this->gen);
    particle.theta = dist_theta(this->gen);
    particle.weight = 1.0;

    // Add particle to list.
    this->particles.push_back(particle);

  }

  // Set initialization flag.
  this->is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

  // Catch initialization.
  if (!this->is_initialized) return;

  // Gaussian distribution generator.
  normal_distribution<double> dist_x(0.0, std_pos[0]);
  normal_distribution<double> dist_y(0.0, std_pos[1]);
  normal_distribution<double> dist_theta(0.0, std_pos[2]);

  // Function constants.
  double delta_d = velocity * delta_t;
  double d_theta = yaw_rate * delta_t;
  double A = velocity / yaw_rate;

  // Update particle states.
  for (int i = 0; i < this->num_particles; i++) {

    // Calculate change in states.
    double p_theta = this->particles[i].theta;

    if (fabs(yaw_rate) < 0.001) {   // Moving straight.
      this->particles[i].x += delta_d * cos(p_theta);
      this->particles[i].y += delta_d * sin(p_theta);
    } else {
      this->particles[i].x += A * (sin(p_theta + d_theta) - sin(p_theta));
      this->particles[i].y += A * (cos(p_theta) - cos(p_theta + d_theta));
      this->particles[i].theta += d_theta;
    }

    // Add measurement noise.
    this->particles[i].x += dist_x(this->gen);
    this->particles[i].y += dist_y(this->gen);
    this->particles[i].theta += dist_theta(this->gen);

  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

  // Determine nearest neighbour for observed measurements.
  for (unsigned int i = 0; i < observations.size(); i++) {

    // Set large distance.
    int id;
    double min_distance = 1.0e99;

    // Compare with all predicted measurements.
    for (unsigned int j = 0; j < predicted.size(); j++) {

      // Determine absolute distance to particle.
      double delta_x = observations[i].x - predicted[j].x;
      double delta_y = observations[i].y - predicted[j].y;
      double delta_d = delta_x*delta_x + delta_y*delta_y;

      // Check if this point is nearer than the previous.
      if (delta_d < min_distance) {
        min_distance = delta_d;
        id = j;
      }
    }

    // Update observation with nearest particle id.
    observations[i].id = id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
    const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

  // Define inputs.
  double sig_x = std_landmark[0];                 // std x
  double sig_y = std_landmark[1];                 // std y
  double sig_x_2 = sig_x*sig_x;
  double sig_y_2 = sig_y*sig_y;
  double A = 0.5 / sig_x_2;
  double B = 0.5 / sig_y_2;
  double C = 1.0 / (2.0 * M_PI * sig_x * sig_y); 
  double max_weight = 0.0;
  double sensor_range_2 = sensor_range*sensor_range;

  // Find landmarks in range of each particle.
  for (int i = 0; i < this->num_particles; i++) {

    // Extract particle values for better understanding.
    double x_p = this->particles[i].x;
    double y_p = this->particles[i].y;
    double theta_p = this->particles[i].theta;

    // Predict measurements to all the map landmarks in range of particle.
    std::vector<LandmarkObs> particles_in_range;
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {

      // Extract car observation values for better understanding.
      int id = map_landmarks.landmark_list[j].id_i;
      double x = map_landmarks.landmark_list[j].x_f;
      double y = map_landmarks.landmark_list[j].y_f;

      // Distance of landmark to particle.
      double delta_x = x - x_p;
      double delta_y = y - y_p;
      double delta_d = delta_x*delta_x + delta_y*delta_y;

      // Check if in range.
      if (delta_d <= sensor_range_2)
        particles_in_range.push_back(LandmarkObs{id,x,y});
    }

    // Transform from car coords to map coords.
    std::vector<LandmarkObs> transformed_observations;
    for (unsigned int j = 0; j < observations.size(); j++) {

      // Extract car observation values for better understanding.
      int id_c = observations[j].id;
      double x_c = observations[j].x;
      double y_c = observations[j].y;

      // Transform coordinates.
      double x_t = x_p + cos(theta_p) * x_c - sin(theta_p) * y_c;
      double y_t = y_p + sin(theta_p) * x_c + cos(theta_p) * y_c;
      transformed_observations.push_back(LandmarkObs{id_c, x_t, y_t});
    }

    // Associate observations to map landmarks.
    this->dataAssociation(particles_in_range, transformed_observations);

    // Calculate the new weight of each particle.
    double new_weight = 1.0;

    for (unsigned int j = 0; j < transformed_observations.size(); j++) {

      // Set parameters.
      int id = transformed_observations[j].id;        // nearest landmark id
      double x_obs = transformed_observations[j].x;   // observed x
      double y_obs = transformed_observations[j].y;   // observed y
      double mu_x = particles_in_range[id].x;         // mean x
      double mu_y = particles_in_range[id].y;         // mean y

      // Calculating & normalize weight.
      double dx = x_obs - mu_x;
      double dy = y_obs - mu_y;
      double dx_2 = dx*dx;
      double dy_2 = dy*dy;

      double weight = C * exp(-(A*dx_2 + B*dy_2));

      if (weight == 0.0) {
        new_weight *= 0.00001;
      } else {
        new_weight *= weight;
      }
    }

    if (new_weight > max_weight) {
      max_weight = new_weight;
    }

    // Update particle weight.
    this->particles[i].weight = new_weight;
    this->weights[i] = new_weight;

  }

  // Normalize weights and push to list.
  for (int i = 0; i < this->num_particles; i++) {
    this->particles[i].weight /= max_weight;
    this->weights[i] /= max_weight;
  }
}

void ParticleFilter::resample() {

  vector<Particle> resampled_particles;

  // Generate random sample.
  default_random_engine gen;
  discrete_distribution<int> index(this->weights.begin(), this->weights.end());

  for (int i = 0; i < this->num_particles; i++) {

    int j = index(this->gen);
    double x = this->particles[j].x;
    double y = this->particles[j].y;
    double theta = this->particles[j].theta;
    resampled_particles.push_back(Particle{j,x,y,theta,1.0});
  }

  this->particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
  
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;

  return particle;
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
