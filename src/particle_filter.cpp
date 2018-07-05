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

	if (is_initialized) {
		return;
	}

	// Set the number of particles
	num_particles = 100;

	// Standard deviation for x, y, theta
	double std_x = std[0];
 	double std_y = std[1];
 	double std_theta = std[2];

 	// Create normal distribution for x, y, theta
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	// Initialize all particles to first position (based on estimates of 
	// x, y, theta and their uncertainties from GPS) and all weights to 1.
	for (int i = 0; i < num_particles; ++i) {
		double sample_x, sample_y, sample_theta;
		
		// Sample from x, y, theta normal distrubtions
        sample_x = dist_x(gen);
        sample_y = dist_y(gen);
        sample_theta = dist_theta(gen);

        Particle p = {
        	i,
        	sample_x,
        	sample_y,
        	sample_theta,
        	1.0
        };
        particles.push_back(p);
	}

	// Filter is initialized
	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	// Standard deviation for x, y, theta
	double std_x = std_pos[0];
 	double std_y = std_pos[1];
 	double std_theta = std_pos[2];

 	// Create normal distribution for x, y, theta
	normal_distribution<double> dist_x(0, std_x);
	normal_distribution<double> dist_y(0, std_y);
	normal_distribution<double> dist_theta(0, std_theta);
	
	for (unsigned int i = 0; i < num_particles; i++) {
		double theta = particles[i].theta;

		if (fabs(yaw_rate) < 0.00001) {
			particles[i].x += velocity * delta_t * cos(theta);
      		particles[i].y += velocity * delta_t * sin(theta);
		} else {
			particles[i].x += velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
			particles[i].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
			particles[i].theta += yaw_rate * delta_t;
		}

		// Add noise
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);
	}


}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

	unsigned int num_observations = observations.size();
  	unsigned int num_predictions = predicted.size();

	for (unsigned int i=0; i < num_observations; i++) {

		// Initialize minimum distance as a really big number.
    	double min_distance = numeric_limits<double>::max();

    	observations[i].id = -1;
		
		for (unsigned int j=0; j < num_predictions; j++) {
			
			double distance = sqrt(pow((observations[i].x - predicted[j].x),2) + pow((observations[i].y - predicted[j].y),2));
			
			if (distance < min_distance) {
				min_distance = distance;
				observations[i].id = predicted[j].id;
			}
		}

	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

	// landmark standard deviation
	double std_landmark_x = std_landmark[0];
	double std_landmark_y = std_landmark[1];

	unsigned int num_landmarks = map_landmarks.landmark_list.size();
	unsigned int num_observations = observations.size();

	for (unsigned int i=0; i < num_particles; i++) {
		// Particle state x, y, theta
		double x = particles[i].x;
    	double y = particles[i].y;
    	double theta = particles[i].theta;
		
		// Find landmarks in the particle's sensor range
    	vector<LandmarkObs> particle_landmarks;
		for (unsigned int j=0; j < num_landmarks; j++) {
			float landmark_x = map_landmarks.landmark_list[j].x_f;
      		float landmark_y = map_landmarks.landmark_list[j].y_f;
      		int landmark_id = map_landmarks.landmark_list[j].id_i;

      		double distance = sqrt(pow((x - landmark_x),2) + pow((y - landmark_y),2));
      		if(distance < sensor_range) {
      			LandmarkObs landmark = {
      				landmark_id,
      				landmark_x,
      				landmark_y
      			};
      			particle_landmarks.push_back(landmark);
      		}
		}

		// TRANSFORM obeservations from vehicle's coordinates to map's coordinates
		vector<LandmarkObs> observations_m;
		for (unsigned int j=0; j < num_observations; j++) {
			double x_m = cos(theta) * observations[j].x - sin(theta) * observations[j].y + x;
      		double y_m = sin(theta) * observations[j].x + cos(theta) * observations[j].y + y;
      		int obs_id = observations[j].id;
      		LandmarkObs observation_m = {
      			obs_id,
      			x_m,
      			y_m
        	};
      		observations_m.push_back(observation_m);
		}

		// ASSOCIATE each transformed observation with a land mark identifier
    	dataAssociation(particle_landmarks, observations_m);

    	// UPDATE WEIGHTS Apply multivariate Gaussian probability dense funciton for each measurement
    	particles[i].weight = 1.0;
    	unsigned int num_observations_m = observations_m.size();
    	unsigned int num_particle_landmarks = particle_landmarks.size();
    	for (unsigned int j=0; j < num_observations_m; j++) {
    		// observation
    		double obs_m_x = observations_m[j].x;
      		double obs_m_y = observations_m[j].y;
      		int obs_m_landmark_id = observations_m[j].id;

      		for (unsigned int k=0; k < num_particle_landmarks; k++) {
      			// landmark
      			int landmark_id = particle_landmarks[k].id;
      			double landmark_x = particle_landmarks[k].x;
      			double landmark_y = particle_landmarks[k].y;

      			if(landmark_id == obs_m_landmark_id) {
      				double gauss_norm = 1 / (2 * M_PI * std_landmark_x * std_landmark_y);
      				double exponent = pow((obs_m_x - landmark_x),2)/(2*pow(std_landmark_x,2)) + pow((obs_m_y - landmark_y),2)/(2*pow(std_landmark_y,2));
      				double weight = gauss_norm * exp(-exponent);

      				if (weight == 0) {
        				particles[i].weight *= 0.00001;
      				} else {
        				particles[i].weight *= weight;
      				}
      				break;
      			}
      		}
    	}


	}
}

void ParticleFilter::resample() {
	// Find max weight
	double max_weight = numeric_limits<double>::min();
	for (unsigned int i = 0; i < num_particles; i++) {
		if (particles[i].weight > max_weight) {
			max_weight = particles[i].weight;
		}
	}

	// Create uniform distribution for index and 2 * max weight
	uniform_real_distribution<double> dist_beta(0.0, 2 * max_weight);
 	uniform_int_distribution<int> dist_index(0, num_particles - 1);

 	// Resampling Wheel
 	// Randomly select a particle index

 	unsigned index = dist_index(gen);
 	double beta = 0;
 	vector<Particle> resampled_particles;
 	for (unsigned int i = 0; i < num_particles; i++) {
 		
 		beta += dist_beta(gen);

 		while (particles[index].weight < beta) {
 			beta -= particles[index].weight;
 			index = (index + 1) % num_particles;
 		}

 		resampled_particles.push_back(particles[index]);
 	}

 	particles = resampled_particles;

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
