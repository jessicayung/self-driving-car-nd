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
#include <math.h> /* for sin, cos */

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	cout << "ParticleFilter::init" << endl;

	// TODO: tweak number of particles
	num_particles = 10;
	default_random_engine gen;
	 
	// Create normal (Gaussian) distributions for x, y, theta
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	
	for (int i = 0; i < num_particles; ++i) {
		double particle_x, particle_y, particle_theta;
				
	 	particle_x = dist_x(gen);
	 	particle_y = dist_y(gen);
	 	particle_theta = dist_theta(gen);	 
		
		// Assemble particle
		// i is index, 1.0 is weight.
		// See struct of Particle in `particle_filter.h` for full key.
		Particle temp = {i, particle_x, particle_y, particle_theta, 1.0};
        
        // Add particle to list of particles
        particles.push_back(temp);
        
        // Add weight to list of weights (see class `ParticleFilter`)
        weights.push_back(1.0f);

	}

	cout << "Number of Weights: " << weights.size() << endl;

	// Function does not return anything
	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	// Noise Gaussian: Mean: updated particle estimate. Std: Measurement noise.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Set up Gaussian noise, as in `ParticleFilter::init`
	default_random_engine gen;
	normal_distribution<double> dist_x(0.0, std_pos[0]);
	normal_distribution<double> dist_y(0.0, std_pos[1]);
	normal_distribution<double> dist_theta(0.0, std_pos[2]);

	for (int i = 0; i < num_particles; i++) {
		
		double x_est, y_est, theta_est;		

		// pointer to each particle
		Particle& particle = particles[i];
		// Extract values from particle for easier reading
		double x = particle.x;
		double y = particle.y;
		double theta = particle.theta; 

		// Calculate estimates of x, y, theta
		theta_est = theta + yaw_rate * delta_t;

		if (yaw_rate > 0.0001) {
			x_est = x + velocity/yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
			y_est = y + velocity/yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
		}
		else {
			x_est += velocity * delta_t * cos(theta);
			y_est += velocity * delta_t * sin(theta);
		}

		// Update each particle's estimate.
		particle.x = x_est + dist_x(gen);
		particle.y = y_est + dist_y(gen);
		particle.theta = theta_est + dist_theta(gen);
	}


}

vector<LandmarkObs> ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	// first arg predicted: Predicted measurements between one particle and all landmarks within sensor range 
	// second arg: observations: actual measurements from radar
	// Fn does NN association

	// initialise associations vector
	vector<LandmarkObs> associations;

	// for each observed measurement
	for (int i=0; i < observations.size(); i++) {

		// make code easier to read
		LandmarkObs obs = observations[i];

		// find predicted measurement closest to observation

		// initialise minimum distance prediction (pred closest to obs)
		LandmarkObs min = predicted[0];
		double min_distance_squared = pow(min.x - obs.x, 2) + pow(min.y - obs.y, 2);
		
		// for each prediction
		for (int j=0; j < predicted.size(); j++) {
			// calculate distance between predicted measurement and obs
			double distance_squared = pow(predicted[j].x - obs.x, 2) + pow(predicted[j].y - obs.y, 2);
			if (distance_squared < min_distance_squared) {
				min = predicted[j];
			}

		}

		// assign said predicted measurement to landmark
		associations.push_back(min);
		

	}

	return associations;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html

	// `observations` is an array of LandmarkObs, each el of which
	// has id, x and y properties (see helper_functions.h)

	// make code easier to read. used to calculate new weights.
	double covariance = pow(std_landmark[0], 2);
	double weights_sum = 0;

	for (int i=0; i < num_particles; i++) {
		// predict measurements to all map landmarks
		Particle& particle = particles[i];
		vector<LandmarkObs> predicted;
		
		for (int j=0; j < observations.size(); j++) {
			LandmarkObs landmark_pred;
			LandmarkObs obs = observations[j];
			landmark_pred.id = obs.id;
			// predict landmark x, y. Equations from trigonometry.
			landmark_pred.x = obs.x * cos(particle.theta) - obs.y * sin(particle.theta) + particle.x;
			landmark_pred.y = obs.x * sin(particle.theta) + obs.y * cos(particle.theta) + particle.y;
			predicted.push_back(landmark_pred);
		}

		// use dataAssociation function to associate sensor measurements to map landmarks 
		vector<LandmarkObs> associations = dataAssociation(predicted, observations);		

		// then calculate new weight of each particle using multi-variate Gaussian (& associations)
		// equation in L14.11 Update Step video

		// initialise unnormalised weight for particle
		// weight is a product so init to 1.0
		double weight = 1.0;

		// weight is a product so use a loop
		for (int j = 0; j < observations.size(); j++) {
			double x_minus_mu = pow(predicted[j].x - associations[j].x, 2) + pow(predicted[j].y - associations[j].y, 2);
			double distance = sqrt(x_minus_mu);
			// TODO: check this + it's inefficient, init at start of fn
			double covar_x_minus_mu = covariance;
			double num = exp(-0.5*x_minus_mu/covar_x_minus_mu);
			double denom = sqrt(abs(2*M_PI*covariance));
			// mutliply particle weight by this obs-weight pair stat
			weight *= num/denom;

		}

		// update particle weight 
		particle.weight = weight;
		// update weight in PF array
		weights[i] = weight;
		// add weight to weights_sum for normalising weights later
		weights_sum += weight;

	}

	// then normalise weights
	for (int i = 0; i < num_particles; i++) {
		// update particle weight
		particles[i].weight /= weights_sum;
		// update weight in PF array
		weights[i] /= weights_sum;
	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	default_random_engine gen;

	// TODO: check
	// Take a discrete distribution with pmf equal to weights
    discrete_distribution<> dd(weights.begin(), weights.end());
    // initialise new particle array
    vector<Particle> newParticles;
    // resample particles
    for (int i = 0; i < num_particles; ++i)
        newParticles.push_back(particles[dd(gen)]);

    particles = newParticles;

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
