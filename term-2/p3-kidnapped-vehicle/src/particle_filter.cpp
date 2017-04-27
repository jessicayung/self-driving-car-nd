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
	
	// TODO: tweak number of particles
	num_particles = 10;
	// TODO: necessary?
	weights.resize(num_particles);
	default_random_engine gen;
	 
	// Create normal (Gaussian) distributions for x, y, theta
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	
	for (int i = 0; i < num_particles; i++) {
		
		Particle new_particle;
		new_particle.id = i;
		new_particle.x = dist_x(gen);
		new_particle.y = dist_y(gen);
		new_particle.theta = dist_theta(gen);
		new_particle.weight = 1.0;
        
        // Add particle to list of particles
        particles.push_back(new_particle);
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
	normal_distribution<double> noise_x(0.0, std_pos[0]);
	normal_distribution<double> noise_y(0.0, std_pos[1]);
	normal_distribution<double> noise_theta(0.0, std_pos[2]);

	if (fabs(yaw_rate) < 0.0001) {
		yaw_rate = 0.0001;
	}

	for (auto&& particle : particles){
    
        particle.x += (velocity / yaw_rate) * (sin(particle.theta + yaw_rate * delta_t) - sin(particle.theta)) + noise_x(gen);
        particle.y += (velocity / yaw_rate) * (cos(particle.theta) - cos(particle.theta + yaw_rate * delta_t)) + noise_y(gen);
        particle.theta += yaw_rate * delta_t + noise_theta(gen);
    }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	// first arg predicted: Predicted measurements between one particle and all landmarks within sensor range 
	// second arg: observations: actual measurements from radar
	// Fn does NN association
	/*
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
	
	// return associations;
	*/
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
	double var_x = pow(std_landmark[0], 2);
	double var_y = pow(std_landmark[1], 2);
	double covar_xy = std_landmark[0] * std_landmark[1];
	double weights_sum = 0;	
	
	for (int i=0; i < num_particles; i++) {
		// predict measurements to all map landmarks
		Particle& particle = particles[i];

		// initialise unnormalised weight for particle
		// weight is a product so init to 1.0
		long double weight = 1;
		
		for (int j=0; j < observations.size(); j++) {
			// transform vehicle's observation to global coordinates
			LandmarkObs obs = observations[j];
			
			// predict landmark x, y. Equations from trigonometry.
			double predicted_x = obs.x * cos(particle.theta) - obs.y * sin(particle.theta) + particle.x;
			double predicted_y = obs.x * sin(particle.theta) + obs.y * cos(particle.theta) + particle.y;

			// initialise terms
			Map::single_landmark_s nearest_landmark;
			double min_distance = sensor_range;
			double distance = 0;

			// associate sensor measurements to map landmarks 
			for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {

				Map::single_landmark_s landmark = map_landmarks.landmark_list[k];

				// calculate distance between landmark and transformed observations
				// approximation (Manhattan distance)
				distance = fabs(predicted_x - landmark.x_f) + fabs(predicted_y - landmark.y_f);

				// update nearest landmark to obs
				if (distance < min_distance) {
					min_distance = distance;
					nearest_landmark = landmark;
				}


			} // end associate nearest landmark

			// then calculate new weight of each particle using multi-variate Gaussian (& associations)
			// equation in L14.11 Update Step video
			double x_diff = predicted_x - nearest_landmark.x_f;
			double y_diff = predicted_y - nearest_landmark.y_f;
			double num = exp(-0.5*((x_diff * x_diff)/var_x + (y_diff * y_diff)/var_y));
			double denom = 2*M_PI*covar_xy;
			// multiply particle weight by this obs-weight pair stat
			weight *= num/denom;

		} // end observations loop

		cout << "weight: " << weight << endl;

		// update particle weight 
		particle.weight = weight;
		// update weight in PF array
		weights[i] = weight;
		// add weight to weights_sum for normalising weights later
		weights_sum += weight;

	}
	
	/* Normalise weights: but wasn't necessary, didn't reduce error
	cout << "weights_sum: " << weights_sum << endl;
	// then normalise weights
	for (int i = 0; i < num_particles; i++) {
		// update particle weight
		particles[i].weight /= weights_sum;
		// update weight in PF array
		weights[i] /= weights_sum;
	}	
	*/

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	default_random_engine gen;

	// Take a discrete distribution with pmf equal to weights
    discrete_distribution<> weights_pmf(weights.begin(), weights.end());
    // initialise new particle array
    vector<Particle> newParticles;
    // resample particles
    for (int i = 0; i < num_particles; ++i)
        newParticles.push_back(particles[weights_pmf(gen)]);

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
