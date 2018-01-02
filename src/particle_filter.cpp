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
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 100;
	weights.resize(num_particles);
	particles.resize(num_particles);

	// Create normal distributions for x, y and theta.
	default_random_engine gen;
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for (int i = 0; i < num_particles; ++i)
	{
		//particles.push_back(new Particle());
		particles[i].id = i;
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		particles[i].weight = 1.0;
		//v.push_back(1);
	}
	is_initialized = 1;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Create normal distributions for x, y and theta.
	default_random_engine gen;
	normal_distribution<double> dist_x(0.0, std_pos[0]);
	normal_distribution<double> dist_y(0.0, std_pos[1]);
	normal_distribution<double> dist_theta(0.0, std_pos[2]);

	for (int i = 0; i < num_particles; ++i)
	{
		if (fabs(yaw_rate) > 0.0001)
		{
			double theta = particles[i].theta + yaw_rate * delta_t;
			particles[i].x += (sin(theta) - sin(particles[i].theta)) * velocity / yaw_rate + dist_x(gen);
			particles[i].y += (cos(particles[i].theta) - cos(theta)) * velocity / yaw_rate + dist_y(gen);
			particles[i].theta = theta + dist_theta(gen);
		}
		else
		{
			particles[i].x += velocity * delta_t * cos(particles[i].theta) + dist_x(gen);
			particles[i].y += velocity * delta_t * sin(particles[i].theta) + dist_y(gen);
			particles[i].theta += dist_theta(gen);
		}
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	double cur_err, error;
	double dx, dy;
	for(int i = 0; i < observations.size(); ++i)
	{
		error = 1.0e99;
		for (int j = 0; j < predicted.size(); ++j)
		{
			dx = predicted[j].x - observations[i].x;
			dy = predicted[j].y - observations[i].y;
			cur_err = sqrt(dx*dx + dy*dy);
			if (cur_err < error)
			{
				error = cur_err;
				observations[i].id = j;
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	double px, py, ptheta; //particle parameters
	double ox, oy; //observation parametes
	double mx, my, dx, dy, error; //landmark parameters

	// constants used later for calculating the new weights
	const double stdx = std_landmark[0];
	const double stdy = std_landmark[1];
	const double na = 0.5 / (stdx * stdx);
	const double nb = 0.5 / (stdy * stdy);
	const double d = 2.0 * M_PI * stdx * stdy;

	for (int i = 0; i < num_particles; ++i)
	{
		px = particles[i].x;
		py = particles[i].y;
		ptheta = particles[i].theta;

		//STEP 1: transform each observations to map coordinate
		vector<LandmarkObs> map_observations;
		for (int j = 0; j < observations.size(); ++j)
		{
			ox = observations[j].x;
			oy = observations[j].y;

			LandmarkObs observation = 
			{
				observations[j].id,
				px + ox * cos(ptheta) - oy * sin(ptheta),
				py + oy * cos(ptheta) + ox * sin(ptheta)
			};

			map_observations.push_back(observation);
		}

		//STEP 2: Find map landmarks within the sensor range
		vector<LandmarkObs> landmarks_in_range;
		for (int j = 0; j < map_landmarks.landmark_list.size(); j++)
		{
			const int mid = map_landmarks.landmark_list[j].id_i;
			mx = map_landmarks.landmark_list[j].x_f;
			my = map_landmarks.landmark_list[j].y_f;

			dx = mx - px;
			dy = my - py;
			error = sqrt(dx * dx + dy * dy);

			if (error < sensor_range) 
			{
					LandmarkObs landmark_in_range = {
						mid,
						mx,
						my};

				landmarks_in_range.push_back(landmark_in_range);
			}
		}

		//STEP 3: Associate landmark in range(id) to landmark observations
		dataAssociation(landmarks_in_range, map_observations);

		//STEP 4: update the particle weight 
		double w = 1.0;

		for (int j = 0; j < map_observations.size(); j++) 
		{
			const int oid = map_observations[j].id;
			const double dx = map_observations[j].x - landmarks_in_range[oid].x;
			const double dy = map_observations[j].y - landmarks_in_range[oid].y;
			const double r = exp(-(na*dx*dx + nb*dy*dy)) / d;
			w *= r;
		}

		particles[i].weight = w;
		weights[i] = w;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	vector<Particle> resampled_particles;
	default_random_engine gen;
	discrete_distribution<int> index(weights.begin(), weights.end());

	for (int j = 0; j < num_particles; ++j) 
	{
		const int i = index(gen);

		Particle p{
			i,
			particles[i].x,
			particles[i].y,
			particles[i].theta,
			1.0
		};

		resampled_particles.push_back(p);
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
