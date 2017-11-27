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

double ParticleFilter::sample_from_gaussian(double mean, double std){
	normal_distribution<double> norm_dist(mean, std);
	return norm_dist(gen);
}

double ParticleFilter::NormalizeAngle(double angle)
{
  while (angle> M_PI) angle-=2.*M_PI;
  while (angle<-M_PI) angle+=2.*M_PI;
	return angle;
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	num_particles = 100;

	for(int i=0; i<num_particles; ++i){
		Particle p;

		// Set values
		p.id = i;
		p.x  = sample_from_gaussian(x, std[0]);
		p.y  = sample_from_gaussian(y, std[1]);
		p.theta = NormalizeAngle(sample_from_gaussian(theta, std[2]));
		p.weight = 1.0;

		cout << p.x << " - " << p.y << " - " << p.theta << endl;

		particles.push_back(p);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	double theta_dt = delta_t * yaw_rate;
	double factor = velocity / yaw_rate;

	// Update each particle
	for(int i=0; i<num_particles; ++i){
		Particle p = particles[i];
		x = p.x + factor*(sin(p.theta + theta_dt) - sin(p.theta))
		y = p.y + factor*(cos(p.theta) - cos(p.theta + theta_dt));
		theta = p.theta + theta_dt;

		// Add noise to predictions and update particle
		particles[i].x = sample_from_gaussian(x, std_pos[0]);
		particles[i].y = sample_from_gaussian(y, std_pos[1]);
		particles[i].theta = NormalizeAngle(sample_from_gaussian(theta, std_pos[2]));
	}
}

/*
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

}*/
void ParticleFilter::dataAssociation(LandmarkObs predicted, const Map &map_landmarks) {
	// TODO: No, we need to change this
	// Find the closest landmark to the predicted observation coordinates
	for(int i=0; i<map_landmarks.landmark_list.size();++i){
		d = dist(predicted.x, predicted.y, double(map_landmarks[i].x_f), double(map_landmarks[i].y_f));
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

	// For each particle
	for(int i=0; i<num_particles; ++i){
		Particle p = particles[i];

		// Transform each observation into map coordinate system
		vector<LandmarkObs> particle_observations;
		for(int j=0; j<observations.size();++j){
			LandmarkObs predicted;

			predicted.x = p.x + (cos(p.theta)*observations[j].x) - (sin(p.theta)*observations[j].y);
			predicted.y = p.y + (sin(p.theta)*observations[j].x) - (cos(p.theta)*observations[j].y);

			// Set sensex, sensey in particle
			// Set associations
			// from associations, get corrdinates of closest landmark
			// compute cumulative weight
			// TODO: at some point we need to consider sensor_range (ignore predictions larger than this value? or clipped them to this value)



			particle_observations.push_back(obs);
		}

		// Associate each observation to closest landmark in map


	}
}

void ParticleFilter::resample() {
	// Delete previous weights
	weights.clear();

	// Update weights`
	for(int i=0; i<num_particles; ++i){
		weights.push_back(particles[i].weight);
	}

	// Resample particles
	std::vector<Particle> temporal;
	for(int i=0; i<num_particles; ++i){
		discrete_distribution<int> particle_dist(weights.begin(),weights.end());
		int index = particle_dist(gen);
		temporal.push_back(particles[index]);
	}
	particles = temporal;

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
