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

#define EPS 0.00001 // Just a small number
#define NUMBER_OF_PARTICLES 100

double ParticleFilter::sample_from_gaussian(double mean, double std){
	normal_distribution<double> norm_dist(mean, std);
	return norm_dist(gen);
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	num_particles = NUMBER_OF_PARTICLES;

	for(int i=0; i<num_particles; ++i){
		Particle particle;
		// Set values
		particle.id = i;
		particle.x  = sample_from_gaussian(x, std[0]);
		particle.y  = sample_from_gaussian(y, std[1]);
		particle.theta = sample_from_gaussian(theta, std[2]);
		particle.weight = 1.0;

		particles.push_back(particle);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	double theta_dt = delta_t * yaw_rate;

	// Update each particle
	for(int i=0; i<num_particles; ++i){
		if (fabs(yaw_rate) < EPS){
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
			particles[i].y += velocity * delta_t * sin(particles[i].theta);
		}
		else{
			double factor = velocity / yaw_rate;
			particles[i].x += factor*(sin(particles[i].theta + theta_dt) - sin(particles[i].theta));
			particles[i].y += factor*(cos(particles[i].theta) - cos(particles[i].theta + theta_dt));
			particles[i].theta += theta_dt;
		}

		// Add noise to predictions and update particle
		particles[i].x += sample_from_gaussian(0, std_pos[0]);
    particles[i].y += sample_from_gaussian(0, std_pos[1]);
		particles[i].theta += sample_from_gaussian(0, std_pos[2]);
	}
}


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	for(unsigned int i=0; i<observations.size();++i){
		// Current observation
    LandmarkObs o = observations[i];

    // min distance to largest possible value
		double min_dist = numeric_limits<double>::max();
		int landmark_idx = -1;

		for (unsigned int l=0; l< predicted.size(); ++l){
			// Current prediction
			LandmarkObs p = predicted[l];

			double d = dist(o.x, o.y, p.x, p.y);

			if(d < min_dist){
				min_dist = d;
				landmark_idx = p.id;
			}
		}
		observations[i].id = landmark_idx;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {

	// calculate normalization term
	double sigma_xx = std_landmark[0]*std_landmark[0];
	double sigma_yy = std_landmark[1]*std_landmark[1];
	double gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);

	// For each particle
	for(int i=0; i<num_particles; ++i){
		double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;

		double sensor_range_2 = sensor_range * sensor_range;

		// Find landmarks within sensor range
		vector<LandmarkObs> predictions;
		for (unsigned int l=0; l< map_landmarks.landmark_list.size(); ++l){
			float landmarkX = map_landmarks.landmark_list[l].x_f;
      float landmarkY = map_landmarks.landmark_list[l].y_f;
			int id = map_landmarks.landmark_list[l].id_i;

			double range_x = landmarkX - x;
			double range_y = landmarkY - y;

			if(range_x*range_x + range_y*range_y <= sensor_range_2){
				predictions.push_back(LandmarkObs{ id, landmarkX, landmarkY });
			}
		}

		// Transform each observation into map coordinate system
		vector<LandmarkObs> transformed_obs;
		for(unsigned int j=0; j<observations.size();++j){
			double xx = x + (cos(theta)*observations[j].x) - (sin(theta)*observations[j].y);
			double yy = y + (sin(theta)*observations[j].x) + (cos(theta)*observations[j].y);
			transformed_obs.push_back(LandmarkObs{ observations[j].id, xx, yy });
		}

		// Associate observations to landmarks
		dataAssociation(predictions, transformed_obs);

		vector<int> associations;
		vector<double> sense_x;
		vector<double> sense_y;

		// Calculate particle weight
		particles[i].weight = 1.0;
		for(unsigned int j=0; j<transformed_obs.size();++j){
			int associated_id = transformed_obs[j].id;

			double mu_x, mu_y;
			// get the x,y coordinates of the landmark
      for (unsigned int k = 0; k < predictions.size(); k++) {
        if (predictions[k].id == associated_id) {
          mu_x = predictions[k].x;
          mu_y = predictions[k].y;
        }
			}

			// Calculating weight.
      double dX = transformed_obs[j].x - mu_x;
			double dY = transformed_obs[j].y - mu_y;

			double weight = gauss_norm * exp( -( dX*dX/(2*sigma_xx) + (dY*dY/(2*sigma_yy)) ) );
			if (weight == 0) {
        	particles[i].weight *= EPS;
      }else {
        particles[i].weight *= weight;
			}


			associations.push_back(associated_id);
			sense_x.push_back(transformed_obs[j].x);
			sense_y.push_back(transformed_obs[j].y);
		}

		// Set particle associations for debugging
		particles[i] = SetAssociations(particles[i], associations, sense_x, sense_y);		//particles[i] =
	}
}

void ParticleFilter::resample() {

	// Update weights`
	weights.clear();
	for(int i=0; i<num_particles; ++i){
		weights.push_back(particles[i].weight);
	}

	discrete_distribution<int> particle_dist(weights.begin(),weights.end());

	// Resample particles
	vector<Particle> new_particles;
	new_particles.resize(num_particles);
	for(int i=0; i<num_particles; ++i){

		auto index = particle_dist(gen);
		new_particles[i] = std::move(particles[index]);
	}
	particles = std::move(new_particles);

}

Particle ParticleFilter::SetAssociations(Particle particle, const std::vector<int> associations,
                                     const std::vector<double> sense_x, const std::vector<double> sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

		//Clear the previous associations
		particle.associations.clear();
		particle.sense_x.clear();
		particle.sense_y.clear();

  	particle.associations= associations;
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
