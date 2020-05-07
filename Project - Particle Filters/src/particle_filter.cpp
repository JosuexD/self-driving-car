/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

static std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {

  // Initiating the number of particles. For this case we'll stick to 100.
  num_particles = 150;

  // Creating gaussian distribution for x, y, and theta
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  // Initiating particles instances
  for(int i = 0; i < num_particles; ++i)
  {
      // Default values are set for the particles. Randomizing their distances using default_random_engine
      Particle p;
      p.id = i;
      p.weight = 1.0;
      p.x = dist_x(gen);
      p.y = dist_y(gen);
      p.theta = dist_theta(gen);
      particles.push_back(p);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

    for (int i = 0; i < num_particles; ++i)
    {
        // Locally saving the previous values before overriding them with the newer calculations
        double x_old = particles[i].x;
        double y_old = particles[i].y;
        double theta_old = particles[i].theta;

        double theta_predicted, x_predicted, y_predicted;

        // Motion model while turning. Taking into account yaw rate
        if(abs(yaw_rate) > 1e-5)
        {
            theta_predicted = theta_old + yaw_rate * delta_t;
            x_predicted = x_old + velocity / yaw_rate * (sin(theta_predicted) - sin(theta_old));
            y_predicted = y_old + velocity / yaw_rate * (cos(theta_old) - cos(theta_predicted));
        } else // Motion model of going straight, taking into account delta_t
        {
            theta_predicted = theta_old;
            x_predicted = x_old + velocity * delta_t * cos(theta_old);
            y_predicted = y_old + velocity * delta_t * sin(theta_old);
        }

        // Initializing normal distribution on the new predicted value
        std::normal_distribution<double> dist_x (x_predicted, std_pos[0]);
        std::normal_distribution<double> dist_y(y_predicted, std_pos[1]);
        std::normal_distribution<double> dist_theta(theta_predicted, std_pos[2]);

        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
   for(auto& observation : observations)
   {
       // sets the minimum value to the highest possible double value
       double min = std::numeric_limits<double>::max();
       for(const auto& predicted_observation: predicted)
       {
           // Calculates distance between observed and predicted
           double d = dist(observation.x, observation.y, predicted_observation.x, predicted_observation.y);
           if (d < min) // If the distance is smaller than previous distance then it will assign the observation to be the predicted
           {
               observation.id = predicted_observation.id;
               min = d; // This line will ensure that the next iteration will be a distance that is closer, and closer.
           }
       }

   }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  double std_x = std_landmark[0];
  double std_y = std_landmark[1];

    for (int i = 0; i <num_particles; ++i)
    {
        // Saving current particle values locally
        double p_x = particles[i].x;
        double p_y = particles[i].y;
        double p_theta = particles[i].theta;

        vector<LandmarkObs> predicted_landmarks;
        for(const auto& map_landmark: map_landmarks.landmark_list)
        {
            // TODO, may need to do some type conversion on here for the landmark x and y
            int l_id = map_landmark.id_i;
            double l_x = map_landmark.x_f;
            double l_y = map_landmark.y_f;

            // Getting distance between current particle and known landsmarks
            double distance = dist(p_x, p_y, l_x, l_y);
            if (distance < sensor_range )
            {
                LandmarkObs l_predicted;
                l_predicted.id = l_id;
                l_predicted.x =l_x;
                l_predicted.y = l_y;
                predicted_landmarks.push_back(l_predicted);
            }
        }

        vector<LandmarkObs> observed_landmarks_map_refs;
        for (uint16_t j = 0; j < observations.size(); ++j)
        {
            // Converting obsercations into data that the  map oordinata system can understand
            LandmarkObs rot_translates_obs;
            rot_translates_obs.x = cos(p_theta) * observations[j].x - sin(p_theta) * observations[j].y + p_x;
            rot_translates_obs.y = sin(p_theta)  * observations[j].x + cos(p_theta) * observations[j].y + p_y;

            observed_landmarks_map_refs.push_back(rot_translates_obs);
        }
        //
        dataAssociation(predicted_landmarks, observed_landmarks_map_refs);

        double particle_likelihood = 1.0;

        double mu_x, mu_y;

        for (const auto& obs : observed_landmarks_map_refs)
        {
            for(const auto& land : predicted_landmarks)
            {
                if(obs.id == land.id)
                {
                    mu_x = land.x;
                    mu_y = land.y;
                    break;
                }
            }

            double norm_factor = 2 * M_PI * std_x * std_y;
            double prob = exp( -( pow(obs.x - mu_x, 2) /
                    (2 * std_x * std_x) + pow(obs.y - mu_y, 2) / (2 * std_y * std_y) ) );
            particle_likelihood *= prob / norm_factor;
        }
        particles[i].weight = particle_likelihood;
    }


    double norm_factor = 0.0;
    for(const auto& particle : particles)
    {
        norm_factor += particle.weight;
    }

    for(auto& particle : particles)
    {
        particle.weight /= (norm_factor + std::numeric_limits<double>::epsilon());
    }


}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  vector<double> particle_weights;
  for(const auto& particle : particles)
  {
      particle_weights.push_back(particle.weight);
  }

  std::discrete_distribution<int> weighted_distribution(particle_weights.begin(), particle_weights.end());

  vector<Particle> resampled_particles;
  for(int i = 0; i < num_particles; ++i)
  {
      int j = weighted_distribution(gen);
      resampled_particles.push_back(particles[j]);
  }

  particles = resampled_particles;

  // Resetting all the weights for the particles
  for(auto& particle: particles)
  {
      particle.weight = 1.0f;
  }

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}