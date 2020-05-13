#include "particle_filter.hpp"

using namespace std;
random_device rd;
default_random_engine gen(rd());

void ParticleFilter::initParticles(double x, double y, double theta)
{
    num_particles = 100;
    particles.resize(num_particles);

    //Set Gaussian Distribution with mean and stddev
    normal_distribution<double> dist_x(x, sigma_pos[0]);
    normal_distribution<double> dist_y(y, sigma_pos[1]);
    normal_distribution<double> dist_theta(theta, sigma_pos[2]);
    for (int i = 0; i < num_particles; i++)
    {
        particles[i].id = i;
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
        particles[i].weight = 1;
    }
}

void ParticleFilter::prediction(double delta_t, double velocity, double yaw_rate)
{

    //Adding noise to movement
    normal_distribution<double> dist_x(0, sigma_pos[0]);
    normal_distribution<double> dist_y(0, sigma_pos[1]);
    normal_distribution<double> dist_theta(0, sigma_pos[2]);
    for (int i = 0; i < num_particles; i++)
    {
        //Motion modell
        double theta = particles[i].theta;
        double d_theta_t = yaw_rate * delta_t;
        if (yaw_rate > 0.01)
        {
            particles[i].x += (velocity / yaw_rate) * (sin(theta + d_theta_t) - sin(theta));
            particles[i].y += (velocity / yaw_rate) * (-cos(theta + d_theta_t) + cos(theta));
            particles[i].theta += d_theta_t;
        }
        else
        {
            particles[i].x += velocity * delta_t * cos(theta);
            particles[i].y += velocity * delta_t * sin(theta);
        }
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
    }
}
double dist(double x1, double y1, double x2, double y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
double getWeight(double x, double y, double ux, double uy, double sigx, double sigy)
{
    double faktor = 1 / (2 * M_PI * sigx * sigy);
    double exp_x = pow((x - ux), 2) / (2 * pow(sigx, 2));
    double exp_y = pow((y - uy), 2) / (2 * pow(sigy, 2));
    return faktor * exp(-(exp_x + exp_y));
}
void ParticleFilter::updateWeights(double sensor_range, const vector<LandmarkObs> &observations, const MyPointCloud2D &map)
{
    //Re-init for resampling
    weights.resize(num_particles);
    for (size_t i = 0; i < particles.size(); i++)
    {
        //Init particle
        particles[i].associations.clear();
        particles[i].sense_x.clear();
        particles[i].sense_y.clear();
        particles[i].weight = 1.0;

        //Transform observations to map coordinate system
        vector<LandmarkObs> mapped_obs;
        mapped_obs = TransformToMapCoords(particles[i], observations);

        //Search closest landmark for every observation at one particle
        for (size_t j = 0; j < mapped_obs.size(); j++)
        {
            vector<double> distances_lm_sm; // distances between landmark and sensor measurements
            for (size_t k = 0; k < map.pts.size(); k++)
            {
                //Calc distance between landmark and current particle
                int lm_id = map.ids[k];
                double lm_x = map.pts[k].x;
                double lm_y = map.pts[k].y;
                double distance_lm_pt = dist(particles[i].x, particles[i].y, lm_x, lm_y);
                if (distance_lm_pt <= sensor_range)
                {
                    // Calc distance between observation and all landmarks within sensor range
                    double distance_lm_sm = dist(mapped_obs[j].x, mapped_obs[j].y, lm_x, lm_y);
                    distances_lm_sm.push_back(distance_lm_sm);

                    //Association for visualization
                    particles[i].associations.push_back(lm_id);
                    particles[i].sense_x.push_back(lm_x);
                    particles[i].sense_y.push_back(lm_y);
                }
                else
                {
                    //If landmark is out of range, then distance is inf.
                    distances_lm_sm.push_back(99999999.9);
                }
            }
            int closest_lm_index = distance(distances_lm_sm.begin(), min_element(distances_lm_sm.begin(), distances_lm_sm.end()));
            double lm_closest_x = map.pts[closest_lm_index].x;
            double lm_closest_y = map.pts[closest_lm_index].y;
            particles[i].weight *= getWeight(lm_closest_x, lm_closest_y, mapped_obs[j].x, mapped_obs[j].y, sigma_landmark[0], sigma_landmark[1]);
        }
        weights[i] = particles[i].weight;
    }
}

void ParticleFilter::resample()
{
    //Resampling Wheel Algorithm
    vector<Particle> newParticles;
    int index = rand() % (num_particles); // 0-100
    double beta = 0.0;
    double mw = *max_element(weights.begin(), weights.end());
    for (int i = 0; i < num_particles; i++)
    {
        beta += ((double)rand() / RAND_MAX) * 2.0 * mw;
        while (beta > weights[index])
        {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        particles[i].id = i;
        newParticles.push_back(particles[index]);
    }
    particles = newParticles;
}

vector<LandmarkObs> ParticleFilter::TransformToMapCoords(Particle particle, vector<LandmarkObs> observation)
{
    //From perspective of particle
    double trans_obs_x, trans_obs_y;
    for (size_t o = 0; o < observation.size(); o++)
    {
        trans_obs_x = particle.x + (cos(particle.theta) * observation[o].x) - (sin(particle.theta) * observation[o].y);
        trans_obs_y = particle.y + (sin(particle.theta) * observation[o].x) + (cos(particle.theta) * observation[o].y);
        observation[o].x = trans_obs_x;
        observation[o].y = trans_obs_y;
    }

    return observation;
}