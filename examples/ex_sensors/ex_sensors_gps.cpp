#include<sensors/gps.h>
#include<iostream>
#include<Eigen/Dense>

using namespace Eigen;
using GpsD =  iav::state_predictor::sensors::GpsD;
using StateVector   = typename GpsD::StateVector;
using TransformationMatrix   = typename GpsD::TransformationMatrix;

int main()
{
    std::cout<< "EXAMPLE GPS SENSOR"<<std::endl;
    bool update_vector[15] = { true,true,true,
                               true,true,true,
                               true,true,true,
                               true,true,true,
                               true,true,true};
    std::ostream* out_stream;
    GpsD gps("gps_topic", update_vector, 12.0, out_stream, false);


    double latitude=51.058171,
           longitude=13.741558,
           hae_altitude=0.0;

    StateVector state = StateVector::Ones();
    gps.initialize(state, Eigen::Isometry3d::Identity(), latitude, longitude, hae_altitude, Eigen::Isometry3d::Identity());
    latitude=51.057564;
    longitude=13.746013;
    gps.gps_callback(state, latitude, longitude, hae_altitude, Eigen::Isometry3d::Identity());
}