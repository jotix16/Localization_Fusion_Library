#include<iostream>

#include<Eigen/Dense>

#include<motion_model/motion_model.h>
#include<filter/filter_ekf.h>

using namespace Eigen;

int main()
{
    using State = iav::state_predictor::filter::Ctrv_EKF2D::StateVector;
    using Matris = iav::state_predictor::filter::Ctrv_EKF2D::StateMatrix;

    std::cout<< "Filter: EKF"<<std::endl;

    iav::state_predictor::filter::Ctrv_EKF2D my_ekf;
    State x0;
    x0 << 1, 1 ,0, 0.5, 0.5, 0.5;
    Matris init_cov;
    init_cov.setIdentity()*1e-6;
    my_ekf.reset(x0, init_cov),

    std::cout<<"Your state is: \n"<< my_ekf.get_state() << "\n";
    std::cout<<"Your covariance is: \n"<< my_ekf.get_covariance() << "\n";
}