#include<motion_model/motion_model.h>
#include<iostream>
#include<Eigen/Dense>

using namespace Eigen;

int main()
{
    std::cout<< "MOTION MODEL"<<std::endl;
    iav::state_predictor::motion_model::Ctra2D my_ctra;
    using State = iav::state_predictor::motion_model::Ctra2D::StateVector;
    using Matris = iav::state_predictor::motion_model::Ctra2D::StateMatrix;
    Matris x;
    x.setIdentity(8,8);
    x(1,2) = 2;
    x(1,3) = 3;

    State s;
    s << 1,1,1,1,1,1,1,1;
    my_ctra.predict(s,1);
    std::cout<<"Your state is: "<<s << "\n";
}