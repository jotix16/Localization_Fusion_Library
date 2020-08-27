#include<iostream>

#include<vector>
#include<random>
#include<math.h>
#include<Eigen/Dense>

#include<motion_model/motion_model.h>
#include<filter/filter_ekf.h>
#include"matplotlibcpp.h"

using namespace Eigen;

namespace plt = matplotlibcpp;

// int main()
// {
//     // Prepare data.
//     int n = 5000;
//     std::vector<double> x(n), y(n), z(n), w(n,2);
//     for(int i=0; i<n; ++i) {
//         x.at(i) = i*i;
//         y.at(i) = sin(2*3.14*i/360.0);
//         z.at(i) = log(i);
//     }

//     // Set the size of output image to 1200x780 pixels
//     plt::figure_size(1200, 780);
//     // Plot line from given x and y data. Color is selected automatically.
//     plt::plot(x, y);
//     // Plot a red dashed line from given x and y data.
//     plt::plot(x, w,"r--");
//     // Plot a line whose name will show up as "log(x)" in the legend.
//     plt::named_plot("log(x)", x, z);
//     // Set x-axis to interval [0,1000000]
//     plt::xlim(0, 1000*1000);
//     // Add graph title
//     plt::title("Sample figure");
//     // Enable legend.
//     plt::legend();
//     // Save the image (file format is determined by the extension)
//     plt::show();
//     plt::save("./basic.png");
// }
// noise generator
std::random_device rd;
std::mt19937 gen(rd());
std::normal_distribution<double> d(0,1);
double normal_dist()
{
    return d(gen);
}


struct MotionModel
{
    using StateMatrix = typename Matrix<double, 3,3>;
    using StateVector = typename Matrix<double, 3,1>;
    using tTime = typename iav::state_predictor::tTime;
    struct States
    {
        constexpr static int X = 0U;
        constexpr static int V_X = 1U;
        constexpr static int V = 0U;
        static constexpr std::array<int,0> ANGLEidx = {};  ///< indexes of angles
    };

    static void compute_jacobian_and_predict(StateMatrix& jacobi, StateVector& state, const tTime& dt)
    {
        // update jacobi
        jacobi.setIdentity();
        jacobi(States::X, States::V_X) = dt;

        predict(state, dt);


    }
    static void predict(StateVector& state, const tTime& dt)
    {
        state(0,0) += dt * state(1,0);
    }

    // Simulates the radar signal retuns from an object flying at a constant altitude and velocity in 1d
    static StateVector simulate(StateVector& state, const tTime& dt)
    {
        // update state after adding process noise
        state(1,0) += 0.1 * normal_dist();
        state(2,0) += 0.1 * normal_dist();
        state(0,0) += dt * state(1,0);
        return state;
    }
    static double get_measurement(const StateVector& state){
        // add measurement noise
        double err = state(0,0) * 0.05 * normal_dist();
        double meas = sqrt(state(0,0)*state(0,0)+ state(1,0)*state(1,0));
        return meas + err ;
    }
};


struct MeasurementModel
{
    // using MeasurementVector = typename Matrix<double, 1,1>;
    // using JacobiMatrix = typename Matrix<double, 1,3>;
    using MeasurementVector = typename Matrix<double, -1,1>;
    using JacobiMatrix = typename Matrix<double, -1,-1>;
    using StateVector = typename Matrix<double, 3,1>;
    using tTime = typename iav::state_predictor::tTime;


    static MeasurementVector h(const StateVector& state, const tTime& dt)
    {
       MeasurementVector z(1);
    //    z.resize(1,1);
       z(0,0) = sqrt(state(0,0)*state(0,0)+ state(2,0)*state(2,0)); 
       return z;
    }

    static JacobiMatrix H(const StateVector& state, const tTime& dt)
    {
        // update jacobi
        JacobiMatrix jacobi(1,3);
        // jacobi.resize(1,3);
        jacobi.setZero();

        double d = sqrt(state(0,0)*state(0,0)+ state(2,0)*state(2,0));
        jacobi(0,0) = state(0,0)/d;
        jacobi(0,2) = state(2,0)/d;
        return jacobi;
    }
};

using Ctra_EKF3D = iav::state_predictor::filter::FilterEkf<MotionModel, 3, double>;

void check_normal_dist()
{
  // check the noise generator 
  int p[10]={};
  const int nrolls = 1000;
  const int nstars = 100;
  
  for (int i=0; i<nrolls; ++i) {
    double number = sqrt(2.0)*normal_dist() + 5;
    if ((number>=0.0)&&(number<10.0)) ++p[int(number)];
  }

  std::cout << "normal_distribution (5.0,2.0):" << std::endl;

  for (int i=0; i<10; ++i) {
    std::cout << i << "-" << (i+1) << ": ";
    std::cout << std::string(p[i]*nstars/nrolls,'*') << std::endl;
  }
}



int main()
{
    using StateVector = MotionModel::StateVector;
    using StateMatrix = MotionModel::StateMatrix;
    using Measurement = MeasurementModel::MeasurementVector;
    using JacobiMatrix = MeasurementModel::JacobiMatrix;

    // time step
    double dt = 0.05;
    // state inits
    StateVector state0, state;
    double x = 0.0, vx = 100, y = 1000.0;
    state << x, vx, y; // the real state
    state0 << x-5, vx-10, y+100; // init state

    // COVARIANCES
    // State covariance init
    StateMatrix P0;
    P0.setIdentity();
    P0 *= 50.0;

    // Process noise covariance
    StateMatrix Q;
    Q.setIdentity();
    Q *= 0.1;
    // Q(0,0) *= 0.25 * std::pow(dt,4) * 0.1;
    // Q(1,1) *= 0.5 * std::pow(dt,2) * 0.1;
    // Q(0,1) *= std::pow(dt,3) * 0.1;
    // Q(1,0) *= 0.5 * std::pow(dt,3) * 0.1;

    // Q(2,2) *= 0.1;
    
    // Measurement noise covariance
    double range_std = 3;
    MatrixXd R(1,1);
    R.setIdentity();
    R *= range_std*range_std;

    // Kalman Filter
    Ctra_EKF3D ekf;
    ekf.reset(state0, P0, Q);

    std::vector<double> track_x, track_vx, track_y, filter_estimate_x, filter_estimate_vx, filter_estimate_y, times;
    StateVector state_temp;
    std::vector<StateVector> track, filter_estimate;
    Measurement z = MeasurementModel::h(state, dt);
    JacobiMatrix H = MeasurementModel::H(state, dt);

    int timesteps = 60.0/dt;
    for (int i = 0; i < timesteps; i++)
    {
        times.push_back(i);
        state_temp = MotionModel::simulate(state, dt);
        track.push_back(state_temp);
        track_x.push_back(state_temp[0]);
        track_vx.push_back(state_temp[1]);
        track_y.push_back(state_temp[2]);

        z = MeasurementModel::h(state, dt);
        H = MeasurementModel::H(state, dt);
        ekf.observation_update(z, H, R, 1000000);
        filter_estimate.push_back(ekf.get_state());
        
        state_temp = ekf.get_state();
        filter_estimate_x.push_back(state_temp[0]);
        filter_estimate_vx.push_back(state_temp[1]);
        filter_estimate_y.push_back(state_temp[2]);
        ekf.temporal_update(dt);
    }
    
    // for (int i = 0; i < timesteps; i++)
    // {
    //     std::cout<< i <<"_1: "<< track.at(i).transpose() << "\n";
    //     std::cout<< i <<"_2: "<< filter_estimate.at(i).transpose() << "\n";
    // }
    
    // Set the size of output image to 1200x780 pixels
    plt::figure_size(1200, 780);
    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(times, track_y);
    // Plot a red dashed line from given x and y data.
    plt::plot(times, filter_estimate_y,"r--");
    // Plot a line whose name will show up as "log(x)" in the legend.
    // plt::named_plot("log(x)", x, z);
    // Set x-axis to interval [0,1000000]
    // plt::xlim(0, 1000*1000);
    // Add graph title
    plt::title("Sample figure");
    // Enable legend.
    plt::legend();
    // Save the image (file format is determined by the extension)
    plt::show();

    std::cout<<"Real state is: \n"<< state << "\n";
    std::cout<<"Estimated state is: \n"<< ekf.get_state() << "\n";

    return 0;
}

// int main()
// {
//     using State = iav::state_predictor::filter::Ctrv_EKF2D::StateVector;
//     using Matris = iav::state_predictor::filter::Ctrv_EKF2D::StateMatrix;

//     std::cout<< "Filter: EKF"<<std::endl;

//     iav::state_predictor::filter::Ctrv_EKF2D my_ekf;
//     State x0;
//     x0 << 1, 1 ,0, 0.5, 0.5, 0.5;
//     Matris init_cov;
//     init_cov.setIdentity()*1e-6;
//     my_ekf.reset(x0, init_cov),

//     std::cout<<"Your state is: \n"<< my_ekf.get_state() << "\n";
//     std::cout<<"Your covariance is: \n"<< my_ekf.get_covariance() << "\n";
// }

void pass(const Matrix<double ,-1, -1>& m)
{
    std::cout<< "Rows: " << m.rows() <<std::endl;
    std::cout<< "Cols: " << m.cols() <<std::endl;
}
int main2()
{
    Matrix<double,-1,-1> mym(2,12);
    mym.setZero();
    mym(0,0) = 14.0;
    mym(0,3) = 14.0;
    std::cout<< "Matrix: \n" << mym <<std::endl;
    // pass(mym);
    return 0;
}

int main4()
{
    return 1;
    // main3();
    // main2();
}