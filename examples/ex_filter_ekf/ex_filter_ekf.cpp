#include<iostream>

#include<string>
#include<vector>
#include<random>
#include<math.h>
#include<Eigen/Dense>

#include<utilities/filter_utilities.h>
#include<motion_model/motion_model.h>
#include<filter/filter_ekf.h>
#include"matplotlibcpp.h"

using namespace Eigen;

namespace plt = matplotlibcpp;

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
    using StateMatrix = Matrix<double, 3,3>;
    using StateVector = Matrix<double, 3,1>;
    using tTime = typename iav::state_predictor::tTime;
    struct States
    {
        using uint = typename iav::state_predictor::uint;
        constexpr static uint X = 0U;
        constexpr static uint Y = 1U;
        constexpr static uint V_X = 2U;
        static constexpr uint STATE_SIZE = 3U;
        static constexpr uint POSITION_OFFSET = X;
        static constexpr uint ORIENTATION_OFFSET = V_X;
        static constexpr uint POSITION_V_OFFSET = V_X;
        static constexpr uint ORIENTATION_V_OFFSET = STATE_SIZE;
        static constexpr uint POSITION_A_OFFSET = STATE_SIZE;
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
        state(States::X,States::X) += dt * state(States::V_X, States::X);
    }

    // The two functions below simulate the radar signal returns from an object flying
    // at a constant altitude and velocity in 2d.
    static StateVector simulate(StateVector& state, const tTime& dt)
    {
        // update state after adding process noise
        state(States::V_X) += 0.1 * normal_dist();
        state(States::Y) += 0.1 * normal_dist();
        state(States::X) += dt * state(States::V_X);
        return state;
    }
    static double get_measurement(const StateVector& state){
        // add measurement noise
        double err = state(States::X) * 0.05 * normal_dist();
        double meas = sqrt(state(States::X)*state(States::X)
        + state(States::Y)*state(States::Y));
        return meas + err ;
    }
};

struct MeasurementModel
{
    using States = MotionModel::States;
    using JacobiMatrix = Matrix<double, -1,3>;
    using MeasurementVector = Matrix<double, -1,1>;
    using StateVector = Matrix<double, 3,1>;
    using tTime = iav::state_predictor::tTime;

    static MeasurementVector get_measurement(const StateVector& state){
        // add measurement noise
       MeasurementVector z(1);
        double err = state(States::X) * 0.05 * normal_dist();
        double meas = sqrt(state(States::X)*state(States::X)+ state(States::Y)*state(States::Y));
        z(0,0) = meas + err;
        return z;
    }

    static MeasurementVector h(const StateVector& state)
    {
       MeasurementVector z(1);
    //    z.resize(1,1);
       z(0,0) = sqrt(state(States::X)*state(States::X)+ state(States::Y)*state(States::Y));
       return z;
    }

    static JacobiMatrix H(const StateVector& state, const tTime& dt)
    {
        // update jacobi
        JacobiMatrix jacobi;
        jacobi.resize(1,3);
        jacobi.setZero();

        double d = sqrt(state(States::X)*state(States::X)+ state(States::Y)*state(States::Y));
        jacobi(States::X, States::X) = state(States::X)/d;
        jacobi(States::X, States::Y) = state(States::Y)/d;
        return jacobi;
    }
};

using Ctra_EKF3D = iav::state_predictor::filter::FilterEkf<MotionModel, 3, double>;

void plot(std::vector<double>x, std::vector<double> y_ground_truth, std::vector<double> y_estimated, bool save, const std::string y_axis, const std::string title= "My Title")
{
    // Set the size of output image to 1200x780 pixels
    plt::figure_size(1200, 780);
    // Plot line from given x and y data. Color is selected automatically.
    plt::named_plot("Filter estimation", x, y_estimated);

    // Plot a red dashed line from given x and y data.
    plt::named_plot("Ground truth", x, y_ground_truth, "r--");
    plt::xlabel("timesteps");
    plt::ylabel(y_axis);
    // Add graph title
    plt::title(title);
    // Enable legend.
    plt::legend();
    // Save the image (file format is determined by the extension)
    if(save)
    {
        std::cout<<"Saving name: ./"+ std::string(y_axis) +".png\n";
        plt::save("./"+ std::string(y_axis) +".png");
    }
    // SHOW
    plt::show();


}


int main()
{
    using States = MotionModel::States;
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
    state0 << x-50, vx+100, y+1000; // init state

    // COVARIANCES
    // State covariance init
    StateMatrix P0;
    P0.setIdentity();
    P0 *= 50.0;

    // Process noise covariance
    StateMatrix Q;
    Q.setIdentity();
    // Q *= 0.1;
    Q(States::X, States::X) *= 0.25 * std::pow(dt,4) * 0.1;
    Q(States::V_X, States::V_X) *= 0.5 * std::pow(dt,2) * 0.1;
    Q(States::X, States::V_X) *= std::pow(dt,3) * 0.1;
    Q(States::V_X, States::X) *= 0.5 * std::pow(dt,3) * 0.1;

    Q(States::Y, States::Y) *= 0.1;

    // Measurement noise covariance
    double range_std = 1;
    MatrixXd R(1,1);
    R.setIdentity();
    R *= range_std*range_std;

    // Kalman Filter
    Ctra_EKF3D ekf;
    ekf.reset(state0, P0, Q);

    std::vector<double> ground_truth_x, ground_truth_vx, ground_truth_y, filter_estimate_x, filter_estimate_vx, filter_estimate_y, times;
    StateVector state_temp;
    std::vector<StateVector> ground_truth, filter_estimate;
    Measurement z = MeasurementModel::h(state);
    JacobiMatrix H = MeasurementModel::H(state, dt);

    double timesteps = 60.0/dt;
    for (int i = 0; i < (int)timesteps; i++)
    {
        times.push_back(i);
        state_temp = MotionModel::simulate(state, dt);
        ground_truth.push_back(state_temp);
        ground_truth_x.push_back(state_temp[0]);
        ground_truth_vx.push_back(state_temp[2]);
        ground_truth_y.push_back(state_temp[1]);

        z = MeasurementModel::get_measurement(state);
        H = MeasurementModel::H(state, dt);
        ekf.observation_update(z, H, R, 1000000);
        filter_estimate.push_back(ekf.get_state());

        state_temp = ekf.get_state();
        filter_estimate_x.push_back(state_temp[0]);
        filter_estimate_vx.push_back(state_temp[2]);
        filter_estimate_y.push_back(state_temp[1]);
        ekf.temporal_update(dt);
    }

    // PLOT
    plot(times, ground_truth_x, filter_estimate_x, false, "x position", "Airplane Simulation: tracking x");
    plot(times, ground_truth_vx, filter_estimate_vx, false, "x velocity", "Airplane Simulation: tracking vx");
    plot(times, ground_truth_y, filter_estimate_y, false, "altitude", "Airplane Simulation: tracking altitude");

    std::cout<<"Ground truth state is: \n"<< state << "\n";
    std::cout<<"Estimated state is: \n"<< ekf.get_state() << "\n";

    return 0;
}


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