/*! \file
*
* Copyright (c) 1983-2020 IAV GmbH. All rights reserved.
*
* \authors Mikel Zhobro, Robert Treiber IAV GmbH
*
* Correspondence should be directed to IAV.
*
* IAV GmbH\n
* Carnotstraße 1\n
* 10587 Berlin
*
* GERMANY
*
* \note
* Neither IAV GmbH nor the authors admit liability
* nor provide any warranty for any of this software.
* Until the distribution is granted by IAV GmbH
* this source code is under non disclosure and must
* only be used within projects with controlling
* interest of IAV GmbH.
*/

#include <gtest/gtest.h>
#include "TestHelper.h"
#include <motion_model/motion_model.h>



TEST(TestCtra, PredictAndJacobi)
{
    const iav::state_predictor::motion_model::Ctra2D model;
    using States = iav::state_predictor::motion_model::Ctra2D::States;
    using CtrvState = iav::state_predictor::motion_model::Ctra2D::StateVector;
    using CtrvMatris = iav::state_predictor::motion_model::Ctra2D::StateMatrix;

    // dts
    const double seconds = 1;
    const double milliseconds_10 = 0.01;
    const double milliseconds_100 = 0.1;

    // set state
    const double vx = -1.0F;
    const double vy = 2.0F;
    const double ax = -0.5F;
    const double ay = 0.5F;
    const double yaw = iav::state_predictor::PI/3;

    CtrvState x;
    x << 0.0F, 0.0F, yaw, vx, vy, 0, ax, ay;

    // predict
    //define lambda for quick testing
    auto test_predict = [=](CtrvState x, double dt)
    {
        std::cout << "Ctra: Testing Predict with dt=" << dt*1000 << "ms.\n";
        model.predict(x,dt);
        CtrvState x1;
        x1 << vx*dt*cos(yaw) - vy*dt*sin(yaw) + 0.5*dt*dt*ax*cos(yaw) - 0.5*dt*dt*ay*sin(yaw),
            vx*dt*sin(yaw) + vy*dt*cos(yaw) + 0.5*dt*dt*ax * sin(yaw) +  0.5*dt*dt*ay*cos(yaw),
            yaw,
            vx + dt*ax,
            vy + dt*ay,
            0,
            ax,
            ay;
        ASSERT_MATRIX_NEAR(x,x1,1e-7);
    };
    test_predict(x, seconds);
    test_predict(x, milliseconds_10);
    test_predict(x, milliseconds_100);

    // jacobi
    auto test_jacobi= [=](CtrvState x, double dt)
    {
        std::cout << "Ctra: Testing Jacobi with dt=" << dt*1000 << "ms.\n";
        CtrvMatris jacobi;
        model.compute_jacobian(jacobi, x, dt);

        CtrvMatris jacobi1;
        double dt_c_yaw = dt*cos(yaw);
        double dt_s_yaw = dt*sin(yaw);
        double dt_dt_c_yaw_05 = dt_c_yaw * dt * 0.5;
        double dt_dt_s_yaw_05 = dt_s_yaw * dt * 0.5;

        jacobi1.setIdentity();
        jacobi1(States::X, States::V_X) = dt_c_yaw;
        jacobi1(States::X, States::V_Y) = -dt_s_yaw;
        jacobi1(States::X, States::YAW) = -dt_s_yaw * vx - dt_c_yaw * vy
                                        - dt_dt_s_yaw_05 * ax - dt_dt_c_yaw_05 * ay;
        jacobi1(States::X, States::A_X) = dt_dt_c_yaw_05;
        jacobi1(States::X, States::A_Y) = -dt_dt_s_yaw_05;
        jacobi1(States::V_X, States::A_X) = dt;

        jacobi1(States::Y, States::V_X) = dt_s_yaw;
        jacobi1(States::Y, States::V_Y) = dt_c_yaw;
        jacobi1(States::Y, States::YAW) = dt_c_yaw * vx - dt_s_yaw * vy
                                        + dt_dt_c_yaw_05 * ax - dt_dt_s_yaw_05 * ay;
        jacobi1(States::Y, States::A_X) = dt_dt_s_yaw_05;
        jacobi1(States::Y, States::A_Y) = dt_dt_c_yaw_05;
        jacobi1(States::V_Y, States::A_Y) = dt;
        jacobi1(States::YAW, States::V_YAW) = dt;

        ASSERT_MATRIX_NEAR(jacobi, jacobi1, 1e-7);
    };
    test_jacobi(x, seconds);
    test_jacobi(x, milliseconds_10);
    test_jacobi(x, milliseconds_100);

    // jacobi & predict
    auto test_jacobi_and_predict= [=](CtrvState x, double dt)
    {
        std::cout << "Ctra: Testing JacobiAndPredict with dt=" << dt*1000 << "ms.\n";
        // since we tested above we can use the functions predict() and compute_jacobian()
        CtrvMatris jacobi1;
        CtrvState x1 = x;
        model.predict(x1, dt);
        model.compute_jacobian(jacobi1, x, dt);
        
        CtrvMatris jacobi;
        model.compute_jacobian_and_predict(jacobi, x, dt);

        // ASSERT_EQ(x(3), x1(3));
        ASSERT_MATRIX_NEAR(x, x1, 1e-5);
        ASSERT_MATRIX_NEAR(jacobi, jacobi1, 1e-7);
    };
    test_jacobi_and_predict(x, seconds);
    test_jacobi_and_predict(x, milliseconds_10);
    test_jacobi_and_predict(x, milliseconds_100);
};