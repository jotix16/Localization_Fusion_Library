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

#pragma once

#include <gtest/gtest.h>

#include <motion_model/motion_model.h>
#include "TestHelper.h"


TEST(TestCtrv, PredictAndJacobi)
{
    const iav::state_predictor::motion_model::Ctrv2D model;
    using States = iav::state_predictor::motion_model::Ctrv2D::States;
    using CtrvState = iav::state_predictor::motion_model::Ctrv2D::StateVector;
    using CtrvMatris = iav::state_predictor::motion_model::Ctrv2D::StateMatrix;

    // dts
    const double seconds = 1;
    const double milliseconds_10 = 0.01;
    const double milliseconds_100 = 0.1;

    // set state
    const double vx = -1.0F;
    const double vy = 2.0F;
    const double yaw = iav::state_predictor::PI/3;

    CtrvState x;
    x << 0.0F, 0.0F, yaw, vx, vy, 0;

    // predict
    //define lambda for quick testing
    auto test_predict = [=](CtrvState x, double dt)
    {
        std::cout << "Ctrv: Testing Predict with dt=" << dt*1000 << "ms.\n";
        model.predict(x,dt);
        CtrvState x1;
        x1 << vx * dt * cos(yaw) - vy * dt * sin(yaw),
            vx * dt * sin(yaw) + vy * dt * cos(yaw),
            yaw,
            vx,
            vy,
            0;
        ASSERT_MATRIX_EQ(x,x1);
    };
    test_predict(x, seconds);
    test_predict(x, milliseconds_10);
    test_predict(x, milliseconds_100);

    // jacobi
    auto test_jacobi= [=](CtrvState x, double dt)
    {
        std::cout << "TestHelper: Testing Jacobi with dt=" << dt*1000 << "ms.\n";
        CtrvMatris jacobi;
        model.compute_jacobian(jacobi, x, dt);

        CtrvMatris jacobi1;

        double dt_c_yaw = dt*cos(yaw);
        double dt_s_yaw = dt*sin(yaw);

        jacobi1.setIdentity();
        jacobi1(States::X, States::V_X) = dt_c_yaw;
        jacobi1(States::X, States::V_Y) = -dt_s_yaw;
        jacobi1(States::X, States::YAW) = -dt_s_yaw * vx - dt_c_yaw * vy;
        jacobi1(States::Y, States::V_X) = dt_s_yaw;
        jacobi1(States::Y, States::V_Y) = dt_c_yaw;
        jacobi1(States::Y, States::YAW) = dt_c_yaw * vx - dt_s_yaw * vy;
        jacobi1(States::YAW, States::V_YAW) = dt;

        ASSERT_MATRIX_EQ(jacobi, jacobi1);
    };
    test_jacobi(x, seconds);
    test_jacobi(x, milliseconds_10);
    test_jacobi(x, milliseconds_100);

    // jacobi & predict
    auto test_jacobi_and_predict= [=](CtrvState x, double dt)
    {
        std::cout << "Ctrv: Testing JacobiAndPredict with dt=" << dt*1000 << "ms.\n";
        CtrvMatris jacobi1;
        CtrvState x1 = x;
        model.predict(x1, dt);
        model.compute_jacobian(jacobi1, x, dt);
        
        CtrvMatris jacobi;
        model.compute_jacobian_and_predict(jacobi, x, dt);

        ASSERT_MATRIX_EQ(x, x1);
        ASSERT_MATRIX_EQ(jacobi, jacobi1);
    };
    test_jacobi_and_predict(x, seconds);
    test_jacobi_and_predict(x, milliseconds_10);
    test_jacobi_and_predict(x, milliseconds_100);
};