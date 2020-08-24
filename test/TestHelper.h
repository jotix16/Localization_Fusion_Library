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

/**
 * Macro to assert two matrices being nearly identical (elementwise)
 */
#define ASSERT_MATRIX_NEAR(A,B,delta)           \
    ASSERT_EQ(A.rows(), B.rows());              \
    ASSERT_EQ(A.cols(), B.cols());              \
    for(int i = 0; i < A.rows(); ++i)           \
    {                                           \
        for(int j = 0; j < A.cols(); ++j)       \
        {                                       \
            ASSERT_NEAR(A(i,j), B(i,j), delta); \
        }                                       \
    }

#define ASSERT_MATRIX_EQ(A,B)                   \
    ASSERT_EQ(A.rows(), B.rows());              \
    ASSERT_EQ(A.cols(), B.cols());              \
    for(int i = 0; i < A.rows(); ++i)           \
    {                                           \
        for(int j = 0; j < A.cols(); ++j)       \
        {                                       \
            ASSERT_EQ(A(i,j), B(i,j));          \
        }                                       \
    }