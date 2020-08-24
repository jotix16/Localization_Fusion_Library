# pragma once

// #include <gtest/gtest.h>

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