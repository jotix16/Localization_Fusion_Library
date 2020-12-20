cmake_minimum_required(VERSION 3.13)

set(HEADERS
TestHelper.h
)

# Unit Tests
set(TESTS
    # Motion Model
    tst_motion_model_ctrv.cpp
    tst_motion_model_ctra.cpp
)

set(BUILD_TESTING true)
if(BUILD_TESTING)
#   add_executable(tst_motion_model_ctra ${TESTS} ${HEADERS})
  # motion model ctrv
  add_executable(tst_motion_model_ctrv tst_motion_model_ctrv.cpp ${HEADERS})
  target_link_libraries(tst_motion_model_ctrv gtest_main estimator)
  add_test(MotionModelCtrv tst_motion_model_ctrv)

  # motion model ctra
  add_executable(tst_motion_model_ctra tst_motion_model_ctra.cpp ${HEADERS})
  target_link_libraries(tst_motion_model_ctra gtest_main estimator)
  add_test(MotionModelCtra tst_motion_model_ctra)

  # motion model ctra3d
  add_executable(tst_motion_model_ctra3d tst_motion_model_ctra3d.cpp ${HEADERS})
  target_link_libraries(tst_motion_model_ctra3d gtest_main estimator)
  add_test(MotionModelCtra3d tst_motion_model_ctra3d)
endif()