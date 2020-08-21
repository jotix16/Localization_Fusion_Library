# Now simply link against gtest or gtest_main as needed. Eg
add_executable(tst_lib tst_lib.cpp)
target_link_libraries(tst_lib gtest_main)
add_test(NAME example_test COMMAND tst_lib)