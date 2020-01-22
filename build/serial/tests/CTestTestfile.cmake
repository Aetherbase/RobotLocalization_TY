# CMake generated Testfile for 
# Source directory: /root/ws/tyros_ws/src/serial/tests
# Build directory: /root/ws/tyros_ws/build/serial/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_serial_gtest_serial-test "/root/ws/tyros_ws/build/catkin_generated/env_cached.sh" "/usr/sbin/python3" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/root/ws/tyros_ws/build/test_results/serial/gtest-serial-test.xml" "--return-code" "/root/ws/tyros_ws/devel/lib/serial/serial-test --gtest_output=xml:/root/ws/tyros_ws/build/test_results/serial/gtest-serial-test.xml")
set_tests_properties(_ctest_serial_gtest_serial-test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;143;add_test;/opt/ros/melodic/share/catkin/cmake/test/gtest.cmake;89;catkin_run_tests_target;/opt/ros/melodic/share/catkin/cmake/test/gtest.cmake;28;_catkin_add_google_test;/root/ws/tyros_ws/src/serial/tests/CMakeLists.txt;2;catkin_add_gtest;/root/ws/tyros_ws/src/serial/tests/CMakeLists.txt;0;")
add_test(_ctest_serial_gtest_serial-test-timer "/root/ws/tyros_ws/build/catkin_generated/env_cached.sh" "/usr/sbin/python3" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/root/ws/tyros_ws/build/test_results/serial/gtest-serial-test-timer.xml" "--return-code" "/root/ws/tyros_ws/devel/lib/serial/serial-test-timer --gtest_output=xml:/root/ws/tyros_ws/build/test_results/serial/gtest-serial-test-timer.xml")
set_tests_properties(_ctest_serial_gtest_serial-test-timer PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;143;add_test;/opt/ros/melodic/share/catkin/cmake/test/gtest.cmake;89;catkin_run_tests_target;/opt/ros/melodic/share/catkin/cmake/test/gtest.cmake;28;_catkin_add_google_test;/root/ws/tyros_ws/src/serial/tests/CMakeLists.txt;9;catkin_add_gtest;/root/ws/tyros_ws/src/serial/tests/CMakeLists.txt;0;")
