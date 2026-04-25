include(CTest)
include(Catch)

add_executable(test_smoke test_smoke.cpp)
target_link_libraries(test_smoke PRIVATE pendulum_core Catch2::Catch2WithMain)
catch_discover_tests(test_smoke)

add_executable(test_types test_types.cpp)
target_link_libraries(test_types PRIVATE pendulum_core Catch2::Catch2WithMain)
catch_discover_tests(test_types)

add_executable(test_pendulum_n1 test_pendulum_n1.cpp)
target_link_libraries(test_pendulum_n1 PRIVATE pendulum_core Catch2::Catch2WithMain)
catch_discover_tests(test_pendulum_n1)

add_executable(test_dopri5 test_dopri5.cpp)
target_link_libraries(test_dopri5 PRIVATE pendulum_core Catch2::Catch2WithMain)
catch_discover_tests(test_dopri5)

add_executable(test_engine_n1 test_engine_n1.cpp)
target_link_libraries(test_engine_n1 PRIVATE pendulum_core Catch2::Catch2WithMain)
catch_discover_tests(test_engine_n1)

add_executable(test_validation_n1 test_validation_n1.cpp)
target_link_libraries(test_validation_n1 PRIVATE pendulum_core Catch2::Catch2WithMain)
catch_discover_tests(test_validation_n1)
