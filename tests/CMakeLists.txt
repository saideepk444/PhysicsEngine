enable_testing()

add_executable(physics_tests
    PhysicsTests.cpp
)

target_link_libraries(physics_tests
    PRIVATE
    PhysicsCore
    gtest
    gtest_main
)

add_test(NAME physics_tests COMMAND physics_tests) 