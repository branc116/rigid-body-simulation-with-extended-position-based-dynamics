cmake_minimum_required(VERSION 3.0.0)
project(PBDBODIES VERSION 0.1.0)

include(CTest)
enable_testing()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(blaze CONFIG REQUIRED)
find_package(libcopp CONFIG REQUIRED)



SET(CMAKE_CXX_FLAGS  "-Wall ${CMAKE_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS  "-fcoroutines ${CMAKE_CXX_FLAGS}")


add_executable(PBDBODIES main.cpp
    AddShit.hpp
    Core.hpp
    ITensors.hpp
    Types.hpp)

target_link_libraries(PBDBODIES PRIVATE blaze::blaze)
target_link_libraries(PBDBODIES PRIVATE libcopp::copp libcopp::cotask)


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})

include(CPack)
