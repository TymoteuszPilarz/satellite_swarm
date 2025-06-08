add_executable(satellite_swarm_simulator 
    main.cpp
    simulator.cpp
    satellite_swarm.cpp
    satellite.cpp
    antenna.cpp
    transmission_channel.cpp
)

find_package(Eigen3 REQUIRED NO_MODULE)

target_link_libraries(satellite_swarm_simulator 
    PRIVATE 
        Eigen3::Eigen
)
