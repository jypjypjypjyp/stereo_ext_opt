add_executable(stereo_ext_opt
        config.cpp
        estimator.cpp
        frame.cpp
        frontend.cpp
        landmark.cpp
        node.cpp
        parameters.cpp
        utility.cpp)

target_link_libraries(stereo_ext_opt ${THIRD_PARTY_LIBS} blas)
target_compile_features(stereo_ext_opt PRIVATE cxx_std_14)