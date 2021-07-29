# Build GTest
include(OhmGTest)

enable_testing()
add_subdirectory(data)
add_subdirectory(gputiltest)
add_subdirectory(ohmtestcommon)
add_subdirectory(ohmtest)
add_subdirectory(ohmtestgpu)
add_subdirectory(ohmtestheightmap)
add_subdirectory(slamiotest)
