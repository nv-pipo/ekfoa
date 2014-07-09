cmake_minimum_required(VERSION 2.4.6)
project (ekfoa)

if(COMMAND cmake_policy)
   cmake_policy(SET CMP0003 NEW) #Do not produce search linker paths for libraries linked via full path
   cmake_policy(SET CMP0017 NEW) #Prefer calls include() or find_package() from the CMake module directory over CMAKE_MODULE_PATH
endif(COMMAND cmake_policy)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH bin)

#### Extra flags ####
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GNU_SOURCE -D_REENTRANT -D_EIGEN_MPL2_ONLY -D_NDEBUG -O3 -fopenmp -fstack-protector -frounding-math")

#### PTHREADS ####
find_package (Threads)

#### BOOST ####
find_package( Boost 1.49 COMPONENTS thread system REQUIRED )
include_directories(${Boost_INCLUDE_DIRS})

#### OpenCV ####
find_package(OpenCV REQUIRED)

#### CGAL ####
find_package(CGAL REQUIRED)

#Add path to FindGMP.cmake and FindMPFR.cmake:
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CGAL_MODULES_DIR})
find_package( GMP REQUIRED )
find_package( MPFR REQUIRED )

#### EIGEN #### 
# the CGAL_MODULES_DIR should contain FindEigen3.cmake
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#### OpenMP #### 
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


#### GLFW ####
find_package(X11)
find_package(OpenGL)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

include_directories(${X11_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLFW_INCLUDE_DIRS})

#### ARM Optimizations ####
if (CMAKE_SYSTEM_PROCESSOR MATCHES "armv7l")
   set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__ARM_NEON__ -fPIC -march=armv7-a -mfloat-abi=hard -mfpu=neon -ftree-vectorize -mvectorize-with-neon-quad -mcpu=cortex-a9 -mtune=cortex-a9")
endif()

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

add_executable(ekfoa src/main.cpp src/gui.cpp src/opengl_utils/arcball.cpp src/ekfoa.cpp src/camera.cpp src/feature.cpp src/kalman.cpp src/motion_model.cpp src/motion_tracker_of.cpp src/print.cpp)
target_link_libraries(ekfoa ${CGAL_LIBRARY} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${OPENGL_glu_LIBRARY} ${GLFW_STATIC_LIBRARIES})