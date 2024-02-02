# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(MyCProject)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)  # You can change 11 to 14, 17, or 20 depending on your requirements
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find Eigen3 package
find_package(Eigen3 REQUIRED)
# Check if the Eigen3 package was found
if(Eigen3_FOUND)
    # Include directories for Eigen3
    include_directories(${Eigen3_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "Eigen3 not found. Please install it or provide the correct path.")
endif()

# Create a subdirectory for the executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Create the executable from source files
# add_executable(test_main 
# 	src/test_main.cpp 
# 	src/OdeSolver.cpp
# 	src/OdeSolarsail.cpp
# 	src/SetRRT.cpp
# 	src/MyODE.cpp
# 	src/myownlibrary.cpp
# 	src/Graph.cpp
# 	src/MySetRRT.cpp
# )

# Create the executable from source files
add_executable(test_setrrt 
	src/test_setrrt.cpp
	src/OdeVirtual.cpp
	src/OdeSolver.cpp
	src/OdeSolarsail.cpp
	src/PlannerVirtual.cpp
	src/Graph.cpp
	src/SetRRT.cpp
)


# Link the Eigen3 library
#target_link_libraries(test_main Eigen3::Eigen)

# Link the Eigen3 library
target_link_libraries(test_setrrt Eigen3::Eigen)
