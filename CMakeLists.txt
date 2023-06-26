cmake_minimum_required (VERSION 3.16)
project (sfml-app)

set(CMAKE_CXX_STANDARD 11)

# Add the SFML library
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Add the source files
set(SOURCES
    geometry.h
    mapRRT.cpp
)



find_package(OpenCV REQUIRED)


include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( . )

add_executable (${PROJECT_NAME} ${SOURCES} )



# Link with the SFML and OpenCV libraries

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${OpenCV_LIBS}
    sfml-graphics
    sfml-window
    sfml-system
    
)


