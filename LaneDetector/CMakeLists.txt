FIND_PACKAGE( OpenCV REQUIRED )
add_executable(lanes demo.cpp)
TARGET_LINK_LIBRARIES( lanes ${OpenCV_LIBS} )
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
