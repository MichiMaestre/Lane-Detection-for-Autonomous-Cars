set(GTEST_SHUFFLE 1)
FIND_PACKAGE( OpenCV REQUIRED )
add_executable(
    lanes-test
    main.cpp
    test.cpp
)

target_include_directories(lanes-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(lanes-test PUBLIC gtest ${OpenCV_LIBS})

include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
