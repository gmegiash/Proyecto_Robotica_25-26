# Sources set
LIST(APPEND SOURCES
    ../src/specificworker.cpp
    ../src/hungarian.cpp
    ../src/ransac_line_detector.cpp
    ../src/room_detector.cpp
)

# Headers set
LIST(APPEND HEADERS
    ../src/specificworker.h
)

LIST(APPEND LIBS ${LIBS} tbb)

SET (LIBS ${LIBS} tbb Qt6PrintSupport)

INCLUDE($ENV{ROBOCOMP}/cmake/modules/opencv4.cmake)

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-O4 -fmax-errors=1 -fno-char8_t)


