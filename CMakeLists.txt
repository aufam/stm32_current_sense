file(READ version.txt CURRENT_SENSE_VERSION)
message("CURRENT_SENSE_VERSION : ${CURRENT_SENSE_VERSION}")

# sources
file(GLOB_RECURSE CURRENT_SENSE_SOURCES current_sense/*.*)
add_library(current_sense ${CURRENT_SENSE_SOURCES})

# include dirs
target_include_directories(current_sense PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# defines
target_compile_definitions(current_sense PUBLIC -DCURRENT_SENSE_VERSION="${CURRENT_SENSE_VERSION}")

# depends
target_link_libraries(current_sense etl)
target_link_libraries(current_sense periph)