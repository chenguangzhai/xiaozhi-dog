# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# On Windows the expanded compile command can exceed the CreateProcess limit
# (32767 chars) for include-heavy targets. Force Ninja to pass command lines
# through response files instead.
if(WIN32)
    set(CMAKE_NINJA_FORCE_RESPONSE_FILE ON)
endif()

# Add this line to disable the specific warning
add_compile_options(-Wno-missing-field-initializers)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)

set(PROJECT_VER "2.5.0")
project(xiaozhi)
