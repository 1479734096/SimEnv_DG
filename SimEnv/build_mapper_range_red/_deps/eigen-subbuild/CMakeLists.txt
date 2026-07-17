# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.16.3)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(eigen-populate NONE)

include(ExternalProject)
ExternalProject_Add(eigen-populate
                     "UPDATE_DISCONNECTED" "False" "URL" "https://github.com/nachovizzo/eigen/archive/refs/tags/3.4.90.tar.gz"
                    SOURCE_DIR          "/home/xbh/tzb26/codeDG/SimEnv/build_mapper_range_red/_deps/eigen-src"
                    BINARY_DIR          "/home/xbh/tzb26/codeDG/SimEnv/build_mapper_range_red/_deps/eigen-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)
