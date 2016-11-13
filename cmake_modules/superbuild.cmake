include(ExternalProject)

# Base directory for storing components of external projects
set_property(DIRECTORY PROPERTY EP_BASE Dependencies)

set(EIGEN_VERSION "3.2.9")
set(EIGEN_MD5 "de11bfbfe2fd2dc4b32e8f416f58ee98")
ExternalProject_add(ep_eigen
    URL "http://bitbucket.org/eigen/eigen/get/${EIGEN_VERSION}.tar.bz2"
    URL_MD5 "${EIGEN_MD5}"
    CMAKE_ARGS
        "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
        "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
        -DEIGEN_TEST_NO_OPENGL=ON
)

ExternalProject_add(ep_g2o
    SOURCE_DIR "${PROJECT_SOURCE_DIR}/Thirdparty/g2o"
    CMAKE_ARGS
        "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
        "-DCMAKE_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/Dependencies/Install/ep_eigen"
    INSTALL_COMMAND ""
    DEPENDS ep_eigen
)

ExternalProject_add(ep_orbslam2
        SOURCE_DIR "${PROJECT_SOURCE_DIR}"
        CMAKE_ARGS
        "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
        "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
        -DDO_SUPERBUILD=OFF
        "-DDEPENDENCIES_PATH=${CMAKE_CURRENT_BINARY_DIR}/Dependencies"
        INSTALL_COMMAND ""
        DEPENDS ep_eigen ep_g2o
        )
