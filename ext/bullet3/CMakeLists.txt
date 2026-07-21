cmake_minimum_required(VERSION 3.16)

project(bullet3_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Ruby REQUIRED COMPONENTS Development)

execute_process(
  COMMAND ${RUBY_EXECUTABLE} -rrubygems -e "print Gem::Specification.find_by_name('rice').full_gem_path"
  OUTPUT_VARIABLE RICE_GEM_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE RICE_LOOKUP_RESULT
)

if(NOT RICE_LOOKUP_RESULT EQUAL 0)
  message(FATAL_ERROR "Rice gem is required to build bullet3")
endif()

set(BULLET3_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/bullet3")
if(NOT EXISTS "${BULLET3_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "vendor/bullet3 is missing. Run: git submodule update --init --recursive")
endif()

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_CPU_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_EXTRAS OFF CACHE BOOL "" FORCE)
set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_PYBULLET OFF CACHE BOOL "" FORCE)
set(BUILD_ENET OFF CACHE BOOL "" FORCE)
set(BUILD_CLSOCKET OFF CACHE BOOL "" FORCE)
set(USE_GLUT OFF CACHE BOOL "" FORCE)
set(INSTALL_LIBS OFF CACHE BOOL "" FORCE)

add_subdirectory("${BULLET3_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/bullet3")

add_library(bullet3 MODULE
  bullet3.cpp
  collision/rb_collision.cpp
  collision/shapes/rb_collision_shape.cpp
  constraints/rb_constraints.cpp
  dynamics/rb_dynamics.cpp
  io/rb_io.cpp
  linear_math/rb_matrix3x3.cpp
  linear_math/rb_quaternion.cpp
  linear_math/rb_transform.cpp
  linear_math/rb_vector3.cpp
  multi_body/rb_multi_body.cpp
  soft_body/rb_soft_body.cpp
  util/rb_debug_draw.cpp
  vehicle/rb_vehicle.cpp
  "${BULLET3_SOURCE_DIR}/Extras/Serialize/BulletFileLoader/bChunk.cpp"
  "${BULLET3_SOURCE_DIR}/Extras/Serialize/BulletFileLoader/bDNA.cpp"
  "${BULLET3_SOURCE_DIR}/Extras/Serialize/BulletFileLoader/bFile.cpp"
  "${BULLET3_SOURCE_DIR}/Extras/Serialize/BulletFileLoader/btBulletFile.cpp"
  "${BULLET3_SOURCE_DIR}/Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp"
  "${BULLET3_SOURCE_DIR}/Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp"
)

target_include_directories(bullet3 PRIVATE
  "${BULLET3_SOURCE_DIR}/src"
  "${BULLET3_SOURCE_DIR}/Extras/Serialize/BulletFileLoader"
  "${BULLET3_SOURCE_DIR}/Extras/Serialize/BulletWorldImporter"
)

target_include_directories(bullet3 SYSTEM PRIVATE
  ${Ruby_INCLUDE_DIRS}
  "${RICE_GEM_PATH}/include"
)

target_link_libraries(bullet3 PRIVATE
  ${Ruby_LIBRARIES}
  BulletSoftBody
  BulletDynamics
  BulletCollision
  LinearMath
)

if(MSVC)
  target_compile_options(bullet3 PRIVATE /W4)
else()
  target_compile_options(bullet3 PRIVATE -Wall -Wextra)
endif()

set_target_properties(bullet3 PROPERTIES
  PREFIX ""
  SUFFIX "${RUBY_EXTENSION_SUFFIX}"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)
