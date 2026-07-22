cmake_minimum_required(VERSION 3.16)

project(bullet3_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Ruby REQUIRED COMPONENTS Development)
find_package(PkgConfig QUIET)

execute_process(
  COMMAND ${RUBY_EXECUTABLE} -rrubygems -e "print Gem::Specification.find_by_name('rice').full_gem_path"
  OUTPUT_VARIABLE RICE_GEM_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE RICE_LOOKUP_RESULT
)

if(NOT RICE_LOOKUP_RESULT EQUAL 0)
  message(FATAL_ERROR "Rice gem is required to build bullet3")
endif()

set(BULLET_ROOT "" CACHE PATH "Path to a Bullet Physics SDK installation")
if(NOT BULLET_ROOT AND DEFINED ENV{BULLET_ROOT})
  set(BULLET_ROOT "$ENV{BULLET_ROOT}")
endif()

set(BULLET3_SEARCH_PREFIXES
  "${BULLET_ROOT}"
  "$ENV{BULLET_ROOT}"
  /opt/homebrew/opt/bullet
  /usr/local
  /usr
)
list(REMOVE_ITEM BULLET3_SEARCH_PREFIXES "")

set(BULLET3_INCLUDE_DIRS "")
set(BULLET3_LIBRARIES "")
set(BULLET3_LINK_TARGETS "")
set(BULLET3_LIBRARY_DIRS "")

find_package(Bullet QUIET)
if(Bullet_FOUND OR BULLET_FOUND)
  if(DEFINED BULLET_INCLUDE_DIRS)
    set(BULLET3_INCLUDE_DIRS ${BULLET_INCLUDE_DIRS})
  elseif(DEFINED BULLET_INCLUDE_DIR)
    set(BULLET3_INCLUDE_DIRS ${BULLET_INCLUDE_DIR})
  endif()
  set(BULLET3_LIBRARIES ${BULLET_LIBRARIES})
elseif(PkgConfig_FOUND)
  pkg_check_modules(BULLET_PC QUIET IMPORTED_TARGET bullet)
  if(BULLET_PC_FOUND)
    if(TARGET PkgConfig::BULLET_PC)
      list(APPEND BULLET3_LINK_TARGETS PkgConfig::BULLET_PC)
    else()
      set(BULLET3_INCLUDE_DIRS ${BULLET_PC_INCLUDE_DIRS})
      set(BULLET3_LIBRARIES ${BULLET_PC_LIBRARIES})
      set(BULLET3_LIBRARY_DIRS ${BULLET_PC_LIBRARY_DIRS})
    endif()
  endif()
endif()

if(NOT BULLET3_LINK_TARGETS AND NOT BULLET3_LIBRARIES)
  find_path(BULLET3_INCLUDE_DIR
    NAMES btBulletDynamicsCommon.h
    HINTS ${BULLET3_SEARCH_PREFIXES}
    PATH_SUFFIXES include/bullet include
  )

  set(BULLET3_REQUIRED_BULLET_LIBRARIES
    BulletSoftBody
    BulletDynamics
    BulletCollision
    LinearMath
  )
  set(BULLET3_MISSING_BULLET_LIBRARIES "")
  foreach(BULLET3_LIBRARY_NAME IN LISTS BULLET3_REQUIRED_BULLET_LIBRARIES)
    string(TOUPPER "${BULLET3_LIBRARY_NAME}" BULLET3_LIBRARY_VARIABLE)
    find_library(BULLET3_${BULLET3_LIBRARY_VARIABLE}_LIBRARY
      NAMES ${BULLET3_LIBRARY_NAME}
      HINTS ${BULLET3_SEARCH_PREFIXES}
      PATH_SUFFIXES lib lib64
    )
    if(BULLET3_${BULLET3_LIBRARY_VARIABLE}_LIBRARY)
      list(APPEND BULLET3_LIBRARIES ${BULLET3_${BULLET3_LIBRARY_VARIABLE}_LIBRARY})
    else()
      list(APPEND BULLET3_MISSING_BULLET_LIBRARIES ${BULLET3_LIBRARY_NAME})
    endif()
  endforeach()

  if(BULLET3_INCLUDE_DIR AND NOT BULLET3_MISSING_BULLET_LIBRARIES)
    set(BULLET3_INCLUDE_DIRS ${BULLET3_INCLUDE_DIR})
  endif()
endif()

if(NOT BULLET3_LINK_TARGETS AND (NOT BULLET3_INCLUDE_DIRS OR NOT BULLET3_LIBRARIES))
  message(FATAL_ERROR "Bullet development files are required to build bullet3. Install Bullet or set BULLET_ROOT to the installation prefix.")
endif()

set(BULLET3_OPTIONAL_SOURCES "")
set(BULLET3_OPTIONAL_INCLUDE_DIRS "")
set(BULLET3_OPTIONAL_LIBRARIES "")

set(BULLET3_EXTRAS_SOURCE_DIR "" CACHE PATH "Optional path to Bullet Extras/Serialize sources for .bullet import support")
if(NOT BULLET3_EXTRAS_SOURCE_DIR AND DEFINED ENV{BULLET3_EXTRAS_SOURCE_DIR})
  set(BULLET3_EXTRAS_SOURCE_DIR "$ENV{BULLET3_EXTRAS_SOURCE_DIR}")
endif()

if(BULLET3_EXTRAS_SOURCE_DIR)
  set(BULLET3_FILE_LOADER_DIR "${BULLET3_EXTRAS_SOURCE_DIR}/BulletFileLoader")
  set(BULLET3_WORLD_IMPORTER_DIR "${BULLET3_EXTRAS_SOURCE_DIR}/BulletWorldImporter")

  if(NOT EXISTS "${BULLET3_FILE_LOADER_DIR}/btBulletFile.cpp" OR NOT EXISTS "${BULLET3_WORLD_IMPORTER_DIR}/btBulletWorldImporter.cpp")
    message(FATAL_ERROR "BULLET3_EXTRAS_SOURCE_DIR must point to Bullet's Extras/Serialize directory")
  endif()

  list(APPEND BULLET3_OPTIONAL_SOURCES
    "${BULLET3_FILE_LOADER_DIR}/bChunk.cpp"
    "${BULLET3_FILE_LOADER_DIR}/bDNA.cpp"
    "${BULLET3_FILE_LOADER_DIR}/bFile.cpp"
    "${BULLET3_FILE_LOADER_DIR}/btBulletFile.cpp"
    "${BULLET3_WORLD_IMPORTER_DIR}/btBulletWorldImporter.cpp"
    "${BULLET3_WORLD_IMPORTER_DIR}/btWorldImporter.cpp"
  )
  list(APPEND BULLET3_OPTIONAL_INCLUDE_DIRS
    "${BULLET3_FILE_LOADER_DIR}"
    "${BULLET3_WORLD_IMPORTER_DIR}"
  )
  set(BULLET3_WITH_WORLD_IMPORTER TRUE)
else()
  find_path(BULLET3_WORLD_IMPORTER_INCLUDE_DIR
    NAMES btBulletWorldImporter.h
    HINTS ${BULLET3_SEARCH_PREFIXES}
    PATH_SUFFIXES
      include/bullet/Extras/Serialize/BulletWorldImporter
      include/Extras/Serialize/BulletWorldImporter
  )
  find_library(BULLET3_WORLD_IMPORTER_LIBRARY
    NAMES BulletWorldImporter
    HINTS ${BULLET3_SEARCH_PREFIXES}
    PATH_SUFFIXES lib lib64
  )
  find_library(BULLET3_FILE_LOADER_LIBRARY
    NAMES BulletFileLoader Bullet2FileLoader
    HINTS ${BULLET3_SEARCH_PREFIXES}
    PATH_SUFFIXES lib lib64
  )

  if(BULLET3_WORLD_IMPORTER_INCLUDE_DIR AND BULLET3_WORLD_IMPORTER_LIBRARY AND BULLET3_FILE_LOADER_LIBRARY)
    list(APPEND BULLET3_OPTIONAL_INCLUDE_DIRS ${BULLET3_WORLD_IMPORTER_INCLUDE_DIR})
    list(APPEND BULLET3_OPTIONAL_LIBRARIES ${BULLET3_WORLD_IMPORTER_LIBRARY} ${BULLET3_FILE_LOADER_LIBRARY})
    set(BULLET3_WITH_WORLD_IMPORTER TRUE)
  endif()
endif()

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
  ${BULLET3_OPTIONAL_SOURCES}
)

target_include_directories(bullet3 PRIVATE
  ${BULLET3_INCLUDE_DIRS}
  ${BULLET3_OPTIONAL_INCLUDE_DIRS}
)

target_include_directories(bullet3 SYSTEM PRIVATE
  ${Ruby_INCLUDE_DIRS}
  "${RICE_GEM_PATH}/include"
)

set(BULLET3_RUBY_LIBRARIES "")
foreach(RUBY_LIBRARY IN LISTS Ruby_LIBRARIES)
  if(RUBY_LIBRARY AND NOT RUBY_LIBRARY MATCHES "-NOTFOUND$")
    list(APPEND BULLET3_RUBY_LIBRARIES ${RUBY_LIBRARY})
  endif()
endforeach()

if(BULLET3_LIBRARY_DIRS)
  target_link_directories(bullet3 PRIVATE ${BULLET3_LIBRARY_DIRS})
endif()

target_link_libraries(bullet3 PRIVATE
  ${BULLET3_RUBY_LIBRARIES}
  ${BULLET3_LINK_TARGETS}
  ${BULLET3_LIBRARIES}
  ${BULLET3_OPTIONAL_LIBRARIES}
)

if(APPLE)
  target_link_options(bullet3 PRIVATE -undefined dynamic_lookup)
endif()

if(BULLET3_WITH_WORLD_IMPORTER)
  target_compile_definitions(bullet3 PRIVATE BULLET3_WITH_WORLD_IMPORTER)
endif()

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
