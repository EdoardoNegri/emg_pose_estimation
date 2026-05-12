cmake_minimum_required(VERSION 3.20)

project(BodyBasicsD2D LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(WIN32 AND NOT MSVC)
    message(FATAL_ERROR
        "This Kinect v2 application must be built with the Microsoft Visual C++ toolchain. "
        "CMake selected '${CMAKE_CXX_COMPILER_ID}' instead. "
        "Configure with a Visual Studio generator, for example: "
        "cmake -S . -B temp -G \"Visual Studio 17 2022\" -A x64")
endif()

set(CODE_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(CAPTURE_SRC_DIR "${CODE_DIR}/capture_data/src")
set(INCLUDE_DIR "${CODE_DIR}/include")
set(OUTPUT_DIR "${CMAKE_CURRENT_LIST_DIR}/capture_data/build")

set(BODYBASICS_SOURCES
    "${CAPTURE_SRC_DIR}/main.cpp"
    "${CAPTURE_SRC_DIR}/data.cpp"
    "${CAPTURE_SRC_DIR}/renderer.cpp"
    "${CAPTURE_SRC_DIR}/storage.cpp"
)

set(BODYBASICS_HEADERS
    "${INCLUDE_DIR}/BodyBasics.h"
    "${INCLUDE_DIR}/resource.h"
    "${INCLUDE_DIR}/stdafx.h"
    "${INCLUDE_DIR}/tracking_renderer.h"
    "${INCLUDE_DIR}/tracking_storage.h"
)

add_executable(tracking WIN32
    ${BODYBASICS_SOURCES}
    ${BODYBASICS_HEADERS}
)

target_include_directories(tracking PRIVATE
    "${INCLUDE_DIR}"
    "${CODE_DIR}"
    "$ENV{KINECTSDK20_DIR}/inc"
)

target_link_directories(tracking PRIVATE
    "$<$<EQUAL:${CMAKE_SIZEOF_VOID_P},8>:$ENV{KINECTSDK20_DIR}/lib/x64>"
    "$<$<EQUAL:${CMAKE_SIZEOF_VOID_P},4>:$ENV{KINECTSDK20_DIR}/lib/x86>"
)

target_compile_definitions(tracking PRIVATE
    WIN32
    _WINDOWS
    UNICODE
    _UNICODE
    $<$<CONFIG:Debug>:_DEBUG>
    $<$<NOT:$<CONFIG:Debug>>:NDEBUG>
    ENABLE_RENDERING
    ENABLE_RECORDING
)

target_link_libraries(tracking PRIVATE
    kinect20
    kernel32
    user32
    gdi32
    winspool
    comdlg32
    advapi32
    shell32
    ole32
    oleaut32
    uuid
    odbc32
    odbccp32
)

set_target_properties(tracking PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${OUTPUT_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${OUTPUT_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${OUTPUT_DIR}"
)
