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
set(SRC_DIR "${CODE_DIR}/src")
set(INCLUDE_DIR "${CODE_DIR}/include")
set(OUTPUT_DIR "${CMAKE_CURRENT_LIST_DIR}/build")

function(resolve_bodybasics_file out_var filename)
    set(_candidates
        "${CODE_DIR}/${filename}"
        "${SRC_DIR}/${filename}"
        "${INCLUDE_DIR}/${filename}"
    )

    foreach(_candidate IN LISTS _candidates)
        if(EXISTS "${_candidate}")
            set(${out_var} "${_candidate}" PARENT_SCOPE)
            return()
        endif()
    endforeach()

    if(ARGC GREATER 2)
        foreach(_alt IN LISTS ARGN)
            set(_alt_candidates
                "${CODE_DIR}/${_alt}"
                "${SRC_DIR}/${_alt}"
                "${INCLUDE_DIR}/${_alt}"
            )

            foreach(_candidate IN LISTS _alt_candidates)
                if(EXISTS "${_candidate}")
                    set(${out_var} "${_candidate}" PARENT_SCOPE)
                    return()
                endif()
            endforeach()
        endforeach()
    endif()

    message(FATAL_ERROR "Required file not found under code/, code/src, or code/include: ${filename}")
endfunction()

resolve_bodybasics_file(BODYBASICS_CPP "BodyBasics.cpp" "body_basics.cpp" "data.cpp")
resolve_bodybasics_file(MAIN_CPP "main.cpp")
resolve_bodybasics_file(TRACKING_RENDERER_CPP "tracking_renderer.cpp" "renderer.cpp")
resolve_bodybasics_file(TRACKING_STORAGE_CPP "storage.cpp")
resolve_bodybasics_file(BODYBASICS_H "BodyBasics.h")
resolve_bodybasics_file(RESOURCE_H "resource.h")
resolve_bodybasics_file(STDAFX_H "stdafx.h")
resolve_bodybasics_file(TRACKING_RENDERER_H "tracking_renderer.h")
resolve_bodybasics_file(TRACKING_STORAGE_H "tracking_storage.h")

set(BODYBASICS_SOURCES
    "${MAIN_CPP}"
    "${BODYBASICS_CPP}"
    "${TRACKING_RENDERER_CPP}"
    "${TRACKING_STORAGE_CPP}"
)

set(BODYBASICS_HEADERS
    "${BODYBASICS_H}"
    "${RESOURCE_H}"
    "${STDAFX_H}"
    "${TRACKING_RENDERER_H}"
    "${TRACKING_STORAGE_H}"
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
