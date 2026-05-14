CXX_APP_DIR := capture_data
BUILD_DIR := $(CXX_APP_DIR)\build
RUN_APP_DIR := run_data
RUN_BUILD_DIR := $(RUN_APP_DIR)\build
TEMP_DIR := temp
POWERSHELL := powershell
VSWHERE := C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe
CMAKE := $(shell "$(POWERSHELL)" -NoProfile -Command "$$paths = @('C:\Program Files\CMake\bin\cmake.exe'); $$vsCMake = Get-ChildItem 'C:\Program Files\Microsoft Visual Studio' -Filter cmake.exe -Recurse -ErrorAction SilentlyContinue | Where-Object { $$_.FullName -like '*CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' } | Select-Object -First 1 -ExpandProperty FullName; if ($$vsCMake) { $$paths += $$vsCMake }; $$paths | Where-Object { Test-Path $$_ } | Select-Object -First 1")
CONFIG ?= Debug
GENERATOR ?= $(shell "$(POWERSHELL)" -NoProfile -Command "if (Test-Path '$(VSWHERE)') { $$instance = & '$(VSWHERE)' -all -products * -format json | ConvertFrom-Json | Where-Object { $$_.isComplete -and $$_.isLaunchable } | Select-Object -First 1; if ($$instance) { $$major = $$instance.installationVersion.Split('.')[0]; $$year = [regex]::Match($$instance.catalog.productLineVersion, '20\d{2}').Value; if (-not $$year) { $$year = [regex]::Match($$instance.displayName, '20\d{2}').Value }; if ($$major -and $$year) { 'Visual Studio ' + $$major + ' ' + $$year } } }")
PLATFORM ?= x64
MSVC_ON_PATH := $(shell "$(POWERSHELL)" -NoProfile -Command "if (Get-Command cl.exe -ErrorAction SilentlyContinue) { 'yes' }")
IS_VISUAL_STUDIO_GENERATOR := $(findstring Visual Studio,$(GENERATOR))
TARGET_EXE := tracking.exe
RUN_TARGET_EXE := run_data_app.exe
PYTHON ?= python
PREPROCESS_SCRIPT := process_data\preprocess.py
VISUALIZE_SCRIPT := tests\visualize.py
MODEL_SCRIPT := process_data\model.py
EVALUATE_SCRIPT := tests\evaluate.py
INDEX ?= 0

ifdef GENERATOR
ifneq ($(strip $(GENERATOR)),)
CMAKE_GENERATOR_ARG := -G "$(GENERATOR)"
endif
endif

ifdef PLATFORM
ifneq ($(strip $(PLATFORM)),)
ifneq (,$(IS_VISUAL_STUDIO_GENERATOR))
CMAKE_PLATFORM_ARG := -A $(PLATFORM)
endif
endif
endif

# Avoid passing GNU make flags from mingw32-make into NMake through CMake.
unexport MAKEFLAGS
unexport MFLAGS

.PHONY: help all configure build rebuild clean run-app run-capture run-preprocess run-model run-visualization run-evaluate

help:
	@echo Available targets:
	@echo   make build              Configure and build the C++ capture app
	@echo   make rebuild            Clean, configure, and build
	@echo   make clean              Remove build and temp directories
	@echo   make run-capture        Run $(BUILD_DIR)\$(TARGET_EXE)
	@echo   make run-app            Run $(RUN_BUILD_DIR)\$(RUN_TARGET_EXE)
	@echo   make run-preprocess     Run $(PREPROCESS_SCRIPT) with INDEX=$(INDEX)
	@echo   make run-model          Run $(MODEL_SCRIPT) with INDEX=$(INDEX)
	@echo   make run-visualization  Run $(VISUALIZE_SCRIPT) with INDEX=$(INDEX)
	@echo   make run-evaluate       Evaluate prediction against raw ground truth with INDEX=$(INDEX)
	@echo.
	@echo Variables:
	@echo   INDEX=0                 Override input index, for example: make run-model INDEX=3
	@echo   CONFIG=Debug            Build config
	@echo   GENERATOR=...           Optional generator override, defaults to installed Visual Studio or CMake default
	@echo   PLATFORM=x64            Visual Studio generator platform, default: x64
	@echo.
	@echo Examples:
	@echo   mingw32-make build
	@echo   mingw32-make build GENERATOR="Visual Studio 17 2022" PLATFORM=x64
	@echo   mingw32-make build GENERATOR=Ninja PLATFORM=

all: help

configure:
	@if "$(CMAKE)"=="" (echo Could not find a Windows-native cmake.exe. Install CMake for Windows or Visual Studio CMake tools.& exit /b 1)
	@if "$(GENERATOR)"=="" if "$(MSVC_ON_PATH)"=="" (echo Could not detect an installed Visual Studio generator or cl.exe on PATH. Install Visual Studio Build Tools with Desktop development with C++, or run this from a Developer PowerShell and set GENERATOR=Ninja PLATFORM=.& exit /b 1)
	@if not "$(GENERATOR)"=="" if "$(IS_VISUAL_STUDIO_GENERATOR)"=="" if "$(MSVC_ON_PATH)"=="" (echo Generator "$(GENERATOR)" requires cl.exe on PATH for this project. Run from a Developer PowerShell or use an installed Visual Studio generator.& exit /b 1)
	@if not exist "$(TEMP_DIR)" mkdir "$(TEMP_DIR)"
	$(CMAKE) -S . -B $(TEMP_DIR) $(CMAKE_GENERATOR_ARG) $(CMAKE_PLATFORM_ARG)

build: configure
	$(CMAKE) --build $(TEMP_DIR) --config $(CONFIG)
	@if exist "$(TEMP_DIR)" rmdir /S /Q "$(TEMP_DIR)"

rebuild: clean build

clean:
	@if exist "$(BUILD_DIR)" rmdir /S /Q "$(BUILD_DIR)"
	@if exist "$(RUN_BUILD_DIR)" rmdir /S /Q "$(RUN_BUILD_DIR)"
	@if exist "$(TEMP_DIR)" rmdir /S /Q "$(TEMP_DIR)"

run-capture:
	$(BUILD_DIR)\$(TARGET_EXE)

run-app:
	$(RUN_BUILD_DIR)\$(RUN_TARGET_EXE)

run-preprocess:
	$(PYTHON) $(PREPROCESS_SCRIPT) $(INDEX)

run-model:
	$(PYTHON) $(MODEL_SCRIPT) $(INDEX)

run-visualization:
	$(PYTHON) $(VISUALIZE_SCRIPT) $(INDEX)

run-evaluate:
	$(PYTHON) $(EVALUATE_SCRIPT) $(INDEX)
