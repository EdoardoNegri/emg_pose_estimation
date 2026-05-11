BUILD_DIR := build
TEMP_DIR := temp
POWERSHELL := powershell
VSWHERE := C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe
CMAKE := $(shell "$(POWERSHELL)" -NoProfile -Command "$$paths = @('C:\Program Files\CMake\bin\cmake.exe'); $$vsCMake = Get-ChildItem 'C:\Program Files\Microsoft Visual Studio' -Filter cmake.exe -Recurse -ErrorAction SilentlyContinue | Where-Object { $$_.FullName -like '*CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' } | Select-Object -First 1 -ExpandProperty FullName; if ($$vsCMake) { $$paths += $$vsCMake }; $$paths | Where-Object { Test-Path $$_ } | Select-Object -First 1")
CONFIG ?= Debug
GENERATOR ?= $(shell "$(POWERSHELL)" -NoProfile -Command "if (Test-Path '$(VSWHERE)') { $$instance = & '$(VSWHERE)' -all -format json | ConvertFrom-Json | Where-Object { $$_.isComplete -and $$_.isLaunchable } | Select-Object -First 1; if ($$instance) { $$year = [regex]::Match($$instance.displayName, '20\d{2}').Value; if ($$year) { 'Visual Studio ' + $$instance.catalog.productLineVersion + ' ' + $$year } } }")
PLATFORM ?= x64
TARGET_EXE := tracking.exe
PYTHON ?= python
PREPROCESS_SCRIPT := preprocess.py
VISUALIZE_SCRIPT := visualize.py
MODEL_SCRIPT := model.py
EVALUATE_SCRIPT := evaluate.py
INDEX ?= 0

ifdef GENERATOR
ifneq ($(strip $(GENERATOR)),)
CMAKE_GENERATOR_ARG := -G "$(GENERATOR)"
endif
endif

ifdef PLATFORM
ifneq ($(strip $(PLATFORM)),)
CMAKE_PLATFORM_ARG := -A $(PLATFORM)
endif
endif

# Avoid passing GNU make flags from mingw32-make into NMake through CMake.
unexport MAKEFLAGS
unexport MFLAGS

.PHONY: help all configure build rebuild clean run-app run-preprocess run-visualize run-model run-evaluate

help:
	@echo Available targets:
	@echo   make build              Configure and build the C++ app
	@echo   make rebuild            Clean, configure, and build
	@echo   make clean              Remove build and temp directories
	@echo   make run-app            Run $(BUILD_DIR)\$(TARGET_EXE)
	@echo   make run-preprocess     Run $(PREPROCESS_SCRIPT) with INDEX=$(INDEX)
	@echo   make run-visualize      Run $(VISUALIZE_SCRIPT) with INDEX=$(INDEX)
	@echo   make run-model          Run $(MODEL_SCRIPT) with INDEX=$(INDEX)
	@echo   make run-evaluate       Evaluate prediction against raw ground truth with INDEX=$(INDEX)
	@echo.
	@echo Variables:
	@echo   INDEX=0                 Override input index, for example: make run-model INDEX=3
	@echo   CONFIG=Debug            Build config
	@echo   GENERATOR=...           Optional generator override, defaults to installed Visual Studio
	@echo   PLATFORM=x64            Generator platform, default: x64
	@echo.
	@echo Examples:
	@echo   mingw32-make build
	@echo   mingw32-make build GENERATOR="Visual Studio 18 2026" PLATFORM=x64
	@echo   mingw32-make build GENERATOR=Ninja

all: help

configure:
	@if "$(CMAKE)"=="" (echo Could not find a Windows-native cmake.exe. Install CMake for Windows or Visual Studio CMake tools.& exit /b 1)
	@if "$(GENERATOR)"=="" (echo Could not detect an installed Visual Studio generator. Set GENERATOR explicitly or install Visual Studio.& exit /b 1)
	@if not exist "$(TEMP_DIR)" mkdir "$(TEMP_DIR)"
	$(CMAKE) -S . -B $(TEMP_DIR) $(CMAKE_GENERATOR_ARG) $(CMAKE_PLATFORM_ARG)

build: configure
	$(CMAKE) --build $(TEMP_DIR) --config $(CONFIG)
	@if exist "$(TEMP_DIR)" rmdir /S /Q "$(TEMP_DIR)"

rebuild: clean build

clean:
	@if exist "$(BUILD_DIR)" rmdir /S /Q "$(BUILD_DIR)"
	@if exist "$(TEMP_DIR)" rmdir /S /Q "$(TEMP_DIR)"

run-app:
	$(BUILD_DIR)\$(TARGET_EXE)

run-preprocess:
	$(PYTHON) $(PREPROCESS_SCRIPT) $(INDEX)


run-visualize:
	$(PYTHON) $(VISUALIZE_SCRIPT) $(INDEX)

run-model:
	$(PYTHON) $(MODEL_SCRIPT) $(INDEX)

run-evaluate:
	$(PYTHON) $(EVALUATE_SCRIPT) $(INDEX)
