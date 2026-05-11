BUILD_DIR := build
TEMP_DIR := temp
CMAKE := cmake
CONFIG ?= Debug
GENERATOR ?= NMake Makefiles
PLATFORM ?= x64
VSDEVCMD ?= C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat
TARGET_EXE := tracking.exe
PYTHON ?= python
PREPROCESS_SCRIPT := preprocess.py
VISUALIZE_SCRIPT := visualize.py
MODEL_SCRIPT := model.py
EVALUATE_SCRIPT := evaluate.py
INDEX ?= 0

# Avoid passing GNU make flags from mingw32-make into NMake through CMake.
unexport MAKEFLAGS
unexport MFLAGS

ifeq ($(PLATFORM),x64)
VS_ARCH := amd64
else ifeq ($(PLATFORM),Win32)
VS_ARCH := x86
else
VS_ARCH := $(PLATFORM)
endif

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
	@echo   PLATFORM=x64            Visual Studio platform

all: help

configure:
	@if not exist "$(TEMP_DIR)" mkdir "$(TEMP_DIR)"
	cmd.exe /c call "$(VSDEVCMD)" -arch=$(VS_ARCH) "&&" $(CMAKE) -S . -B $(TEMP_DIR) -G "$(GENERATOR)"

build: configure
	cmd.exe /c call "$(VSDEVCMD)" -arch=$(VS_ARCH) "&&" $(CMAKE) --build $(TEMP_DIR) --config $(CONFIG)
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
