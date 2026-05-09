BUILD_DIR := build
TEMP_DIR := temp
CMAKE := cmake
CONFIG ?= Debug
GENERATOR ?= NMake Makefiles
PLATFORM ?= x64
VSDEVCMD ?= C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat
TARGET_EXE := tracking.exe

ifeq ($(PLATFORM),x64)
VS_ARCH := amd64
else ifeq ($(PLATFORM),Win32)
VS_ARCH := x86
else
VS_ARCH := $(PLATFORM)
endif

.PHONY: all configure build rebuild clean run

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

run:
	$(BUILD_DIR)\$(TARGET_EXE)
