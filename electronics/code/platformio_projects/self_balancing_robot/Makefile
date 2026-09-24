PIO ?= $(HOME)/.platformio/penv/bin/pio
CXX ?= g++

SHARED_INCLUDE := ../../esp_idf_projects/hello_world/main/include
READ_SERIAL_SOURCE := src/read_serial.cpp
READ_SERIAL_HEADER := $(SHARED_INCLUDE)/data_structs.h
READ_SERIAL_BINARY := .pio/read_serial

.PHONY: all firmware read_serial upload run-reader

all: firmware read_serial

firmware:
	$(PIO) run

upload:
	$(PIO) run --target upload

read_serial: $(READ_SERIAL_BINARY)

$(READ_SERIAL_BINARY): $(READ_SERIAL_SOURCE) $(READ_SERIAL_HEADER)
	@mkdir -p $(dir $@)
	$(CXX) -std=c++20 -o $@ -I$(SHARED_INCLUDE) $<

run-reader: read_serial
	$(READ_SERIAL_BINARY)
