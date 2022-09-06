.PHONY: all clean configure build install-dependencies run

-include build/conanbuildinfo.mak

all: configure build

clean:
	rm -rf bin build

configure:
	mkdir -p build
	cd build && cmake ..

build:
	cd build && make

install-dependencies:
	mkdir -p build
	cd build && conan install .. --build missing

run:
	mkdir -p output && ./build/bin/oculomatic
