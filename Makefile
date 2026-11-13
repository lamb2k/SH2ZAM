.PHONY: all test clean
BUILD ?= build
all:
	cmake -S . -B $(BUILD) -DCMAKE_BUILD_TYPE=Release
	cmake --build $(BUILD) --parallel
test: all
	ctest --test-dir $(BUILD) --output-on-failure
clean:
	cmake --build $(BUILD) --target clean
