.PHONY: build run clean

build:
	@cmake -Bbuild . 
	@cmake --build build -j$(nproc)

run: build
	./build/src/apps/optimiluca

clean:
	@rm -rf build
