.PHONY: build run clean

build:
	@cmake -Bbuild . 
	@cmake --build build -j$(nproc)

run: build
	./build/src/apps/optimiluca -s 4 --max-iters 10 --verbose 1

clean:
	@rm -rf build
