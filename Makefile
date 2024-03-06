.PHONY: build run clean

build:
	@cmake -Bbuild . 
	@cmake --build build -j$(nproc)

run: build
	./build/src/apps/optimiluca -s 100 --max-iters 100000 --verbose 1

clean:
	@rm -rf build
