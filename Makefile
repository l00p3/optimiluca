.PHONY: build run clean

build:
	@cmake -Bbuild . 
	@cmake --build build -j$(nproc)

run: build
	./build/src/apps/optimiluca -s 5 --max-iters 1000 -v 2

clean:
	@rm -rf build
