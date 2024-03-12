.PHONY: build run clean

build:
	@cmake -Bbuild . 
	@cmake --build build -j$(nproc)

run: build
	./bin/optimiluca -s 500 --max-iters 1000 --n-closures 50 -v 1

clean:
	@rm -rf build bin
