.PHONY: build run clean

build:
	@cmake -Bbuild . 
	@cmake --build build -j$(nproc)

run: build
	./bin/optimiluca -s 10 --max-iters 100 --n-closures 0 -v 1

clean:
	@rm -rf build bin
