.PHONY: build run clean

build:
	@cmake -Bbuild . 
	@cmake --build build -j$(nproc)

run_GN: build
	./bin/optimiluca -s 500 -c 500 -i 1000 -v 1 -d 0

run_DL: build
	./bin/optimiluca -s 500 -c 500 -i 1000 -v 1 -d 1

clean:
	@rm -rf build bin
