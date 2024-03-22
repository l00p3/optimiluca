.PHONY: build run clean

build:
	@cmake -Bbuild . 
	@cmake --build build -j$(nproc)

run_GN: build
	./bin/optimiluca -s 80 -c 0 -i 10000 -v 1 -d 0

run_DL: build
	./bin/optimiluca -s 100 -c 100 -i 100 -v 1 -d 1

clean:
	@rm -rf build bin
