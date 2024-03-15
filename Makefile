.PHONY: build run clean

build:
	@cmake -Bbuild . 
	@cmake --build build -j$(nproc)

run: build
	./bin/optimiluca -s 1000 -c 500 -i 100 -v 1

clean:
	@rm -rf build bin
