.PHONY: build run clean

build:
	@cmake -Bbuild . 
	@cmake --build build -j$(nproc)

run: build
	./bin/optimiluca -s 100 -c 100 -i 100 -v 1

run_DL: build
	./bin/optimiluca -s 5 -c 5 -i 100 -v 1 -d 1

clean:
	@rm -rf build bin
