.PHONY: build run

build:
	@cmake -Bbuild . 
	@cmake --build build -j$(nproc)

run: build
	./build/src/apps/optimiluca
