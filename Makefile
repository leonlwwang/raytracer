.PHONEY: build, run

build: program

run: program
	./program $(file)

program: main.cpp raytracer.cpp lodepng.cpp
	clang++ -std=c++17 -O3 -I. main.cpp raytracer.cpp lodepng.cpp -o program