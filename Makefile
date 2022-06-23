# This is under MIT licence
# Also, I am not at all proud of this makefile, feel free to make better

all:
	g++ -o smallvcm ./src/smallvcm.cpp -O3 -std=c++14 -fopenmp

oldrng:
	g++ -o smallvcm ./src/smallvcm.cpp -O3 -std=c++14 -fopenmp -DLEGACY_RNG

clean:
	rm smallvcm

unreport:
	rm *.bmp index.html
