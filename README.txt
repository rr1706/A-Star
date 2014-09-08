A* implementation in C++
 by Connor Monahan

This can be used potentially as a library in other software written for
computer vision purposes in the future. It can also be used from the
command line, taking options as arguments, obstacles on stdin, and the
final path on stdout.

To run the command line application, use the following command:
 astar -s 25 2,1 24,24 < in > out
where 'in' is a list of obstacles and 'out' is the output file.

To generate graphs from the output, use the following command:
 graph -Tpng out --reposition 0 0 1 -S 8 -m 0 --blankout 0 in | display -
where 'out' is the result of running astar and 'in' is the list of
obstacles provided to astar on stdin.

Format of obstacles and output is a list of points. Each point is a pair of
two integers x and y. Each pair is separated by a newline.
Example:
 1 1
 2 3
 5 5

Format of 'start' and 'target' is a pair of points delimited by a comma.
Example:
 2,5

Optimizing the library:

By default CMake compiles in debug mode, which is slow because the intended
purpose is testing. If you want to build the library for use in another
application and it is working correctly, you can run another command to create
a faster library.

I have found clang to be the best compiler for the purpose of optimization.
The process I follow to do this is follows, assuming that the CMakeFiles are
in the current directory.

make
clang++ -c -o CMakeFiles/astar.dir/astar.cpp.o -std=c++11 -O3 -Wall astar.cpp
make

