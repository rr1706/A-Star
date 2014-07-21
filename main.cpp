#include <iostream>
#include <unistd.h>
#include <err.h>
#include "astar.h"

using namespace std;

const char *program_name;

void usage(void)
{
    fprintf(stderr, "usage: %s [-s size] startx,y targetx,y\n", program_name);
    exit(1);
}

void test_astar();

int main(int argc, char **argv)
{
    test_astar();
    return 0;
    int size, ch;
    program_name = argv[0];
    size = 10;
    while ((ch = getopt(argc, argv, "s:")) != -1) {
        switch (ch) {
            case 's':
                sscanf(optarg, "%d", &size);
                break;
            case '?':
                warn("illegal option -- %c", ch);
            default:
                usage();
        }
    }
    argc -= optind;
    argv += optind;
    if (argc != 2)
        usage();
    Point start, target;
    sscanf(argv[0], "%d,%d", &start.x, &start.y);
    sscanf(argv[0], "%d,%d", &target.x, &target.y);
    Path obstacles;
    Point current;
    while ((ch = scanf("%d %d", &current.x, &current.y)) != EOF) {
        if (ch == 2)
            obstacles.push_back(current);
    }
    Path final = astar(size, start, target, obstacles);
    for (Path::reverse_iterator it = final.rbegin(); it != final.rend(); ++it) {
        printf("%d %d", it->x, it->y);
    }
    return 0;
}

void test_astar()
{
    Point start = {3, 5}, target = {9, 9};
    Path obstacles = {{7, 0}, {7, 1}, {7, 2}, {7,6}, {7, 7}, {7,8}};
    Path final = astar(10, start, target, obstacles);
}

