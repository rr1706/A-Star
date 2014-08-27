//
//  main.cpp
//  astar
//
//  Created by Connor Monahan on 7/20/14.
//  Copyright (c) 2014 Ratchet Rockers. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <err.h>
#include "astar.h"

const char *program_name;

void usage(void)
{
    fprintf(stderr, "usage: %s [-s size] startx,y targetx,y\n", program_name);
    exit(1);
}

int main(int argc, char **argv)
{
    int size, ch;
    program_name = argv[0];
    size = 10;
    while ((ch = getopt(argc, argv, "s:")) != -1) {
        switch (ch) {
            case 's':
                sscanf(optarg, "%d", &size);
                break;
            case '?':
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
    sscanf(argv[1], "%d,%d", &target.x, &target.y);
    Path obstacles;
    Point current;
    while ((ch = scanf("%d %d\n", &current.x, &current.y)) != EOF) {
        if (ch == 2)
            obstacles.push_back(current);
    }
    Path final = astar(size, start, target, obstacles);
    for (Path::reverse_iterator it = final.rbegin(); it != final.rend(); ++it) {
        printf("%d %d\n", it->x, it->y);
    }
    return EXIT_SUCCESS;
}
