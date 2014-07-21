//
//  astar.h
//  astar
//
//  Created by Connor Monahan on 7/20/14.
//  Copyright (c) 2014 Ratchet Rockers. All rights reserved.
//

#ifndef __astar__astar__
#define __astar__astar__

#include <vector>

struct Point {
    int x, y;
    bool operator==(const Point& oth);
};

typedef std::vector<Point> Path;

Path astar(int size, Point start, Point target, Path obstacles);

#endif /* defined(__astar__astar__) */
