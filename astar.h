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
    Point(int x = 0, int y = 0, Point *parent = NULL);
    // position
    int x, y;
    // part of cost calculation
    int g, h;
    // maintains a list of the point's parent in the path so it can be traversed backwards
    Point *parent;
    // used to calculate cost
    Point *goal;
    // check if both points have the same position
    bool operator==(const Point& oth);
    friend bool operator<(const Point& one, const Point& two);
    // find the total "cost" of traversing to this point
    // the program will attempt to find the "cheapest" path possible
    int total_cost();
    // recalculate h (used in total cost)
    void eucH();
    /**
     * Find potential next points in the path. Only includes points next to this point,
     * excluding obstacles and the current point.
     * @param obstacles List of positions to skip
     * @param size Maximum X & Y size of the grid
     * @return list of points that may be followed
     */
    std::vector<Point> successors(std::vector<Point>& obstacles, int size);
    std::string str();
};

struct Node {
    int x, y, g, h;
    Node *parent;
};

typedef std::vector<Point> Path;
typedef std::vector<Point*> pPath;

/**
 * Run the A* path-finding algorithm
 * @param size Maximum X & Y size of the map
 * @param start Position to begin searching from
 * @param target Position to end the path at
 * @param obstacles List of positions to skip
 * @return list of points in the computed path from start to target
 */
Path astar(int size, Point start, Point target, Path obstacles);

#endif /* defined(__astar__astar__) */
