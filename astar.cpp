//
//  astar.cpp
//  astar
//
//  Created by Connor Monahan on 7/20/14.
//  Copyright (c) 2014 Ratchet Rockers. All rights reserved.
//

#include "astar.h"
#include <math.h>
#include <map>
#include <set>
#include <sstream>
#include <memory>

static double distance(Point one, Point two)
{
    return sqrt(pow(one.x - two.x, 2) + pow(one.y - two.y, 2));
}

Point::Point(int x, int y, Point *parent) : x(x), y(y), parent(parent), goal(NULL)
{
    if (parent != NULL) {
        g = distance(*parent, *this);
        goal = parent->goal;
    } else {
        g = 1;
    }
    eucH();
}

bool Point::operator==(const Point& oth)
{
    return x == oth.x && y == oth.y;
}

int Point::total_cost()
{
    return g + h;
}

std::string Point::str()
{
    std::stringstream ss;
    ss << "x: " << x << " y: " << y;
    return ss.str();
}

void Point::eucH()
{
    if (goal != NULL) {
        h = distance(*this, *goal);
    } else {
        h = 0;
    }
}

static bool obst_open(Path& list, int size, Point pt)
{
    if (pt.x < 0 || pt.y < 0 || pt.x > size || pt.y > size)
        return false;
    for (Point& p : list)
        if (p == pt)
            return false;
    return true;
}

static pPath::iterator get(pPath &list, Point pt)
{
    for (pPath::iterator it = list.begin(); it < list.end(); ++it)
        if (**it == pt)
            return it;
    return list.end();
}

std::vector<Point> Point::successors(Path& obstacles, int size)
{
    std::vector<Point> successors;
    for (int xd = -1; xd <= 1; xd++) {
        for (int yd = -1; yd <= 1; yd++) {
            Point n(x + xd, y + yd, this);
            if (obst_open(obstacles, size, n)) {
                if (n == *this || (parent != NULL && n == *parent))
                    continue;
                successors.push_back(n);
            }
        }
    }
    return successors;
}

bool operator<(const Point& one, const Point& two)
{
    return one.x < two.x && one.y < two.y;
}

static Point* min(std::vector<Point*> &stp)
{
    if (stp.empty())
        throw std::runtime_error("stp empty");
    Point *current = *stp.begin();
    int H = current->total_cost();
    for (Point *pt : stp) {
        if (pt->total_cost() < H) {
            H = pt->total_cost();
            current = pt;
        }
    }
    return current;
}

// adapted from a C# implementation
Path astar(int size, Point start, Point target, Path obstacles)
{
    start.goal = &target;
    start.eucH();
    // open array contain points that still need to be processed
    // closed array contains that have
    std::vector<Point*> open, closed;
    open.push_back(new Point(start));
    while (!open.empty()) {
        // get the point from the open array with the smallest total cost
        // this will be the next point the path will follow
        Point *current = min(open);
        for (pPath::iterator it = open.begin(); it < open.end(); ++it) {
            if (*it == current) {
                open.erase(it); // find the point in the open array and delete it (because it is moving to closed)
                break;
            }
        }
        if (*current == target) {
            // reached the target, done
            target.parent = current->parent;
            break;
        }
        // get a list of all points that the path can potentially take from the current point
        // it checks to make sure it does not chart a course through an obstacle
        for (Point successor : current->successors(obstacles, size)) {
            pPath::iterator oIt = get(open, successor), cIt = get(closed, successor);
            if ((oIt != open.end() && (*oIt)->total_cost() - current->total_cost() <= 0)
                || (cIt != closed.end() && (*cIt)->total_cost() - current->total_cost() <= 0))
                continue; // if this point is already in a list and has more cost, ignore it
            // remove the point from an array if its already there
            // the new one might have a different parent or cost
            if (oIt != open.end()) {
                open.erase(oIt);
                // may leak memory here, but using delete causes lots of errors
            }
            if (cIt != closed.end()) {
                closed.erase(cIt);
            }
            // submit this new point to process next time to extend the path
            open.push_back(new Point(successor));
        }
        // mark the current point as traversed
        closed.push_back(current);
    }
    Path results;
    // Traverse backwards from the target by jumping to each successive parent
    Point *p = &target;
    while (p != NULL) {
        results.push_back(*p);
        p = p->parent;
    }
    // Free some memory from the pointers
    for (Point *pt : open)
        delete pt;
    for (Point *pt : closed)
        delete pt;
    return results;
}
