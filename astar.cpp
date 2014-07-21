//
//  astar.cpp
//  astar
//
//  Created by Connor Monahan on 7/20/14.
//  Copyright (c) 2014 Ratchet Rockers. All rights reserved.
//

#include "astar.h"
#include <math.h>

bool Point::operator==(const Point& oth)
{
    return x == oth.x && y == oth.y;
}

struct open_t {
    int on_list, x, y, parent_x, parent_y;
    double hn, gn, fn;
};

typedef Point closed_t;

struct exp_t {
    int x, y;
    double hn, gn, fn;
};

typedef std::vector<open_t> open_l;
typedef std::vector<closed_t> closed_l;
typedef std::vector<exp_t> exp_l;

static int node_index(open_l& open, Point p)
{
    for (int i = 0; i < open.size(); ++i) {
        if (open[i].x == p.x && open[i].y == p.y)
            return i;
    }
    return -1;
}

static int min_fn(open_l& open, Point target)
{
    int goal_index = 0, min = 0, sz = 0;
    double fn = open[0].fn;
    bool flag = false;
//    open_l temp_array;
    for (int j = 0; j < open.size(); j++) {
        open_t ji = open[j];
        if (ji.on_list == 1) {
//            temp_array.push_back(ji);
            ++sz;
            if (ji.x == target.x && ji.y == target.y) {
                flag = true;
                goal_index = j;
            }
            if (ji.fn < fn) {
                fn = ji.fn;
                min = j;
            }
        }
    }
    if (flag) {
        return goal_index;
    } else if (sz != 0) {
        return min;
    } else {
        return -1;
    }
}

static double distance(Point one, Point two)
{
    return sqrt(pow(one.x - two.x, 2) + pow(one.y - two.y, 2));
}

static exp_l expand_array(Point node, int hn, Point target, closed_l closed, int max)
{
    exp_l exp_array;
    for (int k = -1; k < 2; k++) {
        for (int j = -1; j < 2; j++) {
            if (k == j && k == 0)
                continue;
            Point s = {node.x + k, node.y + j};
            if (s.x < 0 || s.x > max || s.y < 0 || s.y > max)
                continue;
            bool flag = true;
            for (Point& i : closed)
                if (s == i)
                    flag = false;
            if (!flag)
                continue;
            double nhn = hn + distance(node, s), ngn = distance(target, s);
            exp_t expi = {s.x, s.y, nhn, ngn, nhn + ngn};
            exp_array.push_back(expi);
        }
    }
    return exp_array;
}

Path astar(int size, Point start, Point target, Path obstacles)
{
    Path results;
    open_l open;
    closed_l closed;
    for (Point obstacle : obstacles) {
        closed.push_back(obstacle);
    }
    Point node(start);
    double cost = 1;
    bool no_path = true;
    double goal_distance = distance(node, target);
    {
        open_t intl = {0, node.x, node.y, node.x, node.y, cost, goal_distance, goal_distance};
        open.push_back(intl);
        closed.push_back(node);
    }
    while (!(node == target) && no_path) {
        exp_l exp_array = expand_array(node, cost, target, closed, size);
        for (exp_t expi : exp_array) {
            bool flag = false;
            for (open_t& openi : open) {
                if (expi.x == openi.x && expi.y == openi.y) {
                    openi.fn = fmin(openi.fn, expi.fn);
                    if (openi.fn == expi.fn) {
                        openi.parent_x = node.x;
                        openi.parent_y = node.y;
                        openi.hn = expi.hn;
                        openi.gn = expi.gn;
                    }
                    flag = true;
                }
            }
            if (flag == false) {
                open_t intl = {1, expi.x, expi.y, node.x, node.y, expi.hn, expi.gn, expi.fn};
                open.push_back(intl);
            }
        }
        int index_min_node = min_fn(open, target);
        if (index_min_node != -1) {
            Point nnode = {open[index_min_node].x, open[index_min_node].y};
            node = nnode;
            cost = open[index_min_node].hn;
            closed.push_back(node);
            open[index_min_node].on_list = 0;
        } else {
            no_path = false;
        }
    }
    Point final = closed.back();
    results.push_back(final);
    if (final == target) {
        int inode = node_index(open, final);
        Point parent = {open[inode].parent_x, open[inode].parent_y};
        while (!(parent == start)) {
            results.push_back(parent);
            inode = node_index(open, parent);
            Point grandparent = {open[inode].parent_x, open[inode].parent_y};
            parent = grandparent;
        }
    }
    return results;
}
