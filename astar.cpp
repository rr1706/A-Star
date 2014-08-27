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
#include <stdexcept>
#include <algorithm>

class Node {
private:
    double distance(Node *one, Node *two)
    {
        return sqrt(pow(one->x - two->x, 2) + pow(one->y - two->y, 2));
    }
    bool position_open(Node &pos, Path &obstacles, int size)
    {
        if (pos.x < 0 || pos.y < 0 || pos.x > size || pos.y > size)
            return false; // out of bounds
        for (auto it = obstacles.begin(); it < obstacles.end(); ++it)
            if (*it == pos)
                return false; // blocked
        return true;
    }
public:
    void compute_costs()
    {
        if (parent != NULL) {
            g = distance(parent, this);
            if (goal == NULL)
                goal = parent->goal;
        } else {
            g = 1;
        }
        if (goal != NULL) {
            h = distance(this, goal);
        } else {
            h = 0;
        }
    }
    Node(void)
    {
    }
    Node(Point pt, Node *parent = NULL, Node *goal = NULL)
        : x(pt.x), y(pt.y), parent(parent), goal(goal)
    {
        compute_costs();
    }
    Node(int x, int y, Node *parent = NULL, Node *goal = NULL)
        : x(x), y(y), parent(parent), goal(goal)
    {
        compute_costs();
    }
    void copy(Node &other)
    {
        x = other.x;
        y = other.y;
        parent = other.parent;
        goal = other.goal;
        compute_costs();
    }
    int x, y, g, h;
    Node *parent;
    Node *goal;
    friend bool operator==(const Node& one, const Node& two)
    {
        return one.x == two.x && one.y == two.y;
    }
    friend bool operator!=(const Node& one, const Node& two)
    {
        return one.x != two.x || one.y != two.y;
    }
    friend bool operator<(const Node& one, const Node& two)
    {
        return one.x < two.x && one.y < two.y;
    }
    int total_cost(void)
    {
        return g + h;
    }
    template<class Container>
    std::vector<Node> successors(Container &obstacles, int size)
    {
        std::vector<Node> successors;
        for (int xd = -1; xd <= 1; xd++) {
            for (int yd = -1; yd <= 1; yd++) {
                Node n(x + xd, y + yd, this);
                if (!position_open(n, obstacles, size))
                    continue;
                if (n == *this || (parent != NULL && n == *parent))
                    continue;
                successors.push_back(n);
            }
        }
        return successors;
    }
};

Point::Point(int x, int y) : x(x), y(y)
{
}

template<class Object>
class Stack {
private:
    Object *list;
    int max, next;
public:
    Stack(int size) : max(size)
    {
        list = new Object[sizeof(Object) * size];
    }
    ~Stack()
    {
        delete [] list;
    }
    Object* create(Object tmpl)
    {
        if (next >= max)
            throw new std::runtime_error("Stack out of memory");
        // gets an element from a blank position in the list
        Object *elem = list + sizeof(Object) * next++;
        // copies the template data into the blank element
        elem->copy(tmpl);
        return elem;
    }
};

// adapted from a C# implementation
Path astar(int size, Point start, Point target, Path obstacles)
{
    // a preallocated stack is used to store node pointers
    // pointers are required in order to calculate paths to the parent
    //  node when the algorithm finishes
    // this class automatically manages the memory for this
    // the memory is preallocated because otherwise calling malloc/new
    //  for every new successor slowed the program a lot
    Stack<Node> nodeMem(size * size * 10);
    Node targetNode(target);
    // open array contain points that still need to be processed
    // closed array contains that have
    std::vector<Node*> open, closed;
    {
        Node startNode(start, NULL, &targetNode);
        Node *startNodeS = nodeMem.create(startNode);
        open.push_back(startNodeS);
    }
    while (!open.empty()) {
        // get the point from the open array with the smallest total cost
        // this will be the next point the path will follow
        std::sort(open.begin(), open.end(), [] (Node *i, Node *j) {
            return i->total_cost() < j->total_cost();
        });
        Node *current = open[0];
        // find the point in the open array and delete it (because it is moving to closed)
        open.erase(open.begin());
        if (*current == targetNode) {
            // reached the target, done
            targetNode.parent = current->parent;
            break;
        }
        // get a list of all points that the path can potentially take from the current point
        // it checks to make sure it does not chart a course through an obstacle
        for (Node successor : current->successors(obstacles, size)) {
            auto oIt = std::find_if(open.begin(), open.end(), [successor] (Node *nod) {
                return successor == *nod;
            });
            auto cIt = std::find_if(closed.begin(), closed.end(), [successor] (Node *nod) {
                return successor == *nod;
            });
            if ((oIt != open.end() && (*oIt)->total_cost() - current->total_cost() <= 0)
                || (cIt != closed.end() && (*cIt)->total_cost() - current->total_cost() <= 0))
                continue; // if this point is already in a list and has more cost, ignore it
            // remove the point from an array if its already there
            // the new one might have a different parent or cost
            if (oIt != open.end()) {
                open.erase(oIt);
            }
            if (cIt != closed.end()) {
                closed.erase(cIt);
            }
            Node *valid = nodeMem.create(successor); // get a new node from the stack
            // submit this new point to process next time to extend the path
            open.push_back(valid);
        }
        // mark the current point as traversed
        closed.push_back(current);
    }
    Path results;
    // Traverse backwards from the target by jumping to each successive parent
    Node *p = &targetNode;
    while (p != NULL) {
        results.push_back(Point(p->x, p->y));
        p = p->parent;
    }
    // Free some memory from the pointers
    return results;
}
