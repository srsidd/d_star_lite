#include <iostream>
#include <math.h>
#include <limits>

class Position {
protected:
    int x = std::numeric_limits<int>::max();;
    int y = std::numeric_limits<int>::max();;

public:
    Position () {};

    void setX(int x_coord);

    void setY(int y_coord);

    int getX();

    int getY();
};


// If type = 0, it's a node, if type = 2 it's a path, if type = inf it's an obstacle
class Node: public Position {
protected:
    float g_cost = 0;
    float f_cost = 0;
    // bool open_node = false;
    // bool closed_node = false;
    // int type = 0;

public:
    Node() {};

    void set_g_cost(int cost);

    void set_f_cost(int cost);

    // void set_type(int val);

    // void add_to_open();

    // void add_to_closed();

    // void removed_from_closed();

    // void removed_from_open();

    int get_g_cost();

    int get_f_cost();

    // int get_type();

    // bool is_open();

    // bool is_closed();
};
