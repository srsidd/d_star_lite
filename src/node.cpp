#include "node.hpp"


// Populate methods of Position class
    void Position::setX(int x_coord) {
        this->x = x_coord;
    }

    void Position::setY(int y_coord) {
        this->y = y_coord;
    }

    int Position::getX() {
        return this->x;
    }

    int Position::getY() {
        return this->y;
    }


// Populate methods of Node class
    void Node::set_g_cost(int cost) {
        this->g_cost = cost;
    }

    void Node::set_f_cost(int cost) {
        this->f_cost = cost;
    }

    // void Node::set_type(int val) {
    //     this->type = val;
    // }

    // void Node::add_to_open() {
    //     this->open_node = true;
    // }

    // void Node::add_to_closed() {
    //     this->closed_node = true;
    // }

    // void Node::removed_from_closed() {
    //     this->closed_node = false;
    // }

    // void Node::removed_from_open() {
    //     this->open_node = false;
    // }

    int Node::get_g_cost() {
        return this->g_cost;
    }

    int Node::get_f_cost() {
        return this->f_cost;
    }

    // int Node::get_type() {
    //     return this->type;
    // }

    // bool Node::is_open() {
    //     return this->open_node;
    // }

    // bool Node::is_closed() {
    //     return this->closed_node;
    // }
