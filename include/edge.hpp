#ifndef EDGE_H
#define EDGE_H

#include "node.hpp"

template <typename G>
class Edge {
public:
	typedef typename G::E E;
	typedef typename G::node node;

private:
	E data;
	bool dir;
	
public:
	node* nodes[2];
};

#endif
