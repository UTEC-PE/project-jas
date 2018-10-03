#ifndef EDGE_H
#define EDGE_H

#include "node.hpp"

template <typename G>
class Edge {
	E data;
	bool dir;
	
public:
	typedef typename G::E E;
	typedef typename G::node node;

	node* nodes[2];
};

#endif
