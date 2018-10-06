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
	E weight;

	Edge(node* begin, node* end, E w)
	{
		nodes[0] = begin;
		nodes[1] = end;
		weight = w;
	};



	~Edge() {};
	
};

#endif
