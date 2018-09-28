#ifndef GRAPH_HPP
#define GRAPH_HPP
#include "node.hpp"

template <typename T>
class Graph{
	int nnodes;
	Nodes<T>* nodes;
	bool directed;
public:
	Graph(){};
	Graph(int nodos, bool directed = false)
		: nnodes(nodos), directed(directed)
	{
		nodes = new Nodes[nnodes];
	}
	void insert(T value, T neighbord, int x,int y, int z){
		int pos;
		for(pos = 0; i<nnodes; i++){

		}
	}
};
#endif
