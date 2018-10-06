#ifndef NODE_HPP
#define NODE_HPP

template <typename G>
class Node {
public:
	typedef typename G::N N;
	typedef typename G::E E;
	typedef typename G::node node;
	typedef typename G::edge edge;
	typedef typename G::EdgeSeq EdgeSeq;
	
	// This is the Adjacency List
	EdgeSeq edges;

private:
	N data;
	double x;
	double y;
	
public:
	Node(N newData): data(newData) {};

	void addEdge(node* begin, node* end)
	{
		edge* newEdge = new edge(begin, end);
		edges.push_back(newEdge);
	}

	N getData() { return data; }
};

#endif
