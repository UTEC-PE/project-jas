#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <list>
#include "node.hpp"
#include "edge.hpp"

struct Traits {
	typedef char N;
	typedef int E;
};

template <typename Tr>
class Graph {
	NodeSeq nodes;
	NodeIte ni;
	EdgeIte ei;
	
public:
	typedef Graph<Tr> self;

    /* The whole graph is being passed as a trait to Node so it can have N and E
    in it without explicitly declaring so. Same thing with Edge. */
	typedef Node<self> node;
	typedef Edge<self> edge;

    // Vector of nodes
	typedef std::vector<node*> NodeSeq;

    /* A helper typedef for the core function.
    This is NOT the Adjacency List. Each Node as one instead. */
	typedef std::list<edge*> EdgeSeq;
    
	typedef typename Tr::N N;
	typedef typename Tr::E E;
	typedef typename NodeSeq::iterator NodeIte;
	typedef typename EdgeSeq::iterator EdgeIte;
};

typedef Graph<Traits> graph;
#endif