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
	typedef Node<self> node;
	typedef Edge<self> edge;
	typedef std::vector<node*> NodeSeq;
	typedef std::list<edge*> EdgeSeq;
	typedef typename Tr::N N;
	typedef typename Tr::E E;
	typedef typename NodeSeq::iterator NodeIte;
	typedef typename EdgeSeq::iterator EdgeIte;
};

typedef Graph<Traits> graph;

#endif
