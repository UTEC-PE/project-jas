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

private:
	NodeSeq nodes;
	NodeIte ni;
	EdgeIte ei;
	
public:
    Graph() {};
    Graph(N data) { addNode(data); };

    void addNode(N data)
    {
        node* newNode = new node(data);
    }

};

/* 
    Now Graph<Traits> is defined as graph. 
    Hence, the correct instantiation of a Graph would be:
        graph* myGraph = new graph;
*/
typedef Graph<Traits> graph;
#endif