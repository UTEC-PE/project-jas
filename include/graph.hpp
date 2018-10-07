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

    /* A helper typedef for the core functions.
    This is NOT the Adjacency List. It is inherited by the nodes. */
	typedef std::list<edge*> EdgeSeq;
    
	typedef typename Tr::N N;
	typedef typename Tr::E E;
	typedef typename NodeSeq::iterator NodeIte;
	typedef typename EdgeSeq::iterator EdgeIte;

private:
	NodeSeq nodes;
	NodeIte ni;
	EdgeIte ei;
    int nodeCount;
    E edgeWeight;
	bool directed;
public:
    Graph(bool directed=false): nodeCount(0), directed(directed){};
    Graph(N data, bool directed=false)
		: nodeCount(0), edgeWeight(0), directed(directed)
	{ addNode(data); };

    int getNodeCount() { return nodeCount; }
    E getEdgeWeight() { return edgeWeight; }
	
    void addNode(N data)
    {
        node* newNode = new node(data);
        nodes.push_back(newNode);
        nodeCount++;
    }
	bool isDirected(){ return directed;}

    node* findNodeByData(N data)
    {
		for(ni = nodes.begin(); ni!=nodes.end(); ++ni){
			if((*ni)->getData() == data) break;
		}
		return (*ni);
    }

    void deleteNode(node* nodeToDelete)
    {
        int indexOfNodeToDelete = 0;

        // Deletes all edges like [* - nodeToDelete]
        for(size_t i = 0; i < nodeCount; i++)
        {
            if (nodes[i] == nodeToDelete) indexOfNodeToDelete = i;
            else
            {
                EdgeSeq* nodeEdges = &(nodes[i]->edges);
            
                for(auto it = (*nodeEdges).begin(); it != (*nodeEdges).end();)
                {
                    if ( (*it)->nodes[1] == nodeToDelete ) 
                    {
                        edgeWeight -= (*it)->weight;
                        it = (*nodeEdges).erase(it);
                    }
                    else ++it;
                }
            }
        }

        // Deletes all edges like [nodeToDelete - *]
        EdgeSeq* edgesOfNodeToDelete = &(nodeToDelete->edges);
        for(auto it = (*edgesOfNodeToDelete).begin(); it != (*edgesOfNodeToDelete).end(); ++it)
        {
            edgeWeight -= (*it)->weight;
        }

        nodes.erase(nodes.begin() + indexOfNodeToDelete);
        delete nodeToDelete;
        nodeCount--;
    }

    void deleteNode(N nodeToDelete) { deleteNode(findNodeByData(nodeToDelete)); }

    void addEdge(node* begin, node* end, E weight)
    {
        begin->addEdge(begin,end, weight);
        edgeWeight += weight;
    }

    void addEdge(node* begin, node* end)
    {
        addEdge(begin, end, 0);
    }

    void addEdge(N begin, N end, E weight)
    {
        addEdge( findNodeByData(begin), findNodeByData(end), weight );
    }

    void addEdge(N begin, N end)
    {
        addEdge(begin, end, 0);
    }

    void printNodes()
    {
        for(size_t i = 0; i < nodeCount; i++)
        {
            std::cout << nodes[i]->getData() << " ";
        }  
        std::cout << std::endl;
    }

    void printAdjacencyList()
    {
        for(size_t i = 0; i < nodeCount; i++)
        {
            std::cout << nodes[i]->getData() << " -> ";
            EdgeSeq* nodeEdges = &(nodes[i]->edges);
            
            for(auto it : *nodeEdges)
            {
                std::cout << (*it).nodes[1]->getData() << " ";
            }
            std::cout << std::endl;
        }   
    }

};

/* 
    Now Graph<Traits> is defined as graph. 
    Hence, the correct instantiation of a Graph would be:
        graph* myGraph = new graph;
*/
typedef Graph<Traits> graph;
#endif
