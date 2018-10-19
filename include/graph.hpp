#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <list>
#include "node.hpp"
#include "edge.hpp"
#include <algorithm>
#include <map>

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
	bool directed;
    double parameterOfDensity = 0.8;

    std::map<N,int> disjointSetPlaces;

public:
    Graph(): directed(false), nodeCount(0) {};

    Graph(bool directed=false): nodeCount(0), directed(directed){};

    Graph(N data, bool directed=false): nodeCount(0), edgeWeight(0), directed(directed){ addNode(data); };

    int nodeCount;
    int edgeCount;
    E edgeWeight;

    int getNodeCount() { return nodeCount; }
    E getEdgeWeight() { return edgeWeight; }
    double getParameterOfDensity() { return parameterOfDensity; }
	
    void addNode(N data)
    {
        if (findNode(data) == nullptr)
        {
            node* newNode = new node(data);
            nodes.push_back(newNode);
            nodeCount++;
        }
    }
	bool isDirected(){ return directed;}

    node* findNode(N data)
    {
        if (nodeCount == 0) return nullptr;
		for(ni = nodes.begin(); ni!=nodes.end(); ++ni)
        {
			if((*ni)->getData() == data) break;
		}

		return (*ni);
    }

    node* findNode(node n)
    {
        if (nodeCount == 0) return nullptr;
        bool found = false;
        for(ni = nodes.begin(); ni!=nodes.end(); ++ni)
        {
			if((*ni)->getData() == n->getData())
            {
                found = true;
                break;
            }
		}

        if (!found) return nullptr;
        else return (*ni);
    }

    node* findNode(node* n, NodeSeq where)
    {
        if (nodeCount == 0) return nullptr;
        bool found = false;
        for(ni = where.begin(); ni!=where.end(); ++ni)
        {
			if((*ni)->getData() == n->getData())
            {
                found = true;
                break;
            }
		}

        if (!found) return nullptr;
        else return (*ni);
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

    void deleteNode(N nodeToDelete) { deleteNode(findNode(nodeToDelete)); }

    void addEdge(node* begin, node* end, E weight)
    {
        begin->addEdge(begin, end, weight);
        edgeWeight += weight;
        edgeCount ++;
        if (!directed) end->addEdge(end, begin, weight);
    }

    void addEdge(node* begin, node* end)
    {
        addEdge(begin, end, 1);
    }

    void addEdge(N begin, N end, E weight)
    {
        addEdge( findNode(begin), findNode(end), weight );
    }

    void addEdge(N begin, N end)
    {
        addEdge(begin, end, 1);
    }

    void deleteEdge(node* begin, node* end)
    {
        int weightLost = begin->deleteEdge(begin, end);
        
        edgeCount--;
        edgeWeight -= weightLost;

        if(!directed) end->deleteEdge(end, begin);
    }

    void deleteEdge(N begin, N end)
    {
        deleteEdge(findNode(begin), findNode(end));
    }

    double calculateDensity()
    {
        if (directed) return (double) edgeCount/(nodeCount*(nodeCount-1));
        return 2*((double) edgeCount/(nodeCount*(nodeCount-1)));
    }

    bool isDense()
    {
        if (calculateDensity() >= parameterOfDensity) return true;
        return false;
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

    self prim()
    {
        if (!directed)
        {
            Graph *primMST = new Graph(false);

        primMST->addNode(nodes[0]->getData());

        NodeSeq &visited = (primMST->nodes);

        while(visited.size() != nodeCount)
        {
            int minWeight = __INT_MAX__;
            edge *edgeToAdd = new edge();
            for(int i = 0; i < primMST->nodeCount; i++)
            {
                EdgeSeq* nodeEdges = &((findNode(visited[i]->getData())->edges));
            
                for(auto it = (*nodeEdges).begin(); it != (*nodeEdges).end(); ++it)
                {
                    edge* &currentEdge = *it;
                    if ( currentEdge->weight < minWeight ) 
                    {
                        bool isNotInVisited = findNode(currentEdge->nodes[1], visited) == nullptr;
                        if ( isNotInVisited )
                        {
                            minWeight = currentEdge->weight;
                            edgeToAdd = currentEdge;
                        }
                    }
                }
            }
            primMST->addNode(edgeToAdd->nodes[1]->getData());
            primMST->addEdge(edgeToAdd->nodes[0]->getData(), edgeToAdd->nodes[1]->getData(), edgeToAdd->weight);
        }

        return *primMST;
        } else throw std::runtime_error("No se puede aplicar el Algoritmo de Prim a un Grafo direccionado");
    } 

    struct edgeCmp
    {
        bool operator()(const edge* a, const edge* b)
        {
            return (a->weight < b->weight);
        }
    };

    bool createsCycle(edge* &newEdge, int* disjointSet)
    {
        int nodeBeginParent = disjointSetPlaces[newEdge->nodes[0]->getData()];
        int nodeEndParent = disjointSetPlaces[newEdge->nodes[1]->getData()];

        while (disjointSet[nodeBeginParent] >= 0) nodeBeginParent = disjointSet[nodeBeginParent];
        
        while (disjointSet[nodeEndParent] >= 0) nodeEndParent = disjointSet[nodeEndParent];

        if (nodeBeginParent == nodeEndParent) return true;
        else
        {
            disjointSet[nodeEndParent] = nodeBeginParent;
            return false;
        }
    }

    self kruskal()
    {
        Graph* kruskalMST = new Graph(false);
        EdgeSeq allEdges;
        int disjointSet[nodeCount] = {};
        
        for(size_t i = 0; i < nodeCount; i++)
        {
            disjointSet[i] = -1;
            disjointSetPlaces[nodes[i]->getData()] = i;
        }
        
        for(size_t i = 0; i < nodeCount; i++)
        {
            EdgeSeq &currentNodeEdges = nodes[i]->edges;
            allEdges.insert(allEdges.end(), currentNodeEdges.begin(), currentNodeEdges.end());
        }

        allEdges.sort(edgeCmp());
        
        for(auto i = allEdges.begin(); i != allEdges.end(); ++i)
        {
            if (!createsCycle(*i, disjointSet))
            {
                kruskalMST->addNode((*i)->nodes[0]->getData());
                kruskalMST->addNode((*i)->nodes[1]->getData());

                kruskalMST->addEdge(
                    (*i)->nodes[0]->getData(),
                    (*i)->nodes[1]->getData()
                );
            }
        }

        return *kruskalMST;
    }

};

/* 
    Now Graph<Traits> is defined as graph. 
    Hence, the correct instantiation of a Graph would be:
        graph* myGraph = new graph;
*/
typedef Graph<Traits> graph;
#endif
