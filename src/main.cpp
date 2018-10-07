#include <iostream>
#include "graph.hpp"
using namespace std;

int main()
{
    graph* myGraph = new graph;
    myGraph->addNode('A');
    myGraph->addNode('B');
    myGraph->addNode('C');
    myGraph->addNode('D');
    myGraph->addNode('E');

    cout << "Printing nodes..." << endl;
    myGraph->printNodes();

    myGraph->addEdge('A','E',2);
    myGraph->addEdge('A','B',3);
    myGraph->addEdge('A','C',2);
    myGraph->addEdge('B','C',0);
    myGraph->addEdge('D','A',3);
    myGraph->addEdge('C','B',5);

    cout << "Printing adjacency list" << endl;
    myGraph->printAdjacencyList();

    cout << "Node count: " << myGraph->getNodeCount() << endl;
    cout << "Edge weight: " << myGraph->getEdgeWeight() << endl;

    cout << endl << "Deleting node C..." << endl << endl;
    myGraph ->deleteNode('C');

    cout << "Printing nodes..." << endl;
    myGraph->printNodes();

    cout << "Node count: " << myGraph->getNodeCount() << endl;

    cout << "Printing adjacency list" << endl;
    myGraph->printAdjacencyList();

    cout << "Node count: " << myGraph->getNodeCount() << endl;
    cout << "Edge weight: " << myGraph->getEdgeWeight() << endl;

    return 0;
}

