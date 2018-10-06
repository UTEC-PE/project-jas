#include <iostream>
#include "../include/graph.hpp"
using namespace std;

main(int argc, char const *argv[])
{
    graph* myGraph = new graph;
    myGraph->addNode('A');
    myGraph->addNode('B');
    myGraph->addNode('C');
    myGraph->addNode('D');
    myGraph->addNode('E');

    cout << "Printing nodes..." << endl;
    myGraph->printNodes();

    myGraph->addEdge('A','E');
    myGraph->addEdge('A','B');
    myGraph->addEdge('A','C');
    myGraph->addEdge('B','C');
    myGraph->addEdge('D','A');
    myGraph->addEdge('C','B');

    cout << "Printing adjacency list" << endl;
    myGraph->printAdjacencyList();

    return 0;
}

