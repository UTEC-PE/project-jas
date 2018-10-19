#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "graph.hpp"
using namespace std;

int main()
{
    graph* myGraph = new graph(false);

    myGraph->addNode('A');
    myGraph->addNode('B');
    myGraph->addNode('C');
    myGraph->addNode('D');
    myGraph->addNode('E');
    myGraph->addNode('F');
    myGraph->addNode('G');

    myGraph->addEdge('A', 'B', 2);
    myGraph->addEdge('A', 'D', 3);
    myGraph->addEdge('A', 'C', 3);
    myGraph->addEdge('B', 'E', 3);
    myGraph->addEdge('B', 'C', 4);
    myGraph->addEdge('C', 'D', 5);
    myGraph->addEdge('C', 'F', 6);
    myGraph->addEdge('C', 'E', 1);
    myGraph->addEdge('D', 'F', 7);
    myGraph->addEdge('E', 'F', 8);
    myGraph->addEdge('F', 'G', 9);


    myGraph->printAdjacencyList();

    myGraph->DeepFirstSearch('A');
    cout<<endl;
    myGraph->BreadthFirstSearch('A');


    return 0;
}

