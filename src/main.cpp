#include <iostream>
#include <string>
#include "graph.hpp"
#include "read.hpp"
using namespace std;

int main()
{
	Read<graph> read("grafo.txt");
    graph myGraph = read.getGraph();

    myGraph.printAdjacencyList();

    cout << endl;

    graph pGraph = myGraph.prim();
    pGraph.printAdjacencyList();

    cout << endl;

    graph kGraph = myGraph.kruskal();
    kGraph.printAdjacencyList();

    return 0;
}

