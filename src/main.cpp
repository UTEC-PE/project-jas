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
    myGraph.prim().printAdjacencyList();



    return 0;
}

