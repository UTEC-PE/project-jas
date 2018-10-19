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

    cout << "is connected: " << myGraph.isConnected() << endl;
    cout << "is connected: " << myGraph.isFuertementeConexo() << endl;


    cout << "find edge H-Z: " << (myGraph.findEdge('H','Z') != nullptr) << endl;

    return 0;
}

