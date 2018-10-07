#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "graph.hpp"
using namespace std;

int main()
{
    graph* myGraph = new graph;

	//Lectura del archivo
	ifstream input ("grafo.txt");
	string line;
	getline(input, line);
	stringstream ss(line);
	int nodes,edges;
	bool direction;
	ss>>nodes>>edges>>direction;
	getline(input, line);
	ss.str(line);
	ss.clear();
	for(int i = 0; i<nodes ;i++ ){
		char value;
		ss>>value;
		myGraph->addNode(value);
	}
	for(int i=0;i<edges;i++){
		char b,e;
		int w;
		getline(input, line);
		ss.str(line);
		ss.clear();
		ss>>b>>e>>w;
		myGraph->addEdge(b,e,w);
	}
	//Fin lectura de archivo
	
    cout << "Printing nodes..." << endl;
    myGraph->printNodes();


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

