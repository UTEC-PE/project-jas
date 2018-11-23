#include <iostream>
#include <string>
#include "graph.hpp"
#include "read.hpp"
#include <limits>
using namespace std;

int main()
{
	Read<graph> read1("grafo3.txt");
    graph myGraph1 = read1.getGraph();

    Read<graph> read2("grafo2.txt");
    graph myGraph2 = read2.getGraph(); 



    myGraph2.printAdjacencyList();
    
    cout << endl;
    
   /* myGraph2.printAdjacencyList();

    cout << endl;

    graph pGraph1 = myGraph1.prim();
    pGraph1.printAdjacencyList();

    cout << endl;

    //graph kGraph = myGraph.kruskal();
    //kGraph.printAdjacencyList();

    cout << "is connected: " << myGraph1.isConnected() << endl;
    cout << "is fuerteconnected: " << myGraph1.isFuertementeConexo() << endl;

    cout << "is connected: " << myGraph2.isConnected() << endl;
    cout << "is fuerteconnected: " << myGraph2.isFuertementeConexo() << endl;*/
    //map<Node*, int> distance;
    
    int int_max = std::numeric_limits<int>::max();
    cout<<int_max<<endl;

    vector<Node<graph>*> answer;
    vector<int> v = myGraph2.bellmanFord('A');

    for (int i = 0; i < myGraph2.getNodeCount(); ++i)
    {
        cout<<v[i]<<" ";
    }

    //myGraph1.greedy_bfs('A','B');

    //cout << "find edge H-Z: " << (myGraph.findEdge('H','Z') != nullptr) << endl;

    return 0;
}

