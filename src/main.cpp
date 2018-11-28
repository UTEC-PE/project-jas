#include <iostream>
#include <string>
#include "graph.hpp"
#include "read.hpp"
using namespace std;

int main()
{
    graph* g = new graph(true);

    g->addNode('A');
    g->addNode('B');
    g->addNode('C');
    g->addNode('D');
    g->addNode('E');
    g->addNode('F');
    g->addNode('G');
    g->addNode('H');
    g->addNode('I');

    g->addEdge('A', 'B', 5);
    g->addEdge('A', 'C', 25);
    g->addEdge('B', 'F', 33);
    g->addEdge('B', 'E', 11);
    g->addEdge('C', 'D', 17);
    g->addEdge('C', 'F', 38);
    g->addEdge('D', 'G', 12);
    g->addEdge('E', 'F', 21);
    g->addEdge('E', 'I', 1);
    g->addEdge('F', 'D', 4);
    g->addEdge('F', 'G', 3);
    g->addEdge('G', 'H', 50);
    g->addEdge('H', 'F', 41);
    g->addEdge('I', 'F', 2);
    g->addEdge('I', 'H', 21);
    
    /* cout << "A*" << endl;
    g->Astar('A','I');
    cout << endl; */

    cout << "BELLMAN FORD" << endl;
    graph bf = g->bellmanFord('A');
    bf.printRoute();
    cout << endl;

    /* cout << "DIJKSTRA" << endl;
    g->dijkstra(g->findNode('A')); */

    cout << "GREEDY BFS" << endl;
    graph gbfs = g->greedy_bfs('A', 'H');
    gbfs.printRoute();
    cout << endl;

    cout << "FLOYD WARSHALL" << endl;
    g->floydWarshall();
    cout << endl;
}