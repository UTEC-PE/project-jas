#include <iostream>
#include <vector>
#include <algorithm>
#include <random>
#include <thread>
#include <functional>
#include "graph.hpp"
#include "read.hpp"
using namespace std;
int ncore = 0;
int nthreads = 10;
void par(graph* g,char a, char b){
    g->Astar(a, b);	
}
int main(){
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

	vector<thread> threads;
	for(int i = 0; i<nthreads; ++i) {
		threads.emplace_back(bind(par,g,'A','F'));

	}
	cout << "KK" << "\n";
	for(int i = 0; i<nthreads; ++i) {
		threads[i].join();
	}
	//printM(C,n,p);


	return 0;
}

