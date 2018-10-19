#ifndef READ_H
#define READ_H

#include <fstream>
#include <string>
#include <sstream>

#include "graph.hpp"

/**
 * Clase de ayuda para leer el grafo desde un archivo,
 * no es necesario que la utilicen, podrían implementar su lector
 * desde su grafo o algún otro lado
 **/
template <typename G>
class Read {
	typedef typename G::N N;
	typedef typename G::E E;
	std::ifstream file;
public:
	Read(std::string filename ) : file(filename)
	{}
		
	graph& getGraph() {
		std::string line;
		std::getline(file, line);
		std::stringstream ss(line);
		int nodes,edges;
		bool direction;
		ss>>nodes>>edges>>direction;
		graph* g = new graph(direction);
		getline(file, line);
		ss.str(line);
		ss.clear();
		for(int i = 0; i<nodes ;i++ ){
			char value;
			ss>>value;
			g->addNode(value);
		}
		for(int i=0;i<edges;i++){
			char b,e;
			int w;
			getline(file, line);
			ss.str(line);
			ss.clear();
			ss>>b>>e>>w;
			g->addEdge(b,e,w);
		}
		return *g;
	}
};

#endif
