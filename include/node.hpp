#ifndef NODE_HPP
#define NODE_HPP

#include <chrono>
#include <random>

template <typename G>
class Node {
public:
	typedef typename G::N N;
	typedef typename G::E E;
	typedef typename G::node node;
	typedef typename G::edge edge;
	typedef typename G::EdgeSeq EdgeSeq;
	
	// This is the Adjacency List
	EdgeSeq edges;

private:
	N data;
	double x;
	double y;
	
public:
	Node(N newData): data(newData), inDegree(0), outDegree(0) {
		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		std::default_random_engine generator (seed);
		std::uniform_real_distribution<double> distribution(0.0,200.0);
		x = distribution(generator);
		y = distribution(generator);
	};
	Node(const Node<G> &copy){
		data = copy.data;
		x = copy.x;
		y = copy.y;
	};
	int inDegree;
	int outDegree;	// If the graph is not directed, only outDegree will be used
	int &degree = outDegree;

	void addEdge(node* begin, node* end, E weight)
	{
		edge* newEdge = new edge(begin, end, weight);
		edges.push_back(newEdge);
	}

	int deleteEdge(node* begin, node* end)
	{
		
		for(auto it = edges.begin(); it != edges.end(); ++it)
		{
			if ((*it)->nodes[0] == begin && (*it)->nodes[1] == end)
			{
				int weightLost = (*it)->weight;
				delete *it;
				edges.erase(it);
				return weightLost;
			}
		}
		return 0;
	}

	N getData() { return data; }


	int route(int peso){
		if (edges.size())
		{
			peso = edges.front()->nodes[1]->route(peso);
			std::cout<<data<<" ";
			return (peso + edges.front()->weight);		
		}
		std::cout<<data<<" ";
		return 0;
	}


	double getX(){ return x;}
	double getY(){ return y;}

	~Node() {
		while(!edges.empty())
		{
			delete edges.front();
			edges.pop_front();
		}
	};
};

#endif
