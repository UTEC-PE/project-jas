#ifndef NODE_HPP
#define NODE_HPP

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
	Node(N newData): data(newData), inDegree(0), outDegree(0) {};

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

	~Node() {
		while(!edges.empty())
		{
			delete edges.front();
			edges.pop_front();
		}
	};
};

#endif
