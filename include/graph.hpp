#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <queue>
#include <vector>
#include <list>
#include "node.hpp"
#include "edge.hpp"
#include <algorithm>
#include <map>
#include <utility>
#include <queue>
#include <limits>
#include <iomanip>

struct Traits {
	typedef char N;
	typedef int E;
};

template <typename Tr>
class Graph {
public:
	typedef Graph<Tr> self;

    /* The whole graph is being passed as a trait to Node so it can have N and E
	   in it without explicitly declaring so. Same thing with Edge. */
	typedef Node<self> node;
	typedef Edge<self> edge;

    // Vector of nodes
	typedef std::vector<node*> NodeSeq;

    /* A helper typedef for the core functions.
	   This is NOT the Adjacency List. It is inherited by the nodes. */
       // ded
	typedef std::vector<edge*> EdgeSeq;
    
	typedef typename Tr::N N;
	typedef typename Tr::E E;
	typedef typename NodeSeq::iterator NodeIte;
	typedef typename EdgeSeq::iterator EdgeIte;

private:
	NodeSeq nodes;
	NodeIte ni;
	EdgeIte ei;
	bool directed;
    double parameterOfDensity = 0.8;

    std::map<N,int> disjointSetPlaces;
	double manhattan(std::map<N , std::pair<double, double>> coord, N i, N f){
		double x = coord[i].first - coord[f].first;
		double y = coord[i].second - coord[f].second;
		return fabs(x) + fabs(y);
	}
public:
    Graph(): directed(false), nodeCount(0) {};

    Graph(bool directed=false): nodeCount(0), directed(directed){};

    Graph(N data, bool directed=false): nodeCount(0), edgeWeight(0), directed(directed){ addNode(data); };

    int nodeCount;
    int edgeCount;
    E edgeWeight;

    int getNodeCount() { return nodeCount; }
    E getEdgeWeight() { return edgeWeight; }
    double getParameterOfDensity() { return parameterOfDensity; }
	
    void addNode(N data)
    {
        //std::cout<<"-"<<data<<" ";
        //std::cout<<"°"<<findNode(data)<<" ";
        if (findNode(data) == nullptr)
			{
				node* newNode = new node(data);
				nodes.push_back(newNode);
				nodeCount++;
			}
    }
	bool isDirected(){ return directed;}

    node* findNode(N data)
    {
        if (nodeCount == 0) return nullptr;

		for(ni = nodes.begin(); ni!=nodes.end(); ++ni)
			{
				if((*ni)->getData() == data) break;
			}

		return (*ni);
    }

    node* findNode(node n)
    {
        if (nodeCount == 0) return nullptr;
        bool found = false;
        for(ni = nodes.begin(); ni!=nodes.end(); ++ni)
			{
				if((*ni)->getData() == n->getData())
					{
						found = true;
						break;
					}
			}

        if (!found) return nullptr;
        else return (*ni);
    }

    node* findNode(node* n, NodeSeq where)
    {
        if (nodeCount == 0) return nullptr;
        bool found = false;
        for(ni = where.begin(); ni!=where.end(); ++ni)
			{
				if((*ni)->getData() == n->getData())
					{
						found = true;
						break;
					}
			}

        if (!found) return nullptr;
        else return (*ni);
    }

    void deleteNode(node* nodeToDelete)
    {
        int indexOfNodeToDelete = 0;

        // Deletes all edges like [* - nodeToDelete]
        for(size_t i = 0; i < nodeCount; i++)
			{
				if (nodes[i] == nodeToDelete) indexOfNodeToDelete = i;
				else
					{
						EdgeSeq* nodeEdges = &(nodes[i]->edges);
            
						for(auto it = (*nodeEdges).begin(); it != (*nodeEdges).end();)
							{
								if ( (*it)->nodes[1] == nodeToDelete ) 
									{
										edgeWeight -= (*it)->weight;
										it = (*nodeEdges).erase(it);
									}
								else ++it;
							}
					}
			}

        // Deletes all edges like [nodeToDelete - *]
        EdgeSeq* edgesOfNodeToDelete = &(nodeToDelete->edges);
        for(auto it = (*edgesOfNodeToDelete).begin(); it != (*edgesOfNodeToDelete).end(); ++it)
			{
				edgeWeight -= (*it)->weight;
			}

        nodes.erase(nodes.begin() + indexOfNodeToDelete);
        delete nodeToDelete;
        nodeCount--;
    }

    void deleteNode(N nodeToDelete) { deleteNode(findNode(nodeToDelete)); }

    void addEdge(node* begin, node* end, E weight)
    {
        begin->addEdge(begin, end, weight);
        edgeWeight += weight;
        edgeCount++;
        begin->outDegree++;
        if (!directed)
			{
				end->addEdge(end, begin, weight);
				end->outDegree++;
			}
        else end->inDegree++;
    }

    void addEdge(node* begin, node* end)
    {
        addEdge(begin, end, 1);
    }

    void addEdge(N begin, N end, E weight)
    {
		if(!findEdge(begin, end)){
			addEdge( findNode(begin), findNode(end), weight );
		}
    }

    void addEdge(N begin, N end)
    {
        addEdge(begin, end, 1);
    }

    void deleteEdge(node* begin, node* end)
    {
        int weightLost = begin->deleteEdge(begin, end);
        
        edgeCount--;
        edgeWeight -= weightLost;

        if(!directed) end->deleteEdge(end, begin);
    }

    void deleteEdge(N begin, N end)
    {
        deleteEdge(findNode(begin), findNode(end));
    }

    edge* findEdge(node* start, node* end)
    {
        for(size_t i = 0; i < nodeCount; i++)
			{
				EdgeSeq* nodeEdges = &(nodes[i]->edges);
            
				for(auto it : *nodeEdges)
					{
						if ((it)->nodes[0] == start && (it)->nodes[1] == end) return (it);
					}
			}
        return nullptr;   
    }

    edge* findEdge(N start, N end)
    {
        return findEdge(findNode(start), findNode(end));
    }

    double calculateDensity()
    {
        if (directed) return (double) edgeCount/(nodeCount*(nodeCount-1));
        return 2*((double) edgeCount/(nodeCount*(nodeCount-1)));
    }

    bool isDense()
    {
        if (calculateDensity() >= parameterOfDensity) return true;
        return false;
    }

    void printNodes()
    {
        for(size_t i = 0; i < nodeCount; i++)
			{
				std::cout << nodes[i]->getData() << " ";
			}  
        std::cout << std::endl;
    }

    void printAdjacencyList()
    {
        for(size_t i = 0; i < nodeCount; i++)
			{
				std::cout << nodes[i]->getData() << " -> ";
				EdgeSeq* nodeEdges = &(nodes[i]->edges);
            
				for(auto it : *nodeEdges)
					{
						std::cout << (*it).nodes[1]->getData() << " ";
					}
				std::cout << std::endl;
			}
    }

    self prim()
    {
        if (!directed)
			{
				Graph *primMST = new Graph(false);

				primMST->addNode(nodes[0]->getData());

				NodeSeq &visited = (primMST->nodes);

				while(visited.size() != nodeCount)
					{
						int minWeight = __INT_MAX__;
						edge *edgeToAdd = new edge();
						for(int i = 0; i < primMST->nodeCount; i++)
							{
								EdgeSeq* nodeEdges = &((findNode(visited[i]->getData())->edges));
            
								for(auto it = (*nodeEdges).begin(); it != (*nodeEdges).end(); ++it)
									{
										edge* &currentEdge = *it;
										if ( currentEdge->weight < minWeight ) 
											{
												bool isNotInVisited = findNode(currentEdge->nodes[1], visited) == nullptr;
												if ( isNotInVisited )
													{
														minWeight = currentEdge->weight;
														edgeToAdd = currentEdge;
													}
											}
									}
							}
						primMST->addNode(edgeToAdd->nodes[1]->getData());
						primMST->addEdge(edgeToAdd->nodes[0]->getData(), edgeToAdd->nodes[1]->getData(), edgeToAdd->weight);
					}

				return *primMST;
			} else throw std::runtime_error("No se puede aplicar el Algoritmo de Prim a un Grafo direccionado");
    } 


    ~Graph()
    {
        /* while(!nodes.empty())
			{
				delete nodes.back();
				nodes.pop_back();
			} */
    }
    
    void DeepFirstSearch(N data)
    {
        NodeSeq visitedNodes,test;
        test= DeepFirstSearch( findNode(data), visitedNodes);
        printNodeSeq(test);
    }

    NodeSeq DeepFirstSearch_NodeSeq(N data)
    {
        NodeSeq visitedNodes,test;
        test= DeepFirstSearch( findNode(data), visitedNodes);
        return test;
    }

    NodeSeq DeepFirstSearch(node* origen, NodeSeq visitedNodes){
		visitedNodes.push_back(origen);
            
		EdgeSeq* nodeEdges = &(origen->edges);
		for(auto it : *nodeEdges)
            {   
                if (!visited((*it).nodes[1], visitedNodes ))
					{
						visitedNodes= DeepFirstSearch((*it).nodes[1], visitedNodes);
					}

            }
		return visitedNodes;        
    }

    bool visited(node* node , NodeSeq visitedNodes){
        for (int i = 0; i < visitedNodes.size(); ++i)
        {
				if (node == visitedNodes[i])
					{
						return true;
					}
			}
        return false;
    }

    void printNodeSeq(NodeSeq list){
        for (int i = 0; i < list.size(); ++i)
			{
				std::cout<<list[i]->getData()<<' ';;
			}
		std::cout<<std::endl;
    }

    void BreadthFirstSearch(N data)
    {
        NodeSeq visitedNodes , queue , result;
        queue.push_back(findNode(data));
        result = BreadthFirstSearch( findNode(data), visitedNodes, queue );
        printNodeSeq(result);

    }

    NodeSeq BreadthFirstSearch (node* origen, NodeSeq visitedNodes, NodeSeq queue)
    {
        origen = queue[0];
        visitedNodes.push_back(origen);
        EdgeSeq* nodeEdges = &(origen->edges);

        for(auto it : *nodeEdges)
			{   
				if (!visited((*it).nodes[1], visitedNodes ) && !visited((*it).nodes[1], queue ))
					{
						queue.push_back((*it).nodes[1]); 
					}
			}
        queue.erase(queue.begin());
        
        if (queue.empty() )
			{
				return visitedNodes;
			}else{
			return visitedNodes =BreadthFirstSearch(origen, visitedNodes, queue);
           
        }
    }

    struct edgeCmp
    {
        bool operator()(const edge* a, const edge* b)
        {
            return (a->weight < b->weight);
        }
    };

    bool createsCycle(edge* newEdge, int* disjointSet)
    {
        int nodeBeginParent = disjointSetPlaces[newEdge->nodes[0]->getData()];
        int nodeEndParent = disjointSetPlaces[newEdge->nodes[1]->getData()];
        
        while (disjointSet[nodeBeginParent] >= 0) nodeBeginParent = disjointSet[nodeBeginParent];
        
        while (disjointSet[nodeEndParent] >= 0) nodeEndParent = disjointSet[nodeEndParent];

        if (nodeBeginParent == nodeEndParent) return true;
        else
			{
				disjointSet[nodeEndParent] = nodeBeginParent;
				return false;
			} 
    }

    self kruskal()
    {
        Graph* kruskalMST = new Graph(false);
        EdgeSeq allEdges;
        int disjointSet[nodeCount] = {};

        for(size_t i = 0; i < nodeCount; i++)
			{
				disjointSet[i] = -1;
				disjointSetPlaces[nodes[i]->getData()] = i;
			}

        for(size_t i = 0; i < nodeCount; i++)
			{
				EdgeSeq &currentNodeEdges = nodes[i]->edges;
				allEdges.insert(allEdges.end(), currentNodeEdges.begin(), currentNodeEdges.end());
			}

        allEdges.sort(edgeCmp());

        for(auto i = allEdges.begin(); i != allEdges.end(); ++i)
			{
				if (!createsCycle(*i, disjointSet))
					{
						kruskalMST->addNode((*i)->nodes[0]->getData());
						kruskalMST->addNode((*i)->nodes[1]->getData());

						kruskalMST->addEdge(
											(*i)->nodes[0]->getData(),
											(*i)->nodes[1]->getData()
											);
					}
			}

        return *kruskalMST;
    }

    bool isConnected()
    {
        EdgeSeq allEdges;
        int disjointSet[nodeCount] = {};

        for(size_t i = 0; i < nodeCount; i++)
			{
				disjointSet[i] = -1;
				disjointSetPlaces[nodes[i]->getData()] = i;
			}

        for(size_t i = 0; i < nodeCount; i++)
			{
				EdgeSeq &currentNodeEdges = nodes[i]->edges;
				allEdges.insert(allEdges.end(), currentNodeEdges.begin(), currentNodeEdges.end());
			}
        
        for(auto i = allEdges.begin(); i != allEdges.end(); ++i)
			{
				createsCycle(*i, disjointSet);
			}

        int counter = 0;

        
        for(size_t i = 0; i < nodeCount; i++)
			{
				if (disjointSet[i] < 0) counter++;   
			}

        if (counter > 1) return false;
        return true;
    }

	bool isBipartite() 
    {
		std::map<N, int> color;
		for(int i=0; i<nodeCount; i++){
			//-1 no visitado
			color[nodes[i]->getData()] = -1;
		}
		color[nodes[0]->getData()] = 1;
		std::queue<node*> q;
		q.push(nodes[0]);
		while(!q.empty()){
			node* u = q.front(); q.pop();
			EdgeSeq* nodeEdges = &(u->edges);
			for(auto it : *nodeEdges){
				if(color[(*it).nodes[1]->getData()] == -1 ){
					color[(*it).nodes[1]->getData()] = 1 - color[u->getData()];
					q.push((*it).nodes[1]);
				}else if(color[(*it).nodes[1]->getData()] == color[u->getData()]){
					return false;
				}

            }
		}
		return true;
	}
    bool isFuertementeConexo()
    {
        for(size_t i = 0; i < nodeCount; i++)
			{
				int cant_nodes = DeepFirstSearch_NodeSeq(nodes[i]->getData()).size();

				if (cant_nodes!= nodeCount)
					{
						return false;
					}
			}
        return true;

    }

    self bellmanFord(N start){
        Graph* bellmanFord = new Graph(true);
    
        int int_max = std::numeric_limits<int>::max();
        std::vector<edge*> totalEdges;
        std::map<node*, int> distance;
        std::map<node*, int> cost;
        std::map<node*,node*> prev;


        for (int i = 0; i < nodes.size(); ++i)

        {
            bellmanFord->addNode(nodes[i]->getData());

            for (edge* n : nodes[i]->edges)
            {
                totalEdges.push_back(n);
            }
        }

        for (int i = 0; i < nodes.size(); ++i)
        {
            distance[nodes[i]]=int_max;
            prev[nodes[i]]=nullptr;
        }

        distance[findNode(start)]=0;
        for(int j=0; j< nodes.size()-1;j++){

            for (int i = 0; i < totalEdges.size(); ++i)
                {

                    if((distance[totalEdges[i]->nodes[0]]) != int_max )  
                        {
                            if ((distance[totalEdges[i]->nodes[0]]+totalEdges[i]->weight) < distance[totalEdges[i]->nodes[1]])
                                {
                                    distance[totalEdges[i]->nodes[1]] = (distance[totalEdges[i]->nodes[0]]+totalEdges[i]->weight);
                                    prev[totalEdges[i]->nodes[1]] = totalEdges[i]->nodes[0];    
                                    cost[totalEdges[i]->nodes[1]] = totalEdges[i]->weight;
                                }
                        }

                }
		}

        for (int i = 0; i < nodes.size(); ++i){
            if(prev[nodes[i]]){
                bellmanFord->addEdge(nodes[i]->getData(),prev[nodes[i]]->getData(),cost[nodes[i]]);
            }
        }
        for (int i = 0; i < totalEdges.size(); ++i)
                {
                    if((distance[totalEdges[i]->nodes[0]]) != int_max )  {
                        if ((distance[totalEdges[i]->nodes[0]]+totalEdges[i]->weight) < distance[totalEdges[i]->nodes[1]])
                        {
                            std::cout<<"This graph has negative cycles."<<std::endl;
                        }

                    }
			}
        
        return *bellmanFord;
    }
    
    self greedy_bfs(N start, N end){
        Graph* greedy = new Graph(true);
    
        node* begin  = this->findNode(start);
        node* last = this->findNode(end);
        node* temp = begin;
        NodeSeq visited_list , route;
        visited_list.push_back(begin);
        route.push_back(begin);
        

        for (int i = 0; i < nodes.size(); ++i)
        {
            greedy->addNode(nodes[i]->getData());   
        }

        while(visited_list.back() != last)
        { 
            node* menor_vecino = nullptr;
            int menor_weight = std::numeric_limits<int>::max();
            EdgeSeq* nodeEdges = &(temp->edges);

            for(auto it : *nodeEdges){
                if (it->weight < menor_weight && !visited(((*it).nodes[1]),visited_list)){
                               menor_weight = it->weight;
                               menor_vecino = it->nodes[1];
                    }
            }

            if(menor_vecino){
                visited_list.push_back(menor_vecino);
                route.push_back(menor_vecino);
                temp= route.back();
            }else{
                route.pop_back();
                temp= route.back();
            }
        }        
 
        for (int i = 0; i < route.size()-1; ++i)
        {
           greedy->addEdge(    route[i+1]->getData(),route[i]->getData()  , (this-> findEdge( route[i]->getData() , route[i+1]->getData() )  ) ->weight );
        }
        return *greedy;

    }


void printRoute(){
    for (int i = 0; i < nodes.size(); ++i)
    {
        std::cout<<nodes[i]->getData()<<" -> ";
        int peso = 0;
        peso = nodes[i]->route(peso);
        std::cout<<"\tPeso: "<<peso<<std::endl;
    }
}

    std::map<N, std::pair<double, N>> Astar(N begin, N end)
    {
		//bfs to get coords
		std::queue<node*> q;
		std::map<N, bool> visit;
		q.push(nodes[0]);
		std::map<N , std::pair<double, double>> coord;
		while(!q.empty()){
			node* u = q.front(); q.pop();
			EdgeSeq* nodeEdges = &(u->edges);

			for(auto it : *nodeEdges){
				if( !visit[(*it).nodes[1]->getData()] ){
					visit[(*it).nodes[1]->getData()] = true;
					coord[(*it).nodes[1]->getData()] = std::make_pair((*it).nodes[1]->getX(), (*it).nodes[1]->getY());
					q.push((*it).nodes[1]);
				}
            }
		}
		//fin get coords
		//DISTANCIA: manhattan(coord,begin,end);
		std::map<N,double> h;
		for(auto it : coord) {
			h[it.first] = manhattan(coord, it.first, end);
		}
		//Structure to save data 
		//Graph* astar = new Graph(false);
		std::map<N, std::pair<double, N>> table;
		std::queue<node*> q2;

		visit.clear();
		q2.push(findNode(begin));
		visit[begin] = true;
		table[begin] = std::make_pair(h[begin], '\0');
		node* u;
		do{
			u = q2.front(); q2.pop();
			EdgeSeq* nodeEdges = &(u->edges);
			double min = std::numeric_limits<double>::max();
			for(auto it : *nodeEdges){
				if( !visit[(*it).nodes[1]->getData()] ){
					N data = (*it).nodes[1]->getData();
					double w = (*it).weight;
					if( min > h[data] + w){
						min = h[data] + w;
					}
					table[data] = std::make_pair(h[data] + w, u->getData());
				}
            }
			min = std::numeric_limits<double>::max();
			N datamin;
			for(auto it : table){
				if( !visit[it.first] && min > it.second.first ){
					min = it.second.first;
					datamin = it.first;
				}
			}
			visit[datamin] = true;
			q2.push(findNode(datamin));			
		}while( !q2.empty() && u->getData()!=end);

		N iter = end;
		while( table[iter].second != begin){
			//isenter it table[iter.second ]
			//astar->addEdge(iter, table[iter].second, findEdge(iter, table[iter].second)->weight);
			iter = table[iter].second;
		}
		//astar->addEdge(iter, table[iter].second, findEdge(iter, table[iter].second)->weight);

        
        for(auto i: table)
        {
            std::cout << i.first << " " << i.second.first << " " << i.second.second << std::endl; 
        }
        

		return table;
    }
    
    struct nodeCmp
    {
        bool operator()(node* a, node* b)
        {
            return (a->getData() < b->getData());
        }
    };

    std::vector<std::vector<int>> floydWarshall()
    {
        sort(nodes.begin(), nodes.end(), nodeCmp());
        int inf = 999999;
        std::vector<std::vector<int>> iterations(nodeCount, std::vector<int>(nodeCount, 0));

        for (int i = 0; i < nodeCount; i++)
        {
            int k = 1;
            for (int j = 0; j < nodeCount; j++)
            {
                if (i != j)
                {
                    iterations[i][j] = k;
                }
                    
                k++;
            }
        }

        int n_size = nodes.size();

        std::map<node*, int> pos;

        for (int i = 0; i < nodes.size(); i++)
        {
            pos[nodes[i]] = i;
        }

        std::vector<std::vector<int>> dist(n_size, std::vector<int>(n_size, inf));

        for (int i = 0; i < nodes.size(); i++)
        {
            dist[i][i] = 0;
        }

        for (ni = nodes.begin(); ni != nodes.end(); ++ni)
        {
            EdgeSeq &edgeList = (*ni)->edges;
            for (ei = edgeList.begin(); ei != edgeList.end(); ++ei)
            {
                edge* &currentEdge = *ei;
                dist[pos[currentEdge->nodes[0]]][pos[currentEdge->nodes[1]]] = currentEdge->weight;
            }
        }

        for (int k = 0; k < n_size; k++)
        {
            for (int i = 0; i < n_size; i++)
            {
                for (int j = 0; j < n_size; j++)
                {
                    if (dist[i][j] > dist [i][k] +  dist[k][j])
                    {
                        dist[i][j] = dist [i][k] +  dist[k][j];
                        iterations[i][j] = k;
                    }
                }
            }
        }

        // DEBUG
        for (int i = 0; i < dist.size(); i++)
        {
            for (int j = 0; j < dist[i].size(); j++)
            {
                if (dist[i][j] != inf)
                    std::cout << dist[i][j] << "\t";
                else
                    std::cout << "x\t";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        
        for (int i = 0; i < dist.size(); i++)
        {
            for (int j = 0; j < dist[i].size(); j++)
            {
                std::cout << iterations[i][j] << "\t";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;

        return dist;
    }
    
    int minDistance(std::vector<std::pair<node*, int>> distances, std::vector<bool> visited)
    {
        int min = 9999;
        int min_index = 9999;
        
        for (int i = 0; i < distances.size(); i++)
        {
            if (visited[i] == false && std::get<1>(distances[i]) <= min)
            {
                min = std::get<1>(distances[i]);
                min_index = i;
            }
        }
        return min_index;
    }

    int findIndexOfNode(node* n)
    {
        for (int i = 0; i < nodeCount; i++)
        {
            if (n == nodes[i])
                return i;
        }
        return -1;
    }

    void dijkstra(node* start)
    {
        sort(nodes.begin(), nodes.end(), nodeCmp());
        int inf = 9999;
        std::vector<std::pair<node*, int>> distances(nodeCount, std::make_pair(nullptr, 9999));

        std::vector<bool> visited(nodeCount, false);

        for (int i = 0; i < distances.size(); i++)
        {
            std::get<0>(distances[i]) = nodes[i];
        }

        distances[findIndexOfNode(start)] = std::make_pair(start, 0);

        for (int i = 0; i < nodeCount; i++)
        {
            
            int u = minDistance(distances, visited);
            visited[u] = true;

            node* &currentNode = nodes[u];

            for (int j = 0; j < currentNode->edges.size(); j++)
            {
                int v = findIndexOfNode(currentNode->edges[j]->nodes[1]);
                
                if (!visited[v] && (std::get<1>(distances[u]) + currentNode->edges[j]->weight < std::get<1>(distances[v])))
                    {
                        std::get<1>(distances[v]) = std::get<1>(distances[u]) + currentNode->edges[j]->weight;
                    }
            }
        }

        //DEBUG: Print
        std::cout << "Distances, starting from " << start->getData() <<":" << std::endl;
        for (int i = 0; i < distances.size(); i++)
        {
            std::cout << (std::get<0>(distances[i]))->getData() << "  " << std::get<1>(distances[i]) << std::endl;
        }
    }
};

/* 
    Now Graph<Traits> is defined as graph. 
    Hence, the correct instantiation of a Graph would be:
        graph* myGraph = new graph;
*/
typedef Graph<Traits> graph;
#endif
