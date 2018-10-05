<<<<<<< HEAD
#include <iostream>
#include <vector>
using namespace std;

template <typename T>
class Node
{
private:
    vector <Node<T>*> adjList;
    int outDegree;
public:
    T data;
    Node<T>(): outDegree(0) {};
    Node<T>(T newData): data(newData), outDegree(0) {};

    void addAdjacent(Node<T>* adjNode)
    {
        adjList.push_back(adjNode);
        outDegree++;
    }

    void removeAdjacent(Node<T>* adjNode)
    {
        for(int i = 0; i < adjList.size(); i++)
        {
            if (adjList[i] == adjNode)
            {
                adjList.erase(i);
                outDegree--;
                break;
            }
        }
    }

    int numberOfAdjacentNodes()
    {
        return outDegree;
    }

};
=======
#ifndef NODE_HPP
#define NODE_HPP

template <typename G>
class Node {
	N data;
	double x;
	double y;
	
public:
	typedef typename G::N N;
	typedef typename G::E E;
	typedef typename G::edge edge;
	typedef typename G::EdgeSeq EdgeSeq;
	EdgeSeq edges;
};

#endif
>>>>>>> 0033e74d202639283ff9fd1b825941e2cc59271b
