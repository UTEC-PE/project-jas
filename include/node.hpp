#ifndef NODE_HPP
#define NODE_HPP

template <typename G>
class Node {
public:
	typedef typename G::N N;
	typedef typename G::E E;
	typedef typename G::edge edge;
	typedef typename G::EdgeSeq EdgeSeq;
	EdgeSeq edges;

private:
	N data;
	double x;
	double y;
	
public:

};

#endif
