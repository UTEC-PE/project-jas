#ifndef NODE_HPP
#define NODE_HPP

struct Point{
	double x,y,z;
	Point(double x=0, double y=0, double z=0)
		: x(x), y(y), z(z)
	{};
	Point(Point p){
		(*this) = p;
	}
	Point operator=(const Point other){
		x = other.x;
		y = other.y;
		z = other.z;
		return *this;
	}
};

template <typename T>
struct Node{
	T data;
	Point position;
	Node* next;
	Node (T data, Point position, Node* next = nullptr)
		: data(data), next(next), position(position)
	{}
	void killSelf(){
		if(next) next->killSelf();
		delete this;
	}
	bool operator==(const Node other){
		return data == other;
	}
};

struct NodeWeight{
	int weight;
	//TODO
};

#endif
