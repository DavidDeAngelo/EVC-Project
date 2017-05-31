#include "Edge.h"

using namespace EVC;



Edge::Edge(Node* parent, Node* child, btScalar scale) {
	parentNode = parent;
	childNode = child;
	childScale = scale;
}

Edge::~Edge() {

}

Node* Edge::getParent() {
	return parentNode;
}

Node* Edge::getChild() {
	return childNode;
}

btScalar Edge::getScale() {
	return childScale;
}

//use Queue Ids, to identify the same edge in the queue multiple times
//start at default 0
//pop current, get current depth
//visit using current, increment depth by +1
//if we push the same edge more then once, we visit using current+1, increment depth by 1
