#include "Node.h"
#include "Edge.h"
#include <iostream>


using namespace EVC;


Node::Node(const btVector3& ori, const btVector3& objectSize) {
	orientation = ori;
	size = objectSize;
}
Node::~Node() {

}

void Node::addInEdge(Edge* edge) {
	parentEdges.push_back(edge);
}

void Node::addOutEdge(Edge* edge) {
	childEdges.push_back(edge);
}

int Node::getDepth() {
	return recursiveDepth;
}

