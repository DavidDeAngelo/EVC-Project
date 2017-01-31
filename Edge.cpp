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

void Edge::visit() {
	recursiveDepth++;
	//std::cout << "Recursive Limit:" << recursiveDepth;

}

