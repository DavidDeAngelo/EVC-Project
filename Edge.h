#pragma once
#ifndef EDGE_DEF_HEADER
#define EDGE_DEF_HEADER
#include <iostream>
#include "btBulletDynamicsCommon.h"
#include "Node.h"

namespace EVC {


	class Node;
	class Edge {
		Node* parentNode;
		Node* childNode;
		btScalar childScale;
	public:
		Edge(Node* parent, Node* child, btScalar scale);
		~Edge();
		Node* getParent();
		Node* getChild();
		btScalar getScale();
	};

}


#endif // !EDGE_DEF_HEADER

