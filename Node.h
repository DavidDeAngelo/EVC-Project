#pragma once

#ifndef NODE_DEF_GUARD
#define NODE_DEF_GUARD
#include <vector>
#include "btBulletDynamicsCommon.h"
#include "Edge.h"

namespace EVC {
	class Edge;
	class Node {
		btVector3 orientation = btVector3(0.f, 0.f, 0.f);
		btVector3 size = btVector3(.5f, .5f, .5f);
		int recursiveDepth = 0;
	public:
		std::vector<Edge*> parentEdges;
		std::vector<Edge*> childEdges;
		Node(const btVector3& ori, const btVector3& size);
		~Node();
		void addInEdge(Edge* edge);
		void addOutEdge(Edge* edge);
		void visit();
		const std::vector<Edge*>& getChildren() const { return childEdges; }

	};

}
#endif // !NODE_DEF_GUARD
