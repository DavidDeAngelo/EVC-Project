#pragma once

#ifndef GRAPH_DEF_GUARD
#define GRAPH_DEF_GUARD
#include <vector>
#include "btBulletDynamicsCommon.h"
#include "Node.h"
#include "Edge.h"
#include <algorithm>
#include <iostream>

namespace EVC {

	class Graph {
		std::vector<Node*> nodes;
		std::vector<Edge*> edges;
		int recursive_limit = 5;
		Node* root;
	public:
		Graph(const btVector3& ori, const btVector3& objectSize);
		~Graph();
		void addNode(const btVector3& ori, const btVector3& objectSize);
		bool addEdge(Node* parent, Node* child);
		bool addEdge(int pIndex, int cIndex);
		Node* getNode(int index);
		Edge* getEdge(int index);
		Node* getRoot();
		void resetGraph();
		void breathFirstSearch(int limit);

	};

}

#endif // !GRAPH_DEF_GUARD
