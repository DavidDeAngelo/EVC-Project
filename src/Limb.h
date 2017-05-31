#pragma once
#ifndef  LIMB_DEF_HEADER
#define LIMB_DEF_HEADER
#include "Edge.h"
#include "Node.h"
namespace EVC {
	class Node;
	class Edge;
	class Limb {

		int parentLinkIndex;
		int childLinkIndex;
		Edge* edge;
	public:
		Limb(int parent, int child, Edge* edge);
		~Limb();


	};

}


#endif // ! LIMB_DEF_HEADER
