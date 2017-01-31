#pragma once
#ifndef CREATURE_HEADER_GUARD
#define CREATURE_HEADER_GUARD
#include <vector>
#include "Limb.h"
#include "Graph.h"
namespace EVC {

	class Creature {
		std::vector<Limb*> limbs;
		Graph* genome;
	public:
		Creature();
		Creature(Graph* genome, int limit);
		~Creature();


	};

}

#endif // !CREATURE_HEADER_GUARD
