#pragma once
#ifndef CREATURE_HEADER_GUARD
#define CREATURE_HEADER_GUARD
#include <vector>
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "Limb.h"
#include "Graph.h"

namespace EVC {

	class Creature {
		btMultiBody* body;
		std::vector<Limb*> limbs;
		Graph* genome = nullptr;
		Graph* phenotype = nullptr;
		int recursiveLimit = 2;
		int numLinks = 0;
		void calcNumLinks();
	public:
		Creature();
		Creature(Graph* genome, int limit);
		~Creature();
		bool createCreature(); //returns true if creature is parsed into multibody


	};

}

#endif // !CREATURE_HEADER_GUARD
