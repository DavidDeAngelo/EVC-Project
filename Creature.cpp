#include "Creature.h"


using namespace EVC;
using namespace std;

Creature::Creature() {

}

Creature::~Creature() {
	delete genome;
	std::for_each(std::begin(limbs), std::end(limbs), [](Limb*l) {delete l; });

}

Creature::Creature(Graph* graph, int limit) {
	genome = graph;
	queue<Edge*> Queue = queue<Edge*>();
	for (auto &i : genome->getRoot()->getChildren()) {
		Queue.push(i);
	}

	while (Queue.size() != 0) {
		std::cout << "::" << Queue.size();
		Edge* currentEdge = Queue.front();
		Node* currentNode = currentEdge->getChild();
		Queue.pop();
		currentEdge->visit();
		if (currentEdge->recursiveDepth <= limit) {
			std::cout << "Recursive Limit:" << currentEdge->recursiveDepth;
			for (auto &i : currentNode->getChildren()) {
				Queue.push(i);
			}
		}
	}

}