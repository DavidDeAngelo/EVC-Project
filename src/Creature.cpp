#include "Creature.h"
#include <algorithm>
#include <stack>
#include <tuple>
#include <map>
//#include <stdexcept>
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
	recursiveLimit = limit;
}

void Creature::calcNumLinks() {
	/*
	numLinks = 0;
	int currentDepth = 0;
	queue<Edge*> Queue = queue<Edge*>();
	queue<int> depth = queue<int>();

	for (auto &i : genome->getRoot()->getChildren()) {
		if (i->recursiveDepth <= recursiveLimit) {
			numLinks++;
			Queue.push(i);
			depth.push(1);
		}
	}
	while (Queue.size() != 0) {
		Edge* currentEdge = Queue.front();
		Node* currentNode = currentEdge->getChild();
		currentDepth = depth.front();
		Queue.pop();
		depth.pop();
		if (currentDepth <= recursiveLimit) {
			for (auto &i : currentNode->getChildren()) {
				numLinks++;
				Queue.push(i);
				depth.push(currentDepth + 1);
			}
		}
	}

	//genome->resetGraph();
	*/
}
/*
bool Creature::createCreature() {
	if (genome == nullptr)
		return false;
	btVector3 baseHalfExtents(1, 1, 1);
	int currentParentIndex = -1; //start at base 
	int currentChildIndex = 0;
	stack<tuple<Node*, vector<Node*>> > stack;
	//
	vector<Node*> current;
	current.push_back(genome->getRoot());
	stack.push(make_tuple(genome->getRoot(), current));
	
	while (stack.size() != 0) {
		tuple<Node*, vector<Node*>> currentTuple = stack.top();
		stack.pop();
		Node* currentNode = get<0>(currentTuple);
		vector<Node*> currentHistory = get<1>(currentTuple);
		int count = std::count(currentHistory.begin(), currentHistory.end(), currentNode);
		if (count <= recursiveLimit) {
			numLinks++;
			for (auto &i : currentNode->getChildren()) {
				Node* pushNode = i->getChild();
				vector<Node*> pushVector = currentHistory;
				pushVector.push_back(pushNode);
				tuple<Node*, vector<Node*>> pushTuple= make_tuple(pushNode,pushVector);
				stack.push(pushTuple);
			}
		}
	}

	cout << numLinks << "::";
	genome->resetGraph();

	btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
	float baseMass = 1.f;

	if (baseMass)
	{
		btCollisionShape *pTempBox = new btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));
		pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
		delete pTempBox;
	}

	bool canSleep = false;
	btMultiBody *pMultiBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, false, canSleep);


	return true;


}

*/



bool Creature::createCreature() {
	if (genome == nullptr)
		return false;
	btVector3 baseHalfExtents(1, 1, 1);
	int currentParentIndex = -1; //start at base 
	int currentChildIndex = 0;
	stack<tuple<Node*, map<Node*,int>> > stack;
	//
	map<Node*, int> root;
	root[genome->getRoot()]++;
	//cout << root[genome->getRoot()];
	stack.push(make_tuple(genome->getRoot(), root));

	while (stack.size() != 0) {
		tuple<Node*, map<Node*, int>> currentTuple = stack.top();
		stack.pop();
		Node* currentNode = get<0>(currentTuple);
		map<Node*, int> currentHistory = get<1>(currentTuple);
		//int count = std::count(currentHistory.begin(), currentHistory.end(), currentNode);
		int count = currentHistory[currentNode];
		if (count <= recursiveLimit) { // visit node and push all children into the stack
			//do linking stuff here
			numLinks++;
			for (auto &i : currentNode->getChildren()) {
				Node* pushNode = i->getChild();
				map<Node*, int> pushMap = currentHistory;
				pushMap[pushNode]++;
				tuple<Node*, map<Node*, int>> pushTuple = make_tuple(pushNode, pushMap);
				stack.push(pushTuple);
			}
		}
	}

	cout << numLinks << "::";

	btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
	float baseMass = 1.f;

	if (baseMass)
	{
		btCollisionShape *pTempBox = new btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));
		pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
		delete pTempBox;
	}

	bool canSleep = false;
	btMultiBody *pMultiBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, false, canSleep);


	return true;


}

