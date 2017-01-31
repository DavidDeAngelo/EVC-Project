#include "Graph.h"
#include "Creature.h"
#include <vld.h>

using namespace EVC;
using namespace std;

Graph::Graph(const btVector3& ori, const btVector3& objectSize) {
	Node* node = new Node(ori, objectSize);
	root = node;
	nodes.push_back(node);
	//addNode(ori,objectSize);
}

Graph::~Graph() {
	std::for_each(std::begin(nodes), std::end(nodes), [](Node* n) {delete n; });
	std::for_each(std::begin(edges), std::end(edges), [](Edge* e) {delete e; });
}

void Graph::addNode(const btVector3& ori, const btVector3& objectSize) {
	nodes.push_back(new Node(ori, objectSize));
}

bool Graph::addEdge(Node * parent, Node * child)
{
	Edge* edge = new Edge(parent, child, 1);
	parent->addOutEdge(edge);
	child->addInEdge(edge);
	edges.push_back(edge);
	return true;
}

bool Graph::addEdge(int pIndex, int cIndex)
{
	return (pIndex > nodes.size() || cIndex > nodes.size()) ? false : addEdge(nodes[pIndex], nodes[cIndex]);
}

Node* Graph::getNode(int index) {
	return (index > nodes.size() - 1) ? nullptr : nodes[index];
}

Edge* Graph::getEdge(int index) {
	return (index > edges.size() - 1) ? nullptr : edges[index];
}

Node* Graph::getRoot() {
	return root;
}

void Graph::breathFirstSearch(int limit) {

	queue<Edge*> Queue = queue<Edge*>();
	for (auto &i : root->getChildren()) {
		Queue.push(i);
	}

	while (Queue.size() != 0) {
		Edge* currentEdge = Queue.front();
		Node* currentNode = currentEdge->getChild();
		Queue.pop();
		currentEdge->visit();
		if (currentEdge->recursiveDepth <= limit)
			std::cout << "Recursive Limit:" << currentEdge->recursiveDepth;
		for (auto &i : currentNode->getChildren()) {
			Queue.push(i);
		}
	}
}
//#define DEBUG_GRAPH
#ifdef DEBUG_GRAPH
int main()
{

	btVector3 ori = btVector3(1, 1, 1);
	Graph* g = new Graph(ori, btVector3(.3, .3, .3));
	g->addEdge(0, 0);
	g->addEdge(0, 0);

	Creature* c = new Creature(g, 1);
	delete c;


	int age;
	std::cin >> age;
}
#endif // DEBUG


