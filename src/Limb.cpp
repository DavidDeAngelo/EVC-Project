#include "Limb.h"

using namespace EVC;


Limb::Limb(int parent, int child, Edge* srcEdge) {

	parentLinkIndex = parent;
	childLinkIndex = child;
	edge = srcEdge;
}
Limb::~Limb() {

}