#include "Optimiser.h"
#include "Util.h"
#define DEBUG_OPTIMISER

void Optimiser::process(DataManager& data, int frameIdx)
{
	if (frameIdx != 0) {
#ifdef DEBUG_OPTIMISER
	cout << "full BA processing node for frameIdx:" << frameIdx << endl;
#endif
		Util::GlobalBundleAdjustemnt(data);
	}
}

bool Optimiser::validationCheck(DataManager &data, int frameIdx) {
	return true;
}