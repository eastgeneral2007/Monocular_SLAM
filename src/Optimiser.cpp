#include "Optimiser.h"
#include "Util.h"
#include "ParamConfig.h"
#define DEBUG_OPTIMISER

void Optimiser::process(DataManager& data, int frameIdx)
{
	if (frameIdx != 0) {
#ifdef DEBUG_OPTIMISER
	cout << "startig full BA processing node for frameIdx:" << frameIdx << endl;
#endif
		Util::GlobalBundleAdjustemnt(data, FULL_BA_ITER); // use the tuned parameter 
		
#ifdef DEBUG_OPTIMISER
	cout << "finished full BA for frameIdx:" << frameIdx << endl;
#endif		
	}
}

bool Optimiser::validationCheck(DataManager &data, int frameIdx) {
	return true;
}