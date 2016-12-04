//
// Created by JimXing on 19/11/16.
//

#include "../../src/CommonCV.h"
#include "../../src/Converter.h"
#include "../../src/Util.h"
#include "../../src/DataManager.h"
#include "../../src/ParamConfig.h"

#ifndef SUPERBUILD_UNITTESTG2O_H
#define SUPERBUILD_UNITTESTG2O_H

class UnitTestg2o {
public:
    static void unitTestCvMatToG2oSE3();
    static void unitTestFullBA();
    static double differenceInRtL2(Frame *f1, Frame *f2);
    static double differenceInXYZL2(MapPoint *m1, MapPoint *m2);
};

#endif //SUPERBUILD_UNITTESTG2O_H
