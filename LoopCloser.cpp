//
// Created by JimXing on 13/11/16.
//

#include "LoopCloser.h"


LoopCloser::LoopCloser(const string &name, DataManager &data_reference) : ProcessingEngine(name),
                                                                          data_reference(data_reference) {}
