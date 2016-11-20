//
// Created by JimXing on 13/11/16.
//

#include "LocalMapper.h"

LocalMapper::LocalMapper(const string &name, DataManager &data_reference)
        : ProcessingEngine("LocalMapper"), data_reference(data_reference) {}
