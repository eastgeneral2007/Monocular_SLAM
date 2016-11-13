//
// Created by JimXing on 13/11/16.
//

#include "LocalMapper.h"

LocalMapper::LocalMapper(const string &name, KeyFrame *current_keyframe, DataManager &data_reference)
        : ProcessingEngine(name), current_keyframe(current_keyframe), data_reference(data_reference) {}
