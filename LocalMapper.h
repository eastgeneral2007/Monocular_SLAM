//
// Created by JimXing on 13/11/16.
//

#ifndef MONOCULAR_SLAM_LOCALMAPPER_H
#define MONOCULAR_SLAM_LOCALMAPPER_H


#include "ProcessingEngine.h"

class LocalMapper : public ProcessingEngine{
public:
    // Constructor
    LocalMapper();

    // localMapper main function
    virtual void process(DataManager& data) override;



};


#endif //MONOCULAR_SLAM_LOCALMAPPER_H
