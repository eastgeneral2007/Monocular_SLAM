#ifndef OPTIMISER_H
#define OPTIMISER_H
#include "ProcessingNode.h"

class Optimiser : public ProcessingNode {
public:
    // add any local variable needed

public:
    // Constructor
    Optimiser():ProcessingNode("Optimiser"){};

    // Optimiser process
    virtual void process(DataManager& data, int frameIdx) override;
    virtual bool validationCheck(DataManager& data, int frameIdx) override;
};

#endif