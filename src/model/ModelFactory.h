/*
 * ModelFactory.h
 *
 *  Created on: Nov 30, 2015
 *      Author: plueckin
 */

#include "Model.h"

#pragma once

class ModelFactory {
public:
    static Model::Ptr create(std::string const &type);
};

