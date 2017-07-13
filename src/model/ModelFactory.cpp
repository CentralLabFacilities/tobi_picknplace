/*
 * ModelFactory.cpp
 *
 *  Created on: Nov 30, 2015
 *      Author: plueckin
 */

#include "ModelFactory.h"
#include <ros/ros.h>

#include "types/Katana.h"
#include "types/KatanaSim.h"
#define KATANA_NAME "katana"
#define KATANA_SIM_NAME "katana_sim"

Model::Ptr ModelFactory::create(std::string const &type) {

    if(type == KATANA_NAME){
        return Katana::Ptr(new Katana());
    } else {
        return KatanaSim::Ptr(new KatanaSim());
    }

}
