/*
 * ModelFactory.cpp
 *
 *  Created on: Nov 30, 2015
 *      Author: plueckin
 */

#include "ModelFactory.h"
#include <ros/ros.h>


#include "types/H2R5.h"
#ifdef USE_KATANA
#include "types/Katana.h"
#include "types/KatanaSim.h"
#endif //USE_KATANA

#define KATANA_NAME "katana"
#define H2R5_NAME "h2r5"

Model::Ptr ModelFactory::create(std::string const &type) {

#ifdef USE_KATANA
    if(type == KATANA_NAME)
        return Katana::Ptr(new Katana());
    if(type == KATANA_SIM_NAME)
        return KatanaSim::Ptr(new KatanaSim());
#endif //USE_KATANA
    //Only for wrong Build, searching Error only with SegFault is no fun.
    if(type == KATANA_NAME)
	ROS_ERROR("You use Model Katana, but didn't build picknplace with the Parameter WITH_KATANA=On");
    if(type == H2R5_NAME)
        return H2R5::Ptr(new H2R5());

    return Model::Ptr();

}
