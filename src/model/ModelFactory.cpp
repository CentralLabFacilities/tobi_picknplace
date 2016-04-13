/*
 * ModelFactory.cpp
 *
 *  Created on: Nov 30, 2015
 *      Author: plueckin
 */

#include "ModelFactory.h"

#include "types/H2R5.h"
#ifdef USE_KATANA
#include "types/Katana.h"
#include "types/KatanaSim.h"
#endif //USE_KATANA

Model::Ptr ModelFactory::create(std::string const &type) {

#ifdef USE_KATANA
    if(type == KATANA_NAME)
        return Katana::Ptr(new Katana());
    if(type == KATANA_SIM_NAME)
        return KatanaSim::Ptr(new KatanaSim());
#endif //USE_KATANA
    if(type == H2R5_NAME)
        return H2R5::Ptr(new H2R5());

    return Model::Ptr();

}
