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
    if(type == KATANA_NAME){
	Model::Ptr katana;
	katana = Model::Ptr(new Katana());
        return katana;}
    //if(type == KATANA_SIM_NAME)
    //    return KatanaSim::Ptr(new KatanaSim());
#endif //USE_KATANA
    if(type == H2R5_NAME){
        return H2R5::Ptr(new H2R5());}
    std::cout << "Load default Model" << std::endl;
    return Model::Ptr();

}
