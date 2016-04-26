/*
 * ModelFactory.cpp
 *
 *  Created on: Nov 30, 2015
 *      Author: plueckin
 */

#include "ModelFactory.h"

#include "types/H2R5.h"
//#ifdef USE_KATANA
#include "types/Katana.h"
#include "types/KatanaSim.h"
//#endif //USE_KATANA

Model::Ptr ModelFactory::create(std::string const &type) {

  std::cout << "Factory" << std::endl;
//#ifdef USE_KATANA
    if(type == KATANA_NAME){
        std::cout << "before Katana" << std::endl;
	Katana::Ptr katana = Katana::Ptr(new Katana());
	std::cout << "create Katana" << std::endl;
        return katana;}
    //if(type == KATANA_SIM_NAME)
   //     return KatanaSim::Ptr(new KatanaSim());
//#endif //USE_KATANA
    if(type == H2R5_NAME){
        return H2R5::Ptr(new H2R5());}
    std::cout << "Found error" << std::endl;
    return Model::Ptr();

}
