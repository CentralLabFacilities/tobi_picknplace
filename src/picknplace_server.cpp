/*
 * picknplace_server.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#include "control/Controller.h"
#include "control/ViaPoseStrategy.h"
#include "model/KatanaModel.h"
#include "model/KatanaSimModel.h"
#include "interface/RsbInterface.h"
#include "interface/ViewInterface.h"
#include <boost/program_options.hpp>

#include <ros/ros.h>

#include <iostream>
#include "util/TransitionReader.h"

using namespace std;
using namespace boost::program_options;

options_description desc("Allowed options");
variables_map vm;

int main(int argc, char **argv) {

	cout << "############################" << endl;
	cout << "## tobi_picknplace_server ##" << endl;
	cout << "############################" << endl;
	cout << endl << "init ..." << endl;

	try {
		//handle cmdline args with boost::programoptions
		desc.add_options() //
		("help", "shows help message") //
		("debug", "debug mode") //
		("sim", "simulation mode") //
		("transitions,t", value<string>(), "file describing possible transitions")  // transitions
				;

		store(
				command_line_parser(argc, argv).options(desc).style(
						command_line_style::unix_style ^ command_line_style::allow_short).run(),
				vm);
		notify(vm);
	} catch (boost::program_options::error &e) {
		cerr << e.what() << " use --help" << endl;
		return 1;
	}

	if (vm.count("help")) {
		cout << desc << endl;
	}

	ros::init(argc, argv, "tobi_picknplace_server");

    ros::console::Level level = ros::console::levels::Info;
    if (vm.count("debug")) {
        level = ros::console::levels::Debug;
    }
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

	Model::Ptr katana;
	if (vm.count("sim")) {
		katana = Model::Ptr(new KatanaSimModel());
	} else {
		KatanaModel::Ptr model(new KatanaModel());
		katana = model;
	}

	ViaPoseStrategy::Ptr strategy(new ViaPoseStrategy(katana));
	if (vm.count("transitions")) {
		TransitionsReader reader;
		vector<Transition> t = reader.read(vm["transitions"].as<string>());
		strategy->setTransitions(t);
	}

	RsbInterface::Ptr rsbInterface(new RsbInterface("/arm/picknplace/server"));
	ViewInterface::Ptr viewInterface(new ViewInterface());

	Controller controller(katana, strategy);
	controller.addControlInterface(rsbInterface);
	controller.addControlInterface(viewInterface);

	cout << "running ..." << endl;

	ros::spin();
	return 0;
}
