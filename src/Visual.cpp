#include <iostream>
#include <vector>
#include <fstream>
#include "MatlabEngine.hpp"
#include "MatlabDataArray.hpp"

using namespace matlab::engine;

std::vector<double> temp;
std::vector<double> xacc;
std::vector<double> yacc;
std::vector<double> zacc;
std::vector<double> kPitch;
std::vector<double> kRoll;
std::vector<double> latitude;
std::vector<double> longitude;
std::vector<double> vWind;
std::vector<double> watt;
std::vector<double> testTime;

//prototypes
void populate(double timeIncrement, std::string input);

int main() {

	std::string wait;

	// starts MATLAB session
	std::unique_ptr<MATLABEngine> ep = matlab::engine::startMATLAB({ u"-desktop" });
	matlab::data::ArrayFactory factory;


	// populating vectors and sending to MATLAB to graph
	populate(0.1, "C:\\Users\\profe\\Desktop\\test1.txt"); // change string to whatever the real destination will be

	auto tempArr = factory.createArray({ 1,temp.size() }, temp.cbegin(), temp.cend());
	auto xaccArr = factory.createArray({ 1,xacc.size() }, xacc.cbegin(), xacc.cend());
	auto yaccArr = factory.createArray({ 1,yacc.size() }, yacc.cbegin(), yacc.cend());
	auto zaccArr = factory.createArray({ 1,zacc.size() }, zacc.cbegin(), zacc.cend());
	auto kPitchArr = factory.createArray({ 1,kPitch.size() }, kPitch.cbegin(), kPitch.cend());
	auto kRollArr = factory.createArray({ 1,kRoll.size() }, kRoll.cbegin(), kRoll.cend());
	auto latitudeArr = factory.createArray({ 1,latitude.size() }, latitude.cbegin(), latitude.cend());
	auto longitudeArr = factory.createArray({ 1,longitude.size() }, longitude.cbegin(), longitude.cend());
	auto wattArr = factory.createArray({ 1,watt.size() }, watt.cbegin(), watt.cend());
	auto timeArr = factory.createArray({ 1,testTime.size() }, testTime.cbegin(), testTime.cend());

	for (int i = 0; i < vWind.size(); i++) {
		vWind.at(i) = vWind.at(i) * 18.91 - 17.92;
	} // for
	auto windArr = factory.createArray({ 1,vWind.size() }, vWind.cbegin(), vWind.cend());

	ep->setVariable(u"time", timeArr);
	ep->setVariable(u"temp", tempArr);
	ep->setVariable(u"xacc", xaccArr);
	ep->setVariable(u"yacc", yaccArr);
	ep->setVariable(u"zacc", zaccArr);
	ep->setVariable(u"kPitch", kPitchArr);
	ep->setVariable(u"kRoll", kRollArr);
	ep->setVariable(u"latitude", latitudeArr);
	ep->setVariable(u"longitude", longitudeArr);
	ep->setVariable(u"watts", wattArr);
	ep->setVariable(u"wind", windArr);

	ep->eval(u"plot(time, temp)");
	ep->eval(u"xlabel('Time (s)')");
	ep->eval(u"ylabel('Temperature (C)')");
	ep->eval(u"figure");

	ep->eval(u"plot(time, xacc)");
	ep->eval(u"xlabel('Time (s)')");
	ep->eval(u"ylabel('X Acceleration (m/s)')");
	ep->eval(u"figure");

	ep->eval(u"plot(time, yacc)");
	ep->eval(u"xlabel('Time (s)')");
	ep->eval(u"ylabel('Y Acceleration (m/s)')");
	ep->eval(u"figure");

	ep->eval(u"plot(time, zacc)");
	ep->eval(u"xlabel('Time (s)')");
	ep->eval(u"ylabel('Z Acceleration (m/s)')");
	ep->eval(u"figure");

	ep->eval(u"plot(time, kPitch)");
	ep->eval(u"xlabel('Time (s)')");
	ep->eval(u"ylabel('Kalman Pitch')");
	ep->eval(u"figure");

	ep->eval(u"plot(time, kRoll)");
	ep->eval(u"xlabel('Time (s)')");
	ep->eval(u"ylabel('Kalman Roll')");
	ep->eval(u"figure");

	ep->eval(u"plot(time, latitude)");
	ep->eval(u"xlabel('Time (s)')");
	ep->eval(u"ylabel('Latitude')");
	ep->eval(u"figure");

	ep->eval(u"plot(time, longitude)");
	ep->eval(u"xlabel('Time (s)')");
	ep->eval(u"ylabel('Longitude')");
	ep->eval(u"figure");

	ep->eval(u"plot(time, watts)");
	ep->eval(u"xlabel('Time (s)')");
	ep->eval(u"ylabel('Wattage Draw')");
	ep->eval(u"figure");

	ep->eval(u"plot(time, wind)");
	ep->eval(u"xlabel('Time (s)')");
	ep->eval(u"ylabel('Wind Speed (mph)')");
	ep->eval(u"figure");

	//image reading
	ep->eval(u"img1 = imread('C:\\Users\\profe\\Desktop\\Full.jpg');");
	ep->eval(u"img2 = imread('C:\\Users\\profe\\Desktop\\Not_full.jpg');");
	ep->eval(u"imshowpair(img1,img2,'diff');");
	ep->eval(u"[ximage,yimage] = ginput(2);");

	// getting y pixel value
	double fuelConst = 0.0087006797;
	auto yimage = ep->getVariable(u"yimage");
	double fuelTop = yimage[0];
	double fuelBot = yimage[1];
	double fuelPix = fuelBot - fuelTop;
	double fuelUsed = fuelPix * fuelConst; // convert to cm height
	fuelUsed = fuelUsed * 10 * 3; // convert to ml
	std::cout << "Fuel used: " << fuelUsed << "ml" << std::endl;



	std::cout << "enter anything to exit MATLAB" << std::endl;
	std::cin >> wait;


	return 0;

}

// function to fill up the given vector using a path for a text file
void populate(double timeIncrement, std::string input) {

	std::string line;
	std::ifstream file;
	size_t place = 0;
	bool hasSpace;
	double timeCounter = 0;

	hasSpace = true; // to see if there is space separating two values

	file.open(input);

	while (getline(file, line)) {
	
		place = line.find('\n');

		if (line.substr(0, place).compare("Temp = ") == 0) {
			getline(file, line);
			place = line.find('\n');
			temp.push_back(stod(line.substr(0, place)));
		}
		else if (line.substr(0, place).compare("X acc = ") == 0) {
			getline(file, line);
			place = line.find('\n');
			xacc.push_back(stod(line.substr(0, place)));
		}
		else if (line.substr(0, place).compare("Y acc = ") == 0) {
			getline(file, line);
			place = line.find('\n');
			yacc.push_back(stod(line.substr(0, place)));
		}
		else if (line.substr(0, place).compare("Z acc = ") == 0) {
			getline(file, line);
			place = line.find('\n');
			zacc.push_back(stod(line.substr(0, place)));
		}
		else if (line.substr(0, place).compare("Kalman Pitch = ") == 0) {
			getline(file, line);
			place = line.find('\n');
			kPitch.push_back(stod(line.substr(0, place)));
		}
		else if (line.substr(0, place).compare("Kalman Roll = ") == 0) {
			getline(file, line);
			place = line.find('\n');
			kRoll.push_back(stod(line.substr(0, place)));
		}
		else if (line.substr(0, place).compare("Latitude: ") == 0) {
			getline(file, line);
			place = line.find('\n');
			latitude.push_back(stod(line.substr(0, place)));
		}
		else if (line.substr(0, place).compare("Longitude: ") == 0) {
			getline(file, line);
			place = line.find('\n');
			longitude.push_back(stod(line.substr(0, place)));
		}
		else if (line.substr(0, place).compare("Wind Speed voltage: ") == 0) {
			getline(file, line);
			place = line.find('\n');
			vWind.push_back(stod(line.substr(0, place)));
		}
		else if (line.substr(0, place).compare("W: ") == 0) {
			getline(file, line);
			place = line.find('\n');
			watt.push_back(stod(line.substr(0, place)));
		}
		else {
			continue;
		}

	}

	for (int i = 0; i < temp.size(); i++) {
		testTime.push_back(i * timeIncrement);
	}

}
