#include <iostream>
#include <vector>
#include <fstream>
#include "MatlabEngine.hpp"
#include "MatlabDataArray.hpp"
                                      
using namespace matlab::engine;

//prototypes
void populate(std::vector<double>& dataVector, std::vector<double>& timeVector, double timeIncrement, std::string input);

int main() {

	std::string wait;

	// creating two vectors, placeholders for data from input file and a time vector starting from t = 0.
	// will likely create the two synchronously so time and parsedData are the same size for plotting.
	std::vector<double> parsedData;
	for (int i = 0; i < 5; i++)
		parsedData.push_back(i);

	std::vector<double> time;
	for (int i = 0; i < 5; i++)
		time.push_back(i);

	// starts MATLAB session
	std::unique_ptr<MATLABEngine> ep = matlab::engine::startMATLAB({ u"-desktop" });
	matlab::data::ArrayFactory factory;


	//basic functionality testing. Not being used / will probably be deleted later
	/*

	// creates a MATLAB array using the vectors created earlier
	auto Y = factory.createArray({ 1,parsedData.size() }, parsedData.cbegin(), parsedData.cend());
	auto X = factory.createArray({ 1,time.size() }, time.cbegin(), time.cend());

	// sets the variables in MATLAB according the MATLAB arrays
	ep->setVariable(u"dependent", Y);
	ep->setVariable(u"time", X);

	// creates a MATLAB plot of the given variables
	ep->eval(u"plot(time, dependent)");
	ep->eval(u"xlabel('time (s)')");
	ep->eval(u"ylabel('dependent')");

	// sample function evaluation of MATLAB vector Y
	auto result = ep->feval(u"sqrt", Y);
	ep->setVariable(u"result", result);
	ep->eval(u"figure");
	ep->eval(u"plot(time, dependent, time, result)");
	ep->eval(u"xlabel('time (s)')");
	ep->eval(u"ylabel('dependent')");

	// displays results from the previous evaluation
	for (int i = 0; i < result.getNumberOfElements(); i++)
		std::cout << (double)result[i] << std::endl;

	*/

	// Below are tests for a sample text file input

	std::vector<double> testInput;
	std::vector<double> testTime;

	populate(testInput, testTime, 1, "C:\\Users\\profe\\Desktop\\test.txt"); // change string to whatever the real destination will be

	auto testDataArray = factory.createArray({ 1,testInput.size() }, testInput.cbegin(), testInput.cend());
	auto testTimeArray = factory.createArray({ 1,testTime.size() }, testTime.cbegin(), testTime.cend());
	ep->setVariable(u"data", testDataArray);
	ep->setVariable(u"time", testTimeArray);
	ep->eval(u"plot(time, data)");
	ep->eval(u"xlabel('time (s)')");
	ep->eval(u"ylabel('data')");

	// everything below this point is testing least squares
	// or linearizing data. Will probably be its own function later
	int N = testInput.size();
	double m, b, top1 = 0, top2a = 0, top2b = 0, bot1 = 0, bot2 = 0;

	for (int i = 0; i < N; i++) {

		top1 += testInput.at(i) * testTime.at(i);
		top2a += testTime.at(i);
		top2b += testInput.at(i);
		bot1 += testTime.at(i) * testTime.at(i);
		bot2 += testTime.at(i);

	} // for

	m = (N * top1 - top2a * top2b) / (N * bot1 - bot2 * bot2);
	b = (top2b - m * top2a) / N;

	auto mA = factory.createScalar(m);
	ep->setVariable(u"m", mA);
	auto bA = factory.createScalar(b);
	ep->setVariable(u"b", bA);

	ep->eval(u"hold on");
	ep->eval(u"fplot(@(x) m * x + b, [0,6])");


	std::cout << "enter anything to exit MATLAB" << std::endl;
	std::cin >> wait;


	return 0;

}

// function to fill up the given vector using a path for a text file
void populate(std::vector<double>& dataVector, std::vector<double>& timeVector, double timeIncrement, std::string input) {

	std:: string line;
	std::ifstream file;
	size_t place = 0;
	//size_t emptyCheck;
	bool hasSpace;
	double timeCounter = 0;

	hasSpace = true; // to see if there is space separating two values

	file.open(input);
	
	getline(file, line);

	while (hasSpace) {

		if (line.find(' ') != std::string::npos) {              // if there is a space character found,
														        // initialize an item for the value found
			place = line.find(' ');                             // before that space. Then, create a new
			dataVector.push_back(stod(line.substr(0, place)));  // substring for everything after the space
			timeVector.push_back(timeCounter);
			timeCounter += timeIncrement;
			line = line.substr(place + 1, std::string::npos);

		}
		else {

			dataVector.push_back(stod(line.substr(0, place)));  // case for the very last value found
			timeVector.push_back(timeCounter);
			hasSpace = false;

		} // if-else

	} // while

}