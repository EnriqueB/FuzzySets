#ifndef FUZZCONTROLLER_H
#define FUZZYCONTROLLER_H
#include <vector>
#include <iostream>
#include <algorithm>
#include "fuzzySet.h"
#include "rule.h"

using namespace std;

class FuzzyController{
	private:
		vector <FuzzySet> inputs;
		vector <FuzzySet> outputs;
		vector <Rule> rules;

	public:
		//constructor
		FuzzyController() {}

		//methods
		void addSets(string file, bool isInput);
		void addRules(string fileName);
		vector <double> evaluateInput(vector <int> inp);
		vector <double> evaluateResponse(vector < vector <double> > out);
		vector <double> getMembership(int data, int inp);
		vector <double> combineValues(int front, int right, vector <double> fSpeeds, vector <double> rSpeeds, double avoidanceCheck, double wallCheck);


};

void FuzzyController::addSets(string fileName, bool isInput){
	//if isInput is true, the file and sets are 
	//for the input fuzzy sets

	/*
	file format:
	Sets count
	MembershipFunctions count for the set 'i'
	a b c d  values for each membership function
	*/
	//open file
	ifstream file;
	file.open(fileName);
	if (!file.is_open()){
		cout << "Error opening the file\n";
		exit(0);
	}
	int setsCount, mfCount, a, b, c, d;
	file >> setsCount;
	for (int i = 0; i < setsCount; i++){
		file >> mfCount;
		FuzzySet fs;
		for (int j = 0; j < mfCount; j++){
			file >> a >> b >> c >> d;
			fs.addMF(a, b, c, d);
		}
		if (isInput)
			inputs.push_back(fs);
		else
			outputs.push_back(fs);
	}

}

void FuzzyController::addRules(string fileName){
	//read from file
	ifstream file;
	file.open(fileName);
	if (!file.is_open()){
		cout << "Error opening the file\n";
		exit(0);
	}
	//should change this to AMMOUNT_RULES
	while (!file.eof()){
		vector <int> ant(inputs.size()), out(2);
		for (int i = 0; i < inputs.size(); i++){
			file >> ant[i];
		}
		for (int i = 0; i < 2; i++){
			file >> out[i];
		}
		Rule r(ant, out);
		rules.push_back(r);
	}
	file.close();

}

vector <double> FuzzyController::evaluateInput(vector <int> inp) {
	vector < vector <double> > output;
	for (int i = 0; i<inp.size(); i++) {
		output.push_back(inputs[i].evaluate(inp[i]));
	}
	return evaluateResponse(output);
}

vector <double> FuzzyController::evaluateResponse(vector <vector <double> > out) {
	double sumResponses = 0.0;
	//should do a memset to 0
	vector <double> sumResponsesCentroid;
	vector <double> responses;
	sumResponsesCentroid.push_back(0);
	sumResponsesCentroid.push_back(0);
	//this should depend on the opperation
	double minimum;
	//cout << "Fired rules: ";
	for (int i = 0; i<rules.size(); i++) {
		//should change for a loop to check N number of antecedents
		if (out[0][rules[i].getAntecedents(0)]>0 && out[1][rules[i].getAntecedents(1)]>0) {
			//rule fires
			//should be a loop for all outputs
			
			
			minimum = min(out[0][rules[i].getAntecedents(0)], out[1][rules[i].getAntecedents(1)]);
			if (inputs.size() > 2){
				if (out[2][rules[i].getAntecedents(2)] == 0)
					continue;
				cout << i << " ";
				minimum = min(out[0][rules[i].getAntecedents(0)], out[1][rules[i].getAntecedents(1)]);
				minimum = min(minimum, out[2][rules[i].getAntecedents(2)]);
			}
			else{
				cout << i << " ";
			}
			sumResponses += minimum;
			sumResponsesCentroid[0] += (minimum*outputs[0].getCentroid(rules[i].getOutputs(0)));
			sumResponsesCentroid[1] += (minimum*outputs[1].getCentroid(rules[i].getOutputs(1)));
		}
	}
	cout << endl;
	responses.push_back(sumResponsesCentroid[0]/sumResponses);
	responses.push_back(sumResponsesCentroid[1]/sumResponses);

	return responses;
}

vector <double> FuzzyController::getMembership(int data, int inp){
	return inputs[inp].evaluate(data);
}

vector <double> FuzzyController::combineValues(int front, int right, vector <double> fSpeeds, vector <double> rSpeeds, double avoidanceCheck, double wallCheck){
	//create output sets with the original velocities
	double centroidLeft = (avoidanceCheck*fSpeeds[0] + wallCheck*rSpeeds[0]) / (wallCheck + avoidanceCheck);
	double centroidRight = (avoidanceCheck*fSpeeds[1] + wallCheck*rSpeeds[1]) / (wallCheck + avoidanceCheck);
	//cout <<endl<<"controlador "<< centroidLeft << " " << centroidRight << endl;
	//cout << "cont " << avoidanceCheck << " " << wallCheck << endl;

	FuzzySet fsLeft, fsRight;
	fsLeft.addMF((int)fSpeeds[0] - 50, (int)fSpeeds[0], (int)fSpeeds[0], (int)fSpeeds[0] + 50);
	fsLeft.addMF((int)rSpeeds[0] - 50, (int)rSpeeds[0], (int)rSpeeds[0], (int)rSpeeds[0] + 50);
	fsLeft.addMF((int)centroidLeft - 10, (int)centroidLeft, (int)centroidLeft, (int)centroidLeft + 10);

	fsRight.addMF((int)fSpeeds[1] - 50, (int)fSpeeds[1], (int)fSpeeds[1], (int)fSpeeds[1] + 50);
	fsRight.addMF((int)rSpeeds[1] - 50, (int)rSpeeds[1], (int)rSpeeds[1], (int)rSpeeds[1] + 50);
	fsRight.addMF((int)centroidRight - 10, (int)centroidRight, (int)centroidRight, (int)centroidRight + 10);

	outputs.push_back(fsLeft);
	outputs.push_back(fsRight);

	vector < vector <double> > output;
	output.push_back(inputs[0].evaluate(front));
	output.push_back(inputs[1].evaluate(right));

	//rules
	vector <double> answer;

	for (int i = 0; i < rules.size(); i++){
		if (output[0][rules[i].getAntecedents(0)] > 0 && output[1][rules[i].getAntecedents(1)] > 0){
			answer.push_back(outputs[0].getCentroid(rules[i].getOutputs(0)));
			answer.push_back(outputs[1].getCentroid(rules[i].getOutputs(1)));
			outputs.pop_back();
			outputs.pop_back();
			return answer;
		}
	}
	cout << "wat";
	outputs.pop_back();
	outputs.pop_back();
	return answer;
}
#endif