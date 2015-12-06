#ifndef FUZZYSET_H
#define FUZZYSET_H
#include <vector>
#include "membershipFunction.h"

using namespace std;

class FuzzySet{
	private:
		vector <MembershipFunction> MFs;

	public:
		//constructors
		FuzzySet() {}

		//methods
		void addMF(int a, int b, int c, int d);
		vector <double> evaluate(int value);
		double getCentroid(int n) { return MFs[n].getCentroid(); }
};

void FuzzySet::addMF(int a, int b, int c, int d){
	MembershipFunction mf(a, b, c, d);
	MFs.push_back(mf);
}

vector <double> FuzzySet::evaluate(int value) {
	vector <double> ans;
	for (int i = 0; i < MFs.size(); i++) {
		ans.push_back(MFs[i].membership(value));
	}
	return ans;
}


#endif