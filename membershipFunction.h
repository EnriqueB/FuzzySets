#ifndef MEMBERSHIPFUNCTION_H
#define MEMBERSHIPFUNCTION_H

using namespace std;

class MembershipFunction{
	private:
		int a;
		int b;
		int c;
		int d;
		double centroid;

	public:
		//constructors
		MembershipFunction(){ a = b = c = d = centroid = 0; }
		MembershipFunction(int a, int b, int c, int d);

		//methods
		double membership(double measurement);

		//setters
		void setA(int val){ a = val; }
		void setB(int val){ b = val; }
		void setC(int val){ c = val; }
		void setD(int val){ d = val; }

		//getters
		int getA(){ return a; }
		int getB(){ return b; }
		int getC(){ return c; }
		int getD(){ return d; }
		double getCentroid() { return centroid; }

};

MembershipFunction::MembershipFunction(int w, int x, int y, int z){
	a = w;
	b = x;
	c = y;
	d = z;


	if (b == c){
		//figure is a triangle, centroid is the center
		centroid = b;
	}
	else{
		//divide into pieces (1001 and calculate the centroid
		double steps = (d - a) / 10000.0;
		double sum_xFx, sum_Fx;
		sum_xFx = sum_Fx = 0;
		double Fx;
		for (double i = a; i<d; i += steps){
			if (i<b){
				Fx = (i - a) / (b - a);
				sum_Fx += Fx;
				sum_xFx += (Fx*i);
			}
			else if (i >= b && i <= c){
				sum_Fx++;
				sum_xFx += i;
			}
			else{
				Fx = (d - i) / (d - c);
				sum_Fx += Fx;
				sum_xFx += (Fx*i);
			}
		}
		centroid = sum_xFx / sum_Fx;
	}
}

double MembershipFunction::membership(double measurement){
	if (measurement >= b && measurement <= c){
		return 1.0;
	}
	if (measurement <= a || measurement >= d){
		//what should I do for measurements for the right shoulder?
		return 0.0;
	}
	if (measurement > a && measurement < b){
		//left slope
		return (measurement - a) / (b - a);
	}
	if (measurement > c && measurement < d){
		//right slope
		return (d - measurement) / (d - c);
	}
}

#endif