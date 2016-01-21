#ifndef RULE_H
#define RULE_H

using namespace std;

class Rule{
	private:
		vector <int> antecedents;
		vector <int> outputs;

	public:
		//constructor
		Rule() {}
		Rule(vector <int> ant, vector <int> out){ antecedents = ant; outputs = out; }
		//getters
		int getAntecedents(int n) { return antecedents[n]; }
		int getOutputs(int n) { return outputs[n]; }

};

#endif
