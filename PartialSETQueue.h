#ifndef PARTIALSETQUEUE_H
#define PARTIALSETQUEUE_H

//PartialSETqueue.h
//
//Header
//

#include "BusPacket.h"
#include "BankState.h"
#include "Transaction.h"
#include "SystemConfiguration.h"
#include "SimulatorObject.h"

using namespace std;

namespace DRAMSim {
class PartialSETQueue: public SimulatorObject {
	ParitalSETQueue();
	ostream &dramsim_log;
public:
	//Entry in PSqueue has two fields one is address, the other is time
	struct entry {
		BusPacket *busPacket;
		unsigned elaspedTime;
	};

	//typedefs
	typedef vector<entry*> Entry1D;
	typedef vector<Entry1D> Entry2D;
	typedef vector<Entry2D> Entry3D;

	//functions
	ParitalSETQueue(vector< vector<BankState> > &states, ostream &dramsim_log);
	virtual ~ParitalSETQueue();
	bool enqueue(BusPacket *newBusPacket);
	void evict();

	//fields

	Entry3D PSqueues; // 3D array of entry pointers
	vector<vector<BankState> > &bankStates;

private:

}
}
