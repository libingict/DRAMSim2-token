#ifndef PARTIALSETQUEUE_H
#define PARTIALSETQUEUE_H

//PartialSETqueue.h
//
//Header
//

#include "BusPacket.h"
#include "BankState.h"
#include "CommandQueue.h"
#include "Transaction.h"
#include "SystemConfiguration.h"
#include "SimulatorObject.h"

using namespace std;

namespace DRAMSim {

class PartialSETQueue: public SimulatorObject {
	PartialSETQueue();
	ostream &dramsim_log;
public:
	//Entry in PSqueue has two fields one is address, the other is time
	struct entry {
		BusPacket *busPacket;
		unsigned elaspedTime;
	};
//    static double RETENTION_TIME;
	//typedefs
	typedef vector<entry*> Entry1D;
	typedef vector<Entry1D> Entry2D;
	typedef vector<Entry2D> Entry3D;

	//functions
	PartialSETQueue(vector< vector<BankState> > &states, ostream &dramsim_log);
	virtual ~PartialSETQueue();
	bool enqueue(BusPacket *newBusPacket);
	void evict();
	vector<BusPacket *> &getPSQueue(unsigned rank, unsigned bank);
	bool idlePredictisLong(unsigned bank);
	void evict(unsigned rank, unsigned bank);
	void emergePartialSET(unsigned rank, unsigned bank,
			unsigned index);
	void update();
	//bool isFull;
	vector< vector<bool> > isFull;

	//fields

	Entry3D PSqueues; // 3D array of entry pointers
	vector<vector<BankState> > &bankStates;

private:

};
}
#endif
