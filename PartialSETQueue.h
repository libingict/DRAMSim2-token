#ifndef PARTIALSETQUEUE_H
#define PARTIALSETQUEUE_H

//PartialSETqueue.h
//
//Header
//

#include "BusPacket.h"
#include "Entry.h"
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
//    static double RETENTION_TIME;
	//typedefs
	typedef vector< Entry* > Entry1D;
	typedef vector<Entry1D> Entry2D;
	typedef vector<Entry2D> Entry3D;

	//functions to restore the idle information : false means long; true means short
	typedef bool IdleType;
	typedef vector<IdleType> BankD;
	typedef vector<BankD> RankD;
	typedef vector<RankD> Table;

	PartialSETQueue(vector< vector<BankState> > &states, CommandQueue &cmdqueue, ostream &dramsim_log);
	virtual ~PartialSETQueue();
	bool enqueue(BusPacket *newBusPacket);
//	vector<Entry *> &getPSQueue(unsigned rank, unsigned bank);
//	bool idlePredictisLong(unsigned rank,unsigned bank);
/*	void evict(unsigned rank, unsigned bank);
	void emergePartialSET(unsigned rank, unsigned bank,
			unsigned index);*/
	void getIdleInterval(); //get the bank interval is long or short;
	void update();
	vector< vector<bool> > isFull;
	//fields
	Entry3D PSqueues; // 3D array of entry pointers
	Table IdleTable;
	vector<vector<BankState> > &bankStates;
	CommandQueue &cmdQueue;
private:
	static const unsigned PARTIAL_QUEUE_DEPTH = 128;
};
}
#endif
