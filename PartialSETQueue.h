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
#include "CancelWrite.h"
#include "PredictionEntry.h"
using namespace std;

namespace DRAMSim {

class PartialSETQueue: public SimulatorObject {
	PartialSETQueue();
	ostream &dramsim_log;
public:
//    static double RETENTION_TIME;
	//typedefs
	typedef vector<Entry*> Entry1D;
	typedef vector<Entry1D> Entry2D;
	typedef vector<Entry2D> Entry3D;

	//functions to restore the idle information : false means long; true means short
	typedef bool IdleType;
	typedef vector<IdleType> BankD;
	typedef vector<BankD> RankD;
	typedef vector<RankD> Table;

	typedef vector<vector<PredictionEntry*> > Predict2D;
	vector<Predict2D> predictTable;

	PartialSETQueue(vector<vector<BankState> > &states,
			CancelWrite &canclewrite, ostream &dramsim_log);
	virtual ~PartialSETQueue();
	bool enqueue(BusPacket *newBusPacket);
	void release(BusPacket *newBusPacket);
	void iniPredictTable(unsigned rank, unsigned bank, uint64_t addr,
			uint64_t rip);
//	vector<Entry *> &getPSQueue(unsigned rank, unsigned bank);
//	bool idlePredictisLong(unsigned rank,unsigned bank);
	bool evict(unsigned &nextrank, unsigned &nextbank, BusPacket **busPacket);
	void getIdleInterval(); //get the bank interval is long or short;
	void print();
	void printIdletable();
	void update();

	vector<vector<bool> > isFull;
	//fields
	Entry3D PSqueues; // 3D array of entry pointers
	Table IdleTable;
	vector<vector<BankState> > &bankStates;
	CancelWrite &cancelWrite;
	vector<vector<uint64_t> > countPSQsetperBank;
	vector<vector<uint64_t> > countPSQpartialsetperBank;
private:
	static const unsigned PARTIAL_QUEUE_DEPTH = 128;
	vector<vector<bool> > idle;
	vector<vector<uint64_t> > begin;
	vector<vector<uint64_t> > duration;

};
}
#endif
