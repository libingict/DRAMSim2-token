/*
 * CancelWrite.h
 *
 *  Created on: 2014-3-13
 *      Author: libing
 */

#ifndef CANCELWRITE_H_
#define CANCELWRITE_H_

#include "SimulatorObject.h"
#include "CommandQueue.h"
#include "BusPacket.h"
#include "BankState.h"

namespace DRAMSim {

class CancelWrite: public SimulatorObject {
public:
	CancelWrite(vector<vector<BankState> > &states, ostream &dramsim_log);
	virtual ~CancelWrite();
	CommandQueue writeQueue;
	CommandQueue readQueue;
	vector<vector<BankState> > &bankStates;
	bool addRequest(Transaction *transaction, BusPacket *buspacket,
			bool &found);
//	BusPacket* returnReadTransaction(BusPacket* readPacket,
//			CommandQueue& cmdqueue);
	bool cancelwrite(BusPacket **busPacket);
	bool issueRequest(unsigned r, unsigned b, BusPacket *&busPacket,
			CommandQueue &requestQueue);
	void update();
	bool isEmpty(unsigned rank);
	vector<vector<BusPacket*> > pendingWR;
	unsigned nextBank;
	void print();
private:
	unsigned writeQueueDepth;
	vector<vector<bool> > writecancel;
	vector<vector<unsigned> > writestarttime;
	vector<vector<unsigned> > readrequest;
	vector<vector<unsigned> > writerequest;
	ostream &dramsim_log;
	vector<BusPacket *> pendingWrite;
	unsigned nextRank;
	unsigned nextRankPRE;
	unsigned nextBankPRE;

};
}

#endif /* CANCELWRITE_H_ */
