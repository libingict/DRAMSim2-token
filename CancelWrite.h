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
#include "Rank.h"
#include "TokenController.h"

namespace DRAMSim {

class CancelWrite: public SimulatorObject {
public:
	CancelWrite(vector<vector<BankState> > &states, ostream &dramsim_log,
			vector<Rank *> *&ranks);
	virtual ~CancelWrite();
	vector<vector<BankState> > &bankStates;
	CommandQueue writeQueue;
	CommandQueue readQueue;
	vector<Rank*> *&ranks;
	bool addRequest(Transaction *transaction, BusPacket *buspacket,
			bool &found);

//	BusPacket* returnReadTransaction(BusPacket* readPacket,
//			CommandQueue& cmdqueue);
	bool cancelwrite(BusPacket **busPacket);
	bool issueRequest(unsigned r, unsigned b, BusPacket *&busPacket,
			CommandQueue &requestQueue);
	void issueWC(unsigned r, unsigned b);
	void update();
	bool isEmpty(unsigned rank);
//	void getToken(BusPacket *buspacket);
	vector<vector<BusPacket*> > ongoingWrite;
//	vector<vector<BusPacket*> > canceledWrite;
	vector<vector<bool> > writepriority;
	vector<vector<bool> > writecancel;
	TokenController* tokenRank;

	unsigned nextRank;
	unsigned nextBank;
	unsigned nextRankPRE;
	unsigned nextBankPRE;
	vector<vector<uint64_t> > coutcanceledwrite; //record the Count of Canceled Write
//	vector<BusPacket *> canceledWrite;
	void print();
private:
	unsigned writeQueueDepth;
//	vector<vector<uint64_t> > readcout;
	ostream &dramsim_log;

};
}

#endif /* CANCELWRITE_H_ */
