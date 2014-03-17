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
	CancelWrite(CommandQueue *cmdqueue, vector<vector<BankState> > &states,
			ostream &dramsim_log);
	CommandQueue writeQueue;
	CommandQueue readQueue;
	CommandQueue *commandQueue;
	vector<vector<BankState> > bankStates;
	bool addRequest(Transaction *transaction, BusPacket *buspacket);
//	BusPacket* returnReadTransaction(BusPacket* readPacket,
//			CommandQueue& cmdqueue);
	bool cancelWrite(BusPacket **busPacket);
	bool issueRequest(unsigned rank, unsigned bank, BusPacket *busPacket,
			CommandQueue &requestQueue);
	void update();
	vector<vector<BusPacket*> > pendingWR;
private:
	unsigned writeQueueDepth;
	vector<vector<bool> > writecancel;
	vector<vector<unsigned> > writestarttime;
	unsigned currentClockCyle;
	ostream &dramsim_log;
	vector<BusPacket *> pendingWrite;

};
}

#endif /* CANCELWRITE_H_ */
