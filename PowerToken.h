/*
 * PowerToken.h
 *
 *  Created on: 2014-7-24
 *      Author: libing
 */

#ifndef POWERTOKEN_H_
#define POWERTOKEN_H_
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
class PowerToken: public SimulatorObject {
public:
	uint64_t startCycle;
		unsigned iterNumber;
		uint64_t demandedToken;
		vector<unsigned> setBits;
		vector<unsigned> resetBits;
		void set_RankBank(unsigned rankid_,unsigned bankid_){
			rank=rankid_;
			bank=bankid_;
		}
		uint64_t getToken(BusPacket *buspacket);
		uint64_t releaseToken(BusPacket *buspacket);
		virtual ~PowerToken();
	private:
		unsigned rank;
		unsigned bank;
	    static unsigned maxPower;

	vector<vector<unsigned> > powerPoll;

	void update();
	bool issuable();

};
}

#endif /* POWERTOKEN_H_ */
