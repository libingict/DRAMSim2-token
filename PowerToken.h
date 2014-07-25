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
	unsigned setbitPower;
	unsigned resetbitPower;
	unsigned availableBit;
	static unsigned maxPower;

	vector<vector<unsigned> > powerPoll;

	void update();
	bool issuable();

};
}

#endif /* POWERTOKEN_H_ */
