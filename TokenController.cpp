/*
 * WriteRequest.cpp
 *
 *  Created on: 2014-9-23
 *      Author: libing
 */

#include "TokenController.h"
#include <cstdlib>
#include <time.h>
#include <math.h>
#define BANKLEVEL(Chip,Offset) (Chip*DEVICE_WIDTH/2)+Offset
using namespace DRAMSim;
TokenController::TokenController(unsigned r, ostream &dramsim_log_) :
		rank(r),writeLatency(0), dramsim_log(dramsim_log_) {
	startCycle = vector<unsigned>(NUM_BANKS, 0);
	valid = vector<bool>(NUM_BANKS, false);
	resetCounts = vector < vector<unsigned>
			> (NUM_BANKS, vector<unsigned>(NUM_DEVICES, 0));
	setCounts = vector < vector<unsigned>
			> (NUM_BANKS, vector<unsigned>(NUM_DEVICES, 0));
	partresetCounts = vector < vector<unsigned>
			> (NUM_BANKS, vector<unsigned>(NUM_DEVICES, 0));
	partsetCounts = vector < vector<unsigned>
			> (NUM_BANKS, vector<unsigned>(NUM_DEVICES, 0));
	tokenPool = vector < uint64_t > (NUM_DEVICES, 80);
//	dataCounts=vector<DataLevel>(DataLevel,0);
	demandedToken = vector < uint64_t > (NUM_DEVICES, 0); //per cell 2-MLC
	iterNumber = vector<unsigned>(NUM_DEVICES * DEVICE_WIDTH / 2, 0); //per cell 2-MLC
	currentClockCycle = 0;
}
TokenController::~TokenController() {

}
void TokenController::initial(BusPacket *buspacket) {
	unsigned b = buspacket->bank;
	if(valid[b] == true){
		return;
	}
	valid[b] = true;
	vector<unsigned> latency = vector<unsigned>(NUM_DEVICES, 0);
	//64bit data, from low to high mapped to chip 0-7
	uint64_t oldata, newdata;
	uint64_t writtendata, tmp;
	//	 PRINT("dataPacket is :"<< buspacket->dataPacket);
	newdata = buspacket->dataPacket->getData();
	oldata = buspacket->dataPacket->getoldData();
	PRINT("newdata is :"<<hex<< newdata<<" oldata is "<< oldata<<dec);
	//	 buspacket->print();
	writtendata = oldata ^ newdata; // same is zero, different is one
	tmp = writtendata;
	//actually this is different for each chip.
	for (unsigned i = 0; i < NUM_DEVICES; i++) {
		if (tmp == 0) {
			demandedToken[i] = 0;
			continue;
		}
		for (unsigned j = 0; j < DEVICE_WIDTH / 2; j = j + 1) {
			if (!(tmp & 0x11)) {
//				demandedToken[i];
//				calculate the demandedToken
				switch (newdata & 0x11) {
				case 0x00:
					resetCounts[b][i]++;
					iterNumber[BANKLEVEL(i,j)] = getiterNumber(0);
					break;
				case 0x01:
					partresetCounts[b][i]++;
					iterNumber[BANKLEVEL(i,j)] = getiterNumber(1);
					break;
				case 0x10:
					partsetCounts[b][i]++;
					iterNumber[BANKLEVEL(i,j)] = getiterNumber(2);
					break;
				case 0x11:
					setCounts[b][i]++;
					iterNumber[BANKLEVEL(i,j)] = getiterNumber(3);
					break;
				}
			}
			tmp = tmp >> 2;
			newdata = newdata >> 2;
		}
		if (partresetCounts[b][i] != 0) {
			latency[i] = (uint64_t)(RESETLatency + 7 * SETLatency);
		} else if (partsetCounts[b][i] != 0) {
			latency[i] = max((uint64_t) latency[i],
					(uint64_t)(RESETLatency + 5 * SETLatency));
		} else if (setCounts[b][i] != 0) {
			latency[i] = max((uint64_t) latency[i],
					(uint64_t)(RESETLatency + SETLatency));
		} else {
			latency[i] = max((uint64_t) latency[i], (uint64_t)(RESETLatency));
		}
		writeLatency = max((uint64_t) latency[i], (uint64_t) writeLatency);
		demandedToken[i] = (resetCounts[b][i] + partresetCounts[b][i]
				+ setCounts[b][i] + partsetCounts[b][i]) * RESETToken;
	}
//	demandedToken[i] = setBits[i] + resetBits[i] * 2;
}
unsigned TokenController::getiterNumber(unsigned datalevel) {
	unsigned iternumber = 0;
	switch (datalevel) {
	case 0:
		iternumber = 1;
		break;
	case 1:
		iternumber = 8;
		break;
	case 2:
		iternumber = 6;
		break;
	case 3:
		iternumber = 2;
		break;
	default:
		iternumber = 8;
		break;
	}
	return iternumber;
}
void TokenController::update() {
	unsigned elapsedCycle;
	for (unsigned b = 0; b < NUM_BANKS; b++) {
		if (valid[b]==false)
			continue;
		elapsedCycle = currentClockCycle - startCycle[b];
		if (elapsedCycle == RESETLatency) {
			for (unsigned i = 0; i < NUM_DEVICES; i++) {
				if (setCounts[b][i] == 0 && resetCounts[b][i] == 0
						&& partsetCounts[b][i] == 0
						&& partresetCounts[b][i] == 0)
					continue;
				for (unsigned j = 0; j < DEVICE_WIDTH / 2; j = j + 1) {
					if (iterNumber[BANKLEVEL(i,j)] != 0) {
						iterNumber[BANKLEVEL(i,j)]--;
						demandedToken[i] = demandedToken[i] - RESETToken;
						tokenPool[i]=tokenPool[i]+RESETToken;
					}
				}
			}
		} else if ((elapsedCycle > RESETLatency)
				&& ((uint64_t)(elapsedCycle - RESETLatency)
						% (uint64_t) SETLatency == 0)
				&& ((elapsedCycle - RESETLatency) / SETLatency <= 8)) {
			for (unsigned i = 0; i < NUM_DEVICES; i++) {
				if (setCounts[b][i] == 0 && resetCounts[b][i] == 0
						&& partsetCounts[b][i] == 0
						&& partresetCounts[b][i] == 0)
					continue;
				for (unsigned j = 0; j < DEVICE_WIDTH / 2; j = j + 1) {
					if (iterNumber[BANKLEVEL(i,j)] != 0) {
						iterNumber[BANKLEVEL(i,j)]--;
						demandedToken[i] = demandedToken[i] - SETToken;
						tokenPool[i]=tokenPool[i]+SETToken;
					}
				}
			}

		} else if ((elapsedCycle - RESETLatency) / SETLatency == 8){
			valid[b]==false;
		}
	}
}
bool TokenController::powerAllowable(BusPacket *buspacket) {
//	initial(currentClockCycle,buspacket);
	unsigned bank=buspacket->bank;
	if(valid[bank]==false){
		initial(buspacket);
	}
	for(unsigned i = 0; i < NUM_DEVICES; i++){
		if(tokenPool[i]<demandedToken[i]){
			return false;
		}
		else{
			tokenPool[i]=tokenPool[i]-demandedToken[i];
		}
	}
	startCycle[bank]=currentClockCycle;
	return true;

}
/*void TokenController::new_update() {
 unsigned elapsedCycle;
 elapsedCycle = currentClockCycle - startCycle;
 if (elapsedCycle == RESETLatency) {
 for (unsigned i = 0; i < NUM_DEVICES; i++) {
 if (setCounts[i] == 0 && resetCounts[i] == 0
 && partsetCounts[i] == 0 && partresetCounts[i] == 0)
 continue;
 for (unsigned j = 0; j < DEVICE_WIDTH / 2; j = j + 1) {
 if (iterNumber[BANKLEVEL(i,j)] != 0) {
 iterNumber[BANKLEVEL(i,j)]--;
 }
 if (setCounts[i] != 0)
 setCounts[i]--;
 if (resetCounts[i] != 0)
 resetCounts[i]--;
 if (partsetCounts[i] != 0)
 setCounts[i]--;
 if (partresetCounts[i] != 0)
 partresetCounts[i]--;
 }
 demandedToken[i] = (resetCounts[i] + partresetCounts[i]
 + setCounts[i] + partsetCounts[i]) * SETToken
 }
 } else if ((elapsedCycle > RESETLatency)
 && ((elapsedCycle - RESETLatency) / SETLatency <= 8)) {
 for (unsigned i = 0; i < NUM_DEVICES; i++) {
 if (setCounts[i] == 0 && resetCounts[i] == 0
 && partsetCounts[i] == 0 && partresetCounts[i] == 0)
 continue;
 for (unsigned j = 0; j < DEVICE_WIDTH / 2; j = j + 1) {
 if (iterNumber[BANKLEVEL(i,j)] != 0) {
 iterNumber[BANKLEVEL(i,j)]--;
 }
 if (setCounts[i] != 0)
 setCounts[i]--;
 if (partsetCounts[i] != 0)
 setCounts[i]--;
 if (partresetCounts[i] != 0)
 partresetCounts[i]--;
 }
 demandedToken[i] = (resetCounts[i] + partresetCounts[i]
 + setCounts[i] + partsetCounts[i]) * SETToken
 }

 }
 }*/
