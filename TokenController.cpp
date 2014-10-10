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
#define REQUESTID(rank,bank) (rank*NUM_BANKS)+bank
extern double setEnergyperCell; //pj
extern double resetEnergyperCell; //pj
using namespace DRAMSim;
TokenController::TokenController(ostream &dramsim_log_) :
		dramsim_log(dramsim_log_) {
	startCycle = vector<unsigned>(REQUESTID(NUM_RANKS,NUM_BANKS), 0);
	valid = vector<bool>(REQUESTID(NUM_RANKS,NUM_BANKS), false);
	existed = vector<bool>(REQUESTID(NUM_RANKS,NUM_BANKS), false);
	writtendata = vector < uint64_t > (REQUESTID(NUM_RANKS,NUM_BANKS), 0);
	requestToken = vector < vector<uint64_t>
			> (REQUESTID(NUM_RANKS,NUM_BANKS), vector < uint64_t
					> (NUM_DEVICES, 0));
	resetCounts =
			vector < vector<unsigned>
					> (REQUESTID(NUM_RANKS,NUM_BANKS), vector<unsigned>(
							NUM_DEVICES, 0));
	setCounts =
			vector < vector<unsigned>
					> (REQUESTID(NUM_RANKS,NUM_BANKS), vector<unsigned>(
							NUM_DEVICES, 0));
	partresetCounts =
			vector < vector<unsigned>
					> (REQUESTID(NUM_RANKS,NUM_BANKS), vector<unsigned>(
							NUM_DEVICES, 0));
	partsetCounts =
			vector < vector<unsigned>
					> (REQUESTID(NUM_RANKS,NUM_BANKS), vector<unsigned>(
							NUM_DEVICES, 0));
	tokenPool = vector < uint64_t > (NUM_DEVICES, 80);
//per cell 2-MLC
	currentClockCycle = 0;
}
TokenController::~TokenController() {
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
			resetCounts[REQUESTID(r,b)].clear();
			partresetCounts[REQUESTID(r,b)].clear();
			partsetCounts[REQUESTID(r,b)].clear();
			setCounts[REQUESTID(r,b)].clear();
			requestToken[REQUESTID(r,b)].clear();
		}
	}
}
/*bool TokenController::hasSame(unsigned rank, unsigned bank) {
	if ((existed[REQUESTID(rank, bank)] == true)
			&& (valid[REQUESTID(rank, bank)] == false)) {
		return true;
	} else {
		return false;
	}
}*/

void TokenController::initial(BusPacket *buspacket) {
	unsigned r = buspacket->rank;
	unsigned b = buspacket->bank;
	//64bit data, from low to high mapped to chip 0-7
	uint64_t oldata, newdata;
	uint64_t tmp;
	if ((existed[REQUESTID(r, b)] == true)
			|| (valid[REQUESTID(r, b)] == true)) {
		return;
	}
	existed[REQUESTID(r,b)]=true;
	newdata = buspacket->dataPacket->getData();
	oldata = buspacket->dataPacket->getoldData();
//	PRINT("newdata is :"<<hex<< newdata<<" oldata is "<< oldata<<dec);
	writtendata[REQUESTID(r,b)] = oldata ^ newdata; // same is zero, different is one
	if (writtendata[REQUESTID(r,b)] == 0) {
		return;
	}
	tmp = writtendata[REQUESTID(r,b)];
	//actually this is different for each chip.
	for (unsigned i = 0; i < NUM_DEVICES; i++) {
		for (unsigned j = 0; j < DEVICE_WIDTH / 2; j = j + 1) {  //Per chip
			if ((tmp & 0x3) != 0) {
//				demandedToken[i];
//				calculate the demandedToken
				switch (newdata & 0x3) {
				case 0x0:  //00
					resetCounts[REQUESTID(r,b)][i]++;
					break;
				case 0x1:  //01
					partresetCounts[REQUESTID(r,b)][i]++;
					break;
				case 0x2:  //10
					partsetCounts[REQUESTID(r,b)][i]++;
					break;
				case 0x3:  //11
					setCounts[REQUESTID(r,b)][i]++;
					break;
				}
			}
			tmp = tmp >> 2;
			newdata = newdata >> 2;
		}
		requestToken[REQUESTID(r,b)][i] = (resetCounts[REQUESTID(r,b)][i]
				+ partresetCounts[REQUESTID(r,b)][i]
				+ setCounts[REQUESTID(r,b)][i]
				+ partsetCounts[REQUESTID(r,b)][i]) * RESETToken;
	}
//	demandedToken[i] = setBits[i] + resetBits[i] * 2;
}
uint64_t TokenController::getLatency(unsigned rank, unsigned bank) {
	uint64_t latency = 0;
	if (writtendata[REQUESTID(rank,bank)] == 0) {
		return latency;
	}
	for (size_t i = 0; i < NUM_DEVICES; i++) {
		if (partresetCounts[REQUESTID(rank, bank)][i] != 0) {
			latency = (uint64_t)(RESETLatency + 7 * (unsigned) SETLatency);
			break;
		}
		if (partsetCounts[REQUESTID(rank, bank)][i] != 0) {
			latency = (uint64_t)(RESETLatency + 5 * (unsigned) SETLatency);
			break;
		}
		if (setCounts[REQUESTID(rank, bank)][i] != 0) {
			latency = (uint64_t)(RESETLatency + (unsigned) SETLatency);
			break;
		}
		if (resetCounts[REQUESTID(rank, bank)][i] != 0) {
			latency = (uint64_t)(RESETLatency);
		}
	}
//	PRINT("latency["<<latency<<"] rank["<<rank<< "] bank["<<bank<<"]");
	return latency;
}
double TokenController::getEnergy(unsigned rank, unsigned bank) {
	double energy = 0;
	unsigned r = rank;
	unsigned b = bank;
	if (writtendata[REQUESTID(r,b)] == 0) {
		return energy;
	}
	for (size_t i = 0; i < NUM_DEVICES; i++) {
		energy = energy
				+ (resetCounts[REQUESTID(r,b)][i]
						+ partresetCounts[REQUESTID(r,b)][i]
						+ setCounts[REQUESTID(r,b)][i]
						+ partsetCounts[REQUESTID(r,b)][i]) * getiterNumber(0)
						* resetEnergyperCell
				+ (partresetCounts[REQUESTID(r,b)][i] * (getiterNumber(1) - 1)
						+ setCounts[REQUESTID(r,b)][i] * (getiterNumber(2) - 1)
						+ partsetCounts[REQUESTID(r,b)][i]
								* (getiterNumber(3) - 1)) * setEnergyperCell;
	}
//	PRINT(
//			"energy ["<<energy<<"] resetEnergyperCell["<<RESETLatency<< "] setEnergyperCell ["<<setEnergyperCell<<"]");
	return energy;
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
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
			unsigned bitsCount = 0;
			unsigned elapsedCycle = 0;
			unsigned delta = 0;
			if (writtendata[REQUESTID(r,b)] == 0
					|| existed[REQUESTID(r,b)] == false
					|| valid[REQUESTID(r,b)] == false) {
				continue;
			}
			elapsedCycle = currentClockCycle - startCycle[REQUESTID(r,b)];
			delta = elapsedCycle - RESETLatency;
			if ((delta != 0)
					&& (delta != (unsigned) SETLatency)&& (delta!= 5*(unsigned)SETLatency)&& (delta!= 7*(unsigned)SETLatency)){
					continue;
				}
			for (unsigned i = 0; i < NUM_DEVICES; i++) {
				if (setCounts[REQUESTID(r,b)][i] == 0
						&& resetCounts[REQUESTID(r,b)][i] == 0
						&& partsetCounts[REQUESTID(r,b)][i] == 0
						&& partresetCounts[REQUESTID(r,b)][i] == 0) {
					continue;
				}
//				PRINTN(
//						"elapsedCycle["<<elapsedCycle<<"] r["<<r <<"] b["<<b<<"] ");
//				PRINTN(
//						"chip["<<i<<"] setCounts["<<setCounts[REQUESTID(r,b)][i]<<"] resetCounts["<<resetCounts[REQUESTID(r,b)][i]<<"] partsetCounts["<<partsetCounts[REQUESTID(r,b)][i]<<"] partresetCounts["<<partresetCounts[REQUESTID(r,b)][i]<<"] tokenPool["<<tokenPool[i]<<"] ");
				if (delta == 0) { //reclaim
					tokenPool[i] = tokenPool[i]
							+ (resetCounts[REQUESTID(r,b)][i]
									+ partresetCounts[REQUESTID(r,b)][i]
									+ setCounts[REQUESTID(r,b)][i]
									+ partsetCounts[REQUESTID(r,b)][i])
									* RESETToken;
					resetCounts[REQUESTID(r,b)][i] = 0;
					tokenPool[i] = tokenPool[i]
							- (partresetCounts[REQUESTID(r,b)][i]
									+ setCounts[REQUESTID(r,b)][i]
									+ partsetCounts[REQUESTID(r,b)][i])
									* SETToken; //reallocated
//					PRINTN("delat=0; tokenPool["<<tokenPool[i]<<"] ");
				} else if (delta == (unsigned) SETLatency) { //reclaim
					tokenPool[i] = tokenPool[i]
							+ (setCounts[REQUESTID(r,b)][i]) * SETToken;
					setCounts[REQUESTID(r,b)][i] = 0;
//					PRINTN("delat=SETLatency; tokenPool["<<tokenPool[i]<<"] ");
				} else if (delta == 5 * (unsigned) SETLatency) {
					tokenPool[i] = tokenPool[i]
							+ (partsetCounts[REQUESTID(r,b)][i]) * SETToken;
					partsetCounts[REQUESTID(r,b)][i] = 0; //reallocated
//					PRINTN("delat=5*SETLatency; tokenPool["<<tokenPool[i]<<"] ");
				} else if (delta == 7 * (unsigned) SETLatency) { //finish no reallocate
					tokenPool[i] = tokenPool[i]
							+ (partresetCounts[REQUESTID(r,b)][i]) * SETToken;
					partresetCounts[REQUESTID(r,b)][i] = 0;
					valid[REQUESTID(r,b)] = false;
//					PRINTN("delat=7*SETLatency; tokenPool["<<tokenPool[i]<<"] ");
				}
				bitsCount = bitsCount + setCounts[REQUESTID(r,b)][i]
						+ resetCounts[REQUESTID(r,b)][i]
						+ partsetCounts[REQUESTID(r,b)][i]
						+ partresetCounts[REQUESTID(r,b)][i];
//				PRINT("bitsCount["<<bitsCount<<"]");
			}
			if (bitsCount == 0) {
//				PRINT("r["<<r <<"] b["<<b<<"] bitsCount["<<bitsCount<<"]");
				valid[REQUESTID(r,b)] = false;
				writtendata[REQUESTID(r,b)] = 0;
				existed[REQUESTID(r, b)] =false;
			}
		}
	}
}
bool TokenController::powerAllowable(BusPacket *buspacket) {
	unsigned rank = buspacket->rank;
	unsigned bank = buspacket->bank;
	if (buspacket->busPacketType != WRITE) {
		return true;
	}
	initial(buspacket);
	if (writtendata[REQUESTID(rank,bank)] == 0) {
		startCycle[REQUESTID(rank,bank)] = currentClockCycle;
		valid[REQUESTID(rank,bank)] = false;
		return true;
	}
	for (unsigned i = 0; i < NUM_DEVICES; i++) {
		if (tokenPool[i] < requestToken[REQUESTID(rank,bank)][i]) {
			startCycle[REQUESTID(rank,bank)] = currentClockCycle;
//			PRINT(
//					"TC false currentClock["<<currentClockCycle<<"] chip["<<i<<"] tokenPool["<<tokenPool[i]<<"] requestToken["<<requestToken[REQUESTID(rank,bank)][i]<<"] r["<<rank <<"] b["<<bank<<"] ");
			return false;
		}
	}
	if (valid[REQUESTID(rank,bank)] == false) {
//		PRINTN(
//				"TC true=== currentClock["<<currentClockCycle<<"] r["<<rank <<"] b["<<bank<<"] ");
		for (unsigned i = 0; i < NUM_DEVICES; i++) {
			tokenPool[i] = tokenPool[i] - requestToken[REQUESTID(rank,bank)][i];
//			PRINTN(
//					"chip["<<i<<"] tokenPool["<<tokenPool[i]<<"] requestToken["<<requestToken[REQUESTID(rank,bank)][i]<<"] ");
		}
//		PRINT("");
		valid[REQUESTID(rank,bank)] = true;
		startCycle[REQUESTID(rank,bank)] = currentClockCycle;
		return true;
	}
}
void TokenController::print() {
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
			PRINTN("r["<<r <<"] b["<<b<<"] ");
			for (unsigned i = 0; i < NUM_DEVICES; i++) {
				PRINTN(
						"chip["<<i<<"] tokenPool["<<tokenPool[i]<<"] requestToken["<<requestToken[REQUESTID(r,b)][i]<<"] setCounts["<<setCounts[REQUESTID(r,b)][i]<<"] resetCounts["<<resetCounts[REQUESTID(r,b)][i]<<"] partsetCounts["<<partsetCounts[REQUESTID(r,b)][i]<<"] partresetCounts["<<partresetCounts[REQUESTID(r,b)][i]<<"] ");
			}
			PRINT("");
		}
	}
}
//vector<BusPacket *> &TokenController::getDemandtoken(unsigned bank, unsigned chip)
//{
//	return demandedToken[bank][chip];
//
//}
/*
 void TokenController::new_update() {
 unsigned elapsedCycle = 0;
 unsigned bitCounts = 0;
 for (size_t r = 0; r < NUM_RANKS; r++) {
 for (size_t b = 0; b < NUM_BANKS; b++) {
 if (startCycle[REQUESTID(r,b)] == 0
 || valid[REQUESTID(r,b)] == false) {
 continue;
 }
 elapsedCycle = currentClockCycle - startCycle[REQUESTID(r,b)];
 for (unsigned i = 0; i < NUM_DEVICES; i++) {
 if (elapsedCycle < RESETLatency) {
 break;
 }
 if (setCounts[REQUESTID(r,b)][i] == 0
 && resetCounts[REQUESTID(r,b)][i] == 0
 && partsetCounts[REQUESTID(r,b)][i] == 0
 && partresetCounts[REQUESTID(r,b)][i] == 0) {
 continue;
 }
 if (elapsedCycle == RESETLatency) { //reclaim
 tokenPool[i] = tokenPool[i]
 + (resetCounts[REQUESTID(r,b)][i]
 + partresetCounts[REQUESTID(r,b)][i]
 + setCounts[REQUESTID(r,b)][i]
 + partsetCounts[REQUESTID(r,b)][i])
 * RESETToken;
 resetCounts[REQUESTID(r,b)][i] = 0;
 tokenPool[i] = tokenPool[i]
 - (partresetCounts[REQUESTID(r,b)][i]
 + setCounts[REQUESTID(r,b)][i]
 + partsetCounts[REQUESTID(r,b)][i])
 * SETToken; //reallocated
 } else if (((uint64_t)(elapsedCycle - RESETLatency)
 % (uint64_t) SETLatency == 0)
 && ((elapsedCycle - RESETLatency)/ SETLatency < 7)) { //reclaim
 if ((elapsedCycle - RESETLatency)== SETLatency) {
 setCounts[REQUESTID(r,b)][i] = 0;
 } else if ((elapsedCycle - RESETLatency)== 5*SETLatency){
 partsetCounts[REQUESTID(r,b)][i] = 0;
 } //reallocated
 } else if ((elapsedCycle - RESETLatency)== 7*SETLatency){
 partresetCounts[REQUESTID(r,b)][i] = 0;
 tokenPool[i] = 80;
 valid[REQUESTID(r,b)] = false;
 }
 bitCounts = setCounts[REQUESTID(r,b)][i]
 + resetCounts[REQUESTID(r,b)][i]
 + partsetCounts[REQUESTID(r,b)][i]
 + partresetCounts[REQUESTID(r,b)][i];
 }
 if ((elapsedCycle > RESETLatency)&& bitCounts== 0){
 valid[REQUESTID(r,b)] = false;
 }
 }
 }
 }
 */
