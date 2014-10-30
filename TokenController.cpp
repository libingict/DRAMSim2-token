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

	tokenQueue = vector < vector<TokenEntry*>
			> (REQUESTID(NUM_RANKS,NUM_BANKS), vector<TokenEntry*>(0, NULL)); //2D
//	tokenQueue.resize(REQUESTID(NUM_RANKS,NUM_BANKS));
//	tokenQueue= vector< vector<TokenEntry*> >();
//		tokenQueue= vector< vector<TokenEntry*> >();
//	for (size_t rank = 0; rank < NUM_RANKS; rank++) {
//		//this loop will run only once for per-rank and NUM_BANKS times for per-rank-per-bank
//		for (size_t bank = 0; bank < NUM_BANKS; bank++) {
//			tokenQueue.resize(CMD_QUEUE_DEPTH);
//		}
//	}
//	latency = vector < vector<uint64_t>
//			> (REQUESTID(NUM_RANKS,NUM_BANKS), vector < uint64_t
//					> (CMD_QUEUE_DEPTH, 0));	//2D
//	energy = vector < vector<double>
//			> (REQUESTID(NUM_RANKS,NUM_BANKS), vector<double>(CMD_QUEUE_DEPTH,
//					0));		//2D
	tokenPool = vector < double > (NUM_DEVICES, 80);
//per cell 2-MLC
	currentClockCycle = 0;
}
TokenController::~TokenController() {/*
 for (size_t r = 0; r < NUM_RANKS; r++) {
 for (size_t b = 0; b < NUM_BANKS; b++) {
 tokenQueue[REQUESTID(r,b)].clear();
 }
 }
 */
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
	if (buspacket->busPacketType != WRITE) {
		return;
	}
	//64bit data, from low to high mapped to chip 0-7
	uint64_t oldata, newdata, writtenData, tmpData;
	uint64_t tmpLatency=0;
	double tmpEnergy=0;
	newdata = buspacket->dataPacket->getData();
	oldata = buspacket->dataPacket->getoldData();
//	PRINT("newdata is :"<<hex<< newdata<<" oldata is "<< oldata<<dec);
	writtenData = oldata ^ newdata; // same is zero, different is one
	if (writtenData == 0) {
		return;
	}
	tmpData = writtenData;
	DataCounts* datacounts = new DataCounts();
	//actually this is different for each chip.
	for (unsigned i = 0; i < NUM_DEVICES; i++) {
		for (unsigned j = 0; j < DEVICE_WIDTH / 2; j = j + 1) {  //Per chip
			if ((tmpData & 0x3) != 0) {
//				demandedToken[i];
//				calculate the demandedToken
				switch (newdata & 0x3) {
				case 0x0:  //00
					datacounts->resetCounts[i]++;
					break;
				case 0x1:  //01
					datacounts->partresetCounts[i]++;
					break;
				case 0x2:  //10
					datacounts->partsetCounts[i]++;
					break;
				case 0x3:  //11
					datacounts->setCounts[i]++;
					break;
				}
			}
			tmpData = tmpData >> 2;
			newdata = newdata >> 2;
		}
		tmpEnergy = tmpEnergy
				+ (datacounts->resetCounts[i] + datacounts->partresetCounts[i]
						+ datacounts->setCounts[i] + datacounts->partsetCounts[i])
						* getiterNumber(0) * resetEnergyperCell
				+ (datacounts->partresetCounts[i] * (getiterNumber(1) - 1)
						+ datacounts->setCounts[i] * (getiterNumber(2) - 1)
						+ datacounts->partsetCounts[i] * (getiterNumber(3) - 1))
						* setEnergyperCell;
		if (datacounts->partresetCounts[i] != 0) {
			tmpLatency = (uint64_t)(RESETLatency +(unsigned) ( 7 * SETLatency));
		} else if (datacounts->partsetCounts[i] != 0) {
			tmpLatency = max(tmpLatency,
					(uint64_t)(RESETLatency + (unsigned) (5 * SETLatency)));
		} else if (datacounts->setCounts[i] != 0) {
			tmpLatency = max(tmpLatency,
					(uint64_t)(RESETLatency + (unsigned) SETLatency));
		} else if (datacounts->resetCounts[i] != 0) {
			tmpLatency = max(tmpLatency, (uint64_t)(RESETLatency));
		}
	}
	tokenQueue[REQUESTID(r,b)].push_back(
			new TokenEntry(0, buspacket->physicalAddress, false, datacounts,
					tmpLatency, tmpEnergy));
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
			TokenEntry* tokenentry;
			unsigned elapsedCycle = 0;
			unsigned delta = 0;
			for (size_t i = 0; i < tokenQueue[REQUESTID(r,b)].size(); i++) {
				tokenentry = tokenQueue[REQUESTID(r,b)][i];
				if (tokenentry->valid == true) {  //update the powerToken
					elapsedCycle = currentClockCycle - tokenentry->startCycle;
					delta = elapsedCycle - RESETLatency;
					if (delta == 7 * (unsigned) SETLatency) {
						DataCounts* data;
						data = tokenentry->dataCounts;
						PRINTN(
								"elapsedCycle["<<elapsedCycle<<"] tokenQueue pa[0x"<<hex<<tokenentry->physicalAddress<<dec<<"] r["<<r <<"] b["<<b<<"] i["<<i<<"]");
						for (unsigned d = 0; d < NUM_DEVICES; d++) {
							if (data->setCounts[d] == 0
									&& data->resetCounts[d] == 0
									&& data->partsetCounts[d] == 0
									&& data->partresetCounts[d] == 0) {
								continue;
							}
							PRINTN(
									"chip["<<d<<"] requestToken["<<tokenentry->requestToken[d]<<"] tokenPool["<<tokenPool[d]<<"] ");
							tokenPool[d] = tokenPool[d]
									+ (data->resetCounts[d]
											+ data->partresetCounts[d]
											+ data->setCounts[d]
											+ data->partsetCounts[d])
											* RESETToken;
							data->resetCounts[d] = 0;
							data->setCounts[d] = 0;
							data->partsetCounts[d] = 0; //reallocated
							data->partresetCounts[d] = 0;
							PRINTN("after tokenPool["<< tokenPool[d]<<"] ");
						}
						PRINT("");
						tokenentry->valid = false;
						delete (tokenQueue[REQUESTID(r,b)][i]);
						tokenQueue[REQUESTID(r,b)].erase(
								tokenQueue[REQUESTID(r,b)].begin() + i);
					}
				}
			}

		}
	}

}
bool TokenController::powerAllowable(BusPacket *buspacket) {
	unsigned r = buspacket->rank;
	unsigned b = buspacket->bank;
	bool found = false;
	if (buspacket->busPacketType != WRITE) {
		return true;
	}

//	//64bit data, from low to high mapped to chip 0-7
	uint64_t oldata, newdata, writtenData;
	newdata = buspacket->dataPacket->getData();
	oldata = buspacket->dataPacket->getoldData();
////	PRINT("newdata is :"<<hex<< newdata<<" oldata is "<< oldata<<dec);
	writtenData = oldata ^ newdata; // same is zero, different is one
	if (writtenData == 0) {
		return true;
	}
//	PRINTN("=== currentClock["<<currentClockCycle<<"] ");
//	buspacket->print();
//	tmpData = writtenData;
////	dataCounts[REQUESTID(r,b)].push_back(new DataCounts());
	DataCounts* data = new DataCounts();
	TokenEntry* tokenentry;
	for (size_t i = tokenQueue[REQUESTID(r,b)].size(); i != 0; i--) {
		tokenentry = tokenQueue[REQUESTID(r,b)][i - 1];
		if (tokenentry->physicalAddress == buspacket->physicalAddress
				&& tokenentry->valid == false) {
			tokenQueue[REQUESTID(r,b)].push_back(
					new TokenEntry(0, buspacket->physicalAddress, false,
							tokenentry->dataCounts,tokenentry->latency,tokenentry->energy));
			delete (tokenQueue[REQUESTID(r,b)][i - 1]);
			tokenQueue[REQUESTID(r,b)].erase(
					tokenQueue[REQUESTID(r,b)].begin() + i - 1);
//			PRINTN(
//					"tokenQueue size["<<tokenQueue[REQUESTID(r,b)].size()<<"] found! ");
			found = true;
			break;
		}
	}
	if (!found) {
//		PRINTN(
//				"tokenQueue size["<<tokenQueue[REQUESTID(r,b)].size()<<"] not found! ");
		initial(buspacket);
	}
	tokenentry = tokenQueue[REQUESTID(r,b)][tokenQueue[REQUESTID(r,b)].size()
			- 1];
//	PRINTN((*tokenentry));
	data = tokenentry->dataCounts;
	for (unsigned d = 0; d < NUM_DEVICES; d++) {
		tokenentry->requestToken[d] = (data->resetCounts[d]
				+ data->partresetCounts[d] + data->setCounts[d]
				+ data->partsetCounts[d]) * RESETToken;
		if (tokenPool[d] < tokenentry->requestToken[d]) {
//			PRINT(
//					" TC false chip["<<d<<"] tokenPool["<<tokenPool[d]<<"] requestToken["<< tokenentry->requestToken[d]<<"] ");
			return false;
		}
	}
	PRINTN(" TC true ");
	for (unsigned d = 0; d < NUM_DEVICES; d++) {
		if (tokenentry->requestToken[d] != 0) {
			PRINTN(
								"chip["<<d<<"] tokenPool["<<tokenPool[d]<<"] requestToken["<<tokenentry->requestToken[d]<<"] ");
			tokenPool[d] = tokenPool[d] - tokenentry->requestToken[d];
		}
	}
	tokenentry->startCycle = currentClockCycle;
	tokenentry->valid = true;
	PRINT("");
	return true;

}
void TokenController::print() {/*
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
 */
}

void TokenController::new_update() {
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
			TokenEntry* tokenentry;
			unsigned elapsedCycle = 0;
			unsigned delta = 0;
			for (size_t i = 0; i < tokenQueue[REQUESTID(r,b)].size(); i++) {
				tokenentry = tokenQueue[REQUESTID(r,b)][i];
				elapsedCycle = currentClockCycle - tokenentry->startCycle;
				if (elapsedCycle < RESETLatency) {
					continue;
				}
				if (tokenentry->valid == true) {
//					PRINT(
//							"tokenQueue r["<<r <<"] b["<<b<<"] i["<<i<<"] ");
					//updateBits;
					//updateReclaim;
					//updateReallocate;
					updateReclaim(tokenentry);
//					updateBits(tokenentry);
					updateReallocate(tokenentry);
				}
				if (elapsedCycle == tokenentry->latency) {
					//				PRINT("r["<<r <<"] b["<<b<<"] bitsCount["<<bitsCount<<"]");
					tokenentry->valid = false;
					delete (tokenQueue[REQUESTID(r,b)][i]);
					tokenQueue[REQUESTID(r,b)].erase(
							tokenQueue[REQUESTID(r,b)].begin() + i);
				}
			}

		}
	}
}

void TokenController::updateReallocate(TokenEntry* tokenentry) {
	if(tokenentry ==NULL){
		return;
	}
	unsigned elapsedCycle = 0;
	unsigned delta = 0;
	elapsedCycle = currentClockCycle - tokenentry->startCycle;
	delta = elapsedCycle - RESETLatency;
	DataCounts* data;
	data = tokenentry->dataCounts;
	if (elapsedCycle <= tokenentry->latency) {
		for (size_t i = 0; i < 7; i++) {
			if (delta == (unsigned) (i * SETLatency)) {
				for (unsigned d = 0; d < NUM_DEVICES; d++) {
					if (data->setCounts[d] == 0 && data->resetCounts[d] == 0
							&& data->partsetCounts[d] == 0
							&& data->partresetCounts[d] == 0) {
						continue;
					}
//					PRINTN(
//							"updateReallocate delta["<<delta<<"] chip["<<d<<"] tokenPool["<<tokenPool[d]<<"] ");
					tokenPool[d] = tokenPool[d]
							- (data->partresetCounts[d] + data->setCounts[d]
									+ data->partsetCounts[d]) * SETToken;
//					PRINT("after tokenPool["<<tokenPool[d]<<"] ");
				}
				break;

			}
		}
	}
}
void TokenController::updateReclaim(TokenEntry* tokenentry) {
	unsigned elapsedCycle = 0;
	unsigned delta = 0;
	unsigned itera = 0;
	elapsedCycle = currentClockCycle - tokenentry->startCycle;
	delta = elapsedCycle - RESETLatency;
	DataCounts* data;
	data = tokenentry->dataCounts;
	if (elapsedCycle == RESETLatency) { //reclaim
		for (unsigned d = 0; d < NUM_DEVICES; d++) {
			tokenPool[d] = tokenPool[d]
					+ (data->resetCounts[d] + data->partresetCounts[d]
							+ data->setCounts[d] + data->partsetCounts[d])
							* RESETToken;
			data->resetCounts[d] = 0; //reallocated
		}
//		PRINTN("delat=0; data->resetCounts["<<data->resetCounts[d] <<"] after tokenPool["<< tokenPool[d]<<"] ");
		return;
	}
	for (size_t i = 0; i <= 7; i++) {
		if (delta > (unsigned) (i * SETLatency)
				&& delta <= (unsigned) ((i + 1) * SETLatency)) {
			itera = delta - (unsigned) (i * SETLatency);
			for (size_t j = 2; j <= 5; j++) {
				if (itera == (unsigned) (j * 0.2 * SETLatency)) {
//					PRINTN(
//							"Reclaim delta["<<delta<<"] i["<<i<<"] j["<<j<<"] itera["<<itera<<"] ");
					for (unsigned d = 0; d < NUM_DEVICES; d++) {
						if (data->setCounts[d] == 0 && data->resetCounts[d] == 0
								&& data->partsetCounts[d] == 0
								&& data->partresetCounts[d] == 0) {
							continue;
						}
//						PRINTN(" chip["<<d<<"] tokenPool["<<tokenPool[d]<<"] ");
						tokenPool[d] = tokenPool[d]
								+ (double) (0.25 * SETToken)
										* (data->partresetCounts[d]
												+ data->setCounts[d]
												+ data->partsetCounts[d]);
//						PRINT("after tokenPool["<<tokenPool[d]<<"] ");
						if (i == 0 && j == 5) { //reclaim
							data->setCounts[d] = 0;
//							PRINT("data->setCounts["<<data->setCounts[d] <<"] ");
						}
						if (i == 4 && j == 5) { //reclaim
							data->partsetCounts[d] = 0;
//							PRINT("data->partsetCounts["<<data->partsetCounts[d] <<"] ");
						}
						if (i == 6 && j == 5) { //reclaim
							data->partresetCounts[d] = 0;
//							PRINTN("data->partsetCounts["<<data->partsetCounts[d] <<"] ");
						}
					}
					break;
				}
			}
			break;
		}
	}
}


