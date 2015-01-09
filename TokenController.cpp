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
TokenController::TokenController(vector<vector<TokenEntry*> >& writequeue,
		ostream &dramsim_log_) :
		tokenQueue(writequeue), dramsim_log(dramsim_log_) {

//	tokenQueue = vector < vector<TokenEntry*>
//			> (REQUESTID(NUM_RANKS,NUM_BANKS), vector<TokenEntry*>(0, NULL)); //2D
//	tokenQueue.resize(REQUESTID(NUM_RANKS,NUM_BANKS));
//	tokenQueue= vector< vector<TokenEntry*> >();
//		tokenQueue= vector< vector<TokenEntry*> >();
//	for (size_t rank = 0; rank < NUM_RANKS; rank++) {
//		//this loop will run only once for per-rank and NUM_BANKS times for per-rank-per-bank
//		for (size_t bank = 0; bank < NUM_BANKS; bank++) {
//			tokenQueue.resize(CMD_QUEUE_DEPTH);
//		}
//	}

//	energy = vector < vector<double>
//			> (REQUESTID(NUM_RANKS,NUM_BANKS), vector<double>(CMD_QUEUE_DEPTH,
//					0));		//2D
	tokenPool = vector<double>(NUM_DEVICES, 80);
	ratio=1;			//
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

bool TokenController::addwriteRequest(vector<TokenEntry*>& writeBank,
		BusPacket* buspacket, bool &found) { ////enqueue for write
	unsigned r = buspacket->rank;
	unsigned b = buspacket->bank;
//	bool found = false;
	vector < uint64_t > resetToken = vector < uint64_t > (NUM_DEVICES, 0);
	uint64_t oldata, newdata, writtenData, tmpData;
	newdata = buspacket->dataPacket->getData();
	oldata = buspacket->dataPacket->getoldData();
	writtenData = oldata ^ newdata; // same is zero, different is one,Flip-N-write
	if (writtenData == 0) {		// data is zero, then 不存，直接返回
		found = true;
		return true;
	}
	if (writeBank.size() > CMD_QUEUE_DEPTH) {
		return false;
	}
	//first update if there is the writequeue for the same address, if there is,update and return;
	for (unsigned i = 0; i < writeBank.size(); i++) {
		BusPacket *bspacket = writeBank[i]->packet;
		if (bspacket->physicalAddress == buspacket->physicalAddress
				&& writeBank[i]->valid == false && writeBank[i]->done == false
				&& writeBank[i]->startCycle == 0) {	//if the write queue has the same address, then the old write should be evicted, updated the newest data.
				//而且没有被发射,也没有执行完
			found = true;
			if ((newdata != bspacket->dataPacket->getData()
					|| oldata != bspacket->dataPacket->getoldData())) { //如果数据不相等
				writeBank.erase(writeBank.begin() + i);
			} else { //if the written data is same, there is no need to update the write queue，如果数据相同，不需要更新
				return true;		//这次插入的写操作已经完成
			}
			break;
		}
	}

//64bit data, from low to high mapped to chip 0-7
//	PRINT("newdata is :"<<hex<< newdata<<" oldata is "<< oldata<<dec);
	DataCounts* datacounts = new DataCounts(); //calculate the data for per chip
	tmpData = writtenData;
	uint64_t tmpLatency = 0;
	double tmpEnergy = 0;
	unsigned maxToken = 0; // max RESET counts for chips, represents the required max power，用于排序
//actually this is different for each chip.
	for (unsigned i = 0; i < NUM_DEVICES; i++) { //Per chip
		for (unsigned j = 0; j < DEVICE_WIDTH / 2; j = j + 1) { //get the number of each level
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
						+ datacounts->setCounts[i]
						+ datacounts->partsetCounts[i]) * getiterNumber(0)
						* resetEnergyperCell
				+ (datacounts->partresetCounts[i] * (getiterNumber(1) - 1)
						+ datacounts->setCounts[i] * (getiterNumber(2) - 1)
						+ datacounts->partsetCounts[i] * (getiterNumber(3) - 1))
						* setEnergyperCell * (double) (0.5+(1/2*(double)ratio));
		if (datacounts->partresetCounts[i] != 0) { //the latency of line determined by the "partialRESET"  state
			tmpLatency = (uint64_t)(RESETLatency + (unsigned) (7 * SETLatency));
		} else if (datacounts->partsetCounts[i] != 0) {
			tmpLatency = max(tmpLatency,
					(uint64_t)(RESETLatency + (unsigned) (5 * SETLatency)));
		} else if (datacounts->setCounts[i] != 0) {
			tmpLatency = max(tmpLatency,
					(uint64_t)(RESETLatency + (unsigned) SETLatency));
		} else if (datacounts->resetCounts[i] != 0) {
			tmpLatency = max(tmpLatency, (uint64_t)(RESETLatency));
		}
		resetToken[i] = (datacounts->resetCounts[i]
				+ datacounts->partresetCounts[i] + datacounts->setCounts[i]
				+ datacounts->partsetCounts[i]) * RESETToken;
		if (resetToken[i] > maxToken) {
			maxToken = resetToken[i];
		}
	}
//	buspacket->busPacketType=WRITE;
//update the tokenQueue, order the token as the power increasing order;
	for (size_t s = 0; s < writeBank.size(); s++) {
		unsigned tmpmaxToken = 0;
		vector<TokenEntry*>::iterator it = writeBank.begin() + s;
//		DataCounts* datacounts = *it->dataCounts;
		if ((*it)->valid == false && (*it)->done == false) {	//the write is not executing or done
			for (unsigned d = 0; d < NUM_DEVICES; d++) { //max Token for chip in tmpToken
				if ((*it)->requestToken[d] > tmpmaxToken) {
					tmpmaxToken = (*it)->requestToken[d];
				}
			}
			if (maxToken < tmpmaxToken) {
				writeBank.insert(it,
						new TokenEntry(0, buspacket, false, false, datacounts,
								tmpLatency, tmpEnergy, resetToken,
								dramsim_log));
//				PRINTN("clock "<<currentClockCycle<<" TC addwriteRequest insert buspacket is ");
//				buspacket->print();
				return true;
			}
		}
	}
	writeBank.push_back(
			new TokenEntry(0, buspacket, false, false, datacounts, tmpLatency,
					tmpEnergy, resetToken, dramsim_log)); //order the queue according to the power
//	PRINTN("clock "<<currentClockCycle<<" TC addwriteRequest push_back buspacket is ");
//	buspacket->print();
	return true;
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
void TokenController::update_Naive() {  //navie的写方式，当完成之后偿还token
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
			TokenEntry* tokenentry;
			unsigned elapsedCycle = 0;
			unsigned delta = 0;
			for (size_t i = 0; i < tokenQueue[REQUESTID(r,b)].size(); i++) {
//				PRINT("TC updateNaive() rank "<<r<<" bank "<<b);
				tokenentry = tokenQueue[REQUESTID(r,b)][i];
				if (tokenentry->valid == true) {  //update the powerToken
					elapsedCycle = currentClockCycle - tokenentry->startCycle;
					delta = elapsedCycle - RESETLatency;
					if (delta == 7 * (unsigned) SETLatency) {
						DataCounts* data;
						BusPacket* bspacket;
						data = tokenentry->dataCounts;
						bspacket = tokenentry->packet;
//						PRINTN(
//								"elapsedCycle["<<elapsedCycle<<"] tokenQueue pa[0x"<<hex<<bspacket->physicalAddress<<dec<<"] r ["<<r <<"] b ["<<b<<"] i ["<<i<<"] ");
						for (unsigned d = 0; d < NUM_DEVICES; d++) {
							if (data->setCounts[d] == 0
									&& data->resetCounts[d] == 0
									&& data->partsetCounts[d] == 0
									&& data->partresetCounts[d] == 0) {
								continue;
							}
//							PRINTN(
//									"chip["<<d<<"] requestToken["<<tokenentry->requestToken[d]<<"] tokenPool["<<tokenPool[d]<<"] ");
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
//							PRINTN("after tokenPool["<< tokenPool[d]<<"] ");
						}
//						PRINT("done ");
//						bspacket->print();
						tokenentry->valid = false;
						tokenentry->done = true; //release the token and write finish
//						delete (tokenQueue[REQUESTID(r,b)][i]);	//not delete, delete by memory controller.
//						tokenQueue[REQUESTID(r,b)].erase(
//								tokenQueue[REQUESTID(r,b)].begin() + i);
					}
				}
			}

		}
	}
}
bool TokenController::release(size_t rank, size_t bank, uint64_t &addr) { //memory controller is able to release the request of writeQueue
	TokenEntry* write;
	addr = 0;
	for (size_t i = 0; i < tokenQueue[REQUESTID(rank,bank)].size(); i++) {
		write = tokenQueue[REQUESTID(rank,bank)][i];
		if (write->done == true) {
			addr = write->packet->physicalAddress;
//			PRINT(
//					"clock "<<currentClockCycle<<" i["<<i<<"] "<<" TC release is 0x"<<hex<<addr<<dec);
			delete (tokenQueue[REQUESTID(rank,bank)][i]); //not delete, delete by memory controller.
			tokenQueue[REQUESTID(rank,bank)].erase(
					tokenQueue[REQUESTID(rank,bank)].begin() + i);
			return true;
		}
	}
	return false;
}
bool TokenController::powerAllowable(TokenEntry*& writerequest) { //isIssuable for write
//	BusPacket *bspacket = writerequest->packet;
	if (writerequest->valid == true || writerequest->done == true) {
		return false;
	}
	DataCounts* data = writerequest->dataCounts;
	for (unsigned d = 0; d < NUM_DEVICES; d++) {
//		writerequest->requestToken[d] = (data->resetCounts[d]
//				+ data->partresetCounts[d] + data->setCounts[d]
//				+ data->partsetCounts[d]) * RESETToken;
		if (tokenPool[d] < writerequest->requestToken[d]) {
//			PRINTN(
//					"TC false chip["<<d<<"] tokenPool["<<tokenPool[d]<<"] requestToken["<< writerequest->requestToken[d]<<"] ");
			writerequest->packet->print();
			return false;
		}
	}
//	PRINTN("TC powerAllowable true ");
	for (unsigned d = 0; d < NUM_DEVICES; d++) {
		if (writerequest->requestToken[d] != 0) {
//			PRINTN(
//					"chip["<<d<<"] tokenPool["<<tokenPool[d]<<"] requestToken["<<writerequest->requestToken[d]<<"] ");
			tokenPool[d] = tokenPool[d] - writerequest->requestToken[d];
		}
	}
	writerequest->packet->print();
	writerequest->startCycle = currentClockCycle;
	writerequest->valid = true;
//	PRINT("");
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

void TokenController::update_SPA() {
//	unsigned ratio = 4;
	double fineToken = (double) SETToken / (double) ratio;
	double fineLatency = ceil(SETLatency / ratio );
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
			for (size_t i = 0; i < tokenQueue[REQUESTID(r,b)].size(); i++) {
				TokenEntry* tokenentry;
				unsigned elapsedCycle = 0;
				unsigned delta = 0;
				unsigned bitsCount = 0;
				tokenentry = tokenQueue[REQUESTID(r,b)][i];
				if (tokenentry->valid == true) {  //update the powerToken
					elapsedCycle = currentClockCycle - tokenentry->startCycle;
					if (elapsedCycle < RESETLatency) {
						continue;
					}
					delta = elapsedCycle - RESETLatency;
					DataCounts* data;
					data = tokenentry->dataCounts;
					unsigned deltaLatency = 0;
					deltaLatency = delta % (unsigned) SETLatency;
//					PRINTN("elapsedCycle["<<elapsedCycle<<"] ");
					for (unsigned d = 0; d < NUM_DEVICES; d++) {		//
						if (data->setCounts[d] == 0 && data->resetCounts[d] == 0
								&& data->partsetCounts[d] == 0
								&& data->partresetCounts[d] == 0) {
							continue;
						}
						if (delta == 0) { //reclaim			如果是RESET迭代完成，回收RESETtoken，分配SETtoken
//							PRINTN(
//									"delta=0; before tokenPool d["<<d<<"] before ["<<tokenPool[d]<<"] ");
							tokenPool[d] = tokenPool[d]
									+ (data->resetCounts[d]
											+ data->partresetCounts[d]
											+ data->setCounts[d]
											+ data->partsetCounts[d])
											* RESETToken;
							data->resetCounts[d] = 0; //RESET完成
							tokenPool[d] =
									tokenPool[d]
											- (data->partresetCounts[d]
													+ data->setCounts[d]
													+ data->partsetCounts[d])
													* SETToken; //reallocated
//							PRINT("after ["<<tokenPool[d]<<"] ");
						} else if (elapsedCycle > RESETLatency) {
//						PRINTN(
//								"chip["<<d<<"] requestToken["<<tokenentry->requestToken[d]<<"] tokenPool["<<tokenPool[d]<<"] ");
							if (deltaLatency % (unsigned) fineLatency == 0) {
//								PRINTN(
//										"fineLatency="<<fineLatency<<" deltaLatency["<<deltaLatency<<"] before tokenPool d["<<d<<"] ["<<tokenPool[d]<<"] ");
								tokenPool[d] =
										tokenPool[d]
												+ fineToken
														* (data->partresetCounts[d]
																+ data->setCounts[d]
																+ data->partsetCounts[d]);
//								PRINT("after ["<<tokenPool[d]<<"] ");
							}
							if (delta == (unsigned) SETLatency) { //reclaim 回收SETtoken
								data->setCounts[d] = 0;
								//							PRINTN("delta=SETLatency; tokenPool["<<tokenPool[d]<<"] ");
							} else if (delta == 5 * (unsigned) SETLatency) { //回收 partset Token
								data->partsetCounts[d] = 0; //reallocated
								//							PRINTN("delta=5*SETLatency; tokenPool["<<tokenPool[d]<<"] ");
							} else if (delta == 7 * (unsigned) SETLatency) { //finish no reallocate
								data->partresetCounts[d] = 0;
//							tokenentry->valid = false;
								//							PRINTN("delta=7*SETLatency; tokenPool["<<tokenPool[d]<<"] ");
							}
							if (deltaLatency == 0) {	//SETLatency的时候，更新token
//								PRINTN(
//										"deltaLatency == 0 delta["<<delta<<"] tokenPool d["<<d<<"] before ["<<tokenPool[d]<<"] ");
								tokenPool[d] =
										tokenPool[d]
												- SETToken
														* (data->partresetCounts[d]
																+ data->setCounts[d]
																+ data->partsetCounts[d]);
//								PRINT("after ["<<tokenPool[d]<<"] ");
							}
						}

						bitsCount = bitsCount + data->setCounts[d]
								+ data->resetCounts[d] + data->partsetCounts[d]
								+ data->partresetCounts[d];
						//				PRINT("bitsCount["<<bitsCount<<"]");
						//							PRINTN("after tokenPool["<< tokenPool[d]<<"] ");
					}
					if (bitsCount == 0) {
						//				PRINT("r["<<r <<"] b["<<b<<"] bitsCount["<<bitsCount<<"]");
//						PRINTN("tokenentry done buspacket is ");
						tokenentry->packet->print();
						tokenentry->valid = false;
						tokenentry->done = true;//release the token and write finish
						//						delete (tokenQueue[REQUESTID(r,b)][i]);
						//						tokenQueue[REQUESTID(r,b)].erase(
						//								tokenQueue[REQUESTID(r,b)].begin() + i);
					}
				}

			}

		}
	}
}

void TokenController::update_FPB() {
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
			TokenEntry* tokenentry;
			unsigned elapsedCycle = 0;
			unsigned delta = 0;
			unsigned bitsCount = 0;
			for (size_t i = 0; i < tokenQueue[REQUESTID(r,b)].size(); i++) {
				tokenentry = tokenQueue[REQUESTID(r,b)][i];
				if (tokenentry->valid == true) {  //update the powerToken
					elapsedCycle = currentClockCycle - tokenentry->startCycle;
					delta = elapsedCycle - RESETLatency;
					if ((delta != 0) && (delta != (unsigned) SETLatency)
							&& (delta != 5 * (unsigned) SETLatency)
							&& (delta != 7 * (unsigned) SETLatency)) { //不是iteration结束的时候
						continue;
					}
					DataCounts* data;
					data = tokenentry->dataCounts;
//					PRINTN("elapsedCycle["<<elapsedCycle<<"] ");
					for (unsigned d = 0; d < NUM_DEVICES; d++) {		//
						if (data->setCounts[d] == 0 && data->resetCounts[d] == 0
								&& data->partsetCounts[d] == 0
								&& data->partresetCounts[d] == 0) {
							continue;
						}
//						PRINTN(
//								"chip["<<d<<"] requestToken["<<tokenentry->requestToken[d]<<"] tokenPool["<<tokenPool[d]<<"] ");
						if (delta == 0) { //reclaim			如果是RESET迭代完成，回收RESETtoken，分配SETtoken
							tokenPool[d] = tokenPool[d]
									+ (data->resetCounts[d]
											+ data->partresetCounts[d]
											+ data->setCounts[d]
											+ data->partsetCounts[d])
											* RESETToken;
							data->resetCounts[d] = 0; //RESET完成
							tokenPool[d] =
									tokenPool[d]
											- (data->partresetCounts[d]
													+ data->setCounts[d]
													+ data->partsetCounts[d])
													* SETToken; //reallocated
							//					PRINTN("delat=0; tokenPool["<<tokenPool[d]<<"] ");
						} else if (delta == (unsigned) SETLatency) { //reclaim 回收SETtoken
							tokenPool[d] = tokenPool[d]
									+ (data->setCounts[d]) * SETToken;
							data->setCounts[d] = 0;
//							PRINTN("delta=SETLatency; tokenPool["<<tokenPool[d]<<"] ");
						} else if (delta == 5 * (unsigned) SETLatency) { //回收 partset Token
							tokenPool[d] = tokenPool[d]
									+ (data->partsetCounts[d]) * SETToken;
							data->partsetCounts[d] = 0; //reallocated
//							PRINTN("delta=5*SETLatency; tokenPool["<<tokenPool[d]<<"] ");
						} else if (delta == 7 * (unsigned) SETLatency) { //finish no reallocate
							tokenPool[d] = tokenPool[d]
									+ (data->partresetCounts[d]) * SETToken;
							data->partresetCounts[d] = 0;
//							PRINTN("delta=7*SETLatency; tokenPool["<<tokenPool[d]<<"] ");
						}
//						PRINTN("tokenentry buspacket is ");
//						tokenentry->packet->print();
						bitsCount = bitsCount + data->setCounts[d]
								+ data->resetCounts[d] + data->partsetCounts[d]
								+ data->partresetCounts[d];
						//				PRINT("bitsCount["<<bitsCount<<"]");
//							PRINTN("after tokenPool["<< tokenPool[d]<<"] ");
					}
					if (bitsCount == 0) {
						//				PRINT("r["<<r <<"] b["<<b<<"] bitsCount["<<bitsCount<<"]");
//						PRINT("done ");
						tokenentry->valid = false;
						tokenentry->done = true; //release the token and write finish
//						delete (tokenQueue[REQUESTID(r,b)][i]);
//						tokenQueue[REQUESTID(r,b)].erase(
//								tokenQueue[REQUESTID(r,b)].begin() + i);
					}
				}

			}
		}
	}
}
void TokenController::update() {
//	update_SPA();
	update_Naive();
}
