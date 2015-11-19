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
//#define actualToken(latency) -1.25*SETToken*(latency-0.2*SETLatency)/SETLatency+SETToken
extern double setEnergyperCell; //pj
extern double resetEnergyperCell; //pj
using namespace DRAMSim;
TokenController::TokenController(vector<vector<TokenEntry*> >& writequeue,
		ostream &dramsim_log_) :
		tokenQueue(writequeue), dramsim_log(dramsim_log_) {
	releasedwriteQueue = vector < vector<TokenEntry*>
			> (REQUESTID(NUM_RANKS,NUM_BANKS), vector<TokenEntry*>(0, NULL)); //2D
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
	tokenPool = vector<double>(NUM_DEVICES, PowerBudget);
	tokenUtilityWrites=vector<double>(NUM_DEVICES, 0.0);
//	ratio = 2;			//
//	ratio = 4;
//per cell 2-MLC
	currentClockCycle = 0;
}
TokenController::~TokenController() {
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
			for (vector<TokenEntry*>::iterator it =
					tokenQueue[REQUESTID(r,b)].begin();
					it != tokenQueue[REQUESTID(r,b)].end(); it++) {
				if (NULL != *it) {
					delete *it;
					*it = NULL;
				}
			}
			for (vector<TokenEntry*>::iterator itrwq =
					releasedwriteQueue[REQUESTID(r,b)].begin();
					itrwq != releasedwriteQueue[REQUESTID(r,b)].end();
					itrwq++) {
				if (NULL != *itrwq) {
					delete *itrwq;
					*itrwq = NULL;
				}
			}
			tokenQueue[REQUESTID(r,b)].clear();
			releasedwriteQueue[REQUESTID(r,b)].clear();
		}
	}

}

bool TokenController::addwriteRequest(vector<TokenEntry*>& writeBank,
		BusPacket* buspacket, bool &found) { ////enqueue for write
//	bool found = false;
	vector < uint64_t > resetToken = vector < uint64_t > (NUM_DEVICES, 0);
	vector < double > tokenUtility = vector < double > (NUM_DEVICES, 0);
	double requestTokens;
	double actualTokens;
	uint64_t oldata, newdata, writtenData, tmpData;
	newdata = buspacket->dataPacket->getData();
	oldata = buspacket->dataPacket->getoldData();
	writtenData = oldata ^ newdata; // same is zero, different is one,Flip-N-write
	if (writtenData == 0) {		// data is zero, then 不存，直接返回,這次寫操作完成，並且可以釋放
		found = true;
		return true;
	}
	if (writeBank.size() >= CMD_QUEUE_DEPTH) {
		return false;
	}
	//first update if there is the writequeue for the same address, if there is,update and return;
	for (unsigned i = 0; i < writeBank.size(); i++) {
		BusPacket *bspacket = writeBank[i]->packet;
		if (bspacket->physicalAddress == buspacket->physicalAddress
				&& writeBank[i]->valid == false
				&& writeBank[i]->startCycle == 0) {	//if the write queue has the same address, then the old write should be evicted, updated the newest data.
				//而且没有被发射,也没有执行完
			found = true;	//该次写操作要被释放/更新，found is true，表示完成一次写操作，为了回应上层。
//1			PRINTN("found =true "<< "writeBank.size "<< writeBank.size()<<" ");
//1			bspacket->print();
			//add new array to store the released write
			writtenData = (bspacket->dataPacket->getoldData()) ^ newdata;
			buspacket->dataPacket->setData(newdata,
					bspacket->dataPacket->getoldData());
			delete (writeBank[i]);
			writeBank.erase(writeBank.begin() + i);	//删除旧数据
			break;
		}
	}
//同时将要写的数据中的power latency 等计算出来。
//64bit data, from low to high mapped to chip 0-7
//	PRINT("newdata is :"<<hex<< newdata<<" oldata is "<< oldata<<dec);
	DataCounts* datacounts = new DataCounts(); //calculate the data for per chip
	tmpData = writtenData;
	uint64_t tmpLatency = 0;
	double tmpEnergy = 0;
//	unsigned maxToken = 0; // max RESET counts for chips, represents the required max power，用于排序
//actually this is different for each chip.
	for (unsigned i = 0; i < NUM_DEVICES; i++) { //Per chip
		for (unsigned j = 0; j < DEVICE_WIDTH / 2; j = j + 1) { //get the number of each level
			if ((tmpData & 0x3) != 0) {				//异或之后的结果，末两位不是0，表示这两位需要被写。
//				demandedToken[i];
//				calculate the demandedToken
				switch (newdata & 0x3) {			//具体要被写的数据。
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
		//tmpEnergy 整体功耗？ 好像不太对,是每次写操作消耗的功耗。
		//对于Naive, FPB 以及SPA来说都不一样才对。
//		tmpEnergy = tmpEnergy
//				+ (datacounts->resetCounts[i] + datacounts->partresetCounts[i]
//						+ datacounts->setCounts[i]
//						+ datacounts->partsetCounts[i]) * getiterNumber(0)
//						* resetEnergyperCell
		//if FPB 根据SET 迭代改变
		if (FPB) {
			tmpEnergy = tmpEnergy
					+ (datacounts->resetCounts[i]
							+ datacounts->partresetCounts[i]
							+ datacounts->setCounts[i]
							+ datacounts->partsetCounts[i]) * getiterNumber(0)
							* resetEnergyperCell
					+ (datacounts->partresetCounts[i] * (getiterNumber(1) - 1)
							+ datacounts->setCounts[i] * (getiterNumber(2) - 1)
							+ datacounts->partsetCounts[i]
									* (getiterNumber(3) - 1))
							* setEnergyperCell;
			requestTokens = (datacounts->resetCounts[i]
					+ datacounts->partresetCounts[i] + datacounts->setCounts[i]
					+ datacounts->partsetCounts[i]) * RESETToken * RESETLatency
					+ (datacounts->partresetCounts[i] * (getiterNumber(1) - 1)
							+ datacounts->setCounts[i] * (getiterNumber(2) - 1)
							+ datacounts->partsetCounts[i]
									* (getiterNumber(3) - 1)) * SETToken
							* SETLatency;
		}
		//if SPA，根据stage 改变
		if (SPA) {
			tmpEnergy = tmpEnergy
					+ (datacounts->resetCounts[i]
							+ datacounts->partresetCounts[i]
							+ datacounts->setCounts[i]
							+ datacounts->partsetCounts[i]) * getiterNumber(0)
							* resetEnergyperCell
					+ (datacounts->partresetCounts[i] * (getiterNumber(1) - 1)
							+ datacounts->setCounts[i] * (getiterNumber(2) - 1)
							+ datacounts->partsetCounts[i]
									* (getiterNumber(3) - 1))
							* (double) (setEnergyperCell
									* (0.6 + 0.4 / (double) ratio));
			requestTokens = (datacounts->resetCounts[i]
					+ datacounts->partresetCounts[i] + datacounts->setCounts[i]
					+ datacounts->partsetCounts[i]) * RESETToken * RESETLatency
					+ (datacounts->partresetCounts[i] * (getiterNumber(1) - 1)
							+ datacounts->setCounts[i] * (getiterNumber(2) - 1)
							+ datacounts->partsetCounts[i]
									* (getiterNumber(3) - 1)) * SETToken
							* (0.6 + 0.4 / (double) ratio) * SETLatency;
		}

		//if Naive 不随着迭代次数改变,都是resetEnergyperCell
		if (Naive) {
			tmpEnergy = tmpEnergy
					+ (datacounts->resetCounts[i]
							+ datacounts->partresetCounts[i]
							+ datacounts->setCounts[i]
							+ datacounts->partsetCounts[i]) * getiterNumber(0)
							* resetEnergyperCell
					+ (datacounts->partresetCounts[i] + datacounts->setCounts[i]
							+ datacounts->partsetCounts[i])
							* (getiterNumber(3) - 1) * resetEnergyperCell;
			requestTokens = (datacounts->resetCounts[i]
					+ datacounts->partresetCounts[i] + datacounts->setCounts[i]
					+ datacounts->partsetCounts[i]) * RESETToken * RESETLatency
					+ (datacounts->partresetCounts[i] + datacounts->setCounts[i]
							+ datacounts->partsetCounts[i])
							* (getiterNumber(3) - 1) * RESETToken * SETLatency;
		}
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
		actualTokens = (datacounts->resetCounts[i]
				+ datacounts->partresetCounts[i] + datacounts->setCounts[i]
				+ datacounts->partsetCounts[i]) * RESETToken * RESETLatency
				+ (datacounts->partresetCounts[i] * (getiterNumber(1) - 1)
						+ datacounts->setCounts[i] * (getiterNumber(2) - 1)
						+ datacounts->partsetCounts[i] * (getiterNumber(3) - 1))
						* SETToken * 0.6 * SETLatency;
		if(requestTokens==0){
			tokenUtility[i]=1;
		} else {
			tokenUtility[i] = actualTokens / requestTokens;
		}
		/*		与排序相关
		 * if (resetToken[i] > maxToken) {
		 maxToken = resetToken[i];
		 }*/
	}
//	buspacket->busPacketType=WRITE;
//update the tokenQueue, order the token as the power increasing order;
//排序，更新后的算法不需要排序，但是需要把相同row的安排在一起
	/*	for (size_t s = 0; s < writeBank.size(); s++) {
	 unsigned tmpmaxToken = 0;
	 vector<TokenEntry*>::iterator it = writeBank.begin() + s;
	 //		DataCounts* datacounts = *it->dataCounts;
	 if ((*it)->valid == false && (*it)->done == false) { //the write is not executing or done
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
	 }*/
	/*TokenEntry(unsigned startCycle_, BusPacket* packet_, bool valid_,bool done_,
	 DataCounts* datacounts, uint64_t latency_, double energy_,vector<uint64_t> token_, ostream &dramsim_log_)；*/
	writeBank.push_back(
			new TokenEntry(0, buspacket, false, false, datacounts, tmpLatency,
					tmpEnergy, resetToken, tokenUtility, dramsim_log)); //order the queue according to the power
//	PRINTN("clock "<<currentClockCycle<<"writeBank size "<<writeBank.size()<<" TC addwriteRequest push_back buspacket is ");
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
void TokenController::update_nolimit() { //no token issue just countdown the latency and remove from the queue.
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
			for (vector<TokenEntry*>::iterator i =
					tokenQueue[REQUESTID(r,b)].begin();
					i != tokenQueue[REQUESTID(r,b)].end();) {
				unsigned elapsedCycle = 0;
				//				PRINT("TC tokenQueue rank "<<r<<" bank "<<b<<" size "<<tokenQueue[REQESTID(r,b)].size());
				TokenEntry* tokenentry = *i;
				if (tokenentry->valid == true) {
					elapsedCycle = currentClockCycle - tokenentry->startCycle;
					if (elapsedCycle == tokenentry->latency) {
						releasedwriteQueue[REQUESTID(r,b)].push_back(*i);
//						delete (tokenQueue[REQUESTID(r,b)][i]);	//not delete, delete by memory controller.
						i = tokenQueue[REQUESTID(r,b)].erase(i);
//							PRINT("tokenQueue erase continue;");
						continue;
						//						PRINT("clock "<<elapsedCycle <<" < RESETLatency continue");
					}
				}
				i++;
			}
		}
	}
	return;
}
void TokenController::update_SPAIdeal() {
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
			//			TokenEntry* tokenentry = NULL;
			for (vector<TokenEntry*>::iterator i =
					tokenQueue[REQUESTID(r,b)].begin();
					i != tokenQueue[REQUESTID(r,b)].end();) {
				unsigned elapsedCycle = 0;
				TokenEntry* tokenentry = *i;
				DataCounts* data = tokenentry->dataCounts;
				if (tokenentry->valid != true) {
					i++;
					continue;
				} else {
					if (tokenentry->latency == 0) {
//						PRINTN(
//								"clock "<<currentClockCycle<<" elapsedCycle["<<elapsedCycle<<"]== tokenentry->latency="<<tokenentry->latency<< " tokenQueue erase continue;");
//						tokenentry->packet->print();
						releasedwriteQueue[REQUESTID(r,b)].push_back(*i);
						//						delete (tokenQueue[REQUESTID(r,b)][i]);	//not delete, delete by memory controller.
						i = tokenQueue[REQUESTID(r,b)].erase(i);
						continue;
						//						PRINT("clock "<<elapsedCycle <<" < RESETLatency continue");
					}
					elapsedCycle = currentClockCycle - tokenentry->startCycle;
					if (elapsedCycle < RESETLatency) {
						i++;
						continue;
					}
					if (elapsedCycle == RESETLatency) {
//						PRINTN(
//								"updateSPAIdeal() elapsedCycle["<<elapsedCycle<<"] tokenQueue pa[0x"<<hex<<tokenentry->packet->physicalAddress<<dec<<"] r ["<<r <<"] b ["<<b<<"] ");

						for (unsigned d = 0; d < NUM_DEVICES; d++) {
							if (data->setCounts[d] == 0
									&& data->resetCounts[d] == 0
									&& data->partsetCounts[d] == 0
									&& data->partresetCounts[d] == 0) {
								continue;
							}
//							PRINTN(
//									" chip["<<d<<"] requestToken["<<tokenentry->requestToken[d]<<"] before tokenPool["<<tokenPool[d]<<"] ");
							tokenPool[d] = tokenPool[d]
									+ tokenentry->requestToken[d];
							data->resetCounts[d] = 0; //RESET完成
//							PRINTN("after tokenPool["<< tokenPool[d]<<"] ");
						}
//						PRINT("");
					}
					if (elapsedCycle == tokenentry->latency) {
						/*	PRINTN(
						 "clock "<<currentClockCycle<<" elapsedCycle["<<elapsedCycle<<"]== tokenentry->latency="<<tokenentry->latency<< " tokenQueue erase continue;");
						 tokenentry->packet->print();*/
						releasedwriteQueue[REQUESTID(r,b)].push_back(*i);
						//						delete (tokenQueue[REQUESTID(r,b)][i]);	//not delete, delete by memory controller.
						i = tokenQueue[REQUESTID(r,b)].erase(i);
						continue;
						//						PRINT("clock "<<elapsedCycle <<" < RESETLatency continue");
					} else {
//						PRINTN(
//								"clock "<<currentClockCycle<<" elapsedCycle["<<elapsedCycle<<"] != tokenentry->latency="<<tokenentry->latency<< " tokenQueue continue;");
//						tokenentry->packet->print();
						i++;
					}
				}
			}
		}
	}
	return;
}
void TokenController::update_Naive() {  //naive的写方式，当完成之后偿还token
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
//			TokenEntry* tokenentry = NULL;
			for (vector<TokenEntry*>::iterator i =
					tokenQueue[REQUESTID(r,b)].begin();
					i != tokenQueue[REQUESTID(r,b)].end();) {
//				PRINT("TC tokenQueue rank "<<r<<" bank "<<b<<" size "<<tokenQueue[REQESTID(r,b)].size());
				unsigned elapsedCycle = 0;
				TokenEntry* tokenentry = *i;
				if (tokenentry->valid != true) {
					i++;
					continue;
				}   //update the powerToken
					if (tokenentry->latency == 0) {
//						PRINTN(
//								"clock "<<currentClockCycle<<" elapsedCycle["<<elapsedCycle<<"]== tokenentry->latency="<<tokenentry->latency<< " tokenQueue erase continue;");
//						tokenentry->packet->print();
						releasedwriteQueue[REQUESTID(r,b)].push_back(*i);
						//						delete (tokenQueue[REQUESTID(r,b)][i]);	//not delete, delete by memory controller.
						i = tokenQueue[REQUESTID(r,b)].erase(i);
						continue;
						//						PRINT("clock "<<elapsedCycle <<" < RESETLatency continue");
					}
					elapsedCycle = currentClockCycle - tokenentry->startCycle;
					if (elapsedCycle < RESETLatency) {
						//actual tokens[d] are(data)*RESETToken
						i++;
						continue;
					}
					if (elapsedCycle == tokenentry->latency) {
						DataCounts* data = NULL;
						data = tokenentry->dataCounts;
//						PRINTN(
//								"updateNaive() elapsedCycle["<<elapsedCycle<<"] tokenQueue pa[0x"<<hex<<bspacket->physicalAddress<<dec<<"] r ["<<r <<"] b ["<<b<<"] i ["<<i<<"] ");
						for (unsigned d = 0; d < NUM_DEVICES; d++) {
							if (data->setCounts[d] == 0
									&& data->resetCounts[d] == 0
									&& data->partsetCounts[d] == 0
									&& data->partresetCounts[d] == 0) {
								continue;
							}
//							PRINTN(
//									"chip["<<d<<"] requestToken["<<tokenentry->requestToken[d]<<"] before tokenPool["<<tokenPool[d]<<"] ");
							tokenPool[d] = tokenPool[d]
									+ tokenentry->requestToken[d];
							data->resetCounts[d] = 0;
							data->setCounts[d] = 0;
							data->partsetCounts[d] = 0; //reallocated
							data->partresetCounts[d] = 0;
//							PRINTN("after tokenPool["<< tokenPool[d]<<"] ");
						}
//						PRINTN("done clock "<<currentClockCycle<<" ");
//						tokenentry->packet->print();
						tokenentry->valid = false;
						tokenentry->done = true; //release the token and write finish
//							tokenentry->packet->print();
						releasedwriteQueue[REQUESTID(r,b)].push_back(*i);
//						delete (tokenQueue[REQUESTID(r,b)][i]);	//not delete, delete by memory controller.
						i = tokenQueue[REQUESTID(r,b)].erase(i);
//							PRINT("tokenQueue erase continue;");
//						PRINT("clock "<<elapsedCycle <<"delta != 7 * (unsigned) SETLatency or continue");
						continue;
					}

//				PRINT("clock "<<currentClockCycle <<" not true");
				i++;
			}
		}
	}
	return;
}
bool TokenController::release(size_t rank, size_t bank, uint64_t &addr) { //memory controller is able to release the request of writeQueue
	for (size_t i = 0; i < releasedwriteQueue[REQUESTID(rank,bank)].size();
			i++) {
		if (releasedwriteQueue[REQUESTID(rank,bank)][i] == NULL) {
//			PRINT("release clock "<<currentClockCycle<<" rank "<<rank<<" bank "<<bank<<" i["<<i<<"] NULL return");
			return false;
		}
		TokenEntry* write = releasedwriteQueue[REQUESTID(rank,bank)][i];
		addr = write->packet->physicalAddress;
//		PRINT(
//					"release clock "<<currentClockCycle<<" rank "<<rank<<" bank "<<bank<<" i["<<i<<"] "<<" TC WRITE ACK is 0x"<<hex<<addr<<dec);
		delete (releasedwriteQueue[REQUESTID(rank,bank)][i]); //not delete, delete by memory controller.
		releasedwriteQueue[REQUESTID(rank,bank)].erase(
				releasedwriteQueue[REQUESTID(rank,bank)].begin() + i);
		return true;
	}
	return false;
}
bool TokenController::powerAllowable(TokenEntry*& writerequest) { //isIssuable for write
//	BusPacket *bspacket = writerequest->packet;
	if (writerequest->valid == true) {
		return false;
	}
	if (nolimit == false) {
		for (unsigned d = 0; d < NUM_DEVICES; d++) {
//		writerequest->requestToken[d] = (data->resetCounts[d]
//				+ data->partresetCounts[d] + data->setCounts[d]
//				+ data->partsetCounts[d]) * RESETToken;
			if (tokenPool[d] < writerequest->requestToken[d]) {
//			PRINTN(
//					"TC false chip["<<d<<"] tokenPool["<<tokenPool[d]<<"] requestToken["<< writerequest->requestToken[d]<<"] ");
				return false;
			}
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
	return true;
}
void TokenController::issueChangestate(TokenEntry*& writerequest) { //isIssuable for write
//	BusPacket *bspacket = writerequest->packet;
	writerequest->startCycle = currentClockCycle;
	writerequest->valid = true;
//	PRINT("");
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
	double fineToken = (double) SETToken / (double) ratio;
	long spanLatency = SETLatency / 5; //单位延迟，span部分所占的时间'
	long fineLatency = ceil((SETLatency - spanLatency) / ratio);
	unsigned deltaLatency = 0;
	unsigned bitsCount = 0;
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
			for (vector<TokenEntry*>::iterator i =
					tokenQueue[REQUESTID(r,b)].begin();
					i != tokenQueue[REQUESTID(r,b)].end();) {
				unsigned elapsedCycle = 0;
				unsigned delta = 0;
				TokenEntry* tokenentry = *i;
				if (tokenentry->valid != true) {
					i++;
					continue;
				}
				if (tokenentry->latency == 0) {
//					PRINTN(
//							"clock "<<currentClockCycle<<" elapsedCycle["<<elapsedCycle<<"]== tokenentry->latency="<<tokenentry->latency<< " tokenQueue erase continue;");
//					tokenentry->packet->print();
					releasedwriteQueue[REQUESTID(r,b)].push_back(*i);
					//						delete (tokenQueue[REQUESTID(r,b)][i]);	//not delete, delete by memory controller.
					i = tokenQueue[REQUESTID(r,b)].erase(i);
					continue;
					//						PRINT("clock "<<elapsedCycle <<" < RESETLatency continue");
				}
				elapsedCycle = currentClockCycle - tokenentry->startCycle;
				DataCounts* data = tokenentry->dataCounts;
				if (elapsedCycle < RESETLatency) {
					i++;
					continue;
				}
				if (elapsedCycle == RESETLatency) { //reclaim			如果是RESET迭代完成，回收RESETtoken，分配SETtoken
//					PRINTN(
//							"elapsedCycle "<<elapsedCycle<<" rank "<<r<<" bank "<<b<<" RESETLatency="<<RESETLatency <<" ");
					for (unsigned d = 0; d < NUM_DEVICES; d++) {
						if (data->setCounts[d] == 0 && data->resetCounts[d] == 0
								&& data->partsetCounts[d] == 0
								&& data->partresetCounts[d] == 0) { //当前chip没有写执行
							continue;
						}
//						PRINTN(
//								" before  chip["<<d<<"] tokenPool["<<tokenPool[d]<<"] ");
						tokenPool[d] = tokenPool[d]
								+ (data->resetCounts[d]
										+ data->partresetCounts[d]
										+ data->setCounts[d]
										+ data->partsetCounts[d]) * RESETToken;
						data->resetCounts[d] = 0; //RESET完成
						tokenPool[d] = tokenPool[d]
								- (data->partresetCounts[d] + data->setCounts[d]
										+ data->partsetCounts[d]) * SETToken; //reallocated
						tokenentry->requestToken[d] = (data->partresetCounts[d]
								+ data->setCounts[d] + data->partsetCounts[d])
								* SETToken;
//						PRINTN(
//								"after ["<<tokenPool[d]<<"]"<<hex<<" pa[0x"<<hex<<tokenentry->packet->physicalAddress<<dec<<"]; ");
						bitsCount = bitsCount + data->setCounts[d]
								+ data->resetCounts[d] + data->partsetCounts[d]
								+ data->partresetCounts[d];
					}
//					PRINT("");
					if (bitsCount == 0) {
						//				PRINT("r["<<r <<"] b["<<b<<"] bitsCount["<<bitsCount<<"]");
						//						PRINT("done ");
						tokenentry->valid = false;
						tokenentry->done = true; //release the token and write finish
						releasedwriteQueue[REQUESTID(r,b)].push_back(*i);
						//						delete (tokenQueue[REQUESTID(r,b)][i]);
						i = tokenQueue[REQUESTID(r,b)].erase(i);
						continue;
					}
				}
				if (elapsedCycle > RESETLatency) {
					delta = elapsedCycle - RESETLatency;
					deltaLatency = delta % (unsigned) SETLatency; //记录第几次迭代
					//等待span time 结束, 大于e_Latency后开始调度SETtoken
					if (((deltaLatency > spanLatency)
							&& ((deltaLatency - spanLatency)
									% (unsigned) fineLatency == 0))
							|| (deltaLatency == 0)) { //根据fineLatency进行功耗调度，每次stage 末进行功耗增加功耗
//						PRINTN(
//								"elapsedCycle "<<elapsedCycle<<" rank "<<r<<" bank "<<b<<hex<<" pa[0x"<<hex<<tokenentry->packet->physicalAddress<<dec<<"] spanLatency="<<spanLatency<<" deltaLatency="<<deltaLatency<<" fineLatency="<<fineLatency<<" ");
						for (unsigned d = 0; d < NUM_DEVICES; d++) {
							if (data->setCounts[d] == 0
									&& data->partsetCounts[d] == 0
									&& data->partresetCounts[d] == 0) { //当前chip没有写执行，查找有写执行的chip
								continue;
							}
//							PRINTN(
//									" before tokenPool d["<<d<<"] ["<<tokenPool[d]<<"] ");
							tokenPool[d] = tokenPool[d]
									+ fineToken
											* (data->partresetCounts[d]
													+ data->setCounts[d]
													+ data->partsetCounts[d]);
//							PRINTN(
//									"after ["<<tokenPool[d]<<hex<<"] pa[0x"<<hex<<tokenentry->packet->physicalAddress<<dec<<"]; ");
							tokenentry->requestToken[d] -= fineToken
									* (data->partresetCounts[d]
											+ data->setCounts[d]
											+ data->partsetCounts[d]);
						}
//						PRINT("");
					}
					if (delta == (unsigned) SETLatency) { //reclaim 回收SETtoken
						for (unsigned d = 0; d < NUM_DEVICES; d++) {
							data->setCounts[d] = 0;
							//							PRINTN("delta=SETLatency; tokenPool["<<tokenPool[d]<<"] ");
						}
					} else if (delta == 5 * (unsigned) SETLatency) { //回收 partset Token
						for (unsigned d = 0; d < NUM_DEVICES; d++) {
							data->partsetCounts[d] = 0; //reallocated
							//							PRINTN("delta=5*SETLatency; tokenPool["<<tokenPool[d]<<"] ");
						}
					} else if (delta == 7 * (unsigned) SETLatency) { //finish no reallocate
						for (unsigned d = 0; d < NUM_DEVICES; d++) {
							data->partresetCounts[d] = 0;
							//							PRINTN("delta=7*SETLatency; tokenPool["<<tokenPool[d]<<"] ");
						}
					}
					if (deltaLatency == 0) {	//SETLatency的时候，更新token
//						PRINTN(
//								"elapsedCycle "<<elapsedCycle<<" deltaLatency == 0 "<<" rank "<<r<<" bank "<<b<<hex<<" pa[0x"<<hex<<tokenentry->packet->physicalAddress<<"] "<<dec);
						for (unsigned d = 0; d < NUM_DEVICES; d++) {
							if (data->setCounts[d] == 0
									&& data->partsetCounts[d] == 0
									&& data->partresetCounts[d] == 0) { //当前chip没有写执行
								continue;
							}
//							PRINTN(
//									" before tokenPool d["<<d<<"] ["<<tokenPool[d]<<"] ");
							tokenPool[d] = tokenPool[d]
									- SETToken
											* (data->partresetCounts[d]
													+ data->setCounts[d]
													+ data->partsetCounts[d]);
							tokenentry->requestToken[d] = SETToken
									* (data->partresetCounts[d]
											+ data->setCounts[d]
											+ data->partsetCounts[d]);
							;
//							PRINTN("after ["<<tokenPool[d]<<"]; ");
						}
//						PRINT("");
					}
				}
				if (elapsedCycle == tokenentry->latency) { //所有片子上的bit数写完，那么写完成，等待释放
					tokenentry->valid = false;
					tokenentry->done = true; //release the token and write finish
//					PRINTN("elapsedCycle "<<elapsedCycle<<" WRITE finish ");
//					tokenentry->packet->print();
					releasedwriteQueue[REQUESTID(r,b)].push_back(*i);
					i = tokenQueue[REQUESTID(r,b)].erase(i);
					continue;
				}
				i++;
			}
		}
	}
}
void TokenController::update_FPB() {
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
			unsigned elapsedCycle = 0;
			unsigned delta = 0;
			unsigned bitsCount = 0;
			for (vector<TokenEntry*>::iterator i =
					tokenQueue[REQUESTID(r,b)].begin();
					i != tokenQueue[REQUESTID(r,b)].end();) {
				TokenEntry* tokenentry = *i;
				if (tokenentry->valid != true) {
					i++; //token->valid!=true;
					continue;
				}  //update the powerToken
					if (tokenentry->latency == 0) {
//							PRINTN(
//											"clock "<<currentClockCycle<<" elapsedCycle["<<elapsedCycle<<"]== tokenentry->latency="<<tokenentry->latency<< " tokenQueue erase continue;");
//											tokenentry->packet->print();
						releasedwriteQueue[REQUESTID(r,b)].push_back(*i);
						//						delete (tokenQueue[REQUESTID(r,b)][i]);	//not delete, delete by memory controller.
						i = tokenQueue[REQUESTID(r,b)].erase(i);
						continue;
						//						PRINT("clock "<<elapsedCycle <<" < RESETLatency continue");
					}
					elapsedCycle = currentClockCycle - tokenentry->startCycle;
					if (elapsedCycle < RESETLatency) {
						i++;
						continue;
					}
					delta = elapsedCycle - RESETLatency;
					if ((delta != 0) && (delta != (unsigned) SETLatency)
							&& (delta != 5 * (unsigned) SETLatency)
							&& (delta != 7 * (unsigned) SETLatency)) { //不是iteration结束的时候
						i++;
						continue;
					}
					DataCounts* data = tokenentry->dataCounts;
//					PRINTN("elapsedCycle["<<elapsedCycle<<"] tokenentry buspacket is ");
//					tokenentry->packet->print();
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
						bitsCount = bitsCount + data->setCounts[d]
								+ data->resetCounts[d] + data->partsetCounts[d]
								+ data->partresetCounts[d];
						//				PRINT("bitsCount["<<bitsCount<<"]");
//							PRINTN("after tokenPool["<< tokenPool[d]<<"] ");
					}
					if (bitsCount == 0 || elapsedCycle == tokenentry->latency) {
//							PRINTN("done r["<<r <<"] b["<<b<<"] bitsCount["<<bitsCount<<"] elapsedCycle "<<elapsedCycle<<" tokenentry->latency "<<tokenentry->latency <<" clock "<<currentClockCycle<<" ");
//							tokenentry->packet->print();
						tokenentry->valid = false;
						tokenentry->done = true; //release the token and write finish
						releasedwriteQueue[REQUESTID(r,b)].push_back(*i);
//						delete (tokenQueue[REQUESTID(r,b)][i]);
						i = tokenQueue[REQUESTID(r,b)].erase(i);
						continue;
					}
				i++;
			}
		}
	}
}
void TokenController::powerUtilization(TokenEntry *writerequest) {
//	const int startCycle = 1000000;
//	const int endCycle = 30000000;
//	if (currentClockCycle < startCycle || currentClockCycle > endCycle) {
//		return;
//	}
	if (writerequest->latency != 0) {
		for (size_t i = 0; i < NUM_DEVICES; i++) {
			tokenUtilityWrites[i] += writerequest->tokenUtility[i];
			utilitywritesNum++;
		}
	}


}
void TokenController::update() {
	if (SPA) {
		update_SPA();
	} else if (FPB) {
		update_FPB();
	} else if (Naive) {
		update_Naive();
	} else if (nolimit) {
		update_nolimit();
	} else if (SPAIdeal) {
		update_SPAIdeal();
	}
	return;
}
