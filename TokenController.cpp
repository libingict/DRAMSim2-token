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
	ratio = 2;			//
//	ratio = 4;
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
			found = true;	//该次写操作要被释放/更新，found is true，表示完成一次写操作，为了回应上层。
			if (newdata != bspacket->dataPacket->getData()) { //如果数据不相等
				delete (writeBank[i]);
				writeBank.erase(writeBank.begin() + i);	//删除旧数据
				writtenData = (bspacket->dataPacket->getoldData()) ^ newdata;
				if(writtenData==0){		//要写的数据跟旧数据相同，那么不需要将新写请求插入队尾。这时候两个写都被删除。
					return true;
				}
				buspacket->dataPacket->setData(newdata,
						bspacket->dataPacket->getoldData());
			}
			//if the written data is same, there is no need to update the write queue，如果数据相同，不需要更新
			else {
				return true;		//这次插入的写操作已经完成
			}
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
	unsigned maxToken = 0; // max RESET counts for chips, represents the required max power，用于排序
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
		tmpEnergy = tmpEnergy
				+ (datacounts->resetCounts[i] + datacounts->partresetCounts[i]
						+ datacounts->setCounts[i]
						+ datacounts->partsetCounts[i]) * getiterNumber(0)
						* resetEnergyperCell
						//if FPB 根据SET 迭代改变
				+ (datacounts->partresetCounts[i] * (getiterNumber(1) - 1)
						+ datacounts->setCounts[i] * (getiterNumber(2) - 1)
						+ datacounts->partsetCounts[i] * (getiterNumber(3) - 1))
						* setEnergyperCell
						* (double) (0.5 + (1 / 2 * (double) ratio));
		//if SPA，根据stage 改变
		/*
		 * + (datacounts->partresetCounts[i] * (getiterNumber(1) - 1)
						+ datacounts->setCounts[i] * (getiterNumber(2) - 1)
						+ datacounts->partsetCounts[i] * (getiterNumber(3) - 1))
						* (double)((setEnergyperCell*2*ratio-1)/5);
		*/
		//if Naive 不随着迭代次数改变
		/*+(datacounts->partresetCounts[i] + datacounts->setCounts[i]+ datacounts->partsetCounts[i])
		  * (getiterNumber(3) - 1)* setEnergyperCell*/
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
			TokenEntry* tokenentry = NULL;
			unsigned elapsedCycle = 0;
			unsigned delta = 0;
			for (size_t i = 0; i < tokenQueue[REQUESTID(r,b)].size(); i++) {
//				PRINT("TC updateNaive() rank "<<r<<" bank "<<b);
				tokenentry = tokenQueue[REQUESTID(r,b)][i];
				if (tokenentry->valid == true) {  //update the powerToken
					elapsedCycle = currentClockCycle - tokenentry->startCycle;
					delta = elapsedCycle - RESETLatency;
					if (delta == 7 * (unsigned) SETLatency) {
						DataCounts* data = NULL;
						BusPacket* bspacket = NULL;
						data = tokenentry->dataCounts;
						bspacket = tokenentry->packet;
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
	TokenEntry* write = NULL;
	addr = 0;
	for (size_t i = 0; i < tokenQueue[REQUESTID(rank,bank)].size(); i++) {
		if (tokenQueue[REQUESTID(rank,bank)][i] == NULL) {
//			PRINT("release clock "<<currentClockCycle<<" rank "<<rank<<" bank "<<bank<<" i["<<i<<"] NULL return");
			return false;
		}
		write = tokenQueue[REQUESTID(rank,bank)][i];
		if (write->done == true) {
			addr = write->packet->physicalAddress;
//			PRINT(
//					"release clock "<<currentClockCycle<<" rank "<<rank<<" bank "<<bank<<" i["<<i<<"] "<<" TC release is 0x"<<hex<<addr<<dec);
			delete (tokenQueue[REQUESTID(rank,bank)][i]); //not delete, delete by memory controller.
			tokenQueue[REQUESTID(rank,bank)].erase(
					tokenQueue[REQUESTID(rank,bank)].begin() + i);
//		PRINT("release clock "<<currentClockCycle<<" r "<<rank<<" b "<<bank<<" &r "<<&rank<<" &bank "<<&bank<<" true");
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
//			writerequest->packet->print();
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
//	writerequest->packet->print();
//	writerequest->startCycle = currentClockCycle;
//	writerequest->valid = true;
//	PRINT("");
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
	long e_Latency = SETLatency / 5; //单位延迟，span部分所占的时间'
	long fineLatency = ceil((SETLatency - e_Latency) / ratio);
	unsigned deltaLatency = 0;
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
			for (size_t i = 0; i < tokenQueue[REQUESTID(r,b)].size(); i++) {
				TokenEntry* tokenentry = NULL;
				unsigned elapsedCycle = 0;
				unsigned delta = 0;
				unsigned bitsCount = 0;
				tokenentry = tokenQueue[REQUESTID(r,b)][i];
				if (tokenentry->valid == true) { //update the powerToken，write is being executed
					elapsedCycle = currentClockCycle - tokenentry->startCycle;
					DataCounts* data = tokenentry->dataCounts;
					if (elapsedCycle < RESETLatency) {
						continue;
					}
					if (elapsedCycle == RESETLatency) { //reclaim			如果是RESET迭代完成，回收RESETtoken，分配SETtoken
						PRINTN(
								"rank "<<r<<" bank "<<b<<" i "<<i<<" elapsedCycle="<<elapsedCycle<<" RESETLatency="<<RESETLatency <<" ");
						for (unsigned d = 0; d < NUM_DEVICES; d++) {
							if (data->setCounts[d] == 0
									&& data->resetCounts[d] == 0
									&& data->partsetCounts[d] == 0
									&& data->partresetCounts[d] == 0) { //当前chip没有写执行
								continue;
							}
							PRINTN(
									" before  chip["<<d<<"] tokenPool["<<tokenPool[d]<<"] ");
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
							PRINT(
									"after ["<<tokenPool[d]<<"]"<<hex<<"] pa[0x"<<hex<<tokenentry->packet->physicalAddress<<dec<<"]; ");
							bitsCount = bitsCount + data->setCounts[d]
									+ data->resetCounts[d]
									+ data->partsetCounts[d]
									+ data->partresetCounts[d];
						}
						if (bitsCount == 0) {  //所有片子上的bit数写完，那么写完成，等待释放
							tokenentry->valid = false;
							tokenentry->done = true; //release the token and write finish
						}
					} else if (elapsedCycle > RESETLatency) {
						delta = elapsedCycle - RESETLatency;
						deltaLatency = delta % (unsigned) SETLatency; //记录第几次迭代
						//等待span time 结束, 大于e_Latency后开始调度SETtoken
						if (((deltaLatency > e_Latency)
								&& (deltaLatency - e_Latency)
										% (unsigned) fineLatency == 0)
								|| (deltaLatency == 0)) { //根据fineLatency进行功耗调度，每次stage 末进行功耗更新，增加功耗
							PRINTN(
									"rank "<<r<<" bank "<<b<<" i "<<i<<" e_Latency="<<e_Latency<<"deltaLatency["<<deltaLatency<<"] fineLatency="<<fineLatency<<" ");
							for (unsigned d = 0; d < NUM_DEVICES; d++) {
								if (data->setCounts[d] == 0
										&& data->partsetCounts[d] == 0
										&& data->partresetCounts[d] == 0) { //当前chip没有写执行，查找有写执行的chip
									continue;
								}
								PRINTN(
										" before tokenPool d["<<d<<"] ["<<tokenPool[d]<<"] ");
								tokenPool[d] =
										tokenPool[d]
												+ fineToken
														* (data->partresetCounts[d]
																+ data->setCounts[d]
																+ data->partsetCounts[d]);
								PRINT(
										"after ["<<tokenPool[d]<<hex<<"] pa[0x"<<hex<<tokenentry->packet->physicalAddress<<dec<<"]; ");
							}
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
							PRINTN(
									"rank "<<r<<" bank "<<b<<" i "<<i<<" e_Latency="<<e_Latency<<"deltaLatency["<<deltaLatency<<"] fineLatency="<<fineLatency<<" ");
							for (unsigned d = 0; d < NUM_DEVICES; d++) {
								if (data->setCounts[d] == 0
										&& data->partsetCounts[d] == 0
										&& data->partresetCounts[d] == 0) { //当前chip没有写执行
									continue;
								}
								tokenPool[d] =
										tokenPool[d]
												- SETToken
														* (data->partresetCounts[d]
																+ data->setCounts[d]
																+ data->partsetCounts[d]);
								PRINT(
										"after ["<<tokenPool[d]<<hex<<"] pa[0x"<<hex<<tokenentry->packet->physicalAddress<<dec<<"]; ");
								bitsCount = bitsCount + data->setCounts[d]
										+ data->resetCounts[d]
										+ data->partsetCounts[d]
										+ data->partresetCounts[d];
							}
							if (bitsCount == 0) {  //所有片子上的bit数写完，那么写完成，等待释放
								tokenentry->valid = false;
								tokenentry->done = true; //release the token and write finish
							}
						}
					}
				}
			}
		}
	}
}
void TokenController::update_FPB() {
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
			TokenEntry* tokenentry = NULL;
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
									+ data->partsetCounts[d]) * RESETToken;
							data->resetCounts[d] = 0;//RESET完成
							tokenPool[d] = tokenPool[d]
							- (data->partresetCounts[d] + data->setCounts[d]
									+ data->partsetCounts[d]) * SETToken;//reallocated
							//					PRINTN("delat=0; tokenPool["<<tokenPool[d]<<"] ");
						} else if (delta == (unsigned) SETLatency) { //reclaim 回收SETtoken
							tokenPool[d] = tokenPool[d]
							+ (data->setCounts[d]) * SETToken;
							data->setCounts[d] = 0;
//							PRINTN("delta=SETLatency; tokenPool["<<tokenPool[d]<<"] ");
						} else if (delta == 5 * (unsigned) SETLatency) { //回收 partset Token
							tokenPool[d] = tokenPool[d]
							+ (data->partsetCounts[d]) * SETToken;
							data->partsetCounts[d] = 0;//reallocated
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
					if (bitsCount == 0) {
						//				PRINT("r["<<r <<"] b["<<b<<"] bitsCount["<<bitsCount<<"]");
//						PRINT("done ");
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
void TokenController::update() {
	update_SPA();
//	update_Naive();
}
