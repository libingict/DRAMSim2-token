/*
 * CancelWrite.cpp
 *
 *  Created on: 2014-3-13
 *      Author: libing
 */

#include "CancelWrite.h"
#include <cstdlib>
#include <time.h>
#include <math.h>
using namespace DRAMSim;
#define REQUESTID(rank,bank) (rank*NUM_BANKS)+bank
CancelWrite::CancelWrite(vector<vector<BankState> > &states,
		ostream &dramsim_log_, vector<Rank *> *&ranks_) :
		bankStates(states), readQueue(states, dramsim_log_), ranks(ranks_), writeQueueDepth(
				CMD_QUEUE_DEPTH), nextRank(0), nextBank(0), nextRankPRE(0), nextBankPRE(
				0), dramsim_log(dramsim_log_) {
	currentClockCycle = 0;
	writecancel = vector < vector<bool>
			> (NUM_RANKS, vector<bool>(NUM_BANKS, false));
	coutcanceledwrite = vector < vector<uint64_t>
			> (NUM_RANKS, vector < uint64_t > (NUM_BANKS, 0)); //record the Count of Canceled Write
	ongoingWrite = vector < vector<BusPacket *>
			> (NUM_RANKS, vector<BusPacket *>(NUM_BANKS, NULL));
	writepriority = vector < vector<bool>
			> (NUM_RANKS, vector<bool>(NUM_BANKS, false));
	sendPRE = vector < vector<bool>
			> (NUM_RANKS, vector<bool>(NUM_BANKS, false));
	windowsize = vector < vector<unsigned>
			> (NUM_RANKS, vector<unsigned>(NUM_BANKS, 0));
	writeQueue = vector < vector<TokenEntry*>
			> (NUM_RANKS * NUM_BANKS, vector<TokenEntry*>(0, NULL)); //2D
	tokenRank = new TokenController(writeQueue, dramsim_log_);
	zerowrite = vector < uint64_t > (NUM_RANKS * NUM_BANKS, 0);
	mergedWritesPerBank = vector < uint64_t > (NUM_RANKS * NUM_BANKS, 0);
	totalToken = vector<double>(NUM_DEVICES, 0.0);
	maxToken = vector<double>(NUM_DEVICES, 0.0);
	averageToken = vector<double>(NUM_DEVICES, 0.0);
	writeburstCycle = vector < vector<uint64_t>
			> (NUM_RANKS * NUM_BANKS, vector < uint64_t > (0, 0));
	readfoundReturn = vector < uint64_t > (NUM_RANKS * NUM_BANKS, 0);
	readFounds = 0;
	countAlg = 0;
	priorityWrites = vector < vector<uint64_t>
			> (NUM_RANKS, vector < uint64_t > (NUM_BANKS, 0));
	falseWrites = 0;
	//64bit data, from low to high mapped to chip 0-7

}
CancelWrite::~CancelWrite() {
	for (size_t r = 0; r < NUM_RANKS; r++) {
		ongoingWrite[r].clear();
		writecancel[r].clear();
		writepriority[r].clear();
		sendPRE[r].clear();
		windowsize[r].clear();
		for (size_t b = 0; b < NUM_BANKS; b++) {
			for (size_t i = 0; i < writeQueue[REQUESTID(r,b)].size(); i++) {
				delete (writeQueue[REQUESTID(r,b)][i]);
			}
			writeQueue[REQUESTID(r,b)].clear();
		}
	}
	delete tokenRank;
}
bool CancelWrite::addRequest(Transaction *transaction, BusPacket *buspacket,
		bool &found) {
	if (transaction->transactionType == DATA_READ) {
		vector<TokenEntry*> &wrbank =
				writeQueue[REQUESTID(buspacket->rank, buspacket->bank)];
		for (unsigned i = 0; i < wrbank.size(); i++) {
			BusPacket *bspacket = wrbank[i]->packet;
			if (bspacket->physicalAddress == transaction->address) { //if write queue has the same address, then the read request can be returned
				transaction->transactionType = RETURN_DATA; //MC got the returned data from write queue.
				if (bspacket->busPacketType == ACTIVATE && wrbank.size() > 1
						&& i < wrbank.size() - 1
						&& wrbank[i + 1]->packet->physicalAddress
								== buspacket->physicalAddress) {
					transaction->set_data(
							wrbank[i + 1]->packet->dataPacket->getData(),
							wrbank[i + 1]->packet->dataPacket->getoldData());
				} else {
					transaction->set_data(bspacket->dataPacket->getData(),
							bspacket->dataPacket->getoldData());
				}
				found = true;
				readfoundReturn[REQUESTID(buspacket->rank, buspacket->bank)]++;
				readFounds++;
				break;
			}
		}
		if (found) {
			return true;
		}
		if (readQueue.hasRoomFor(2, buspacket->rank, buspacket->bank)) {
			readQueue.enqueue(
					new BusPacket(ACTIVATE, buspacket->physicalAddress,
							buspacket->column, buspacket->row, buspacket->rank,
							buspacket->bank, buspacket->dataPacket, dramsim_log,
							buspacket->RIP));
			readQueue.enqueue(
					new BusPacket(READ, buspacket->physicalAddress,
							buspacket->column, buspacket->row, buspacket->rank,
							buspacket->bank, buspacket->dataPacket, dramsim_log,
							buspacket->RIP));
			return true;
		} else {
			delete buspacket;
			return false;
		}
	} else if (transaction->transactionType == DATA_WRITE) { //order the request according to the power
		//update the tokenqueue
		uint64_t writtendata = buspacket->dataPacket->getData()
				^ buspacket->dataPacket->getoldData();		//按位异或
		bool add = false;
		if (writtendata == 0) { //写的数据为0， 不插入
			zerowrite[REQUESTID(buspacket->rank, buspacket->bank)]++;
			found = true;
			return true;
		}
		add = tokenRank->addwriteRequest(
				writeQueue[REQUESTID(buspacket->rank, buspacket->bank)],
				buspacket, found);
		if (found) {
			mergedWritesPerBank[REQUESTID(buspacket->rank,buspacket->bank)]++;
		}
		if (add) {
//			writerequest[buspacket->rank][buspacket->bank]++;
			return true;
		} else {
			delete buspacket;
			return false;
		}
	}
}

bool CancelWrite::issueRead(unsigned r, unsigned b, BusPacket *&busPacket) {
	bool issuable = false;
	bool found = false; //有相同的row
	vector<BusPacket *> &queue = readQueue.getCommandQueue(r, b);
	for (unsigned i = 0; i < queue.size(); i++) {
		BusPacket *request = queue[i];
		if (bankStates[r][b].currentBankState == RowActive
				&& queue[i]->row == bankStates[r][b].openRowAddress) {
			found = true;
		}
		//TODO: check timing and power constraint, the request in the queue is ordered according to the required power
		if (readQueue.isIssuable(request)) { //check for the timing constraint
//			PRINTN("issueRequest readqueue clock "<<currentClockCycle<<" ");
			busPacket = new BusPacket(request->busPacketType,
					request->physicalAddress, request->column, request->row,
					request->rank, request->bank, request->dataPacket,
					dramsim_log, request->RIP);
//			PRINTN(" busPacket ");
//			busPacket->print();
			/*
			 if the bus packet before is an activate, that is the act that was
			 paired with the column access we are removing, so we have to remove
			 that activate as well (check i>0 because if i==0 then there's nothing before it)
			 */
			if (i > 0 && queue[i - 1]->busPacketType == ACTIVATE) {
				// i is being returned, but i-1 is being thrown away, so must delete it here
				delete (queue[i - 1]);
				delete (queue[i]);
				// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
				queue.erase(queue.begin() + i - 1, queue.begin() + i + 1);
			} else // there's no activate before this packet
			{
				//or just remove the one bus packet
				//if write then check the power token per chip. until writes for all chips are done
				delete (queue[i]);
				queue.erase(queue.begin() + i);
			}
			issuable = true;
			break;
		}
	}
	if (issuable == false && found == false) {
		if (bankStates[r][b].currentBankState == RowActive
				&& (sendPRE[r][b] == false)) {
			sendPRE[r][b] = true;
//			issuable = true;
//			busPacket = new BusPacket(PRECHARGE, 0, 0, 0, r, b, NULL,
//					dramsim_log);
//			PRINT("issueRead set PRE r["<<r<<"]b["<<b<<"] row["<<bankStates[r][b].openRowAddress<<"] ");
		}
	}
	return issuable;
}

bool CancelWrite::issueWrite_PAS(unsigned r, unsigned b,
		BusPacket *&busPacket) { //how to issue write
	TokenEntry* request;		//功耗优先，如果功耗可以，那么就发射该请求。
	vector<TokenEntry *> &queue = writeQueue[REQUESTID(r, b)];
	bool issuable = false;
	unsigned range = 0;
	//check the power allowable
	//power priority
	unsigned head = 0;
	for (head = 0; head < writeQueue[REQUESTID(r,b)].size(); head++) {
		if (queue[head]->valid != true) {
			break;
		}
	}
	if (writepriority[r][b] == false) {
		PRINT(
				"r "<<r<<" b "<<b<<" writepriority false range is "<<writeQueue[REQUESTID(r, b)].size()-head);
		range = writeQueue[REQUESTID(r, b)].size() - head;
	} else {
		PRINT(
				"r "<<r<<" b "<<b<<" writepriority true range is "<<windowsize[r][b]);
		range = windowsize[r][b];
	}
	for (unsigned i = head; i < head + range; i++) {
		request = queue[i];
		BusPacket *buspacket = request->packet;
//		PRINT("issueWrite_PAS currentClock is "<<currentClockCycle);
//		bankStates[r][b].print();
//		if (currentClockCycle >= bankStates[r][b].nextPrecharge) {
		if (bankStates[r][b].currentBankState == RowActive
				&& currentClockCycle >= bankStates[r][b].nextWrite) {//row miss
			if (tokenRank->powerAllowable(request)) {
				if (buspacket->row != bankStates[r][b].openRowAddress) {
					PRINTN("Row miss ");
					busPacket = new BusPacket(PREACTWR,
							buspacket->physicalAddress, buspacket->column,
							buspacket->row, buspacket->rank, buspacket->bank,
							buspacket->dataPacket, dramsim_log, buspacket->RIP,
							request->latency, request->energy);
				} else {
					PRINTN("Row hit ");
					busPacket = new BusPacket(WRITE, buspacket->physicalAddress,
							buspacket->column, buspacket->row, buspacket->rank,
							buspacket->bank, buspacket->dataPacket, dramsim_log,
							buspacket->RIP, request->latency, request->energy);
				}
				issuable = true;
			}
		} else if (bankStates[r][b].currentBankState == Idle
				&& currentClockCycle >= bankStates[r][b].nextActivate) {	//空
			if (tokenRank->powerAllowable(request)) {
				busPacket = new BusPacket(ACTWR, buspacket->physicalAddress,
						buspacket->column, buspacket->row, buspacket->rank,
						buspacket->bank, buspacket->dataPacket, dramsim_log,
						buspacket->RIP, request->latency, request->energy);
//			PRINTN("Idle or Precharging ");
				issuable = true;
			}
		}
		if (issuable) {
			if (writepriority[r][b] == true && windowsize[r][b] > 0) { //不是因为readQueue为空
				windowsize[r][b]--;
			}
			tokenRank->issueChangestate(request);
			tokenRank->powerUtilization(request);
			PRINTN("issueWrite_PAS true currentClock is "<<currentClockCycle);
			busPacket->print();
			return issuable;
		}
//		
	}
	if (!issuable) {
		PRINTN("issueWrite_PAS false currentClock is "<<currentClockCycle);
		bankStates[r][b].print();
	}
	return issuable;
	//check the timing constraint
}
void CancelWrite::firstfit(unsigned r, unsigned b, unsigned rowAddr) {
	//rowaddress, power的情况,
	bool powerFeasible = true;
	vector<double> tmpsumToken = vector<double>(NUM_DEVICES, 0);
	//						PRINT("firstfit ");
	vector<double> tokenRemain(tokenRank->tokenPool);
	unsigned head = 0;
	unsigned rank = r;
	unsigned bank = b;
	bank++;
	if (bank == NUM_BANKS) {
		bank = 0;
		rank++;
		if (rank == NUM_RANKS) {
			rank = 0;
		}
	}
	countAlg++;
	do {
		if (writepriority[rank][bank]) {
			vector<TokenEntry *> &queue = writeQueue[REQUESTID(rank, bank)];
			TokenEntry * request;
			for (head = 0; head < queue.size(); head++) {
				if (queue[head]->valid != true) {
//					isFalse = true;
					break;
				}
			}
//			if (!isFalse)
//				continue;
			for (unsigned j = head;
					j < queue.size() && j < head + windowsize[rank][bank];
					j++) {
				request = queue[j];
				if (request->packet->row == rowAddr) { //如果row相同
					//if power 合适，那么把它排在第一位
					for (unsigned d = 0; d < NUM_DEVICES; d++) {
						if (request->requestToken[d] > tokenRemain[d]) { //功耗不能满足，查找writeQueue中的下一个请求，不把该请求加入要处理的请求中如果power 允许
							powerFeasible = false;
							break;
						}
					}
					if (powerFeasible == true) {
						////把该请求放在队列的首位，优先调度
						for (unsigned d = 0; d < NUM_DEVICES; d++) {
							if (request->requestToken[d] != 0) { //统计当前的功耗值；
								tmpsumToken[d] += request->requestToken[d];
								tokenRemain[d] -= request->requestToken[d];
							}
						}
						TokenEntry* tmp = queue[j];
//						tmp->packet->print();
						queue.insert(queue.begin() + head, tmp);
						queue.erase(queue.begin() + j + 1);
						break;
					}
				}
			}
		}
		bank++;
		if (bank == NUM_BANKS) {
			bank = 0;
			rank++;
			if (rank == NUM_RANKS) {
				rank = 0;
			}
		}
	} while (!(rank == r && bank == b));
	return;
}
bool CancelWrite::writeScheduling(unsigned rank_, unsigned bank_,
		BusPacket *&busPacket) {  //当调度某个bank的write 时
	bool issuable = false;	//是否可执行
	bool feasible = false;	//可行的
	unsigned range = 0;
	unsigned rank = rank_;
	unsigned bank = bank_;
	unsigned rowAddress;
	vector<double> &tokenRemain = tokenRank->tokenPool;	//Pool中现有的功耗，只有大于请求需要的功耗才能执行
	vector<TokenEntry *> &queue = writeQueue[REQUESTID(rank, bank)];
	BusPacket *buspacket;
	TokenEntry *request;
	unsigned head = 0;
	unsigned i = 0;
	bool found = false;
//bankStates[rank][bank].print();
//	PRINT("currentClockCycle "<<currentClockCycle);
	for (head = 0; head < writeQueue[REQUESTID(rank,bank)].size(); head++) {
		if (queue[head]->valid != true) {
			found = true;
			break;
		}
	}
	if (!found) {
//		PRINT("All true!")
		return false;
	}
	if (writepriority[rank][bank] == false) {
//			PRINT(
//					"rank "<<rank<<" bank "<<bank<<" writepriority false range is "<<writeQueue[REQUESTID(rank, bank)].size()-head<<" head is "<<head);
		range = writeQueue[REQUESTID(rank, bank)].size() - head;
	} else {
		range = windowsize[rank][bank];
	}
	for (i = head; i < head + range; i++) { //应该是遇到currenttail指针
		request = queue[i];
		buspacket = request->packet;
		if (bankStates[rank][bank].currentBankState == RowActive //row hit or idle
		&& buspacket->row == bankStates[rank][bank].openRowAddress) {
			//那么将所有writepriority为true的bank中的写请求进行排序考察
			feasible = true;
			if (currentClockCycle >= bankStates[rank][bank].nextWrite
					&& tokenRank->powerAllowable(request)) { //如果是Row Active,时序满足
				busPacket = new BusPacket(WRITE, buspacket->physicalAddress,
						buspacket->column, buspacket->row, buspacket->rank,
						buspacket->bank, buspacket->dataPacket, dramsim_log,
						buspacket->RIP, request->latency, request->energy);
				rowAddress = bankStates[rank][bank].openRowAddress;
				issuable = true;
				break;
			}
		}
		if (bankStates[rank][bank].currentBankState == Idle
				&& currentClockCycle >= bankStates[rank][bank].nextActivate) {
			//那么将所有writepriority为true的bank中的写请求进行排序考察
			if (tokenRank->powerAllowable(request)) {
				busPacket = new BusPacket(ACTWR, buspacket->physicalAddress,
						buspacket->column, buspacket->row, buspacket->rank,
						buspacket->bank, buspacket->dataPacket, dramsim_log,
						buspacket->RIP, request->latency, request->energy);
				rowAddress = buspacket->physicalAddress;
				issuable = true;
				break;
			}
		}
	}
	if (issuable) {
		tokenRank->issueChangestate(request);
		if (windowsize[rank][bank] == windowSize) { //不是因为readQueue为空，刚刚为true
			firstfit(rank, bank, rowAddress);
		}
		if (writepriority[rank][bank] == true) {
			priorityWrites[rank][bank]++;
			if (windowsize[rank][bank] > 0) { //不是因为readQueue为空更新windowsize
				windowsize[rank][bank]--;
			}
		} else {
			falseWrites++;
		}
		queue.insert(queue.begin() + head, request);
		queue.erase(queue.begin() + i + 1);
//		PRINTN(
//				"rank "<<rank<<" bank "<<bank<<" writepriority true range is "<<windowsize[rank][bank]<<" head is "<<head);
//		PRINTN(" PAS clock "<<currentClockCycle<<" issueWrite_PAS ");
//		busPacket->print();	//把这个请求放在head部
		tokenRank->powerUtilization(request);
		return true;
	}
//把其他rank以及其他bank的请求都访问一遍,调度下次的请求
	if (bankStates[rank][bank].currentBankState == RowActive) {
		if ((!feasible) && (sendPRE[rank][bank] == false)) {
			sendPRE[rank][bank] = true;
//			PRINT(
//					"PAS clock "<<currentClockCycle<<" r "<<rank<<" b "<<bank<<" write sendPRE true");
		}
		return false;
	}
	return issuable;
}
bool CancelWrite::issueWrite_RP(unsigned r, unsigned b, BusPacket *&busPacket) { //时序优先，必须时序满足，再检测功耗。
//timing priority
	TokenEntry *request;
	BusPacket *buspacket;
	bool issuable = false;
	bool found = false;
	bool isValid = false;
	unsigned range = 0;
	unsigned head = 0;
	for (head = 0; head < writeQueue[REQUESTID(r,b)].size(); head++) {
		if (writeQueue[REQUESTID(r,b)][head]->valid != true) {
			isValid = true;
			break;
		}
	}
	if (!isValid) { //全部都在执行，所以没有合适的write可以被调度
//				PRINT(
//						"clock "<<currentClockCycle<<" r "<<r<<" b "<<b<<" writeQueue size "<<writeQueue[REQUESTID(r, b)].size()<<" all writing return");
		return false;
	}
	if (writepriority[r][b] == false) {
//				PRINT(
//						"r "<<r<<" b "<<b<<" writepriority false range is "<<writeQueue[REQUESTID(r, b)].size());
		range = writeQueue[REQUESTID(r, b)].size() - head;
	} else {
//				PRINT(
//						"r "<<r<<" b "<<b<<" writepriority true range is "<<windowsize[r][b]);
		range = windowsize[r][b];
	}
	if (bankStates[r][b].currentBankState == RowActive) { 	//row active
		for (unsigned i = head;
				i < head + range && i < writeQueue[REQUESTID(r,b)].size();
				i++) {
			request = writeQueue[REQUESTID(r,b)][i];
			buspacket = request->packet;
			if (buspacket->row == bankStates[r][b].openRowAddress) { //same row, not close and/or issue
				//row hit
				found = true;
				if (currentClockCycle >= bankStates[r][b].nextWrite) {
					if (tokenRank->powerAllowable(request)) { //power is not allowed
						busPacket = new BusPacket(WRITE,
								buspacket->physicalAddress, buspacket->column,
								buspacket->row, buspacket->rank,
								buspacket->bank, buspacket->dataPacket,
								dramsim_log, buspacket->RIP, request->latency,
								request->energy);	//写的延迟
//						PRINTN(
//								"RP clock "<<currentClockCycle<<" issueWrite_RP row hit ");
//						busPacket->print();
						issuable = true;
						tokenRank->issueChangestate(request);
						tokenRank->powerUtilization(request);
						if (writepriority[r][b] == true) {
							priorityWrites[r][b]++;
							if (windowsize[r][b] > 0) { //不是因为readQueue为空
								windowsize[r][b]--;
							}
						} else {
							falseWrites++;
						}
						//移到queue的头部
						writeQueue[REQUESTID(r, b)].insert(
								writeQueue[REQUESTID(r, b)].begin() + head,
								request);
						writeQueue[REQUESTID(r, b)].erase(
								writeQueue[REQUESTID(r, b)].begin() + i + 1);
						return true;
					}
//					PRINT(
//							"RP clock "<<currentClockCycle<<" r "<<r<<" b "<<b<<" issueWrite_RP row hit powerAllowable false");
				}
				break;
			}
		}
	} else if (bankStates[r][b].currentBankState == Idle
			&& currentClockCycle >= bankStates[r][b].nextActivate) {		//空
		for (unsigned i = head;
				i < head + range && i < writeQueue[REQUESTID(r,b)].size();
				i++) {
			request = writeQueue[REQUESTID(r,b)][i];
			if (tokenRank->powerAllowable(request)) {	//power is allowed
				buspacket = request->packet;
				busPacket = new BusPacket(ACTWR, buspacket->physicalAddress,
						buspacket->column, buspacket->row, buspacket->rank,
						buspacket->bank, buspacket->dataPacket, dramsim_log,
						buspacket->RIP, request->latency, request->energy);	//写的延迟
//				PRINTN("RP clock "<<currentClockCycle<<" issueWrite_RP Idle ");
//				busPacket->print();
				issuable = true;
				if (writepriority[r][b] == true) {
					priorityWrites[r][b]++;
					;
					if (windowsize[r][b] > 0) { //不是因为readQueue为空
						windowsize[r][b]--;
					}
				} else {
					falseWrites++;
				}
				tokenRank->issueChangestate(request);
				tokenRank->powerUtilization(request);
				//移到queue的头部
				writeQueue[REQUESTID(r, b)].insert(
						writeQueue[REQUESTID(r, b)].begin() + head, request);
				writeQueue[REQUESTID(r, b)].erase(
						writeQueue[REQUESTID(r, b)].begin() + i + 1);
				return true;
			}
//			PRINT("RP clock "<<currentClockCycle<<" r "<<r<<" b "<<b<<" issueWrite_RP Idle powerAllowable false ");
			//break;
		}
	}
	if ((found == false) && (bankStates[r][b].currentBankState == RowActive)
			&& (sendPRE[r][b] == false)) {
		sendPRE[r][b] = true;
//		PRINT("RP clock "<<currentClockCycle<<" r "<<r<<" b "<<b<<" write sendPRE true");
		return false;
	}
	return issuable;
}
bool CancelWrite::writeCancel(unsigned r, unsigned b, BusPacket *&busPacket) { //if there is waiting read request, issue the request. write能否cancel，如果能，恢复现场？如果不能，什么都不做。
	bool issueread = false;
	bool found = false;
	vector<BusPacket *> &rdqueue = readQueue.getCommandQueue(r, b);
	vector<TokenEntry *> &writequeue = writeQueue[REQUESTID(r,b)];
	//找到正在执行的请求，将该请求取消，恢复该TokenEntry的状态
	//更新writeQueue，将该写请求插回到队列中
	if (bankStates[r][b].currentBankState == RowActive
			&& bankStates[r][b].lastCommand == WRITE) {		//正在进行write
		for (size_t i = 0; i < writequeue.size(); i++) {	//pause the write
			if (writequeue[i]->valid == true) {
				writequeue[i]->valid = false;
				writequeue[i]->startCycle = 0;
				break;
			}
		}
	}
	//更新bank的时序
	//更改rank的时序，使得其能够接受这次读？
//	if (bankStates[r][b].currentBankState == RowActive
//			&& bankStates[r][b].lastCommand == WRITE) {		//正在进行write
	//1.push_back the paired of Write and ACTIVATE to the writequeue.
	for (unsigned i = 0; i < rdqueue.size(); i++) {			//查找需要发射的读请求
		BusPacket *packet = rdqueue[i];
		if (bankStates[r][b].openRowAddress == packet->row) {//如果命中，tWTR+tREAD
			issueread = true;
			found = true;
			//modify all timing for all Bank
			/*						bankStates[r][b].nextRead = currentClockCycle;
			 //2.modify the cycle of Rank;
			 //modify all timing for all Bank in Rank
			 (*ranks)[r]->bankStates[b].nextRead = currentClockCycle;*/
			busPacket = new BusPacket(READ_PREHIT, packet->physicalAddress,
					packet->column, packet->row, packet->rank, packet->bank,
					packet->dataPacket, dramsim_log, packet->RIP);
			if (i > 0 && rdqueue[i - 1]->busPacketType == ACTIVATE) {
				// i is being returned, but i-1 is being thrown away, so must delete it here
				delete (rdqueue[i - 1]);
				delete (rdqueue[i]);
				// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
				rdqueue.erase(rdqueue.begin() + i - 1, rdqueue.begin() + i + 1);
			} else // there's no activate before this packet
			{
				//or just remove the one bus packet
				//if write then check the power token per chip. until writes for all chips are done
				delete (rdqueue[i]);
				rdqueue.erase(rdqueue.begin() + i);
			}
			break;
		}
	}
	if (found == false) {		//如果不命中，那么这次读所需的时间为tWTR+PRE+ACT+tREAD
		issueread = true;
		busPacket = new BusPacket(READ_PREMISS, rdqueue[0]->physicalAddress,
				rdqueue[0]->column, rdqueue[0]->row, rdqueue[0]->rank,
				rdqueue[0]->bank, rdqueue[0]->dataPacket, dramsim_log,
				rdqueue[0]->RIP);
		if (rdqueue.size() > 1 && rdqueue[0]->busPacketType == ACTIVATE
				&& rdqueue[1]->busPacketType == READ
				&& rdqueue[1]->row == rdqueue[0]->row) {
			// i is being returned, but i-1 is being thrown away, so must delete it here
			delete (rdqueue[0]);
			delete (rdqueue[1]);
			// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
			rdqueue.erase(rdqueue.begin(), rdqueue.begin() + 2);
		} else // there's no activate before this packet
		{
			//or just remove the one bus packet
			//if write then check the power token per chip. until writes for all chips are done
			delete (rdqueue[0]);
			rdqueue.erase(rdqueue.begin());
		}
	}
	//更新rank中bank的时序
	return issueread;
	//if idle nothing to do , no write block the read
}
bool CancelWrite::issue(BusPacket **busPacket) {
	if (rowBufferPolicy == OpenPage && queuingStructure == PerRankPerBank) {
		bool sendingWR = false;
		bool sendingRD = false;
		unsigned startingRank = nextRank;
		unsigned startingBank = nextBank;
		do {
			vector<TokenEntry *> &writequeue =
					writeQueue[REQUESTID(nextRank, nextBank)];
			vector<BusPacket *> &readqueue = readQueue.getCommandQueue(nextRank,
					nextBank);
			if (!(writequeue.empty() && readqueue.empty())) {
				size_t head = 0;
				if (writequeue.size() > 0) {
					for (head = 0; head < writequeue.size(); head++) { //check the number of buffered write
						if (writequeue[head]->valid != true) {
//							PRINT("break! head "<<head<<" windowqueue.size "<<writequeue.size()<<" RP "<<RP<< " SPAIdeal "<<SPAIdeal<<" ");
							break;
						}
					}
					if ((writepriority[nextRank][nextBank] == false)
							&& (writequeue.size() > head)
							&& ((writequeue.size() - head) >= windowSize)) {
						writepriority[nextRank][nextBank] = true;
						windowsize[nextRank][nextBank] = windowSize;
						writeburstCycle[REQUESTID(nextRank,nextBank)].push_back(
								currentClockCycle);
//						PRINT(
//								"turns true windowsize: "<< windowsize[nextRank][nextBank]<< " currentClock "<<currentClockCycle<<" writequeue.size "<<writequeue.size()<<" head "<<head<<" r["<<nextRank<<"] b["<<nextBank<<"] ");
					}
				}
				if (writepriority[nextRank][nextBank]) {
					if (!WRITECANCEL) {
						if (windowsize[nextRank][nextBank] == 0) { //写完成以后，writepriority是false，保证这段时间是batch调度的。
							writepriority[nextRank][nextBank] = false;
							windowsize[nextRank][nextBank] = 0;
							uint64_t startCycle =
									writeburstCycle[REQUESTID(nextRank,nextBank)][writeburstCycle[REQUESTID(nextRank,nextBank)].size()
											- 1];
//							PRINT(
//									" r["<<nextRank<<"] b["<<nextBank<<"]  burst start at clock "<<startCycle<<" end at currentClock "<<currentClockCycle);
							writeburstCycle[REQUESTID(nextRank,nextBank)][writeburstCycle[REQUESTID(nextRank,nextBank)].size()
									- 1] = currentClockCycle - startCycle;
						} else {
//							sendingWR = issueWrite_PAS(nextRank, nextBank,
//									*busPacket);
							if (PAS) {
								sendingWR = writeScheduling(nextRank, nextBank,
										*busPacket);
							}
							if (RP || nolimit || SPAIdeal) {
								sendingWR = issueWrite_RP(nextRank, nextBank,
										*busPacket);
							}
//							if (sendingWR) {
//							 PRINTN(
//							 " true currentClock "<<currentClockCycle<<" windowsize "<< windowsize[nextRank][nextBank] <<" writequeue.size "<<writequeue.size()<<" head "<<head<<" ");
//							 (*busPacket)->print();
//							 }
						}
					} /*else {						//WriteCancle function
					 if (readqueue.size() > 0 && writequeue.size() <= 0) { //prioritized the ReadRequest and Cancel the on-going write
					 writepriority[nextRank][nextBank] = false;
					 coutcanceledwrite[nextRank][nextBank]++; //record the Count of Canceled Write
					 writecancel[nextRank][nextBank] = true;
					 sendingRD = writeCancel(nextRank, nextBank,
					 *busPacket); //恢复现场？主要是bank 和rank的状态
					 } else {
					 //							sendingWR = issueWrite_PAS(nextRank, nextBank,
					 //									*busPacket);
					 if (PAS) {
					 sendingWR = writeScheduling(nextRank, nextBank,
					 *busPacket);
					 }
					 if (RP) {
					 sendingWR = issueWrite_RP(nextRank, nextBank,
					 *busPacket);
					 }
					 }
					 }*/
				} else {				//writepriority is false!
					if (!readqueue.empty()) {
//						writepriority[nextRank][nextBank] = false;
//						PRINT(
//								"writepriority false issue read currentClock is "<<currentClockCycle);
						sendingRD = issueRead(nextRank, nextBank, *busPacket);
					} /*else if (!writequeue.empty()
					 && (head < writequeue.size())) { //write queue 为非空，且里边有未处理的写
					 //						PRINT(
					 //								"writepriority false clock is "<<currentClockCycle<<" r "<<nextRank<<" b "<<nextBank<<" head "<<head<<" windowSize "<< windowSize<<" ");
					 sendingWR = issueWrite_PAS(nextRank, nextBank,
					 *busPacket);
					 if (PAS) {
					 sendingWR = writeScheduling(nextRank, nextBank,
					 *busPacket);
					 }
					 if (RP || nolimit) {
					 sendingWR = issueWrite_RP(nextRank, nextBank,
					 *busPacket);
					 }
					 }*/
				}
				if (sendingWR || sendingRD) {
//					PRINTN("issue return clock "<<currentClockCycle<<" ");
//					(*busPacket)->print();
					return true;
				}
			}
			readQueue.nextRankAndBank(nextRank, nextBank);
		} while (!(startingRank == nextRank && startingBank == nextBank));
		if ((!sendingWR) && (!sendingRD)) {
			//查看可以关的bank， issue PRE;
			bool sendingPRE = false;
			unsigned startingRank = nextRankPRE;
			unsigned startingBank = nextBankPRE;
			do // round robin over all ranks and banks
			{
				//check if bank is open
				if ((sendPRE[nextRankPRE][nextBankPRE] == true)
						&& ((bankStates[nextRankPRE][nextBankPRE].currentBankState
								== RowActive))) {
					if (currentClockCycle
							>= bankStates[nextRankPRE][nextBankPRE].nextPrecharge) {
						sendingPRE = true;
						*busPacket = new BusPacket(PRECHARGE, 0, 0, 0,
								nextRankPRE, nextBankPRE, NULL, dramsim_log);
//						PRINTN("issue return PRE clock "<<currentClockCycle<<" writepriority is "<<writepriority[nextRankPRE][nextBankPRE]<<" ");
//						(*busPacket)->print();
						sendPRE[nextRankPRE][nextBankPRE] = false;
//						if (writepriority[nextRankPRE][nextBankPRE]) {
						vector<BusPacket *> &readqueue =
								readQueue.getCommandQueue(nextRankPRE,
										nextBankPRE);
						for (unsigned i = 0; i < readqueue.size(); i++) {
							//if there is something going to that bank and row, then we don't want to send a PRE
							if (readqueue[i]->bank == nextBankPRE
									&& readqueue[i]->row
											== bankStates[nextRankPRE][nextBankPRE].openRowAddress) {
								if (readqueue[i]->busPacketType != ACTIVATE) {
									vector<BusPacket*>::iterator it =
											readqueue.begin() + i;
									BusPacket *bpacket = new BusPacket(ACTIVATE,
											readqueue[i]->physicalAddress,
											readqueue[i]->column,
											readqueue[i]->row,
											readqueue[i]->rank,
											readqueue[i]->bank, NULL,
											dramsim_log, readqueue[i]->RIP);
									readqueue.insert(it, bpacket);
								}
								break;
							}
						}
//						}
						break;
					}
				}
				readQueue.nextRankAndBank(nextRankPRE, nextBankPRE);
			} while (!(startingRank == nextRankPRE
					&& startingBank == nextBankPRE));
			//if no PREs could be sent, just return false
			if (!sendingPRE)
				return false;
			else {
				return true;
			}
		}
	}
}
void CancelWrite::updateavailableToken() {
	const int startCycle = 1000000;
	const int endCycle = 7000000;
	if (currentClockCycle < startCycle || currentClockCycle > endCycle) {
		return;
	}
	if (currentClockCycle % INTERVAL == 0) {
		//print
		//1.calculate the average value;
//		PRINT("updateavailableToken clock "<<currentClockCycle<<" INTERVAL ="<<INTERVAL);
		for (unsigned d = 0; d < NUM_DEVICES; d++) {
			averageToken[d] = totalToken[d] / (double) INTERVAL;
//			PRINTN("maxToken d["<<d<<"]: "<<maxToken[d]<<" avgToken["<<d<<"]: "<<averageToken[d]<<" ");
		}
//		PRINT("");
		maxtokenList.push_back(maxToken);
		avgtokenList.push_back(averageToken);
		maxToken = vector<double>(NUM_DEVICES, 0);
		totalToken = vector<double>(NUM_DEVICES, 0);
		averageToken = vector<double>(NUM_DEVICES, 0);
	} else {
		for (unsigned d = 0; d < NUM_DEVICES; d++) {
			totalToken[d] += tokenRank->tokenPool[d];
			maxToken[d] = max(maxToken[d], tokenRank->tokenPool[d]);
		}
	}
}
void CancelWrite::printavailableToken() {
	unsigned no = 0;
//	PRINT(" ---  maxToken list  ---  ");
//	for (vector<vector<double> >::iterator it = maxtokenList.begin();
//			it != maxtokenList.end(); it++, no++) {
//		PRINTN("maxToken No. "<<no<<" : ");
//		for (unsigned d = 0; d < NUM_DEVICES; d++) {
//			PRINTN("d["<<d<<"]: "<<(*it)[d]<<"; ");
//		}
//		PRINT("");
//	}
//	no=0;
	PRINT(" ---  avgToken list  ---  ");
	for (vector<vector<double> >::iterator it = avgtokenList.begin();
			it != avgtokenList.end(); it++, no++) {
		PRINTN("avgToken No. "<<no<<" : ");
		for (unsigned d = 0; d < NUM_DEVICES; d++) {
			PRINTN("d["<<d<<"]: "<<(*it)[d]<<"; ");
		}
		PRINT("");
	}
}
void CancelWrite::printavgTokenUtility() {
	PRINTN("avgTokenUtility " );// Writes # "<< tokenRank->utilitywritesNum);
	for (unsigned d = 0; d < NUM_DEVICES; d++) {
		double avgUtility = tokenRank->tokenUtilityWrites[d]
				/ (double) tokenRank->utilitywritesNum;
		PRINTN(" d["<<d<<"] "<<avgUtility<<" ");
	}
	PRINT("");
}
void CancelWrite::printburstCycles() {
//	PRINT(" ---  maxToken list  ---  ");
//	for (vector<vector<double> >::iterator it = maxtokenList.begin();
//			it != maxtokenList.end(); it++, no++) {
//		PRINTN("maxToken No. "<<no<<" : ");
//		for (unsigned d = 0; d < NUM_DEVICES; d++) {
//			PRINTN("d["<<d<<"]: "<<(*it)[d]<<"; ");
//		}
//		PRINT("");
//	}
//	no=0;
	vector < uint64_t > totalwriteburstCyclesPerBank = vector < uint64_t
			> (REQUESTID(NUM_RANKS,NUM_BANKS), 0);
	vector<double> avgwriteburstLatency = vector<double>(
			REQUESTID(NUM_RANKS,NUM_BANKS), 0);
	double totalwriteburstCycles = 0.0;
	double avgwriteburstCycles = 0.0;
	double totalavgwriteburstLatency = 0.0;
	uint64_t totalprioritWrites=0;
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
			if (priorityWrites[r][b] != 0) {
				for (size_t i = 0; i < writeburstCycle[REQUESTID(r,b)].size();
						i++) {
					totalwriteburstCyclesPerBank[REQUESTID(r,b)] +=
							writeburstCycle[REQUESTID(r,b)][i];
				}
				totalwriteburstCycles +=
						totalwriteburstCyclesPerBank[REQUESTID(r,b)];
				avgwriteburstLatency[REQUESTID(r,b)] =
						totalwriteburstCyclesPerBank[REQUESTID(r,b)]
								/ (double) priorityWrites[r][b];
				PRINT(
						"        Rank "<<r<<" Bank "<<b <<" totalburstCycles "<<totalwriteburstCyclesPerBank[REQUESTID(r,b)]<<" avgwriteburstLatency "<<avgwriteburstLatency[REQUESTID(r,b)]<<" priorityWrites "<<priorityWrites[r][b]);
				totalavgwriteburstLatency +=
						avgwriteburstLatency[REQUESTID(r,b)];
				totalprioritWrites += priorityWrites[r][b];
			}
		}
	}
	if (totalprioritWrites != 0) {
		avgwriteburstCycles = totalwriteburstCycles
				/ (double) totalprioritWrites;
		PRINT(
				"avgwriteBurstCycles "<<avgwriteburstCycles<<" totalavgwriteburstLatency "<<totalavgwriteburstLatency<<" totalprioritWrites "<<totalprioritWrites);
	}
}
bool CancelWrite::isEmpty(unsigned r) {
	if (writeQueue.size() == 0 && readQueue.isEmpty(r)) {
		return true;
	} else
		return false;
}

void CancelWrite::update() {
	readQueue.step();
	tokenRank->update();
	tokenRank->step();
	step();
	updateavailableToken();
}
/*void CancelWrite::print(){

 }*/
