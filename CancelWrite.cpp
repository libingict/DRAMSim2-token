/*
 * CancelWrite.cpp
 *
 *  Created on: 2014-3-13
 *      Author: libing
 */

#include "CancelWrite.h"
using namespace DRAMSim;

CancelWrite::CancelWrite(vector<vector<BankState> > &states,
		ostream &dramsim_log_, vector<Rank *> *&ranks_) :
		dramsim_log(dramsim_log_), bankStates(states), writeQueue(states,
				dramsim_log_), readQueue(states, dramsim_log_), ranks(ranks_), writeQueueDepth(
				CMD_QUEUE_DEPTH), nextRank(0), nextBank(0), nextRankPRE(0), nextBankPRE(
				0) {
	currentClockCycle = 0;
	writecancel = vector < vector<bool>
			> (NUM_RANKS, vector<bool>(NUM_BANKS, false));

	writestarttime = vector < vector<unsigned>
			> (NUM_RANKS, vector<unsigned>(NUM_BANKS, 0));
	readrequest = vector < vector<unsigned>
			> (NUM_RANKS, vector<unsigned>(NUM_BANKS, 0));
	writerequest = vector < vector<unsigned>
			> (NUM_RANKS, vector<unsigned>(NUM_BANKS, 0));
	pendingWR = vector < vector<BusPacket *>
			> (NUM_RANKS, vector<BusPacket *>(NUM_BANKS, NULL));
	writepriority = vector < vector<bool>
			> (NUM_RANKS, vector<bool>(NUM_BANKS, false));

}
CancelWrite::~CancelWrite() {

}
bool CancelWrite::addRequest(Transaction *transaction, BusPacket *buspacket,
		bool &found) {
	if (transaction->transactionType == DATA_READ) {
		vector<BusPacket*> &queue = writeQueue.getCommandQueue(buspacket->rank,
				buspacket->bank);
		for (unsigned i = 0; i < queue.size(); i++) {
			BusPacket *packet = queue[i];
			if (packet->physicalAddress == transaction->address) { //if write queue has the same address, then the read request can be returned
				transaction->transactionType = RETURN_DATA; //MC got the returned data from write queue.
				if (queue[i]->busPacketType == ACTIVATE && queue.size() > 1
						&& i < queue.size() - 1
						&& queue[i + 1]->physicalAddress
								== packet->physicalAddress) {
					transaction->data = queue[i + 1]->data;
				} else {
					transaction->data = packet->data;
				}
				found = true;
				break;
			}
		}
		if (found) {
			readrequest[buspacket->rank][buspacket->bank]++;
			return true;
		}
		if (readQueue.hasRoomFor(2, buspacket->rank, buspacket->bank)) {
			readQueue.enqueue(
					new BusPacket(ACTIVATE, buspacket->physicalAddress,
							buspacket->column, buspacket->row, buspacket->rank,
							buspacket->bank, buspacket->data, dramsim_log,
							buspacket->RIP));
			readQueue.enqueue(
					new BusPacket(READ, buspacket->physicalAddress,
							buspacket->column, buspacket->row, buspacket->rank,
							buspacket->bank, buspacket->data, dramsim_log,
							buspacket->RIP));

			return true;
		} else {
			delete buspacket;
			return false;
		}
	} else if (transaction->transactionType == DATA_WRITE) {
		vector<BusPacket*> &queue = writeQueue.getCommandQueue(buspacket->rank,
				buspacket->bank);
		for (unsigned i = 0; i < queue.size(); i++) {
			BusPacket *packet = queue[i];
			if (packet->physicalAddress == transaction->address) { //if the write queue has the same address, then the old write can be evicted, updated the newest data.
				found = true;
				if (i != 0 && queue[i - 1]->busPacketType == ACTIVATE
						&& queue[i - 1]->physicalAddress
								== packet->physicalAddress) { // the write request is paired
					delete (queue[i - 1]);
					// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
					queue.erase(queue.begin() + i - 1, queue.begin() + i + 1); //[queue.begin() + i - 1, queue.begin() + i + 1)
				} else if (queue[i]->busPacketType == ACTIVATE
						&& queue.size() > 1 && i < queue.size() - 1
						&& queue[i + 1]->physicalAddress
								== packet->physicalAddress) { // the write request is paired
					delete (queue[i + 1]);
					// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
					queue.erase(queue.begin() + i, queue.begin() + i + 2);
				} else {
					//the activated has popped.
					queue.erase(queue.begin() + i);
				}
				break;
			}
		}
		//every bank pending one request.
		if (writeQueue.hasRoomFor(2, buspacket->rank, buspacket->bank)) {
			writeQueue.enqueue(
					new BusPacket(ACTIVATE, buspacket->physicalAddress,
							buspacket->column, buspacket->row, buspacket->rank,
							buspacket->bank, buspacket->data, dramsim_log,
							buspacket->RIP));
			writeQueue.enqueue(buspacket);
			return true;
		} else {
//			PRINT("No Room in Write Queue");
			delete buspacket;
			return false;
		}
	}
}
bool CancelWrite::issueRequest(unsigned r, unsigned b, BusPacket *&busPacket,
		CommandQueue &requestQueue) {
	bool issuable = false;
	vector<BusPacket *> &queue = requestQueue.getCommandQueue(r, b);
	for (unsigned i = 0; i < queue.size(); i++) {
		BusPacket *request = queue[i];
		if (requestQueue.isIssuable(request)) {
			busPacket = request;
			/*
			 if the bus packet before is an activate, that is the act that was
			 paired with the column access we are removing, so we have to remove
			 that activate as well (check i>0 because if i==0 then there's nothing before it)
			 */
			if (i > 0 && queue[i - 1]->busPacketType == ACTIVATE) {
				// i is being returned, but i-1 is being thrown away, so must delete it here
				delete (queue[i - 1]);
				// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
				queue.erase(queue.begin() + i - 1, queue.begin() + i + 1);
			} else // there's no activate before this packet
			{
				//or just remove the one bus packet
				queue.erase(queue.begin() + i);
			}
			issuable = true;
			break;
		} else {
			bool allpending = false;
			switch (request->busPacketType) {
			case ACTIVATE:
				if (currentClockCycle
						< min(bankStates[r][b].nextActivate,
								min(bankStates[r][b].nextWrite,
										bankStates[r][b].nextRead)))
					allpending = true;
				break;
			case READ:
				if (currentClockCycle
						< min(bankStates[r][b].nextActivate,
								bankStates[r][b].nextRead))
					allpending = true;
				break;
			case WRITE:
				if (currentClockCycle
						< min(bankStates[r][b].nextActivate,
								bankStates[r][b].nextWrite))
					allpending = true;
				break;
			default:
				break;
			}
			if (allpending)
				break;
		}
	}
	return issuable;
}
void CancelWrite::issueWC(unsigned r, unsigned b) { //if there is waiting read request, issue the request.
	bool issueRead = false;
	vector<BusPacket *> &rdqueue = readQueue.getCommandQueue(r, b);
	if (bankStates[r][b].currentBankState == RowActive
			&& bankStates[r][b].lastCommand == WRITE) {
		//1.push_back the paired of Write and ACTIVATE to the writequeue.
		for (unsigned i = 0; i < rdqueue.size(); i++) {
			BusPacket *packet = rdqueue[i];
			if (bankStates[r][b].openRowAddress == packet->row) {
				//modify all timing for all Bank
				/*						bankStates[r][b].nextRead = currentClockCycle;
				 //2.modify the cycle of Rank;
				 //modify all timing for all Bank in Rank
				 (*ranks)[r]->bankStates[b].nextRead = currentClockCycle;*/
				issueRead = readQueue.isIssuable(packet);
				if (!issueRead
						&& currentClockCycle < bankStates[r][b].nextRead) { //issue Write Request
					unsigned oldnextRead = bankStates[r][b].nextRead;
					unsigned delta = oldnextRead - currentClockCycle;
					if (pendingWR[r][b] != NULL
							&& delta > (0.75 * WRITE_TO_READ_DELAY_R)) {
						//modify all timing for all Bank
						bankStates[r][b].nextPrecharge -= delta;
						bankStates[r][b].nextWrite -= delta;
						bankStates[r][b].nextRead -= delta;
						//2.modify the cycle of Rank;
						(*ranks)[r]->bankStates[b].nextPrecharge -= delta;
						(*ranks)[r]->bankStates[b].nextRead -= delta;
						(*ranks)[r]->bankStates[b].nextWrite -= delta;
					}
					return;
				}
			}
		}
	}
	//if idle nothing to do , no write block the read
}
bool CancelWrite::cancelwrite(BusPacket **busPacket) {
	if (rowBufferPolicy == OpenPage && queuingStructure == PerRankPerBank) {
		bool issueWrite = false;
		bool issueRead = false;
		bool sendingPRE = false;
		unsigned startingRank = nextRank;
		unsigned startingBank = nextBank;
		do {
			vector<BusPacket *> &writequeue = writeQueue.getCommandQueue(
					nextRank, nextBank);
			vector<BusPacket *> &readqueue = readQueue.getCommandQueue(nextRank,
					nextBank);
			if (!writepriority[nextRank][nextBank]) {	//then read priority
				if (writequeue.size() > 16) {
					writepriority[nextRank][nextBank] = true;
				}
			}
			if (writepriority[nextRank][nextBank]) {
				/*				if (writequeue.size() <= 8 && !readqueue.empty()) {
				 writepriority[nextRank][nextBank] = false;
				 }*/
				if (!readqueue.empty() && writequeue.size() < 30) {
					writepriority[nextRank][nextBank] = false;
					issueWC(nextRank, nextBank);
					issueRead = issueRequest(nextRank, nextBank, *busPacket,
							readQueue);
					if (issueRead) {
						if (bankStates[nextRank][nextBank].lastCommand == WRITE
								&& pendingWR[nextRank][nextBank]->busPacketType
										== WRITE) {
							//only these commands have an implicit state change
							vector<BusPacket*>::iterator it =
									writequeue.begin();
							writequeue.insert(it,
									new BusPacket(WRITE,
											pendingWR[nextRank][nextBank]->physicalAddress,
											pendingWR[nextRank][nextBank]->column,
											pendingWR[nextRank][nextBank]->row,
											pendingWR[nextRank][nextBank]->rank,
											pendingWR[nextRank][nextBank]->bank,
											pendingWR[nextRank][nextBank]->data,
											dramsim_log,
											pendingWR[nextRank][nextBank]->RIP));
							writequeue.insert(it,
									new BusPacket(ACTIVATE,
											pendingWR[nextRank][nextBank]->physicalAddress,
											pendingWR[nextRank][nextBank]->column,
											pendingWR[nextRank][nextBank]->row,
											pendingWR[nextRank][nextBank]->rank,
											pendingWR[nextRank][nextBank]->bank,
											pendingWR[nextRank][nextBank]->data,
											dramsim_log,
											pendingWR[nextRank][nextBank]->RIP));
						}
						pendingWR[nextRank][nextBank] = NULL;
						return true;
					}
				}
				bool found = false;
				for (unsigned i = 0; i < writequeue.size(); i++) {
					//if there is something going to that bank and row, then we don't want to send a PRE
					issueWrite = writeQueue.isIssuable(writequeue[i]);
					if (issueWrite) {		//issue Write Request
						*busPacket = writequeue[i];
						if (writequeue[i]->busPacketType == WRITE) {
							pendingWR[nextRank][nextBank] = new BusPacket(WRITE,
									writequeue[i]->physicalAddress,
									writequeue[i]->column, writequeue[i]->row,
									writequeue[i]->rank, writequeue[i]->bank,
									writequeue[i]->data, dramsim_log,
									writequeue[i]->RIP);
						}
						if (i > 0
								&& writequeue[i - 1]->busPacketType
										== ACTIVATE) {
							// i is being returned, but i-1 is being thrown away, so must delete it here
							delete (writequeue[i - 1]);
							// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
							writequeue.erase(writequeue.begin() + i - 1,
									writequeue.begin() + i + 1);
						} else { // there's no activate before this packet
							//or just remove the one bus packet
							writequeue.erase(writequeue.begin() + i);
						}

						return true;
					}
					//else 1. bank is open but not same row, need precharge;
					//2. bank is idle, timing not allow, need wait; check the next request
					//3. bank is open, same row, timing not allow,need wait;check the next request
					if (bankStates[nextRank][nextBank].currentBankState
							== RowActive && writequeue[i]->bank == nextBank
							&& writequeue[i]->row
									== bankStates[nextRank][nextBank].openRowAddress) { //check the 1st condition
						found = true;  //once find one, no need to precharge;
					}
				}
				if (!found) { //every entry in write queue is not equal with the open row, then close
					if (currentClockCycle
							>= bankStates[nextRank][nextBank].nextPrecharge
							&& bankStates[nextRank][nextBank].currentBankState
									== RowActive) {
						*busPacket = new BusPacket(PRECHARGE, 0, 0, 0, nextRank,
								nextBank, 0, dramsim_log);
						for (unsigned i = 0; i < readqueue.size(); i++) { //assure no dangling READ
							//if there is something going to that bank and row, then we don't want to send a PRE
							if (readqueue[i]->bank == nextBank
									&& readqueue[i]->row
											== bankStates[nextRank][nextBank].openRowAddress) {
								if (readqueue[i]->busPacketType != ACTIVATE) {
									vector<BusPacket*>::iterator it =
											readqueue.begin() + i;
									BusPacket *bpacket = new BusPacket(ACTIVATE,
											readqueue[i]->physicalAddress,
											readqueue[i]->column,
											readqueue[i]->row,
											readqueue[i]->rank,
											readqueue[i]->bank,
											readqueue[i]->data, dramsim_log,
											readqueue[i]->RIP);
									readqueue.insert(it, bpacket);
								}
								break;
							}
						}
						pendingWR[nextRank][nextBank] = NULL;
						return true;
					}
				}
			}
			if (bankStates[nextRank][nextBank].currentBankState == Idle) {
				if (readqueue.empty() && !writequeue.empty()) {
					issueWrite = issueRequest(nextRank, nextBank, *busPacket,
							writeQueue);
				}
			}
			if (!readqueue.empty()) {
				issueRead = issueRequest(nextRank, nextBank, *busPacket,
						readQueue);
			}
			if (issueWrite || issueRead) {
				if ((*busPacket)->busPacketType == WRITE) {
					pendingWR[nextRank][nextBank] = new BusPacket(WRITE,
							(*busPacket)->physicalAddress, (*busPacket)->column,
							(*busPacket)->row, (*busPacket)->rank,
							(*busPacket)->bank, (*busPacket)->data, dramsim_log,
							(*busPacket)->RIP);
				}
				return true;
			}
			writeQueue.nextRankAndBank(nextRank, nextBank);
		} while (!(startingRank == nextRank && startingBank == nextBank));

		if ((!issueWrite) && (!issueRead)) {
			//issue the PRE to the bank
			unsigned startingRank = nextRankPRE;
			unsigned startingBank = nextBankPRE;
			bool found;
			do {
				found = false;
				vector<BusPacket *> &writequeue = writeQueue.getCommandQueue(
						nextRankPRE, nextBankPRE);
				vector<BusPacket *> &readqueue = readQueue.getCommandQueue(
						nextRankPRE, nextBankPRE);
				if (bankStates[nextRankPRE][nextBankPRE].currentBankState
						== RowActive) {
					if (writepriority[nextRankPRE][nextBankPRE]) {
						for (unsigned i = 0; i < writequeue.size(); i++) {
							//if there is something going to that bank and row, then we don't want to send a PRE
							if (writequeue[i]->bank == nextBankPRE
									&& writequeue[i]->row
											== bankStates[nextRankPRE][nextBankPRE].openRowAddress) {
								found = true;
								break;
							}
						}
						if (!found) {
							for (unsigned i = 0; i < readqueue.size(); i++) {
								//if there is something going to that bank and row, then we don't want to send a PRE
								if (readqueue[i]->bank == nextBankPRE
										&& readqueue[i]->row
												== bankStates[nextRankPRE][nextBankPRE].openRowAddress) {
									if (readqueue[i]->busPacketType
											!= ACTIVATE) {
										vector<BusPacket*>::iterator it =
												readqueue.begin() + i;
										BusPacket *bpacket = new BusPacket(
												ACTIVATE,
												readqueue[i]->physicalAddress,
												readqueue[i]->column,
												readqueue[i]->row,
												readqueue[i]->rank,
												readqueue[i]->bank,
												readqueue[i]->data, dramsim_log,
												readqueue[i]->RIP);
										readqueue.insert(it, bpacket);
									}
									break;
								}
							}
						}
					} else {
						for (unsigned i = 0; i < readqueue.size(); i++) {
							//if there is something going to that bank and row, then we don't want to send a PRE
							if (readqueue[i]->bank == nextBankPRE
									&& readqueue[i]->row
											== bankStates[nextRankPRE][nextBankPRE].openRowAddress) {
								found = true;
								return false;
							}
						}
						if (!found) {
							for (unsigned i = 0; i < writequeue.size(); i++) {
								//if there is something going to that bank and row, then we don't want to send a PRE
								if (writequeue[i]->bank == nextBankPRE
										&& writequeue[i]->row
												== bankStates[nextRankPRE][nextBankPRE].openRowAddress) {
									//change to add a ACT to this write and PRE This bank
									if (readqueue.empty()) {
										found = true;
										if (issueRequest(nextRankPRE,
												nextBankPRE, *busPacket,
												writeQueue)) {
											pendingWR[nextRankPRE][nextBankPRE] =
													NULL;
											return true;
										}
									} else if (writequeue[i]->busPacketType
											!= ACTIVATE) {
										vector<BusPacket*>::iterator it =
												writequeue.begin() + i;
										BusPacket *bpacket = new BusPacket(
												ACTIVATE,
												writequeue[i]->physicalAddress,
												writequeue[i]->column,
												writequeue[i]->row,
												writequeue[i]->rank,
												writequeue[i]->bank,
												writequeue[i]->data,
												dramsim_log,
												writequeue[i]->RIP);
										writequeue.insert(it, bpacket);
										break;
									}
									break;
								}
							}
						}
					}
					if (!found) {
						if (currentClockCycle
								>= bankStates[nextRankPRE][nextBankPRE].nextPrecharge) {
							sendingPRE = true;
							*busPacket = new BusPacket(PRECHARGE, 0, 0, 0,
									nextRankPRE, nextBankPRE, 0, dramsim_log);
							pendingWR[nextRank][nextBank] = NULL;
							return true;
						}
					}
				}
				writeQueue.nextRankAndBank(nextRankPRE, nextBankPRE);
			} while (!(startingRank == nextRankPRE
					&& startingBank == nextBankPRE));
			if (!sendingPRE)
				return false;
		}
	}
	return false;
}

bool CancelWrite::isEmpty(unsigned r) {
	if (writeQueue.isEmpty(r) && readQueue.isEmpty(r)) {
		return true;
	} else
		return false;
}
void CancelWrite::update() {
	writeQueue.step();
	readQueue.step();
	step();
}
/*void CancelWrite::print(){

 }*/
