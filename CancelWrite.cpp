/*
 * CancelWrite.cpp
 *
 *  Created on: 2014-3-13
 *      Author: libing
 */

#include "CancelWrite.h"
using namespace DRAMSim;

CancelWrite::CancelWrite(vector<vector<BankState> > &states,
		ostream &dramsim_log_) :
		dramsim_log(dramsim_log_), bankStates(states), writeQueue(states,
				dramsim_log_), readQueue(states, dramsim_log_), writeQueueDepth(
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
							buspacket->bank, buspacket->data, dramsim_log));
			readQueue.enqueue(
					new BusPacket(READ, buspacket->physicalAddress,
							buspacket->column, buspacket->row, buspacket->rank,
							buspacket->bank, buspacket->data, dramsim_log));

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
							buspacket->bank, buspacket->data, dramsim_log));
			writeQueue.enqueue(buspacket);
			return true;
		} else {
//			PRINT("No Room in Write Queue");
			delete buspacket;
			return false;
		}

	}
	return false;

}
bool CancelWrite::issueRequest(unsigned r, unsigned b, BusPacket *&busPacket,
		CommandQueue &requestQueue) {
	bool issuable = false;
	vector<BusPacket *> &queue = requestQueue.getCommandQueue(r, b);
	for (unsigned i = 0; i < queue.size(); i++) {
		BusPacket *request = queue[i];
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
bool CancelWrite::cancelwrite(BusPacket **busPacket) {
	vector < vector<bool> > pendingWrite = vector < vector<bool>
			> (NUM_RANKS, vector<bool>(NUM_BANKS, false));
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
			if (writequeue.size() > 16) {
				issueWrite = issueRequest(nextRank, nextBank, *busPacket,
						writeQueue);
			} else {
				if (bankStates[nextRank][nextBank].currentBankState == Idle) {
					if (readqueue.empty() && !writequeue.empty()) {
						issueWrite = issueRequest(nextRank, nextBank,
								*busPacket, writeQueue);
					}
				}
				if (!readqueue.empty()) {
					issueRead = issueRequest(nextRank, nextBank, *busPacket,
							readQueue);
				}
			}
			if (issueWrite || issueRead) {
				return true;
			}
			writeQueue.nextRankAndBank(nextRank, nextBank);
		} while (!(startingRank == nextRank && startingBank == nextBank));

		if ((!issueWrite) && (!issueRead)) {
			//issue the PRE to the bank
			unsigned startingRank = nextRankPRE;
			unsigned startingBank = nextBankPRE;
			do {
				bool found = false;
				vector<BusPacket *> &writequeue = writeQueue.getCommandQueue(
						nextRankPRE, nextBankPRE);
				vector<BusPacket *> &readqueue = readQueue.getCommandQueue(
						nextRankPRE, nextBankPRE);
				if (bankStates[nextRankPRE][nextBankPRE].currentBankState
						== RowActive) {
					if (writequeue.size() > 16) {
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
									issueRead = issueRequest(nextRankPRE, nextBankPRE, *busPacket,
																readQueue);
									found=true;
									break;
								}
							}
						}
					}
					else {
						for (unsigned i = 0; i < readqueue.size(); i++) {
							//if there is something going to that bank and row, then we don't want to send a PRE
							if (readqueue[i]->bank == nextBankPRE
									&& readqueue[i]->row
											== bankStates[nextRankPRE][nextBankPRE].openRowAddress) {
								found = true;
								break;
							}
						}
						if (!found) {
							for (unsigned i = 0; i < writequeue.size(); i++) {
								//if there is something going to that bank and row, then we don't want to send a PRE
								if (writequeue[i]->bank == nextBankPRE
										&& writequeue[i]->row
												== bankStates[nextRankPRE][nextBankPRE].openRowAddress) {
									issueWrite = issueRequest(nextRankPRE,
											nextBankPRE, *busPacket,
											writeQueue);
									found = true;
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
							break;
						}
					}
				}
				writeQueue.nextRankAndBank(nextRankPRE, nextBankPRE);
			} while (!(startingRank == nextRankPRE
					&& startingBank == nextBankPRE));
			if (!sendingPRE && (!issueWrite || !issueRead)) {
				return false;
			} else {
				return true;
			}
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
