//
//Class file for partialSET queue object
//

#include "PartialSETQueue.h"
#include "MemoryController.h"
#include "Entry.h"
#include "BusPacket.h"
#include <assert.h>

using namespace DRAMSim;

PartialSETQueue::PartialSETQueue(vector<vector<BankState> > &states,
		CommandQueue &cmdqueue, ostream &dramsim_log_) :
		bankStates(states), cmdQueue(cmdqueue), dramsim_log(dramsim_log_) {

	//use numBankQueus below to create queue structure
	size_t numBankqueues;
	if (queuingStructure == PerRank) {
		numBankqueues = 1;
	} else if (queuingStructure == PerRankPerBank) {
		numBankqueues = NUM_BANKS;
	} else {
		ERROR("== Error - Unknown queuing structure");
		exit(0);
	}
	currentClockCycle = 0;
	//CommandQueue::CommandQueue& cmdQueue=cmdqueue;
	isFull = vector < vector<bool> > (NUM_RANKS, vector<bool>(NUM_BANKS, false));
	idle =  vector < vector<bool> > (NUM_RANKS, vector<bool>(NUM_BANKS, false));
	begin = vector < vector<unsigned> >(NUM_RANKS, vector<unsigned>(NUM_BANKS, 0));
	duration = vector < vector<unsigned> >(NUM_RANKS, vector<unsigned>(NUM_BANKS, 0));
	//create queue based on the structure we want
	Entry1D actualQueue;
	Entry2D perBankQueue = Entry2D();
	PSqueues = Entry3D();
	BankD idleBank;
	RankD idleRank = RankD();
	IdleTable = Table();
	for (size_t rank = 0; rank < NUM_RANKS; rank++) {
		//this loop will run only once for per-rank and NUM_BANKS times for per-rank-per-bank
		for (size_t bank = 0; bank < numBankqueues; bank++) {
			actualQueue = Entry1D();
			perBankQueue.push_back(actualQueue);
			idleBank = BankD();
			idleRank.push_back(idleBank);
		}
		for (size_t bank = 0; bank < NUM_BANKS; bank++) {

			idleBank = BankD();
			idleRank.push_back(idleBank);
		}
		PSqueues.push_back(perBankQueue);
		IdleTable.push_back(idleRank);
	}
}
PartialSETQueue::~PartialSETQueue() {
	//ERROR("PartialSET QUEUE destructor");
	size_t bankMax = NUM_RANKS;
	Entry1D::iterator iter;
	if (queuingStructure == PerRank) {
		bankMax = 1;
	}
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < bankMax; b++) {
			for (size_t i = 0; i < PSqueues[r][b].size(); ++i) {
				delete (PSqueues[r][b][i]);
			}
			PSqueues[r][b].clear();
			IdleTable[r][b].clear();
		}
	}
}
//double PartialSETQueue::RETENTION_TIME=1.0e9;
bool PartialSETQueue::enqueue(BusPacket *bspacket) {
	unsigned rank = bspacket->rank;
	unsigned bank = bspacket->bank;
	if (!(bspacket->busPacketType == WRITE || bspacket->busPacketType == WRITE_P)) {
		ERROR("== Error - Enqueued busPacket should be write");
		return false;
	}
	Entry* newEntry;
	newEntry->busPacket = bspacket;
	newEntry->busPacket->busPacketType = PartialSET;
	newEntry->elapsedTime = 0;
	Entry1D::iterator iter;
	if (queuingStructure == PerRank) {
		for (iter = PSqueues[rank][0].begin(); iter != PSqueues[rank][0].end();
				++iter) {
			Entry *tmp = *iter;
			if (tmp->busPacket == bspacket) {
				PSqueues[rank][0].erase(iter);
				break;
			}
		}
		PSqueues[rank][0].push_back(newEntry);
		if (PSqueues[rank][0].size() >= PARTIAL_QUEUE_DEPTH) {
			ERROR("== Error - Enqueued more than allowed in command queue");
			ERROR(
					"						Need to call .hasRoomFor(int numberToEnqueue, unsigned rank, unsigned bank) first");
			//set the queue state full
			isFull[rank][0] = true;
			return false;
		}
		return true;
	} else if (queuingStructure == PerRankPerBank) {
		for (iter = PSqueues[rank][bank].begin();
				iter != PSqueues[rank][bank].end(); ++iter) {
			Entry *tmp = *iter;
			if (tmp->busPacket == bspacket) {
				PSqueues[rank][bank].erase(iter);
				break;
			}
		}
		PSqueues[rank][bank].push_back(newEntry);
		if (PSqueues[rank][bank].size() >= PARTIAL_QUEUE_DEPTH) {
			ERROR("== Error - Enqueued more than allowed in command queue");
			ERROR(
					"						Need to call .hasRoomFor(int numberToEnqueue, unsigned rank, unsigned bank) first");
			//set the queue state full
			isFull[rank][bank] = true;
		}
	} else {
		ERROR("== Error - Unknown queuing structure");
		exit(0);
	}
	return true;
}
/*bool PartialSETQueue::idlePredictisLong(unsigned bank) {
 return true;
 }*/
/*
 //two conditions.1, the queue is full; 2, the elaspedTime of oldest entry reaches the threshold;
 //then issue a SET from PSqueue bank, insert the issued cmd to cmdqueue.
 void PartialSETQueue::evict(unsigned rank, unsigned bank) {

 if (bankStates[rank][bank].currentBankState == Idle) {
 vector<BusPacket *> &cmdqueue = CommandQueue::getCommandQueue(rank,
 bank);
 if (cmdqueue.size() == 0) {
 if (idlePredictisLong(bank) || isFull[rank][bank]) {
 unsigned col = PSqueues[rank][bank][0].busPacket->column;
 unsigned row = PSqueues[rank][bank][0].busPacket->row;
 uint64_t phyAddr =
 PSqueues[rank][bank][0].busPacket->physicsAddress;
 BusPacket *SETcommand = new BusPacket(FullSET, phyAddr, col,
 row, bank, rank, 0, dramsim_log);
 //todo:insert the busPacket to cmdqueue and issue the busPacket to memory
 BusPacket *ACTcommand = new BusPacket(ACTIVATE, phyAddr, col,
 row, bank, rank, 0, dramsim_log);
 cmdqueue.push_back(ACTcommand);
 cmdqueue.push_back(SETcommand);

 PSqueues[rank][bank].erase(PSqueues[rank][bank].begin() + 0); //delete the entry with the largest elapseTime
 }
 }
 }

 }
 //the same address entry is SET from command queue, the relative entry in PSQueue should be deleted
 void release(BusPacket *busPacket) {
 unsigned rank = busPacket->rank;
 unsigned bank = busPacket->bank;
 if (queuingStructure == PerRank) {
 if (!PSqueues[rank][0].empty()) {
 for (unsigned i = 0; i < PSqueues[rank][0].size(); i++) {
 if (PSqueues[rank][0][i].busPacket->physicsAddress
 == newBusPacket->physicalAddress)
 break;
 }
 PSqueues[rank][0].erase(PSqueues[rank][0].begin() + i);
 }
 } else if (queuingStructure == PerRankPerBank) {
 if (!PSqueues[rank][bank].empty()) {
 for (unsigned i = 0; i < PSqueues[rank][bank].size(); i++) {
 if (PSqueues[rank][bank][i].busPacket->physicsAddress
 == newBusPacket->physicalAddress)
 break;
 }
 PSqueues[rank][bank].erase(PSqueues[rank][bank].begin() + i);
 }
 }
 }
 vector<entry *> &PartialSETQueue::getPSQueue(unsigned rank, unsigned bank) {
 if (queuingStructure == PerRankPerBank) {
 return PSqueues[rank][bank];
 } else if (queuingStructure == PerRank) {
 return PSqueues[rank][0];
 } else {
 ERROR("Unknown queue structure");
 abort();
 }
 }
 void PartialSETQueue::emergePartialSET(unsigned rankID, unsigned bankID,
 unsigned index) {
 unsigned rank = rankID;
 unsigned bank;
 if (queuingStructure == PerRankPerBank) {
 bank = bankID;
 } else if (queuingStructure == PerRank) {
 bank = 0;
 } else {
 ERROR("Unknown queue structure");
 abort();
 }
 enrty* newEntry = PSqueues[rank][bank][index];
 newEntry->elapsedTime = 0;
 unsigned col = PSqueues[rank][bank][index].busPacket->column;
 unsigned row = PSqueues[rank][bank][index].busPacket->row;
 uint64_t phyAddr = PSqueues[rank][bank][index].busPacket->physicsAddress;
 BusPacket *Writecommand = new BusPacket(PartialSET, phyAddr, col, row, bank,
 r, 0, dramsim_log);
 //todo:insert the busPacket to cmdqueue and issue the busPacket to memory
 BusPacket *ACTcommand = new BusPacket(ACTIVATE, phyAddr, col, row, bank, r,
 0, dramsim_log);
 cmdqueue.enqueue(ACTcommand);
 cmdqueue.enqueue(Writecommand);

 PSqueues[rank][bank].erase(PSqueues[rank][bank].begin() + index);
 PSqueues[rank][bank].push_back(newEntry);

 }
 */
void PartialSETQueue::getIdleInterval() {
	unsigned r = NUM_RANKS;
	unsigned b = NUM_BANKS;
	for (unsigned i = 0; i < r; ++i) {
		for (unsigned j = 0; j < b; ++j) {
			if (bankStates[i][j].currentBankState == Idle || PowerDown) {
				if (idle[i][j] = false) {
					idle[i][j] = true;
					begin[i][j] = currentClockCycle;
				}
			} else {
				if (idle[i][j] = true) {
					duration[i][j] = begin[i][j] - currentClockCycle;

					if ((duration[i][j] >= WRITE_AUTOPRE_DELAY
							&& rowBufferPolicy == ClosePage)
							|| (duration[i][j] >= WRITE_TO_PRE_DELAY
									&& rowBufferPolicy == OpenPage)) {
						IdleTable[i][j].push_back(true);
					} else
						IdleTable[i][j].push_back(false);
				}
			}
		}
	}
}

void PartialSETQueue::update() {
	//do nothing since pop() is effectively update(),
	//needed for SimulatorObject
	//TODO: make CommandQueue not a SimulatorObject
}

