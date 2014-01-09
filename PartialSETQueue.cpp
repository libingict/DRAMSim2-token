//
//Class file for partialSET queue object
//

#include "PartialSETQueue.h"
#include "MemoryController.h"
#include <assert.h>

using namespace DRAMSim;

PartialSETQueue::PartialSETQueue(vector<vector<BankState> > &states,
		ostream &dramsim_log_) :
		dramsim_log(dramsim_log_), bankStates(states) {
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

	//create queue based on the structure we want
	Entry1D actualQueue;
	Entry2D perBankQueue = Entry2D();
	PSqueues = Entry3D();
	for (size_t rank = 0; rank < NUM_RANKS; rank++) {
		//this loop will run only once for per-rank and NUM_BANKS times for per-rank-per-bank
		for (size_t bank = 0; bank < numBankqueues; bank++) {
			actualQueue = Entry1D();
			perBankQueue.push_back(actualQueue);
			isFull[rank][bank]=false;
		}
		PSqueues.push_back(perBankQueue);
	}
}
PartialSETQueue::~PartialSETQueue() {
	//ERROR("PartialSET QUEUE destructor");
	size_t bankMax = NUM_RANKS;
	if (queuingStructure == PerRank) {
		bankMax = 1;
	}
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < bankMax; b++) {
			for (size_t i = 0; i < PSqueues[r][b].size(); i++) {
				delete (PSqueues[r][b][i]);
			}
			PSqueues[r][b].clear();
			delete isFull[r][b];
		}
	}
}
//double PartialSETQueue::RETENTION_TIME=1.0e9;
void PartialSETQueue::enqueue(BusPacket *newBusPacket) {
	unsigned rank = newBusPacket->rank;
	unsigned bank = newBusPacket->bank;
	entry* newEntry;
	newEntry->busPacket = newBusPacket;
	newEntry->elapsedTime = 0;
	if (queuingStructure == PerRank) {
		for (unsigned i = 0; i < PSqueues[rank][0].size(); i++) {
			if (PSqueues[rank][0][i].busPacket->physicsAddress
					== newBusPacket->physicalAddress) {
				PSqueues[rank][0].erase(PSqueues[rank][0].begin() + i);
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
		for (unsigned i = 0; i < PSqueues[rank][bank].size(); i++) {
			if (PSqueues[rank][bank][i].busPacket->physicsAddress
					== newBusPacket->physicalAddress) {
				PSqueues[rank][bank].erase(PSqueues[rank][bank].begin() + i);
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
	}
		else
	 {
	 ERROR("== Error - Unknown queuing structure");
	 exit(0);
	 }
}
bool PartialSETQueue::idlePredictisLong(unsigned bank){
	return true;
}
//two conditions.1, the queue is full; 2, the elaspedTime of oldest entry reaches the threshold;
//then issue a SET from PSqueue，粒度针对每个bank, insert the issued cmd to cmdqueue.
void PartialSETQueue::evict(unsigned rank, unsigned bank) {

		if (bankStates[rank][bank].currentBankState == Idle) {
			vector<BusPacket *> &cmdqueue = getCommandQueue(rank, bank);
			if (cmdqueue.size() == 0) {
				if (idlePredictisLong(bank) || isFull[rank][bank]) {
					unsigned col = PSqueues[rank][bank][0].busPacket->column;
					unsigned row = PSqueues[rank][bank][0].busPacket->row;
					uint64_t phyAddr =
							PSqueues[rank][bank][0].busPacket->physicsAddress;
					BusPacket *SETcommand = new BusPacket(FullSET, phyAddr, col,
							row, bank, rank, 0, dramsim_log);
					//todo:insert the busPacket to cmdqueue and issue the busPacket to memory
					BusPacket *ACTcommand = new BusPacket(ACTIVATE, phyAddr,
							col, row, bank, rank, 0, dramsim_log);
					cmdqueue.enqueue(ACTcommand);
					cmdqueue.enqueue(SETcommand);

					PSqueues[rank][bank].erase(PSqueues[rank][bank].begin() + 0); //delete the entry with the largest elapseTime
				}
			}
		}

}
//the same address entry is SET from command queue, the relative entry in PSQueue should be deleted
/*void release(BusPacket *busPacket) {
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
}*/
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
	unsigned rank=rankID;
	unsigned bank;
	if (queuingStructure == PerRankPerBank) {
		bank=bankID;
	} else if (queuingStructure == PerRank) {
		bank=0;
	} else {
		ERROR("Unknown queue structure");
		abort();
	}
	enrty* newEntry=PSqueues[rank][bank][index];
	newEntry->elapsedTime=0;
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

/*bool IsFull(unsigned* rank, unsigned* bank) {
	size_t bankMax = NUM_RANKS;
	if (queuingStructure == PerRank) {
		bankMax = 1;
	}
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < bankMax; b++) {
			for (size_t i = 0; i < PSqueues[r][b].size(); i++) {
				delete (PSqueues[r][b][i]);
			}
			PSqueues[r][b].clear();
		}
	}
	if (queuingStructure == PerRank) {
		if (PSqueues[rank][0].size() >= PARTIAL_QUEUE_DEPTH) {
			ERROR("== Error - Enqueued more than allowed in command queue");
			ERROR(
					"						Need to call .hasRoomFor(int numberToEnqueue, unsigned rank, unsigned bank) first");
			//Evict one entry from PSqueue.
			return false;
		}
	} else if (queuingStructure == PerRankPerBank) {
		if (PSqueues[rank][bank].size() >= PARTIAL_QUEUE_DEPTH) {
			ERROR("== Error - Enqueued more than allowed in command queue");
			ERROR(
					"						Need to call .hasRoomFor(int numberToEnqueue, unsigned rank, unsigned bank) first");
			//Evict one entry from PSqueue.
			return false;
		}
	}

}*/

void PartialSETQueue::update()
{
	//do nothing since pop() is effectively update(),
	//needed for SimulatorObject
	//TODO: make CommandQueue not a SimulatorObject
}


