
//
//Class file for partialSET queue object
//

#include "PartialSETQueue.h"
#include "MemoryController.h"
#include <assert.h>

using namespace DRAMSim;

PartialSETQueue :: PartialSETQueue(vector< vector<BankState> > &states, ostream &dramsim_log_) :
		dramsim_log(dramsim_log_),
		bankStates(states)
{
	//use numBankQueus below to create queue structure
	size_t numBankPSqueues;
	if (queuingStructure==PerRank)
	{
		numBankPSqueues = 1;
	}
	else if (queuingStructure==PerRankPerBank)
	{
		numBankPSqueues = NUM_BANKS;
	}
	else
	{
		ERROR("== Error - Unknown queuing structure");
		exit(0);
	}
		//create queue based on the structure we want
	Entry1D actualQueue;
	Entry2D perBankQueue = Entry2D();
	PSqueues = Entry3D();
	for (size_t rank=0; rank<NUM_RANKS; rank++)
	{
		//this loop will run only once for per-rank and NUM_BANKS times for per-rank-per-bank
		for (size_t bank=0; bank<numBankPSqueues; bank++)
		{
			actualQueue	= Entry1D();
			perBankQueue.push_back(actualQueue);
		}
		PSqueues.push_back(perBankQueue);
	}
}
PartialSETQueue :: ~PartialSETQueue(){
	//ERROR("PartialSET QUEUE destructor");
	size_t bankMax = NUM_RANKS;
	if (queuingStructure == PerRank) {
		bankMax = 1; 
	}
	for (size_t r=0; r< NUM_RANKS; r++)
	{
		for (size_t b=0; b<bankMax; b++) 
		{
			for (size_t i=0; i<PSqueues[r][b].size(); i++)
			{
				delete(PSqueues[r][b][i]);
			}
			PSqueues[r][b].clear();
		}
	}
}
bool PartialSETQueue::enqueue(BusPacket *newBusPacket)
{
	unsigned rank = newBusPacket->rank;
	unsigned bank = newBusPacket->bank;
	entry* newEntry;
	newEntry->busPacket= newBusPacket;
	newEntry->elapsedTime=0;
	if (queuingStructure==PerRank)
	{	
		for(unsigned i =0; i<PSqueues[rank][0].size();i++ ){
				if(PSqueues[rank][0][i].busPacket->physicsAddress == newBusPacket->physicalAddress){
					PSqueues[rank][0].erase(PSqueues[rank][0].begin() + i);
					break;
				}
		}
		if (PSqueues[rank][0].size()>=PARTIAL_QUEUE_DEPTH)
		{
			ERROR("== Error - Enqueued more than allowed in command queue");
			ERROR("						Need to call .hasRoomFor(int numberToEnqueue, unsigned rank, unsigned bank) first");
			//Evict one entry from PSqueue.
			return false;
		}
		PSqueues[rank][0].push_back(newEntry);
		return true;
	}
	else if (queuingStructure==PerRankPerBank)
	{
		for(unsigned i =0; i<PSqueues[rank][bank].size();i++ ){
				if(PSqueues[rank][bank][i].busPacket->physicsAddress == newBusPacket->physicalAddress){
					PSqueues[rank][bank].erase(PSqueues[rank][bank].begin() + i);
					break;
				}
		}
		PSqueues[rank][bank].push_back(newEntry);

		if (PSqueues[rank][bank].size()>=PARTIAL_QUEUE_DEPTH)
		{
			ERROR("== Error - Enqueued more than allowed in command queue");
			ERROR("						Need to call .hasRoomFor(int numberToEnqueue, unsigned rank, unsigned bank) first");
			//Evict one entry from PSqueue.
			return false;
		}
		PSqueues[rank][0].push_back(newEntry);
		return true;
	}
/*	else
	{
		ERROR("== Error - Unknown queuing structure");
		exit(0);
	}*/
}

//two conditions.1, the queue is full; 2, the elaspedTime of oldest entry reaches the threshold;
//then issue a SET from PSqueue
BusPacket* evict(bool full){
	
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
		
