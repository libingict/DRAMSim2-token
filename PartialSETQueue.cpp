
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
	size_t numBankQueues;
	if (queuingStructure==PerRank)
	{
		numBankQueues = 1;
	}
	else if (queuingStructure==PerRankPerBank)
	{
		numBankQueues = NUM_BANKS;
	}
	else
	{
		ERROR("== Error - Unknown queuing structure");
		exit(0);
	}
		//create queue based on the structure we want
	Entry1D actualQueue;
	Entry2D perBankQueue = Entry2D();
	queues = Entry3D();
	for (size_t rank=0; rank<NUM_RANKS; rank++)
	{
		//this loop will run only once for per-rank and NUM_BANKS times for per-rank-per-bank
		for (size_t bank=0; bank<numBankQueues; bank++)
		{
			actualQueue	= Entry1D();
			perBankQueue.push_back(actualQueue);
		}
		queues.push_back(perBankQueue);
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
			for (size_t i=0; i<queues[r][b].size(); i++)
			{
				delete(queues[r][b][i]);
			}
			queues[r][b].clear();
		}
	}
}
void PartialSETQueue::enqueue(BusPacket *newBusPacket)
{
	unsigned rank = newBusPacket->rank;
	unsigned bank = newBusPacket->bank;
	entry* newEntry;
	newEntry->busPacket= newBusPacket;
	newEntry->elapsedTime=0;
	if (queuingStructure==PerRank)
	{	
		for(unsigned i =0; i<queues[rank][0].size();i++ ){
				if(queues[rank][0][i].busPacket->physicsAddress == newBusPacket->physicalAddress)
					break;
		}
		queues[rank][0].erase(queues[rank][0].begin() + i);
		queues[rank][0].push_back(newEntry);
		if (queues[rank][0].size()>PARTIAL_QUEUE_DEPTH)
		{
			ERROR("== Error - Enqueued more than allowed in command queue");
			ERROR("						Need to call .hasRoomFor(int numberToEnqueue, unsigned rank, unsigned bank) first");
			exit(0);
		}
	}
	else if (queuingStructure==PerRankPerBank)
	{
		for(unsigned i =0; i<queues[rank][bank].size();i++ ){
				if(queues[rank][bank][i].busPacket->physicsAddress == newBusPacket->physicalAddress)
					break;
		}
		queues[rank][bank].erase(queues[rank][bank].begin() + i);
		queues[rank][bank].push_back(newEntry);
		if (queues[rank][bank].size()>PARTIAL_QUEUE_DEPTH)
		{
			ERROR("== Error - Enqueued more than allowed in command queue");
			ERROR("						Need to call .hasRoomFor(int numberToEnqueue, unsigned rank, unsigned bank) first");
			exit(0);
		}
	}
	else
	{
		ERROR("== Error - Unknown queuing structure");
		exit(0);
	}
}
BusPacket* evict(){
	

}
		
