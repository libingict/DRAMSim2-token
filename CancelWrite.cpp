/*
 * CancelWrite.cpp
 *
 *  Created on: 2014-3-13
 *      Author: libing
 */

#include "CancelWrite.h"
using namespace DRAMSim;

CancelWrite::CancelWrite(CommandQueue *cmdqueue,
		vector<vector<BankState> > &states, ostream &dramsim_log_) :
		dramsim_log(dramsim_log_), bankStates(states), writeQueue(states,
				dramsim_log_), readQueue(states, dramsim_log_), writeQueueDepth(
				CMD_QUEUE_DEPTH), currentClockCyle(0) {
	commandQueue = cmdqueue;
	writecancel = vector < vector<bool>
			> (NUM_RANKS, vector<bool>(NUM_BANKS, false));

	writestarttime = vector < vector<unsigned>
			> (NUM_RANKS, vector<unsigned>(NUM_BANKS, 0));

}
bool CancelWrite::addRequest(Transaction *transaction, BusPacket *buspacket) {
	if (transaction->transactionType == DATA_READ) {
		vector<BusPacket*> &queue = writeQueue.getCommandQueue(buspacket->rank,
				buspacket->bank);
		for (unsigned i = 0; i != queue.size(); i++) {
			BusPacket *packet = queue[i];
			if (packet->physicalAddress == buspacket->physicalAddress) { //if write queue has the same address, then the read request can be returned
#ifndef NO_STORAGE
				banks[packet->bank].read(packet);
#else
				packet->busPacketType = DATA;
#endif
				buspacket->busPacketType = DATA;
				transaction->transactionType = RETURN_DATA; //MC got the returned data from write queue.
				break;
			}
		}
		readQueue.enqueue(
				new BusPacket(ACTIVATE, buspacket->physicalAddress,
						buspacket->column, buspacket->row, buspacket->rank,
						buspacket->bank, buspacket->data, dramsim_log));
		readQueue.enqueue(buspacket);
	} else if (transaction->transactionType == DATA_WRITE) {
		vector<BusPacket*> &queue = writeQueue.getCommandQueue(buspacket->rank,
				buspacket->bank);
		for (unsigned i = 0; i != queue.size(); i++) {
			BusPacket *packet = queue[i];
			if (packet->physicalAddress == buspacket->physicalAddress) { //if the write queue has the same address, then the old write can be evicted, updated the newest data.
				if (i != 0 && queue[i - 1]->busPacketType == ACTIVATE) { // the write request is paired
					delete (queue[i - 1]);
					// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
					queue.erase(queue.begin() + i - 1, queue.begin() + i + 1);
				} else
					//the activated has popped.
					queue.erase(queue.begin() + i);
				break;
			}
		}
		//every bank pending one request.
		writeQueue.enqueue(
				new BusPacket(ACTIVATE, buspacket->physicalAddress,
						buspacket->column, buspacket->row, buspacket->rank,
						buspacket->bank, buspacket->data, dramsim_log));
		writeQueue.enqueue(buspacket);
	}
	if (transaction->transactionType == RETURN_DATA) {
		return true;
	} else
		return false;
}
bool CancelWrite::issueRequest(unsigned r, unsigned b, BusPacket *busPacket,
		CommandQueue &requestQueue) {
	BusPacket *request = NULL;
	bool issuable = false;
	/*	if (rowBufferPolicy == OpenPage && queuingStructure == PerRankPerBank) {
	 for (r = 0; r != NUM_RANKS; r++) {
	 for (b = 0; b != NUM_BANKS; b++) {*/
	vector<BusPacket *> &queue = requestQueue.getCommandQueue(r, b);
//	if (queue.size() > (CMD_QUEUE_DEPTH * 0.8)) { //move to cancel write
	for (unsigned i = 0; i != queue.size(); i++) {
		request = queue[i];
		if (requestQueue.isIssuable(request)) {
//			if (!(writerequest->busPacketType == ACTIVATE))
			//check for dependencies
			bool dependencyFound = false;
			for (unsigned j = 0; j < i; j++) {
				BusPacket *prevPacket = queue[j];
				if (prevPacket->bank == request->bank
						&& prevPacket->row == request->row) {
					if (prevPacket->busPacketType != ACTIVATE) {
						dependencyFound = true;
						break;
					}

				}

			}
			if (dependencyFound)
				continue;
			busPacket = request;
			//if the bus packet before is an activate, that is the act that was
			//	paired with the column access we are removing, so we have to remove
			//	that activate as well (check i>0 because if i==0 then there's nothing before it)
			if (i > 0 && queue[i - 1]->busPacketType == ACTIVATE) {
//									rowAccessCounters[(*busPacket)->rank][(*busPacket)->bank]++;
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
		}
	}
//	}
	return issuable;

}
/*bool CancelWrite::issueWrite(unsigned r, unsigned b, BusPacket **busPacket) {
 BusPacket *writerequest = NULL;

 if (rowBufferPolicy == OpenPage && queuingStructure == PerRankPerBank) {
 for (r = 0; r != NUM_RANKS; r++) {
 for (b = 0; b != NUM_BANKS; b++) {
 vector<BusPacket *> &writequeue = writeQueue.getCommandQueue(r,
 b);
 if (writequeue.size() > (int) (CMD_QUEUE_DEPTH * 0.8)) { //move to cancel write
 for (int i = 0; i != writequeue.size; i++) {
 *writerequest = writequeue[i];
 if (writeQueue.issuable(writerequest)) {
 //	if (!(writerequest->busPacketType == ACTIVATE))

 //check for dependencies
 bool dependencyFound = false;
 for (size_t j = 0; j < i; j++) {
 BusPacket *prevPacket = writequeue[j];
 if (prevPacket->bank == writerequest->bank
 && prevPacket->row
 == writerequest->row) {
 if (prevPacket->busPacketType != ACTIVATE) {
 dependencyFound = true;
 break;
 }

 }

 }
 if (dependencyFound)
 continue;
 *busPacket = writerequest;
 //if the bus packet before is an activate, that is the act that was
 //	paired with the column access we are removing, so we have to remove
 //	that activate as well (check i>0 because if i==0 then there's nothing before it)
 if (i > 0
 && queue[i - 1]->busPacketType
 == ACTIVATE) {
 //									rowAccessCounters[(*busPacket)->rank][(*busPacket)->bank]++;
 // i is being returned, but i-1 is being thrown away, so must delete it here
 delete (queue[i - 1]);
 // remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
 queue.erase(queue.begin() + i - 1,
 queue.begin() + i + 1);
 } else // there's no activate before this packet
 {
 //or just remove the one bus packet
 queue.erase(queue.begin() + i);
 }

 issueWrite = true;
 break;
 }
 }
 }
 return issueWrite;

 }
 if (issueWrite) {
 break;
 }
 }
 }

 }*/
bool CancelWrite::cancelWrite(BusPacket **busPacket) {
	BusPacket *poppedBusPacket = NULL;
//	BusPacket *writerequest = NULL;
	unsigned r = 0;
	unsigned b = 0;
	if (rowBufferPolicy == OpenPage && queuingStructure == PerRankPerBank) {
		bool issueWrite = false;
		bool issueRead = false;
//		unsigned r = RDpacket->rank;
//		unsigned b = RDpacket->bank;
		for (r = 0; r != NUM_RANKS; r++) {
			for (b = 0; b != NUM_BANKS; b++) {
				vector<BusPacket *> &writequeue = writeQueue.getCommandQueue(r,
						b);
				/*				vector<BuaPacket *> &readqueue = readQueue.getCommandQueue(r,
				 b);*/
				if (writequeue.size() > (int) (CMD_QUEUE_DEPTH * 0.8)) { //then issue Write
					if (issueRequest(r, b, poppedBusPacket, writeQueue)) {
						writestarttime[r][b] = currentClockCyle;
						pendingWR[r][b] = poppedBusPacket;
						issueWrite = true;
						//pending the writing request
						break;
					}
					/*					for (int i = 0; i != writequeue.size; i++) {
					 *writerequest = writequeue[i];
					 if (writeQueue.issuable(writerequest)) {
					 if (!(writerequest->busPacketType == ACTIVATE))
					 issueWrite = true;
					 }
					 }*/
					if (!issueWrite) {
						if (issueRequest(r, b, poppedBusPacket, readQueue)) {
							issueRead = true;
							break;
						}
					}

				} else {								//prior the read request
					if (issueRequest(r, b, poppedBusPacket, readQueue)) {
						issueRead = true;
						break;
					}
					if (!issueRead) {
						if (issueRequest(r, b, poppedBusPacket, writeQueue)) {
							//issue the cancel
							issueWrite = true;
							break;
						}
					}

				}
			}
			if (issueWrite || issueRead) {
				break;

			}

		}
	}

	/*	if ((bankStates[r][b].lastCommand == WRITE)
	 || (bankStates[r][b].lastCommand == WRITE_P)) {
	 completeFraction = (currentClockCycle - bankStates[r][b].starttime)
	 / (bankStates[r][b].nextRead);
	 if (completeFraction < Threshold) {
	 //		cout << " bankStates["<<busPacket->rank<<"] [" << busPacket->bank<< "].nextRead " << bankStates[busPacket->rank][busPacket->bank].nextRead ;
	 bankStates[busPacket->rank][busPacket->bank].nextRead =
	 currentClockCycle;
	 vector<BusPacket*> &queue = getCommandQueue(busPacket->rank,
	 busPacket->bank);
	 BusPacket *activeCmd = new BusPacket(ACTIVATE,
	 busPacket->physicalAddress, busPacket->row,
	 busPacket->column, busPacket->rank, busPacket->bank, 0,
	 dramsim_log);
	 BusPacket *cmd = new BusPacket(WRITE, busPacket->physicalAddress,
	 busPacket->row, busPacket->column, busPacket->rank,
	 busPacket->bank, busPacket->data, dramsim_log);
	 queue.push_back(activeCmd);
	 queue.push_back(cmd);
	 //			cout << " currentClockcycle is " << currentClockCycle

	 if(currentClockCycle
	 < bankStates[busPacket->rank][busPacket->bank].nextRead){


	 //			canceledWrite[r][busPacket->bank]++;
	 writecomplete = false;
	 RDpacket->busPacketType = WRITE;
	 pendingWR[r][b](RDpacket);
	 bankStates[busPacket->rank][busPacket->bank].print();
	 cout << " start time is "
	 << bankStates[busPacket->rank][busPacket->bank].starttime
	 << "\t";
	 vector<BusPacket *> &queue = getCommandQueue(busPacket->rank,
	 busPacket->bank);
	 for (int i = 0; i != queue.size(); i++) {
	 queue[i]->print();
	 }
	 bankStates[busPacket->rank][busPacket->bank].nextRead =
	 currentClockCycle;
	 }
	 //<< " cancel write\n ";
	 //			return true;
	 //		}
	 }*/
	return true;
}

void CancelWrite::update() {
	/*	for(unsigned r =0; r!=NUM_RANKS; r++) {
	 for (unsigned b = 0; b != NUM_BANKS; b++) {
	 BusPacket *packet = pendingWR[r][b];
	 if(currentClockCyle>=std::min(bankStates[r][b].nextRead,
	 bankStates[r][b].nextActivate,bankStates[r][b].nextPrecharge,bankStates[r][b].nextPowerUp)
	 }
	 }*/

}
