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
		CancelWrite &canclewrite, ostream &dramsim_log_) :
		bankStates(states), cancelWrite(canclewrite), dramsim_log(dramsim_log_) {
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
	isFull = vector < vector<bool>
			> (NUM_RANKS, vector<bool>(NUM_BANKS, false));
	idle = vector < vector<bool> > (NUM_RANKS, vector<bool>(NUM_BANKS, false));
	begin = vector < vector<unsigned>
			> (NUM_RANKS, vector<unsigned>(NUM_BANKS, 0));
	duration = vector < vector<unsigned>
			> (NUM_RANKS, vector<unsigned>(NUM_BANKS, 0));
	//create queue based on the structure we want
//	predictTable = vector<vector<PredictionEntry*> >(NUM_RANKS,vector<PredictionEntry*>(NUM_BANKS,vector<PredictionEntry*>()));
	Entry1D actualQueue;
	Entry2D perBankQueue = Entry2D();
	PSqueues = Entry3D();
	BankD idleBank;
	RankD idleRank = RankD();
	IdleTable = Table();
	Predict2D predictRank = Predict2D();
	for (size_t rank = 0; rank < NUM_RANKS; rank++) {
		//this loop will run only once for per-rank and NUM_BANKS times for per-rank-per-bank
		for (size_t bank = 0; bank < numBankqueues; bank++) {
			actualQueue = Entry1D();
			perBankQueue.push_back(actualQueue);
			idleBank = BankD();
			idleRank.push_back(idleBank);
			predictRank.push_back(vector<PredictionEntry *>());
		}
		PSqueues.push_back(perBankQueue);
		IdleTable.push_back(idleRank);
		predictTable.push_back(predictRank);
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
			predictTable[r][b].clear();
		}
	}
}
//double PartialSETQueue::RETENTION_TIME=1.0e9;
bool PartialSETQueue::enqueue(BusPacket *bspacket) {
	unsigned rank = bspacket->rank;
	unsigned bank = bspacket->bank;
	if (!(bspacket->busPacketType == WRITE || bspacket->busPacketType == WRITE_P)) {
//		ERROR("== Error - Enqueued busPacket should be write");
		return false;
	}
	vector<BusPacket *> &readqueue = cancelWrite.readQueue.getCommandQueue(
			bspacket->rank, bspacket->bank);
	if (readqueue.empty()) { //the write request could be SET
		return false;
	}
	/*	if (queuingStructure == PerRank) {
	 ;
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
	 } else */
	if (queuingStructure == PerRankPerBank) {
		for (unsigned i = 0; i < PSqueues[rank][bank].size(); i++) {
			Entry *entry = PSqueues[rank][bank][i];
			if (entry->busPacket->physicalAddress
					== bspacket->physicalAddress) { //if the write queue has the same address, then the old write can be evicted, updated the newest data.
				if (i != 0
						&& PSqueues[rank][bank][i - 1]->busPacket->busPacketType
								== ACTIVATE
						&& PSqueues[rank][bank][i - 1]->busPacket->physicalAddress
								== bspacket->physicalAddress) { // the write request is paired
					delete (PSqueues[rank][bank][i - 1]);
					// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
					PSqueues[rank][bank].erase(
							PSqueues[rank][bank].begin() + i - 1,
							PSqueues[rank][bank].begin() + i + 1); //[queue.begin() + i - 1, queue.begin() + i + 1)
				} else {
					//the activated has popped.
					PSqueues[rank][bank].erase(
							PSqueues[rank][bank].begin() + i);
				}
				break;
			}
		}
		PSqueues[rank][bank].push_back(
				new Entry(
						new BusPacket(ACTIVATE, bspacket->physicalAddress,
								bspacket->column, bspacket->row, bspacket->rank,
								bspacket->bank, bspacket->data, dramsim_log,
								bspacket->RIP)));
		PSqueues[rank][bank].push_back(
				new Entry(
						new BusPacket(WRITE, bspacket->physicalAddress,
								bspacket->column, bspacket->row, bspacket->rank,
								bspacket->bank, bspacket->data, dramsim_log,
								bspacket->RIP)));
		if (PSqueues[rank][bank].size() >= PARTIAL_QUEUE_DEPTH) {
			/*ERROR("== Error - Enqueued more than allowed in command queue");
			 ERROR(
			 "						Need to call .hasRoomFor(int numberToEnqueue, unsigned rank, unsigned bank) first");*/
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
//the same address entry is SET from command queue, the relative entry in PSQueue should be deleted
void PartialSETQueue::release(BusPacket *bspacket) {
	unsigned rank = bspacket->rank;
	unsigned bank = bspacket->bank;
	Entry1D::iterator iter;
	if (queuingStructure == PerRank) {
		if (!PSqueues[rank][0].empty()) {
			for (iter = PSqueues[rank][0].begin();
					iter != PSqueues[rank][0].end(); ++iter) {
				Entry *tmp = *iter;
				if (tmp->busPacket->physicalAddress
						== bspacket->physicalAddress)
					break;
			}
			PSqueues[rank][0].erase(iter);
		}
	} else if (queuingStructure == PerRankPerBank) {
		for (unsigned i = 0; i < PSqueues[rank][bank].size(); i++) {
			Entry *entry = PSqueues[rank][bank][i];
			if (entry->busPacket->physicalAddress
					== bspacket->physicalAddress) { //if the write queue has the same address, then the old write can be evicted, updated the newest data.
				if (i != 0
						&& PSqueues[rank][bank][i - 1]->busPacket->busPacketType
								== ACTIVATE
						&& PSqueues[rank][bank][i - 1]->busPacket->physicalAddress
								== bspacket->physicalAddress) { // the write request is paired
					delete (PSqueues[rank][bank][i - 1]);
					PSqueues[rank][bank].erase(
							PSqueues[rank][bank].begin() + i - 1,
							PSqueues[rank][bank].begin() + i + 1); //[queue.begin() + i - 1, queue.begin() + i + 1)
				} else if (PSqueues[rank][bank][i]->busPacket->busPacketType
						== ACTIVATE && PSqueues[rank][bank].size() > 1
						&& i < PSqueues[rank][bank].size() - 1) {
					delete (PSqueues[rank][bank][i + 1]);
					// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
					PSqueues[rank][bank].erase(PSqueues[rank][bank].begin() + i,
							PSqueues[rank][bank].begin() + i + 2);
				} else {
					//the activated has popped.
					PSqueues[rank][bank].erase(
							PSqueues[rank][bank].begin() + i);
				}
				break;
			}
		}
	}
}

/*vector<entry *> &PartialSETQueue::getPSQueue(unsigned rank, unsigned bank) {
 if (queuingStructure == PerRankPerBank) {
 return PSqueues[rank][bank];
 } else if (queuingStructure == PerRank) {
 return PSqueues[rank][0];
 } else {
 ERROR("Unknown queue structure");
 abort();
 }
 }*/
//two conditions.1, the queue is full; 2, the elaspedTime of oldest entry reaches the threshold;
//then issue a SET from PSqueue bank, insert the issued cmd to cmdqueue.
bool PartialSETQueue::evict(unsigned &nextrank, unsigned &nextbank,
		BusPacket **busPacket) {
	unsigned startingRank = nextrank;
	unsigned startingBank = nextbank;
	bool issuable = false;
	do {
		Entry1D &psqueue = PSqueues[nextrank][nextbank];
		if (!PSqueues[nextrank][nextbank].empty()) {
			if (isFull[nextrank][nextbank]) {
				if (bankStates[nextrank][nextbank].currentBankState == Idle) {
					//just issue PartialSET from the PartialSET Queue, out-of-order
					if (currentClockCycle
							>= bankStates[nextrank][nextbank].nextActivate) {
						for (size_t i = 0; i < psqueue.size(); i++) {
							if (psqueue[i]->busPacket->busPacketType
									== ACTIVATE) {
								*busPacket = psqueue[i]->busPacket;
								issuable = true;
								break;
							}
						}

					}
				} else if (bankStates[nextrank][nextbank].currentBankState
						== RowActive) {
					bool foundopen = false;
					for (size_t i = 0; i < psqueue.size(); i++) {
						Entry* entry = psqueue[i];
						if (entry->busPacket->row
								== bankStates[nextrank][nextbank].openRowAddress) {
							foundopen = true;
							if (currentClockCycle
									>= bankStates[nextrank][nextbank].nextWrite) {
								issuable = true;
								if (i > 0
										&& psqueue[i - 1]->busPacket->busPacketType
												== ACTIVATE) {
									*busPacket = psqueue[i]->busPacket;
									delete (psqueue[i - 1]);
									psqueue.erase(psqueue.begin() + i - 1,
											psqueue.begin() + i + 1);
								} else if (psqueue[i]->busPacket->busPacketType
										== ACTIVATE && psqueue.size() > 1
										&& i < psqueue.size() - 1) {
									*busPacket = psqueue[i + 1]->busPacket;
									delete (psqueue[i + 1]);
									// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
									psqueue.erase(psqueue.begin() + i,
											psqueue.begin() + i + 2);
								} else { // there's no activate before this packet
										 //or just remove the one bus packet
									*busPacket = psqueue[i]->busPacket;
									psqueue.erase(psqueue.begin() + i);
								}
								break;
							}
						}
					}
					//check the writeQueue and readQueue ,if there is same open row, issue the request and Precharge.
					//or cancel the request and Precharge
					if (!foundopen) {
						vector<BusPacket *> &writequeue =
								cancelWrite.writeQueue.getCommandQueue(nextrank,
										nextbank);
						vector<BusPacket *> &readqueue =
								cancelWrite.readQueue.getCommandQueue(nextrank,
										nextbank);
						bool found = false;
						for (unsigned i = 0; i < readqueue.size(); i++) { //assure no dangling READ
							if (readqueue[i]->bank == nextbank
									&& readqueue[i]->row
											== bankStates[nextrank][nextbank].openRowAddress) {
								if (readqueue[i]->busPacketType != ACTIVATE) {
									if (currentClockCycle
											>= bankStates[nextrank][nextbank].nextRead) {
										*busPacket = readqueue[i];
										readqueue.erase(readqueue.begin() + i);
										issuable = true;
									}
								}
								found = true;
								break;
							}
						}
						if (!found) {
							for (unsigned i = 0; i < writequeue.size(); i++) { //assure no dangling READ
								//if there is something going to that bank and row, then we don't want to send a PRE
								if (writequeue[i]->bank == nextbank
										&& writequeue[i]->row
												== bankStates[nextrank][nextbank].openRowAddress) {
									if (writequeue[i]->busPacketType
											!= ACTIVATE) {
										if (currentClockCycle
												>= bankStates[nextrank][nextbank].nextWrite) {
											*busPacket = writequeue[i];
											writequeue.erase(
													writequeue.begin() + i);
											issuable = true;
										}
									}
									found = true;
									break;
								}
							}
						}
						if (!issuable
								&& currentClockCycle
										>= bankStates[nextrank][nextbank].nextPrecharge) {
							*busPacket = new BusPacket(PRECHARGE, 0, 0, 0,
									nextrank, nextbank, 0, dramsim_log);
							issuable = true;
						}
					}
				}
				if (issuable) {
					if ((*busPacket)->busPacketType == WRITE) {
						(*busPacket)->busPacketType = FullSET;
						isFull[nextrank][nextbank] = false;
					}
					return true;
				}
			} else {
				Entry* entry = psqueue[0];
				if (entry->elapsedTime == 10000) {
					if (bankStates[nextrank][nextbank].currentBankState
							== Idle) {
						//just issue PartialSET from the PartialSET Queue, out-of-order
						if (currentClockCycle
								>= bankStates[nextrank][nextbank].nextActivate) {
							*busPacket = psqueue[0]->busPacket;
							(*busPacket)->busPacketType = PartialSET;
							psqueue.erase(psqueue.begin());
							psqueue.push_back(new Entry((*busPacket)));
							return true;
						}
					} else if (bankStates[nextrank][nextbank].currentBankState
							== RowActive) {
						//check the writeQueue and readQueue ,if there is same open row, issue the request and Precharge.
						//or cancel the request and Precharge
						bool foundopen = false;
						if (entry->busPacket->row
								== bankStates[nextrank][nextbank].openRowAddress) {
							foundopen = true;
							if (currentClockCycle
									>= bankStates[nextrank][nextbank].nextWrite) {
								if (psqueue[0]->busPacket->busPacketType
										== ACTIVATE) {
									delete (psqueue[0]);
									psqueue.erase(psqueue.begin(),
											psqueue.begin() + 1);
									psqueue.push_back(
											new Entry(
													new BusPacket(ACTIVATE,
															(*busPacket)->physicalAddress,
															(*busPacket)->column,
															(*busPacket)->row,
															nextrank, nextbank,
															(*busPacket)->data,
															dramsim_log,
															(*busPacket)->RIP)));
									psqueue.push_back(
											new Entry(
													new BusPacket(WRITE,
															(*busPacket)->physicalAddress,
															(*busPacket)->column,
															(*busPacket)->row,
															nextrank, nextbank,
															(*busPacket)->data,
															dramsim_log,
															(*busPacket)->RIP)));
								} else { // there's no activate before this packet
									//or just remove the one bus packet
									psqueue.erase(psqueue.begin());
									psqueue.push_back(new Entry((*busPacket)));
								}
								*busPacket = psqueue[0]->busPacket;
								(*busPacket)->busPacketType = PartialSET;
								return true;
							}
						}
						//check the writeQueue and readQueue ,if there is same open row, issue the request and Precharge.
						//or cancel the request and Precharge
						if (!foundopen) {
							if (currentClockCycle
									>= bankStates[nextrank][nextbank].nextPrecharge) {
								*busPacket = new BusPacket(PRECHARGE, 0, 0, 0,
										nextrank, nextbank, 0, dramsim_log);
								vector<BusPacket *> &writequeue =
										cancelWrite.writeQueue.getCommandQueue(
												nextrank, nextbank);
								vector<BusPacket *> &readqueue =
										cancelWrite.readQueue.getCommandQueue(
												nextrank, nextbank);
								bool found = false;
								for (unsigned r = 0; r < readqueue.size();
										r++) { //assure no dangling READ
									//if there is something going to that bank and row, then we don't want to send a PRE
									if (readqueue[r]->bank == nextbank
											&& readqueue[r]->row
													== bankStates[nextrank][nextbank].openRowAddress) {
										if (readqueue[r]->busPacketType
												!= ACTIVATE) {
											vector<BusPacket*>::iterator it =
													readqueue.begin() + r;
											readqueue.insert(it,
													new BusPacket(ACTIVATE,
															readqueue[r]->physicalAddress,
															readqueue[r]->column,
															readqueue[r]->row,
															readqueue[r]->rank,
															readqueue[r]->bank,
															readqueue[r]->data,
															dramsim_log,
															readqueue[r]->RIP));
										}
										found = true;
										break;
									}
								}
								if (!found) {
									for (unsigned w = 0; w < writequeue.size();
											w++) { //assure no dangling READ
										if (writequeue[w]->bank == nextbank
												&& writequeue[w]->row
														== bankStates[nextrank][nextbank].openRowAddress) {
											if (writequeue[w]->busPacketType
													!= ACTIVATE) {
												vector<BusPacket*>::iterator it =
														writequeue.begin() + w;
												writequeue.insert(it,
														new BusPacket(ACTIVATE,
																writequeue[w]->physicalAddress,
																writequeue[w]->column,
																writequeue[w]->row,
																writequeue[w]->rank,
																writequeue[w]->bank,
																writequeue[w]->data,
																dramsim_log,
																writequeue[w]->RIP));
											}
											break;
										}
									} //end for writequeue
								} // !found
							} //nextPrecharge
						} //!foundopen
					} //rowactive
				} //RETENTIONTIME
			} //
		}
		cancelWrite.writeQueue.nextRankAndBank(nextrank, nextbank);
	} while (!(startingRank == nextrank && startingBank == nextbank));
	return false;
}

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
					duration[i][j] = currentClockCycle - begin[i][j];
					if (duration[i][j] >= bankStates[i][j].nextWrite) {
						predictTable[i][j][0]->idleInterval.push_back(true);
					} else {
						predictTable[i][j][0]->idleInterval.push_back(false);
					}
					idle[i][j] = false;
				}
			}
		}
	}
}
void PartialSETQueue::iniPredictTable(unsigned rank, unsigned bank,
		uint64_t addr, uint64_t rip) {
	bool found = false;
	vector<PredictionEntry *> &predictbank = predictTable[rank][bank];
	vector<PredictionEntry*>::iterator it = predictbank.begin();
	if (!predictbank.empty()) {
		for (size_t i = 0; i < predictbank.size(); i++) {
			if (predictbank[i]->RIP != 0 && predictbank[i]->RIP == rip) {
				predictbank[i]->timeAccess = currentClockCycle;
				predictbank[i]->address = addr;
				found = true;
				predictbank.insert(it, new PredictionEntry(*(predictbank[i])));
				predictbank.erase(predictbank.begin() + i);
				break;
			}
		}
		if (!found) {
			predictbank.insert(it,
					new PredictionEntry(rip, addr, currentClockCycle));
		}
//		sort(predictbank.begin(), predictbank.end(), PredictionEntry::compare); //the newest entry is at the beginning;
	}
}
void PartialSETQueue::print() {
	if (queuingStructure == PerRank) {
		PRINT(endl << "== Printing Per Rank Queue");
		for (size_t i = 0; i < NUM_RANKS; i++) {
			PRINT(" = Rank " << i << "  size : " << PSqueues[i][0].size());
			for (size_t j = 0; j < PSqueues[i][0].size(); j++) {
				PRINTN("    "<< j << "]");
				PSqueues[i][0][j]->busPacket->print();
				PRINT("elapsed Time is "<<PSqueues[i][0][j]->elapsedTime);
			}
		}
	} else if (queuingStructure == PerRankPerBank) {
		PRINT("\n== Printing Per Rank, Per Bank Queue");

		for (size_t i = 0; i < NUM_RANKS; i++) {
			PRINT(" = Rank " << i);
			for (size_t j = 0; j < NUM_BANKS; j++) {
				PRINT("    Bank "<< j << "   size : " << PSqueues[i][j].size());

				for (size_t k = 0; k < PSqueues[i][j].size(); k++) {
					PRINTN("       " << k << "]");
					PSqueues[i][j][k]->busPacket->print();
					PRINT("elapsed Time is "<<PSqueues[i][j][k]->elapsedTime);
				}
			}
		}
	}
}

void PartialSETQueue::update() {
//do nothing since pop() is effectively update(),
//needed for SimulatorObject
//TODO: make CommandQueue not a SimulatorObject
	step();
	Entry1D::iterator iter;
	if (queuingStructure == PerRankPerBank) {
		unsigned r = NUM_RANKS;
		unsigned b = NUM_BANKS;
		for (unsigned i = 0; i < r; ++i) {
			for (unsigned j = 0; j < b; ++j) {
				for (iter = PSqueues[i][j].begin();
						iter != PSqueues[i][j].end(); ++iter) {
					Entry *tmp = *iter;
					tmp->elapsedTime++;
				}
			}
		}
	}
}

