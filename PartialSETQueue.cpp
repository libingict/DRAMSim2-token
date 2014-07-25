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
	isFull = vector < vector<bool>
			> (NUM_RANKS, vector<bool>(NUM_BANKS, false));
	idle = vector < vector<bool> > (NUM_RANKS, vector<bool>(NUM_BANKS, false));
	begin = vector < vector<uint64_t>
			> (NUM_RANKS, vector < uint64_t > (NUM_BANKS, 0));
	duration = vector < vector<uint64_t>
			> (NUM_RANKS, vector < uint64_t > (NUM_BANKS, 0));
	countPSQsetperBank = vector < vector<uint64_t>
			> (NUM_RANKS, vector < uint64_t > (NUM_BANKS, 0));
	countPSQpartialsetperBank = vector < vector<uint64_t>
			> (NUM_RANKS, vector < uint64_t > (NUM_BANKS, 0));
	//create queue based on the structure we want
//	predictTable = vector<vector<PredictionEntry*> >(NUM_RANKS,vector<PredictionEntry*>(NUM_BANKS,vector<PredictionEntry*>()));
	Entry1D actualQueue;
	Entry2D perBankQueue = Entry2D(); //vector< vector<> > T= vector< vector<> >();
	PSqueues = Entry3D();
	BankD idleBank;
	RankD idleRank = RankD();
	IdleTable = Table();
	Predict2D predictRank = Predict2D();
	for (size_t rank = 0; rank < NUM_RANKS; rank++) {
		//this loop will run only once for per-rank and NUM_BANKS times for per-rank-per-bank
		for (size_t bank = 0; bank < numBankqueues; bank++) {
			actualQueue = Entry1D(); //vector <>  T = vector<> ();
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
	if (readqueue.empty()
			|| (PSqueues[rank][bank].size() >= PARTIAL_QUEUE_DEPTH)) { //the write request could be SET
		return false;
	}
	if (queuingStructure == PerRankPerBank) {
		for (unsigned i = 0; i < PSqueues[rank][bank].size(); i++) {
			Entry *entry = PSqueues[rank][bank][i];
			if (entry->busPacket->physicalAddress
					== bspacket->physicalAddress) {
				if (entry->busPacket->busPacketType == ACTIVATE
						&& i < PSqueues[rank][bank].size() - 1
						&& PSqueues[rank][bank][i + 1]->busPacket->physicalAddress
								== bspacket->physicalAddress) {
					delete (PSqueues[rank][bank][i]);
					delete (PSqueues[rank][bank][i + 1]);
					PSqueues[rank][bank].erase(PSqueues[rank][bank].begin() + i,
							PSqueues[rank][bank].begin() + i + 2);
				} else if (i > 0
						&& PSqueues[rank][bank][i - 1]->busPacket->busPacketType
								== ACTIVATE
						&& PSqueues[rank][bank][i - 1]->busPacket->physicalAddress
								== bspacket->physicalAddress) {
					delete (PSqueues[rank][bank][i - 1]);
					delete (PSqueues[rank][bank][i]);
					PSqueues[rank][bank].erase(
							PSqueues[rank][bank].begin() + i - 1,
							PSqueues[rank][bank].begin() + i + 1); //[queue.begin() + i - 1, queue.begin() + i + 1)
				} else {
					//the activated has popped.
					delete (PSqueues[rank][bank][i]);
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
//		PRINT(
//				"clock "<<currentClockCycle<<" en psqueue 0x"<<hex<<bspacket->physicalAddress<<dec<<" r["<<rank<<"] b["<<bank<<"] ");
		if (PSqueues[rank][bank].size() >= PARTIAL_QUEUE_DEPTH) {
			/*ERROR("== Error - Enqueued more than allowed in command queue");
			 ERROR(
			 "						Need to call .hasRoomFor(int numberToEnqueue, unsigned rank, unsigned bank) first");*/
			//set the queue state full
			isFull[rank][bank] = true;
		} else {
			isFull[rank][bank] = false;
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
				/*if (i != 0
				 && PSqueues[rank][bank][i - 1]->busPacket->busPacketType
				 == ACTIVATE
				 && PSqueues[rank][bank][i - 1]->busPacket->physicalAddress
				 == bspacket->physicalAddress) { // the write request is paired
				 delete (PSqueues[rank][bank][i - 1]);
				 delete (PSqueues[rank][bank][i]);
				 PSqueues[rank][bank].erase(
				 PSqueues[rank][bank].begin() + i - 1,
				 PSqueues[rank][bank].begin() + i + 1); //[queue.begin() + i - 1, queue.begin() + i + 1)
				 } else*/
				if (PSqueues[rank][bank][i]->busPacket->busPacketType
						== ACTIVATE && i < PSqueues[rank][bank].size() - 1
						&& PSqueues[rank][bank][i + 1]->busPacket->physicalAddress
								== bspacket->physicalAddress) {
					delete (PSqueues[rank][bank][i]);
					delete (PSqueues[rank][bank][i + 1]);
					// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
					PSqueues[rank][bank].erase(PSqueues[rank][bank].begin() + i,
							PSqueues[rank][bank].begin() + i + 2); //[queue.begin() + i , queue.begin() + i + 2)
				} else {
					//the activated has popped.
					delete (PSqueues[rank][bank][i]);
					PSqueues[rank][bank].erase(
							PSqueues[rank][bank].begin() + i);
				}
//				PRINT(
//						"clock "<<currentClockCycle<<" same psQ 0x"<<hex<<bspacket->physicalAddress<<dec<<" r["<<rank<<"] b["<<bank<<"] ");
				break;
			}
		}
	}
	if (PSqueues[rank][bank].size() >= PARTIAL_QUEUE_DEPTH) {
		isFull[rank][bank] = true;
	} else {
		isFull[rank][bank] = false;
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
		if (!psqueue.empty()) {
			Entry* entry = psqueue[0];
			if (entry->elapsedTime < 1.0e9) { //the oldest entry not reaches the threshold
				if (isFull[nextrank][nextbank]) { //the queue is full; then has to issue FullSET from the head of the queue.
					if (bankStates[nextrank][nextbank].currentBankState
							== Idle) {
						//just issue Active from the PartialSET Queue, out-of-order
						if (currentClockCycle
								>= bankStates[nextrank][nextbank].nextActivate) {
							for (size_t i = 0; i < psqueue.size(); i++) {
								if (psqueue[i]->busPacket->busPacketType
										== ACTIVATE) {
									*busPacket =
											new BusPacket(ACTIVATE,
													psqueue[i]->busPacket->physicalAddress,
													psqueue[i]->busPacket->column,
													psqueue[i]->busPacket->row,
													nextrank, nextbank,
													psqueue[i]->busPacket->data,
													dramsim_log,
													psqueue[i]->busPacket->RIP);
									issuable = true;
									delete (psqueue[i]);
									psqueue.erase(psqueue.begin() + i);
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
									if (psqueue[i]->busPacket->busPacketType
											== ACTIVATE && psqueue.size() > 1
											&& i < psqueue.size() - 1
											&& psqueue[i + 1]->busPacket->physicalAddress
													== entry->busPacket->physicalAddress) {
										*busPacket =
												new BusPacket(WRITE,
														psqueue[i + 1]->busPacket->physicalAddress,
														psqueue[i + 1]->busPacket->column,
														psqueue[i + 1]->busPacket->row,
														psqueue[i + 1]->busPacket->rank,
														psqueue[i + 1]->busPacket->bank,
														psqueue[i + 1]->busPacket->data,
														dramsim_log,
														psqueue[i + 1]->busPacket->RIP);
										delete (psqueue[i]);
										delete (psqueue[i + 1]);
										// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
										psqueue.erase(psqueue.begin() + i,
												psqueue.begin() + i + 2);
									} else { // there's no activate before this packet
											 //or just remove the one bus packet
										*busPacket =
												new BusPacket(WRITE,
														psqueue[i]->busPacket->physicalAddress,
														psqueue[i]->busPacket->column,
														psqueue[i]->busPacket->row,
														psqueue[i]->busPacket->rank,
														psqueue[i]->busPacket->bank,
														psqueue[i]->busPacket->data,
														dramsim_log,
														psqueue[i]->busPacket->RIP);
										delete psqueue[i];
										psqueue.erase(psqueue.begin() + i);
									}
									break;
								}
							}
						}
						//check the writeQueue and readQueue ,if there is same open row, do nothing
						if (!foundopen) {
							vector<BusPacket *> &writequeue =
									cancelWrite.writeQueue.getCommandQueue(
											nextrank, nextbank);
							vector<BusPacket *> &readqueue =
									cancelWrite.readQueue.getCommandQueue(
											nextrank, nextbank);
							bool found = false;
							for (unsigned i = 0; i < readqueue.size(); i++) { //assure no dangling READ
								if (readqueue[i]->bank == nextbank
										&& readqueue[i]->row
												== bankStates[nextrank][nextbank].openRowAddress) {
									/*									if (readqueue[i]->busPacketType
									 != ACTIVATE) {
									 if (currentClockCycle
									 >= bankStates[nextrank][nextbank].nextRead) {
									 *busPacket = readqueue[i];//if should issue read request from readqueue
									 readqueue.erase(
									 readqueue.begin() + i);
									 issuable = true;
									 }
									 }*/
									found = true;
									break;
								}
							}
							if (!found) {
								for (unsigned i = 0; i < writequeue.size();
										i++) { //assure no dangling WRITE
									//if there is something going to that bank and row, then we don't want to send a PRE,if should leave it to cancelwrite?
									if (writequeue[i]->bank == nextbank
											&& writequeue[i]->row
													== bankStates[nextrank][nextbank].openRowAddress) {
										/*										if (writequeue[i]->busPacketType
										 != ACTIVATE) {
										 if (currentClockCycle
										 >= bankStates[nextrank][nextbank].nextWrite) {
										 *busPacket = writequeue[i];
										 writequeue.erase(
										 writequeue.begin() + i);
										 return true;
										 }
										 }*/
										found = true;
										break;
									}
								}
							}
							if (!issuable && (!found)
									&& (currentClockCycle
											>= bankStates[nextrank][nextbank].nextPrecharge)) {
								*busPacket = new BusPacket(PRECHARGE, 0, 0, 0,
										nextrank, nextbank, 0, dramsim_log);
//								PRINT(
//										"clock "<<currentClockCycle<<" due full psQ pre r["<<nextrank<<"] b["<<nextbank<<"]");
								issuable = true;
							}
						}
					}
					if (issuable) {
						if ((*busPacket)->busPacketType == WRITE) {
							(*busPacket)->busPacketType = FullSET;
							countPSQsetperBank[(*busPacket)->rank][(*busPacket)->bank]++;
						}
						if (PSqueues[nextrank][nextbank].size()
								>= PARTIAL_QUEUE_DEPTH) {
							isFull[nextrank][nextbank] = true;
						} else {
							isFull[nextrank][nextbank] = false;
						}
						return true;
					} else {
						return false;
					}
				}
			} else {			// if the oldest reach the retention time
				if (bankStates[nextrank][nextbank].currentBankState == Idle) {
					//just issue PartialSET from the PartialSET Queue, out-of-order
					if (currentClockCycle
							>= bankStates[nextrank][nextbank].nextActivate) {
						for (size_t i = 0; i < psqueue.size(); i++) {
							if (psqueue[i]->elapsedTime < 1.0e9) {
								break;
							}
							if (psqueue[i]->busPacket->busPacketType
									== ACTIVATE) {
								*busPacket = new BusPacket(ACTIVATE,
										psqueue[i]->busPacket->physicalAddress,
										psqueue[i]->busPacket->column,
										psqueue[i]->busPacket->row,
										psqueue[i]->busPacket->rank,
										psqueue[i]->busPacket->bank,
										psqueue[i]->busPacket->data,
										dramsim_log,
										psqueue[i]->busPacket->RIP);
								delete (psqueue[i]);
								psqueue.erase(psqueue.begin() + i);
								if (PSqueues[nextrank][nextbank].size()
										>= PARTIAL_QUEUE_DEPTH) {
									isFull[nextrank][nextbank] = true;
								} else {
									isFull[nextrank][nextbank] = false;
								}
								return true;
							}
						}
					}
				} else if (bankStates[nextrank][nextbank].currentBankState
						== RowActive) {
					//check the writeQueue and readQueue ,if there is same open row, issue the request and Precharge.
					//or cancel the request and Precharge
					bool foundopen = false;
					for (size_t i = 0; i < psqueue.size(); i++) {
						Entry* entry = psqueue[i];
						if (entry->busPacket->row
								== bankStates[nextrank][nextbank].openRowAddress) {
							foundopen = true;
							if (entry->elapsedTime < 1.0e9) {
								break;
							}
							if (currentClockCycle
									>= bankStates[nextrank][nextbank].nextWrite) {
								if (psqueue[i]->busPacket->busPacketType
										== ACTIVATE
										&& psqueue[i + 1]->busPacket->row
												== entry->busPacket->row) {
									*busPacket =
											new BusPacket(WRITE,
													psqueue[i + 1]->busPacket->physicalAddress,
													psqueue[i + 1]->busPacket->column,
													psqueue[i + 1]->busPacket->row,
													psqueue[i + 1]->busPacket->rank,
													psqueue[i + 1]->busPacket->bank,
													psqueue[i + 1]->busPacket->data,
													dramsim_log,
													psqueue[i + 1]->busPacket->RIP);
									delete (psqueue[i]);
									delete (psqueue[i + 1]);
									psqueue.erase(psqueue.begin() + i,
											psqueue.begin() + i + 2);
								} else { // there's no activate before this packet
									//or just remove the one bus packet
									*busPacket =
											new BusPacket(WRITE,
													psqueue[i]->busPacket->physicalAddress,
													psqueue[i]->busPacket->column,
													psqueue[i]->busPacket->row,
													psqueue[i]->busPacket->rank,
													psqueue[i]->busPacket->bank,
													psqueue[i]->busPacket->data,
													dramsim_log,
													psqueue[i]->busPacket->RIP);
									delete (psqueue[i]);
									psqueue.erase(psqueue.begin() + i);
								}
								psqueue.push_back(
										new Entry(
												new BusPacket(ACTIVATE,
														(*busPacket)->physicalAddress,
														(*busPacket)->column,
														(*busPacket)->row,
														(*busPacket)->rank,
														(*busPacket)->bank,
														(*busPacket)->data,
														dramsim_log,
														(*busPacket)->RIP)));
								psqueue.push_back(
										new Entry(
												new BusPacket(WRITE,
														(*busPacket)->physicalAddress,
														(*busPacket)->column,
														(*busPacket)->row,
														(*busPacket)->rank,
														(*busPacket)->bank,
														(*busPacket)->data,
														dramsim_log,
														(*busPacket)->RIP)));
//								PRINTN("due retention psQ act ");
//								(*busPacket)->print();
								(*busPacket)->busPacketType = PartialSET;
								countPSQpartialsetperBank[(*busPacket)->rank][(*busPacket)->bank]++;
								if (PSqueues[nextrank][nextbank].size()
										>= PARTIAL_QUEUE_DEPTH) {
									isFull[(*busPacket)->rank][(*busPacket)->bank] = true;
								} else {
									isFull[(*busPacket)->rank][(*busPacket)->bank] = false;
								}
								return true;
							}
						}
					}
					//check the writeQueue and readQueue ,if there is same open row, issue the request and Precharge.
					//or cancel the request and Precharge
					if (!foundopen) {
						if (currentClockCycle
								>= bankStates[nextrank][nextbank].nextPrecharge) {
							*busPacket = new BusPacket(PRECHARGE, 0, 0, 0,
									nextrank, nextbank, 0, dramsim_log);
//							PRINT(
//									"due retention psQ pre r["<<nextrank<<"] b["<<nextbank<<"]");
							vector<BusPacket *> &writequeue =
									cancelWrite.writeQueue.getCommandQueue(
											nextrank, nextbank);
							vector<BusPacket *> &readqueue =
									cancelWrite.readQueue.getCommandQueue(
											nextrank, nextbank);
							bool found = false;
							for (unsigned r = 0; r < readqueue.size(); r++) { //assure no dangling READ
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
										w++) { //assure no dangling WRITE,since the retention entry has the highest priority
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
										found = true;
										break;
									}
								} //end for writequeue
							} // !found
							return true;
						} //nextPrecharge
					} //!foundopen
				} //rowactive
			} //RETENTIONTIME
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
			if ((bankStates[i][j].currentBankState == Idle)
					|| (bankStates[i][j].currentBankState == PowerDown)) {
				if (idle[i][j] == false) {
					idle[i][j] = true;
					begin[i][j] = currentClockCycle;
				}
			} else {
				if (idle[i][j] == true) {
					duration[i][j] = currentClockCycle - begin[i][j];
					if (predictTable[i][j].size() != 0) {
						if (duration[i][j] >= WRITE_TO_READ_DELAY_B) {
							predictTable[i][j][predictTable[i][j].size() - 1]->idleInterval.push_back(
									duration[i][j]);
						} else {
							predictTable[i][j][predictTable[i][j].size() - 1]->idleInterval.push_back(
									duration[i][j]);
						}
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
	if (!predictbank.empty()) {
		for (size_t i = 0; i < predictbank.size(); i++) {
			if (predictbank[i]->RIP == rip) { //the latest access of the same PC is at the tail
				predictbank[i]->timeAccess = currentClockCycle;
				predictbank[i]->address = addr;
				found = true;
				predictbank.push_back(new PredictionEntry(*(predictbank[i])));
				predictbank.erase(predictbank.begin() + i);
				break;
			}
		}
		if (!found) {
			predictbank.push_back(
					new PredictionEntry(rip, addr, currentClockCycle));
		}
//		sort(predictbank.begin(), predictbank.end(), PredictionEntry::compare); //the newest entry is at the beginning;
	} else {
		predictbank.push_back(
				new PredictionEntry(rip, addr, currentClockCycle));
	}
	//DEBUG
	/*	PRINTN(
	 "Cycle "<<currentClockCycle<<" R "<<rank << " B "<<bank<<" predictTable size: "<< predictbank.size()<<" entry is ");
	 PRINTN("PredictionEntry RIP : " <<hex<< rip<<dec);
	 PRINTN(" address is 0x" << hex<<addr<<dec);
	 PRINTN(" last access  is " << currentClockCycle);
	 PRINTN(" idleInterval is [");
	 for (size_t i = 0;
	 i != predictbank[predictbank.size() - 1]->idleInterval.size();
	 i++) {
	 PRINTN(" " << predictbank[predictbank.size() - 1]->idleInterval[i]);
	 }
	 PRINT(" ]");*/
	//predictbank[predictbank.size() - 1]->print();
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
				if (PSqueues[i][j].size() == 0) {
					continue;
				}
				PRINT("    Bank "<< j << "   size : " << PSqueues[i][j].size());
				for (size_t k = 0; k < PSqueues[i][j].size(); k++) {
					PRINTN(
							"       " << k << "] elapsed Time is "<<PSqueues[i][j][k]->elapsedTime<<" ");
					PSqueues[i][j][k]->busPacket->print();

				}
			}
		}
	}
}
void PartialSETQueue::printIdletable() {
	/*	if (queuingStructure == PerRank) {
	 PRINT(endl << "== Printing Per Rank Queue");
	 for (size_t i = 0; i < NUM_RANKS; i++) {
	 PRINT(" = Rank " << i << "  size : " << PSqueues[i][0].size());
	 for (size_t j = 0; j < PSqueues[i][0].size(); j++) {
	 PRINTN("    "<< j << "]");
	 PSqueues[i][0][j]->busPacket->print();
	 PRINT("elapsed Time is "<<PSqueues[i][0][j]->elapsedTime);
	 }
	 }
	 } else*/if (queuingStructure == PerRankPerBank) {
		PRINT("\n== Printing Per Rank, Per Bank Queue");
		for (size_t i = 0; i < NUM_RANKS; i++) {
			PRINT(" = Rank " << i);
			for (size_t j = 0; j < NUM_BANKS; j++) {
				if (predictTable[i][j].size() == 0) {
					continue;
				}
				PRINT(" Bank "<< j << " size : " << predictTable[i][j].size());
				for (size_t k = 0; k < predictTable[i][j].size(); k++) {
					vector<PredictionEntry *> &predictbank = predictTable[i][j];
					PRINTN(
							" ["<<k<<"]"<<" RIP : " <<hex<<predictbank[k]->RIP<<dec);
					PRINTN(" idleInterval : [ ");
					for (size_t d = 0; d != predictbank[k]->idleInterval.size();
							d++) {
						PRINTN(predictbank[k]->idleInterval[d]<< ":");
					}
					PRINT(" ]");
				}
			}
		}
	}
}

void PartialSETQueue::update() {
//needed for SimulatorObject
	step();
	Entry1D::iterator iter;
	if (queuingStructure == PerRankPerBank) {
		unsigned r = NUM_RANKS;
		unsigned b = NUM_BANKS;
		for (unsigned i = 0; i < r; ++i) {
			for (unsigned j = 0; j < b; ++j) {
				if (PSqueues[i][j].size() == 0) {
					continue;
				}
				for (iter = PSqueues[i][j].begin();
						iter != PSqueues[i][j].end(); ++iter) {
					Entry *tmp = *iter;
					tmp->elapsedTime++;
				}
			}
		}
	}
	//getIdleInterval();
}

