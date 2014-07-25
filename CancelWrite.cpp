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
				0), maxToken(DEVICE_WIDTH) {
	currentClockCycle = 0;
	writecancel = vector < vector<bool>
			> (NUM_RANKS, vector<bool>(NUM_BANKS, false));
//	readrequest = vector < vector<uint64_t>
//			> (NUM_RANKS, vector<uint64_t>(NUM_BANKS, 0));
	coutcanceledwrite = vector < vector<uint64_t>
			> (NUM_RANKS, vector < uint64_t > (NUM_BANKS, 0)); //record the Count of Canceled Write
	ongoingWrite = vector < vector<BusPacket *>
			> (NUM_RANKS, vector<BusPacket *>(NUM_BANKS, NULL));
//	canceledWrite =  NULL;
	writepriority = vector < vector<bool>
			> (NUM_RANKS, vector<bool>(NUM_BANKS, false));
	////64bit data, from low to high mapped to chip 0-7
	tokenpool = vector < vector<unsigned>
			> (NUM_DEVICES, vector<unsigned>(NUM_BANKS, maxToken));
	tokencountdown = vector < vector<unsigned>
			> (NUM_DEVICES, vector<unsigned>(NUM_BANKS, WRITE_TO_PRE_DELAY));
}
CancelWrite::~CancelWrite() {
	for (size_t r = 0; r < NUM_RANKS; r++) {
		ongoingWrite[r].clear();
		writecancel[r].clear();
		writepriority[r].clear();
	}
}
bool CancelWrite::addRequest(Transaction *transaction, BusPacket *buspacket,
		bool &found) {
	if (transaction->transactionType == DATA_READ) {
		vector<BusPacket*> &wrqueue = writeQueue.getCommandQueue(
				buspacket->rank, buspacket->bank);
		//check the partial queue at the same time
		for (unsigned i = 0; i < wrqueue.size(); i++) {
			BusPacket *packet = wrqueue[i];
			if (packet->physicalAddress == transaction->address) { //if write queue has the same address, then the read request can be returned
				transaction->transactionType = RETURN_DATA; //MC got the returned data from write queue.
				if (wrqueue[i]->busPacketType == ACTIVATE && wrqueue.size() > 1
						&& i < wrqueue.size() - 1
						&& wrqueue[i + 1]->physicalAddress
								== packet->physicalAddress) {
					transaction->data = wrqueue[i + 1]->data;
				} else {
					transaction->data = packet->data;
				}
				found = true;
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
				if (queue[i]->busPacketType == ACTIVATE && queue.size() > 1
						&& i < queue.size() - 1
						&& queue[i + 1]->physicalAddress
								== packet->physicalAddress) { // the write request is paired
					queue[i]->data = transaction->data;
					queue[i + 1]->data = transaction->data;
					// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
				} else {
					//the activated has popped.
					queue[i]->data = transaction->data;
				}
				break;
			}
		}
		if (found) {
//			writerequest[buspacket->rank][buspacket->bank]++;
			return true;
		}
		//every bank pending one request.
		if (writeQueue.hasRoomFor(2, buspacket->rank, buspacket->bank)) {
			writeQueue.enqueue(
					new BusPacket(ACTIVATE, buspacket->physicalAddress,
							buspacket->column, buspacket->row, buspacket->rank,
							buspacket->bank, buspacket->data, dramsim_log,
							buspacket->RIP));
			writeQueue.enqueue(
					new BusPacket(WRITE, buspacket->physicalAddress,
							buspacket->column, buspacket->row, buspacket->rank,
							buspacket->bank, buspacket->data, dramsim_log,
							buspacket->RIP));
			return true;
		} else {
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
		if (requestQueue.isIssuable(request)) { //check for the timing constraint
			busPacket = new BusPacket(request->busPacketType,
					request->physicalAddress, request->column, request->row,
					request->rank, request->bank, request->data, dramsim_log,
					request->RIP);
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
				queue.erase(queue.begin() + i);
			}
			issuable = true;
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
					if (ongoingWrite[r][b] != NULL
							&& delta > (0.75 * WRITE_TO_READ_DELAY_B)) {
						//modify all timing for all Bank
						bankStates[r][b].nextPrecharge -= delta;
						bankStates[r][b].nextWrite -= delta;
						bankStates[r][b].nextRead -= delta;
						//2.modify the cycle of Rank;
						(*ranks)[r]->bankStates[b].nextPrecharge -= delta;
						(*ranks)[r]->bankStates[b].nextRead -= delta;
						(*ranks)[r]->bankStates[b].nextWrite -= delta;
					}
				}
				break;
			}
		}
	}
	return;
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
			if (!(writequeue.empty() && readqueue.empty())) {
				if (!writepriority[nextRank][nextBank]) {	//then read priority
					if (writequeue.size() > 8) {
						writepriority[nextRank][nextBank] = true;
					}
				}
				if (writepriority[nextRank][nextBank]) {
					/*				if (writequeue.size() <= 8 && !readqueue.empty()) {
					 writepriority[nextRank][nextBank] = false;
					 }*/
					if (!readqueue.empty() && writequeue.size() < 16) {	//prioritized the ReadRequest and Cancel the on-going write
						writepriority[nextRank][nextBank] = false;
						issueWC(nextRank, nextBank); //change the timing of Rank and Bank to issue the ReadRequest
						coutcanceledwrite[nextRank][nextBank]++; //record the Count of Canceled Write
						writecancel[nextRank][nextBank] = true;
						issueRead = issueRequest(nextRank, nextBank, *busPacket,
								readQueue);
						if (issueRead) {
							if (bankStates[nextRank][nextBank].lastCommand
									== WRITE
									&& ongoingWrite[nextRank][nextBank]
											!= NULL) {
//							PRINTN(
//									"clock "<<currentClockCycle<<" writecancel! write 0x"<<hex<<ongoingWrite[nextRank][nextBank]->physicalAddress<<dec<<" r["<<nextRank<<"] b["<<nextBank<<"] ");
								//only these commands have an implicit state change
								vector<BusPacket*>::iterator it =
										writequeue.begin();
								writequeue.insert(it,
										new BusPacket(WRITE,
												ongoingWrite[nextRank][nextBank]->physicalAddress,
												ongoingWrite[nextRank][nextBank]->column,
												ongoingWrite[nextRank][nextBank]->row,
												ongoingWrite[nextRank][nextBank]->rank,
												ongoingWrite[nextRank][nextBank]->bank,
												ongoingWrite[nextRank][nextBank]->data,
												dramsim_log,
												ongoingWrite[nextRank][nextBank]->RIP));
								it = writequeue.begin();
								writequeue.insert(it,
										new BusPacket(ACTIVATE,
												ongoingWrite[nextRank][nextBank]->physicalAddress,
												ongoingWrite[nextRank][nextBank]->column,
												ongoingWrite[nextRank][nextBank]->row,
												ongoingWrite[nextRank][nextBank]->rank,
												ongoingWrite[nextRank][nextBank]->bank,
												ongoingWrite[nextRank][nextBank]->data,
												dramsim_log,
												ongoingWrite[nextRank][nextBank]->RIP));
								delete ongoingWrite[nextRank][nextBank];
								ongoingWrite[nextRank][nextBank] = NULL;
							}
						}
					} else {
						issueWrite = issueRequest(nextRank, nextBank,
								*busPacket, writeQueue);
					}
				} else {				//writepriority is false!
					if (!readqueue.empty()) {
						writepriority[nextRank][nextBank] = false;
						issueRead = issueRequest(nextRank, nextBank, *busPacket,
								readQueue);
					} else {
						if (bankStates[nextRank][nextBank].currentBankState
								== Idle && !writequeue.empty()) {
							issueWrite = issueRequest(nextRank, nextBank,
									*busPacket, writeQueue);
						}
					}
				}
				if (issueWrite || issueRead) {
					getToken(*busPacket);
					return true;
				}
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
				if (!(writequeue.empty() && readqueue.empty())) {
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
								for (unsigned i = 0; i < readqueue.size();
										i++) {
									//if there is something going to that bank and row, then we don't want to send a PRE
									if (readqueue[i]->bank == nextBankPRE
											&& readqueue[i]->row
													== bankStates[nextRankPRE][nextBankPRE].openRowAddress) {
										if (readqueue[i]->busPacketType
												!= ACTIVATE) {
											vector<BusPacket*>::iterator it =
													readqueue.begin() + i;
											BusPacket *bpacket =
													new BusPacket(ACTIVATE,
															readqueue[i]->physicalAddress,
															readqueue[i]->column,
															readqueue[i]->row,
															readqueue[i]->rank,
															readqueue[i]->bank,
															readqueue[i]->data,
															dramsim_log,
															readqueue[i]->RIP);
											readqueue.insert(it, bpacket);
//										PRINT(
//												"writeP, write no found, rollback act to read 0x" << hex << bpacket->physicalAddress <<dec<<" r["<<nextRankPRE<<"]b["<<nextBankPRE<<"] ");
										}
										break;
									}
								}
								if (currentClockCycle
										>= bankStates[nextRankPRE][nextBankPRE].nextPrecharge) {
									sendingPRE = true;
									*busPacket = new BusPacket(PRECHARGE, 0, 0,
											0, nextRankPRE, nextBankPRE, 0,
											dramsim_log);
//								PRINT("writeP, write no found, set PRE r["<<nextRankPRE<<"]b["<<nextBankPRE<<"] ");
									return true;
								}
							}
						} else {
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
								if (!readqueue.empty()) {
									for (unsigned i = 0; i < writequeue.size();
											i++) {
										//if there is something going to that bank and row, then we don't want to send a PRE
										if (writequeue[i]->bank == nextBankPRE
												&& writequeue[i]->row
														== bankStates[nextRankPRE][nextBankPRE].openRowAddress) {
											//change to add a ACT to this write and PRE This bank
											if (writequeue[i]->busPacketType
													!= ACTIVATE) {
												vector<BusPacket*>::iterator it =
														writequeue.begin() + i;
												BusPacket *bpacket =
														new BusPacket(ACTIVATE,
																writequeue[i]->physicalAddress,
																writequeue[i]->column,
																writequeue[i]->row,
																writequeue[i]->rank,
																writequeue[i]->bank,
																writequeue[i]->data,
																dramsim_log,
																writequeue[i]->RIP);
												writequeue.insert(it, bpacket);
//											PRINT(
//													"readP, read not empty, not found in read, rollback act to write 0x" << hex << bpacket->physicalAddress <<dec<<" r["<<nextRankPRE<<"]b["<<nextBankPRE<<"] ");
											}
											break;
										}
									}
									if (currentClockCycle
											>= bankStates[nextRankPRE][nextBankPRE].nextPrecharge) {
										sendingPRE = true;
										*busPacket = new BusPacket(PRECHARGE, 0,
												0, 0, nextRankPRE, nextBankPRE,
												0, dramsim_log);
//									PRINTN(
//											"readP, read not empty, not found in read, set PRE r["<<nextRankPRE<<"]b["<<nextBankPRE<<"] ");
										return true;
									}
								} else {
									for (unsigned i = 0; i < writequeue.size();
											i++) { //check if there is issuable write request
										if (writequeue[i]->bank == nextBankPRE
												&& writequeue[i]->row
														== bankStates[nextRankPRE][nextBankPRE].openRowAddress) {
											found = true;   //then not issue PRE
											if (currentClockCycle
													>= bankStates[nextRankPRE][nextBankPRE].nextWrite) {
												/*
												 if the bus packet i is an activate, and also the next packet i+1 is write,
												 then we pop i and i+1, return i+1 packet
												 */
												*busPacket =
														new BusPacket(WRITE,
																writequeue[i]->physicalAddress,
																writequeue[i]->column,
																writequeue[i]->row,
																writequeue[i]->rank,
																writequeue[i]->bank,
																writequeue[i]->data,
																dramsim_log,
																writequeue[i]->RIP);
												if (writequeue[i]->busPacketType
														== ACTIVATE
														&& writequeue[i + 1]->row
																== bankStates[nextRankPRE][nextBankPRE].openRowAddress) {
													// i is being returned, but i-1 is being thrown away, so must delete it here
													delete (writequeue[i + 1]);
													delete (writequeue[i]);
													// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
													writequeue.erase(
															writequeue.begin()
																	+ i,
															writequeue.begin()
																	+ i + 2);
												} else {
													// there's no activate before this packet
													//or just remove the one bus packet
													writequeue.erase(
															writequeue.begin()
																	+ i);
												}
//												PRINT(
//														"clock "<<currentClockCycle<<" readP, read empty, emit write 0x" << hex << (*busPacket)->physicalAddress << dec<<" r["<<nextRankPRE<<"]b["<<nextBankPRE<<"] "<<" ongoing NULL");
												/*												delete ongoingWrite[nextRankPRE][nextBankPRE];
												 ongoingWrite[nextRankPRE][nextBankPRE] =
												 NULL;*/
												return true;
											}
											break;
										}
									}
									if (!found) {
										//issue PRE
										if (currentClockCycle
												>= bankStates[nextRankPRE][nextBankPRE].nextPrecharge) {
											sendingPRE = true;
											*busPacket = new BusPacket(
													PRECHARGE, 0, 0, 0,
													nextRankPRE, nextBankPRE, 0,
													dramsim_log);
											return true;
										}
									}
								}

							}
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
}
bool CancelWrite::isEmpty(unsigned r) {
	if (writeQueue.isEmpty(r) && readQueue.isEmpty(r)) {
		return true;
	} else
		return false;
}
void CancelWrite::getToken(BusPacket *buspacket) {
	unsigned r = buspacket->rank;
	unsigned b = buspacket->bank;
	//64bit data, from low to high mapped to chip 0-7
	vector<unsigned> demandedtokenperChip = vector<unsigned>(NUM_DEVICES, 0);
	BusPacket* bankpacket;
	uint64_t olddata, newdata;;
	uint64_t writtendata, tmp;
	unsigned setbit, resetbit;
	newdata = (uint64_t) (buspacket->data);
	(*ranks)[r]->banks[b].read(bankpacket);
	olddata = (uint64_t) (bankpacket->data);
	writtendata = olddata ^ newdata;
	tmp = writtendata;
//actually this is different for each chip.
	unsigned bit_width = JEDEC_DATA_BUS_BITS / DEVICE_WIDTH;
	for (size_t i = 0; i < NUM_DEVICES; i++) {
		setbit = 0;
		resetbit = 0;
		for (size_t d = 0; d < bit_width; d++) {
			tmp = tmp >> d;
			if (writtendata & 1)
				setbit++;
			else
				resetbit++;
		}
		demandedtokenperChip[i] = setbit / 2 + resetbit;
		tokenpool[i][b] = tokenpool[i][b] - demandedtokenperChip[i];
	}
}
void CancelWrite::releaseToken() {
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t i = 0; i < NUM_DEVICES; i++) {
			for (size_t b = 0; b < NUM_BANKS; b++) {
				if (ongoingWrite[r][b] != NULL && tokencountdown[i][b] != 0) {
					tokencountdown[i][b]--;
				}
				if (tokencountdown[i][b] <= 0) {
					tokenpool[i][b] = maxToken;
					tokencountdown[i][b] = WRITE_TO_PRE_DELAY;
				}
			}
		}
	}
}
bool CancelWrite::powerAllowable() {
	return false;
}
void CancelWrite::update() {
	writeQueue.step();
	readQueue.step();
	step();
}
/*void CancelWrite::print(){

 }*/
