/*********************************************************************************
 *  Copyright (c) 2010-2011, Elliott Cooper-Balis
 *                             Paul Rosenfeld
 *                             Bruce Jacob
 *                             University of Maryland
 *                             dramninjas [at] gmail [dot] com
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice,
 *        this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright notice,
 *        this list of conditions and the following disclaimer in the documentation
 *        and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

//MemoryController.cpp
//
//Class file for memory controller object
//
#include "MemoryController.h"
#include "MemorySystem.h"
#include "AddressMapping.h"

#define SEQUENTIAL(rank,bank) (rank*NUM_BANKS)+bank

/* Power computations are localized to MemoryController.cpp */
extern unsigned IDD0;
extern unsigned IDD1;
extern unsigned IDD2P;
extern unsigned IDD2Q;
extern unsigned IDD2N;
extern unsigned IDD3Pf;
extern unsigned IDD3Ps;
extern unsigned IDD3N;
extern unsigned IDD4W;
extern unsigned IDD4R;
extern unsigned IDD5;
extern unsigned IDD6;
extern unsigned IDD6L;
extern unsigned IDD7;
extern float Vdd;
extern double readEnergyperCell; //pj
//extern double RETENTION_TIME;

using namespace DRAMSim;

MemoryController::MemoryController(MemorySystem *parent, CSVWriter &csvOut_,
		ostream &dramsim_log_) :
		dramsim_log(dramsim_log_), bankStates(NUM_RANKS,
				vector < BankState > (NUM_BANKS, dramsim_log)), commandQueue(
				bankStates, dramsim_log), cancelwriteQueue(bankStates,
				dramsim_log, *&ranks), poppedBusPacket(NULL), csvOut(csvOut_), totalTransactions(
				0), addedRdTrans(0), addedWrTrans(0), refreshRank(0) {
	//get handle on parent
	parentMemorySystem = parent;

	//bus related fields
	outgoingCmdPacket = NULL;
	outgoingDataPacket = NULL;
	dataCyclesLeft = 0;
	cmdCyclesLeft = 0;
	//set here to avoid compile errors
	currentClockCycle = 0;

	//reserve memory for vectors
	transactionQueue.reserve(TRANS_QUEUE_DEPTH);
	powerDown = vector<bool>(NUM_RANKS, false);
	grandTotalBankAccesses = vector < uint64_t > (NUM_RANKS * NUM_BANKS, 0);
	totalReadsPerBank = vector < uint64_t > (NUM_RANKS * NUM_BANKS, 0);
	totalWritesPerBank = vector < uint64_t > (NUM_RANKS * NUM_BANKS, 0);
	totalReadsPerRank = vector < uint64_t > (NUM_RANKS, 0);
	totalWritesPerRank = vector < uint64_t > (NUM_RANKS, 0);

	totalActPerBank = vector < uint64_t > (NUM_RANKS * NUM_BANKS, 0);
	totalPrePerBank = vector < uint64_t > (NUM_RANKS * NUM_BANKS, 0);
	totalActPerRank = vector < uint64_t > (NUM_RANKS, 0);
	totalPrePerRank = vector < uint64_t > (NUM_RANKS, 0);

	writeDataCountdown.reserve(NUM_RANKS);
	writeDataToSend.reserve(NUM_RANKS);
	refreshCountdown.reserve(NUM_RANKS);

	//Power related packets
	backgroundEnergy = vector < uint64_t > (NUM_RANKS, 0);
	burstEnergy = vector < uint64_t > (NUM_RANKS, 0);
	actpreEnergy = vector < uint64_t > (NUM_RANKS, 0);
	refreshEnergy = vector < uint64_t > (NUM_RANKS, 0);

	writeEnergyperBank = vector<double>(NUM_RANKS * NUM_BANKS, 0.0);

	ripaccessPerBank = vector<vector<RipAccesscount*> >(); // vector< vector<> > T= vector< vector<> >();
	addraccessPerBank = vector<vector<RipAccesscount*> >();
	totalEpochLatency = vector < uint64_t > (NUM_RANKS * NUM_BANKS, 0);

	//staggers when each rank is due for a refresh
	for (size_t i = 0; i < NUM_RANKS; i++) {
		refreshCountdown.push_back(
				(int) ((REFRESH_PERIOD / tCK) / NUM_RANKS) * (i + 1));
		for (size_t j = 0; j < NUM_BANKS; j++) {
			ripaccessPerBank.push_back(vector<RipAccesscount*>());
			addraccessPerBank.push_back(vector<RipAccesscount*>());

		}

	}
}

//get a bus packet from either data or cmd bus
void MemoryController::receiveFromBus(BusPacket *bpacket) {
	if (bpacket->busPacketType != DATA) {
		ERROR(
				"== Error - Memory Controller received a non-DATA bus packet from rank");
		bpacket->print();
		exit(0);
	}

	if (DEBUG_BUS) {
		PRINTN(" -- MC Receiving From Data Bus : ");
		bpacket->print();
	}

	//add to return read data queue
	returnTransaction.push_back(
			new Transaction(RETURN_DATA, bpacket->physicalAddress,
					bpacket->RIP));
	totalReadsPerBank[SEQUENTIAL(bpacket->rank,bpacket->bank)]++;

	// this delete statement saves a mindboggling amount of memory
	delete (bpacket);
}

//sends read data back to the CPU
void MemoryController::returnReadData(const Transaction *trans) {
	if (parentMemorySystem->ReturnReadData != NULL) {
		(*parentMemorySystem->ReturnReadData)(parentMemorySystem->systemID,
				trans->address, currentClockCycle);
	}
}

//gives the memory controller a handle on the rank objects
void MemoryController::attachRanks(vector<Rank *> *ranks) {
	this->ranks = ranks;
}

//memory controller update
void MemoryController::update() {

	//PRINT(" ------------------------- [" << currentClockCycle << "] -------------------------");
	//update bank states
	//PRINT("STEP 0");
	for (size_t i = 0; i < NUM_RANKS; i++) {
		for (size_t j = 0; j < NUM_BANKS; j++) {
//			if (bankStates[i][j].lastCommand == WRITE) {
			uint64_t addr = 0;
			if (parentMemorySystem->WriteDataDone != NULL) {
				if (cancelwriteQueue.tokenRank->release(i, j, addr)) {
//							PRINT("rank "<<i<<" bank "<<j <<"clock "<<currentClockCycle<<" WRITE ACK is 0x"<<hex<<addr<<dec<<" then ongoing NULL");
					(*parentMemorySystem->WriteDataDone)(
							parentMemorySystem->systemID, addr,
							currentClockCycle);
				}
			}
			if (bankStates[i][j].stateChangeCountdown > 0) {
				//decrement counters
				bankStates[i][j].stateChangeCountdown--;
				//if counter has reached 0, change state
				if (bankStates[i][j].stateChangeCountdown == 0) {
					switch (bankStates[i][j].lastCommand) {
					//only these commands have an implicit state change
					case WRITE_P:
					case READ_P:
						bankStates[i][j].currentBankState = Precharging;
						bankStates[i][j].lastCommand = PRECHARGE;
						bankStates[i][j].stateChangeCountdown = tRP;
						break;
					case REFRESH:
					case PRECHARGE:
						bankStates[i][j].currentBankState = Idle;
						break;
					default:
						break;
					}
				}
			}
		}
	}
	//PRINT("STEP 1");
	//check for outgoing command packets and handle countdowns
	if (outgoingCmdPacket != NULL) {
		cmdCyclesLeft--;
		if (cmdCyclesLeft == 0) //packet is ready to be received by rank
				{
			(*ranks)[outgoingCmdPacket->rank]->receiveFromBus(
					outgoingCmdPacket);
			outgoingCmdPacket = NULL;
		}
	}

//check for outgoing data packets and handle countdowns
	if (outgoingDataPacket != NULL) {
		dataCyclesLeft--;
		if (dataCyclesLeft == 0) {
			//inform upper levels that a write is done
			//if lastCommand is write but WriteCancel,then not return;
//			if (!WRITECANCEL) {
//				if (parentMemorySystem->WriteDataDone != NULL) {
//					(*parentMemorySystem->WriteDataDone)(
//							parentMemorySystem->systemID,
//							outgoingDataPacket->physicalAddress,
//							currentClockCycle);
//					PRINTN("Write Done ");
//					outgoingDataPacket->print();
//				}
//			}
			(*ranks)[outgoingDataPacket->rank]->receiveFromBus(
					outgoingDataPacket);
			outgoingDataPacket = NULL;
		}
	}
//if any outstanding write data needs to be sent
//and the appropriate amount of time has passed (WL)
//then send data on bus
//
//write data held in fifo vector along with countdowns
	if (writeDataCountdown.size() > 0) {
		for (size_t i = 0; i < writeDataCountdown.size(); i++) {
			writeDataCountdown[i]--;
		}

		if (writeDataCountdown[0] == 0) {
			//send to bus and print debug stuff
			if (DEBUG_BUS) {
				PRINTN(" -- MC Issuing On Data Bus    : ");
				writeDataToSend[0]->print();
			}

			// queue up the packet to be sent
			if (outgoingDataPacket != NULL) {
				ERROR("== Error - Data Bus Collision");
				exit(-1);
			}

			outgoingDataPacket = writeDataToSend[0];
			dataCyclesLeft = BL / 2;

			totalTransactions++;
			totalWritesPerBank[SEQUENTIAL(writeDataToSend[0]->rank,writeDataToSend[0]->bank)]++;

			writeDataCountdown.erase(writeDataCountdown.begin());
			writeDataToSend.erase(writeDataToSend.begin());
		}
	}

//if its time for a refresh issue a refresh
// else pop from command queue if it's not empty
	if (refreshCountdown[refreshRank] == 0) {
		//comment refresh out only count it down never refresh
		commandQueue.needRefresh(refreshRank);
//		(*ranks)[refreshRank]->refreshWaiting = true;
		refreshCountdown[refreshRank] = REFRESH_PERIOD / tCK;
		refreshRank++;
		if (refreshRank == NUM_RANKS) {
			refreshRank = 0;
		}
	}
//if a rank is powered down, make sure we power it up in time for a refresh
	else if (powerDown[refreshRank] && refreshCountdown[refreshRank] <= tXP) {
//		(*ranks)[refreshRank]->refreshWaiting = true;
	}

//pass a pointer to a poppedBusPacket
//function returns true if there is something valid in poppedBusPacket
	//PRINT("STEP 2");
//	psQueue.getIdleInterval();
	bool popWCQueue = false;
	bool popqueue = false;
	BusPacket *poppedWCPacket;
	popWCQueue = cancelwriteQueue.issue(&poppedWCPacket); //then we could know if the last write is canceled.
	popqueue = popWCQueue;
	poppedBusPacket = poppedWCPacket;
	if (popqueue) {
		/*		psQueue.iniPredictTable(poppedBusPacket->rank, poppedBusPacket->bank,
		 poppedBusPacket->physicalAddress, poppedBusPacket->RIP);*/
		unsigned rank = poppedBusPacket->rank;
		unsigned bank = poppedBusPacket->bank;
		if (poppedBusPacket->busPacketType == WRITE
				|| poppedBusPacket->busPacketType == WRITE_P) {
			BusPacket* bp = new BusPacket(DATA,
					poppedBusPacket->physicalAddress, poppedBusPacket->column,
					poppedBusPacket->row, rank, bank,
					poppedBusPacket->dataPacket, dramsim_log,
					poppedBusPacket->RIP);
			writeDataToSend.push_back(bp);
			writeDataCountdown.push_back(WL);
		}
		//
		//update each bank's state based on the command that was just popped out of the command queue
		//
		//for readability's sake
//		PRINTN("MC update ");
//		poppedBusPacket->print();
//				bankStates[rank][bank].print();
//		uint64_t newdata = poppedBusPacket->dataPacket->getData();
//		uint64_t oldata = poppedBusPacket->dataPacket->getoldData();
//		uint64_t writtendata = newdata ^ oldata;
		switch (poppedBusPacket->busPacketType) {
		case READ_P:
		case READ:
			//add energy to account for total
			accessCount(rank, bank, poppedBusPacket->RIP,
					poppedBusPacket->physicalAddress, 1);
			bankStates[rank][bank].openRowAddress = poppedBusPacket->row;
			bankStates[rank][bank].currentBankState = RowActive;
			if (DEBUG_POWER) {
				PRINT(" ++ Adding Read energy to total energy");
			}
			burstEnergy[rank] += (IDD4R - IDD3N) * BL / 2 * NUM_DEVICES;
			if (poppedBusPacket->busPacketType == READ_P) {
				//Don't bother setting next read or write times because the bank is no longer active
				//bankStates[rank][bank].currentBankState = Idle;
				bankStates[rank][bank].nextActivate = max(
						currentClockCycle + READ_AUTOPRE_DELAY,
						bankStates[rank][bank].nextActivate);
				bankStates[rank][bank].lastCommand = READ_P;
				bankStates[rank][bank].stateChangeCountdown = READ_TO_PRE_DELAY;
			} else if (poppedBusPacket->busPacketType == READ) {
				bankStates[rank][bank].nextPrecharge = max(
						currentClockCycle + READ_TO_PRE_DELAY,
						bankStates[rank][bank].nextPrecharge);
				bankStates[rank][bank].lastCommand = READ;
			}
			for (size_t i = 0; i < NUM_RANKS; i++) {
				for (size_t j = 0; j < NUM_BANKS; j++) {
					if (i != poppedBusPacket->rank) {
						//check to make sure it is active before trying to set (save's time?)
						if (bankStates[i][j].currentBankState == RowActive) {
							bankStates[i][j].nextRead = max(
									currentClockCycle + BL / 2 + tRTRS,
									bankStates[i][j].nextRead);
							bankStates[i][j].nextWrite = max(
									currentClockCycle + READ_TO_WRITE_DELAY,
									bankStates[i][j].nextWrite);
						}
					} else {
						bankStates[i][j].nextRead = max(
								currentClockCycle + max(tCCD, BL / 2),
								bankStates[i][j].nextRead);
						bankStates[i][j].nextWrite = max(
								currentClockCycle + READ_TO_WRITE_DELAY,
								bankStates[i][j].nextWrite);
					}
				}

			}
			if (poppedBusPacket->busPacketType == READ_P) {
				//set read and write to nextActivate so the state table will prevent a read or write
				//  being issued (in cq.isIssuable())before the bank state has been changed because of the
				//  auto-precharge associated with this command
				bankStates[rank][bank].nextRead =
						bankStates[rank][bank].nextActivate;
				bankStates[rank][bank].nextWrite =
						bankStates[rank][bank].nextActivate;
			}
			break;
		case READ_PREHIT:				//读抢占，行命中，bus+read
			//add energy to account for total
			accessCount(rank, bank, poppedBusPacket->RIP,
					poppedBusPacket->physicalAddress, 1);
			if (DEBUG_POWER) {
				PRINT(" ++ Adding Read energy to total energy");
			}
			burstEnergy[rank] += (IDD4R - IDD3N) * BL / 2 * NUM_DEVICES;
			//Don't bother setting next read or write times because the bank is no longer active
			//bankStates[rank][bank].currentBankState = Idle;
			bankStates[rank][bank].nextPrecharge = currentClockCycle
					+ READ_TO_PRE_DELAY + tWTR;
			bankStates[rank][bank].lastCommand = READ;
			for (size_t i = 0; i < NUM_RANKS; i++) {
				for (size_t j = 0; j < NUM_BANKS; j++) {
					if (i != poppedBusPacket->rank) {
						//check to make sure it is active before trying to set (save's time?)
						if (bankStates[i][j].currentBankState == RowActive) {
							bankStates[i][j].nextRead = currentClockCycle
									+ BL / 2 + tRTRS + tWTR;
							bankStates[i][j].nextWrite = currentClockCycle
									+ READ_TO_WRITE_DELAY + tWTR;
						}
					} else {
						bankStates[i][j].nextRead = currentClockCycle
								+ max(tCCD, BL / 2) + tWTR;
						bankStates[i][j].nextWrite = currentClockCycle
								+ READ_TO_WRITE_DELAY + tWTR;
					}
				}
			}
			break;
		case READ_PREMISS:		//读抢占，行缺失，pre+act+read
			accessCount(rank, bank, poppedBusPacket->RIP,
					poppedBusPacket->physicalAddress, 1);
			if (DEBUG_POWER) {
				PRINT(" ++ Adding Read energy to total energy");
			}
			burstEnergy[rank] += (IDD4R - IDD3N) * BL / 2 * NUM_DEVICES;
			bankStates[rank][bank].nextPrecharge = currentClockCycle
					+ READ_TO_PRE_DELAY + tWTR + tRP + tRAS;
			bankStates[rank][bank].lastCommand = READ;
			for (size_t i = 0; i < NUM_RANKS; i++) {
				for (size_t j = 0; j < NUM_BANKS; j++) {
					if (i != poppedBusPacket->rank) {
						//check to make sure it is active before trying to set (save's time?)
						if (bankStates[i][j].currentBankState == RowActive) {
							bankStates[i][j].nextRead = currentClockCycle
									+ BL / 2 + tRTRS + tWTR + tRP + tRAS;
							bankStates[i][j].nextWrite = currentClockCycle
									+ READ_TO_WRITE_DELAY + tWTR + tRP + tRAS;
						}
					} else {
						bankStates[i][j].nextRead = currentClockCycle
								+ max(tCCD, BL / 2) + tWTR + tRP + tRAS;
						bankStates[i][j].nextWrite = currentClockCycle
								+ READ_TO_WRITE_DELAY + tWTR + tRP + tRAS;
					}
				}
			}
			break;
		case WRITE_P:
		case WRITE:
			accessCount(rank, bank, poppedBusPacket->RIP,
					poppedBusPacket->physicalAddress, 0);
			if (poppedBusPacket->busPacketType == WRITE_P) {
				bankStates[rank][bank].nextActivate = max(
						currentClockCycle + WRITE_AUTOPRE_DELAY,
						bankStates[rank][bank].nextActivate);
				bankStates[rank][bank].lastCommand = WRITE_P;
				bankStates[rank][bank].stateChangeCountdown =
						WRITE_TO_PRE_DELAY;
			} else if (poppedBusPacket->busPacketType == WRITE) {
				bankStates[rank][bank].nextPrecharge = max(
						currentClockCycle + poppedBusPacket->latency,
						bankStates[rank][bank].nextPrecharge);
				bankStates[rank][bank].nextWrite = max(
						currentClockCycle + poppedBusPacket->latency,
						bankStates[rank][bank].nextWrite);
				bankStates[rank][bank].nextRead = max(
						currentClockCycle + tWTR + poppedBusPacket->latency,
						bankStates[rank][bank].nextRead);
				bankStates[rank][bank].lastCommand = WRITE;
			}

			//add energy to account for total
			if (DEBUG_POWER) {
				PRINT(" ++ Adding Write energy to total energy");
			}
			burstEnergy[rank] += (IDD4W - IDD3N) * BL / 2 * NUM_DEVICES;
			//TODO:record the write power
			writeEnergyperBank[SEQUENTIAL(rank,bank)] +=
					poppedBusPacket->energy;
			for (size_t i = 0; i < NUM_RANKS; i++) {
				for (size_t j = 0; j < NUM_BANKS; j++) {
					if (i != poppedBusPacket->rank) {
						if (bankStates[i][j].currentBankState == RowActive) {
							bankStates[i][j].nextWrite = max(
									currentClockCycle + BL / 2 + tRTRS,
									bankStates[i][j].nextWrite);
							bankStates[i][j].nextRead = max(
									currentClockCycle + WRITE_TO_READ_DELAY_R,
									bankStates[i][j].nextRead);
						}
					} else if (j != poppedBusPacket->bank) {
						bankStates[i][j].nextWrite = max(
								currentClockCycle + max(BL / 2, tCCD),
								bankStates[i][j].nextWrite);
						bankStates[i][j].nextRead = max(
								currentClockCycle + tWTR,
								bankStates[i][j].nextRead);
					}
				}
			}
			//set read and write to nextActivate so the state table will prevent a read or write
			//  being issued (in cq.isIssuable())before the bank state has been changed because of the
			//  auto-precharge associated with this command
			if (poppedBusPacket->busPacketType == WRITE_P) {
				bankStates[rank][bank].nextRead =
						bankStates[rank][bank].nextActivate;
				bankStates[rank][bank].nextWrite =
						bankStates[rank][bank].nextActivate;
			}
			break;
		case ACTWR:		//TODO:如何把写的延迟传递给上层?
			accessCount(rank, bank, poppedBusPacket->RIP,
					poppedBusPacket->physicalAddress, 0);
			writeEnergyperBank[SEQUENTIAL(rank,bank)] +=
					poppedBusPacket->energy;
			bankStates[rank][bank].openRowAddress = poppedBusPacket->row;
			bankStates[rank][bank].currentBankState = RowActive;
			if (poppedBusPacket->busPacketType == ACTWR) {
				bankStates[rank][bank].nextPrecharge = max(
						currentClockCycle + tRAS + poppedBusPacket->latency,
						bankStates[rank][bank].nextPrecharge);
				bankStates[rank][bank].nextWrite = max(
						currentClockCycle + poppedBusPacket->latency,
						bankStates[rank][bank].nextWrite);
				bankStates[rank][bank].nextRead = max(
						currentClockCycle + tWTR + tRAS
								+ poppedBusPacket->latency,
						bankStates[rank][bank].nextRead);
				bankStates[rank][bank].lastCommand = WRITE;
			}
			for (size_t i = 0; i < NUM_RANKS; i++) {
				for (size_t j = 0; j < NUM_BANKS; j++) {
					if (i != poppedBusPacket->rank) {
						if (bankStates[i][j].currentBankState == RowActive) {
							bankStates[i][j].nextWrite = max(
									currentClockCycle + BL / 2 + tRTRS,
									bankStates[i][j].nextWrite);
							bankStates[i][j].nextRead = max(
									currentClockCycle + WRITE_TO_READ_DELAY_R,
									bankStates[i][j].nextRead);
						}
					} else if (j != poppedBusPacket->bank) {
						bankStates[i][j].nextWrite = max(
								currentClockCycle + max(BL / 2, tCCD) + tRAS,
								bankStates[i][j].nextWrite);
						bankStates[i][j].nextRead = max(
								currentClockCycle + tWTR + tRAS,
								bankStates[i][j].nextRead);
					}
				}
			}
			break;
		case PREACTWR:
			accessCount(rank, bank, poppedBusPacket->RIP,
					poppedBusPacket->physicalAddress, 0);
			bankStates[rank][bank].openRowAddress = poppedBusPacket->row;
			bankStates[rank][bank].currentBankState = RowActive;
			writeEnergyperBank[SEQUENTIAL(rank,bank)] +=
					poppedBusPacket->energy;
			if (poppedBusPacket->busPacketType == PREACTWR) {
				bankStates[rank][bank].nextPrecharge = max(
						currentClockCycle + tRP + tRAS
								+ poppedBusPacket->latency,
						bankStates[rank][bank].nextPrecharge);
				bankStates[rank][bank].nextWrite = max(
						currentClockCycle + poppedBusPacket->latency,
						bankStates[rank][bank].nextWrite);
				bankStates[rank][bank].nextRead = max(
						currentClockCycle + tWTR + tRAS
								+ poppedBusPacket->latency,
						bankStates[rank][bank].nextRead);
				bankStates[rank][bank].lastCommand = WRITE;
			}
			for (size_t i = 0; i < NUM_RANKS; i++) {
				for (size_t j = 0; j < NUM_BANKS; j++) {
					if (i != poppedBusPacket->rank) {
						if (bankStates[i][j].currentBankState == RowActive) {
							bankStates[i][j].nextWrite = max(
									currentClockCycle + BL / 2 + tRTRS,
									bankStates[i][j].nextWrite);
							bankStates[i][j].nextRead = max(
									currentClockCycle + WRITE_TO_READ_DELAY_R,
									bankStates[i][j].nextRead);
						}
					} else if (j != poppedBusPacket->bank) {
						bankStates[i][j].nextWrite = max(
								currentClockCycle + max(BL / 2, tCCD),
								bankStates[i][j].nextWrite) + tRP + tRAS;
						bankStates[i][j].nextRead = max(
								currentClockCycle + tWTR + tRP + tRAS,
								bankStates[i][j].nextRead);
					}
				}
			}
			break;
		case ACTIVATE:
			//add energy to account for total
			if (DEBUG_POWER) {
				PRINT(
						" ++ Adding Activate and Precharge energy to total energy");
			}
			actpreEnergy[rank] += ((IDD0 * tRC)
					- ((IDD3N * tRAS) + (IDD2N * (tRC - tRAS)))) * NUM_DEVICES;
			totalActPerBank[SEQUENTIAL(rank, bank)]++;
			bankStates[rank][bank].currentBankState = RowActive;
			bankStates[rank][bank].lastCommand = ACTIVATE;
			bankStates[rank][bank].openRowAddress = poppedBusPacket->row;
			bankStates[rank][bank].nextActivate = max(currentClockCycle + tRC,
					bankStates[rank][bank].nextActivate);
			bankStates[rank][bank].nextPrecharge = max(currentClockCycle + tRAS,
					bankStates[rank][bank].nextPrecharge);

			//if we are using posted-CAS, the next column access can be sooner than normal operation

			bankStates[rank][bank].nextRead = max(
					currentClockCycle + (tRCD - AL),
					bankStates[rank][bank].nextRead);
			bankStates[rank][bank].nextWrite = max(
					currentClockCycle + (tRCD - AL),
					bankStates[rank][bank].nextWrite);
			for (size_t i = 0; i < NUM_BANKS; i++) {
				if (i != poppedBusPacket->bank) {
					bankStates[rank][i].nextActivate = max(
							currentClockCycle + tRRD,
							bankStates[rank][i].nextActivate);
				}
			}
			break;
		case PRECHARGE:
			totalPrePerBank[SEQUENTIAL(rank, bank)]++;
			bankStates[rank][bank].currentBankState = Precharging;
			bankStates[rank][bank].lastCommand = PRECHARGE;
			bankStates[rank][bank].stateChangeCountdown = tRP;
			bankStates[rank][bank].nextActivate = max(currentClockCycle + tRP,
					bankStates[rank][bank].nextActivate);
			break;
		case REFRESH:
			//add energy to account for total
			if (DEBUG_POWER) {
				PRINT(" ++ Adding Refresh energy to total energy");
			}
			refreshEnergy[rank] += (IDD5 - IDD3N) * tRFC * NUM_DEVICES;
			for (size_t i = 0; i < NUM_BANKS; i++) {
				bankStates[rank][i].nextActivate = currentClockCycle + tRFC;
				bankStates[rank][i].currentBankState = Refreshing;
				bankStates[rank][i].lastCommand = REFRESH;
				bankStates[rank][i].stateChangeCountdown = tRFC;
			}
			break;
		default:
			ERROR(
					"clock "<<currentClockCycle<<"== Error - Popped a command we shouldn't have of type : " << poppedBusPacket->busPacketType)
			;
			exit(0);
		}

		//issue on bus and print debug
		if (DEBUG_BUS) {
			PRINTN(" -- MC Issuing On Command Bus : ");
			poppedBusPacket->print();
		}

		//check for collision on bus
		if (outgoingCmdPacket != NULL) {
			ERROR("== Error - Command Bus Collision");
			exit(-1);
		}
		outgoingCmdPacket = poppedBusPacket;
		cmdCyclesLeft = tCMD;
	}

	for (size_t i = 0; i < transactionQueue.size(); i++) {
		//pop off top transaction from queue
		//
		//	assuming simple scheduling at the moment
		//	will eventually add policies here
		Transaction *transaction = transactionQueue[i];

		//map address to rank,bank,row,col
		unsigned newTransactionChan, newTransactionRank, newTransactionBank,
				newTransactionRow, newTransactionColumn;

		// pass these in as references so they get set by the addressMapping function
		addressMapping(transaction->address, newTransactionChan,
				newTransactionRank, newTransactionBank, newTransactionRow,
				newTransactionColumn);

		//if we have room, break up the transaction into the appropriate commands
		//and add them to the command queue
		DataPacket *datapakcet=new DataPacket(transaction->get_newdata(),
				transaction->get_oldata());
		bool added = false;			//denote the trans added success.
		bool found = false;	//found the same, then can recall the uplevel, this transaction finishes
		BusPacketType bpType = transaction->getBusPacketType();
		BusPacket *command = new BusPacket(bpType, transaction->address,
				newTransactionColumn, newTransactionRow, newTransactionRank,
				newTransactionBank, datapakcet, dramsim_log, transaction->RIP);
		delete (datapakcet);
//			PRINTN("Input command: ");command->print();PRINTN("\n");
		if (transaction->transactionType == DATA_READ) {
			Transaction *trans = new Transaction(*transaction);
			added = cancelwriteQueue.addRequest(trans, command, found);
			addedRdTrans++;
			if (found) {
				returnTransaction.push_back(trans);
				PRINT(
											"MC READ ACK same 0x"<<hex<<transaction->address<<dec);
				totalReadsPerBank[SEQUENTIAL(
						newTransactionRank, newTransactionBank)]++;
			}
			if (added) {
				pendingReadTransactions.push_back(transaction);
				transactionQueue.erase(transactionQueue.begin() + i);
				break;
			}
		} else if (transaction->transactionType == DATA_WRITE) {
			addedWrTrans++;
			added = cancelwriteQueue.addRequest(transaction, command, found);
			if (found) {
				totalTransactions++;
				totalWritesPerBank[SEQUENTIAL(
						newTransactionRank, newTransactionBank)]++;
				if (parentMemorySystem->WriteDataDone != NULL) {
//					PRINT(
//							"MC WRITE ACK same 0x"<<hex<<transaction->address<<dec);
					(*parentMemorySystem->WriteDataDone)(
							parentMemorySystem->systemID, transaction->address,
							currentClockCycle);

				}
			}
			if (added) {
				//PRINT("added Transaction Write "<<*transaction);
				delete transaction;
				transactionQueue.erase(transactionQueue.begin() + i);
				break;
			}
		}
		// just delete the transaction now that it's a buspacket

		/* only allow one transaction to be scheduled per cycle -- this should
		 * be a reasonable assumption considering how much logic would be
		 * required to schedule multiple entries per cycle (parallel data
		 * lines, switching logic, decision logic)
		 */

	}

//calculate power
//  this is done on a per-rank basis, since power characterization is done per device (not per bank)
	for (size_t i = 0; i < NUM_RANKS; i++) {
		if (USE_LOW_POWER) {
			//if there are no commands in the queue and that particular rank is not waiting for a refresh...
			if (commandQueue.isEmpty(i) && cancelwriteQueue.isEmpty(i)
					&& !(*ranks)[i]->refreshWaiting) {
				//check to make sure all banks are idle
				bool allIdle = true;
				for (size_t j = 0; j < NUM_BANKS; j++) {
					if (bankStates[i][j].currentBankState != Idle) {
						allIdle = false;
						break;
					}
				}

				//if they ARE all idle, put in power down mode and set appropriate fields
				if (allIdle) {
					powerDown[i] = true;
					(*ranks)[i]->powerDown();
					for (size_t j = 0; j < NUM_BANKS; j++) {
						bankStates[i][j].currentBankState = PowerDown;
						bankStates[i][j].nextPowerUp = currentClockCycle + tCKE;
					}
				}
			}
			//if there IS something in the queue or there IS a refresh waiting (and we can power up), do it
			else if (currentClockCycle >= bankStates[i][0].nextPowerUp
					&& powerDown[i]) //use 0 since theyre all the same
					{
				powerDown[i] = false;
				(*ranks)[i]->powerUp();
				for (size_t j = 0; j < NUM_BANKS; j++) {
					bankStates[i][j].currentBankState = Idle;
					bankStates[i][j].nextActivate = currentClockCycle + tXP;
				}
			}
		}

		//check for open bank
		bool bankOpen = false;
		for (size_t j = 0; j < NUM_BANKS; j++) {
			if (bankStates[i][j].currentBankState == Refreshing
					|| bankStates[i][j].currentBankState == RowActive) {
				bankOpen = true;
				break;
			}
		}

		//background power is dependent on whether or not a bank is open or not
		if (bankOpen) {
			if (DEBUG_POWER) {
				PRINT(" ++ Adding IDD3N to total energy [from rank "<< i <<"]");
			}
			backgroundEnergy[i] += IDD3N * NUM_DEVICES;
		} else {
			//if we're in power-down mode, use the correct current
			if (powerDown[i]) {
				if (DEBUG_POWER) {
					PRINT(
							" ++ Adding IDD2P to total energy [from rank " << i << "]");
				}
				backgroundEnergy[i] += IDD2P * NUM_DEVICES;
			} else {
				if (DEBUG_POWER) {
					PRINT(
							" ++ Adding IDD2N to total energy [from rank " << i << "]");
				}
				backgroundEnergy[i] += IDD2N * NUM_DEVICES;
			}
		}
	}

//check for outstanding data to return to the CPU
	if (returnTransaction.size() > 0) {
		if (DEBUG_BUS) {
			PRINTN(" -- MC Issuing to CPU bus : " << *returnTransaction[0]);
		}
		totalTransactions++;
		bool foundMatch = false;
		//find the pending read transaction to calculate latency
		for (size_t i = 0; i < pendingReadTransactions.size(); i++) {
			if (pendingReadTransactions[i]->address
					== returnTransaction[0]->address) {
				unsigned chan, rank, bank, row, col;
				addressMapping(returnTransaction[0]->address, chan, rank, bank,
						row, col);
				insertHistogram(
						currentClockCycle
								- pendingReadTransactions[i]->timeAdded, rank,
						bank);
				//return latency
				returnReadData(pendingReadTransactions[i]);

				delete pendingReadTransactions[i];
				pendingReadTransactions.erase(
						pendingReadTransactions.begin() + i);
				foundMatch = true;
				break;
			}
		}
		if (!foundMatch) {
			ERROR(
					"Can't find a matching transaction for 0x"<<hex<<returnTransaction[0]->address<<dec);
			abort();
		}

//		PRINT(
//				" == returnTransaction"<<"("<<currentClockCycle<<")=="<< *returnTransaction[0]);
		delete returnTransaction[0];
		returnTransaction.erase(returnTransaction.begin());
	}

//decrement refresh counters
	for (size_t i = 0; i < NUM_RANKS; i++) {
		refreshCountdown[i]--;
	}

//
//print debug
//
	if (DEBUG_TRANS_Q) {
		PRINT("== Printing transaction queue");
		for (size_t i = 0; i < transactionQueue.size(); i++) {
			PRINTN("  " << i << "] "<< *transactionQueue[i]);
		}
	}

	if (DEBUG_BANKSTATE) {
		//TODO: move this to BankState.cpp
		PRINT("== Printing bank states (According to MC)");
		for (size_t i = 0; i < NUM_RANKS; i++) {
			for (size_t j = 0; j < NUM_BANKS; j++) {
				if (bankStates[i][j].currentBankState == RowActive) {
					PRINTN("[" << bankStates[i][j].openRowAddress << "] ");
				} else if (bankStates[i][j].currentBankState == Idle) {
					PRINTN("[idle] ");
				} else if (bankStates[i][j].currentBankState == Precharging) {
					PRINTN("[pre] ");
				} else if (bankStates[i][j].currentBankState == Refreshing) {
					PRINTN("[ref] ");
				} else if (bankStates[i][j].currentBankState == PowerDown) {
					PRINTN("[lowp] ");
				}
			}
			PRINT(""); // effectively just cout<<endl;
		}
	}

	if (DEBUG_CMD_Q) {
		commandQueue.print();
	}
	commandQueue.step();
	cancelwriteQueue.update();
}

bool MemoryController::WillAcceptTransaction() {
	return transactionQueue.size() < TRANS_QUEUE_DEPTH;
}

//allows outside source to make request of memory system
bool MemoryController::addTransaction(Transaction *trans) {
	if (WillAcceptTransaction()) {
		trans->timeAdded = currentClockCycle;
		transactionQueue.push_back(trans);
//		PRINT("("<<currentClockCycle<<")=="<< *trans);
		return true;
	} else {
		return false;
	}
}

void MemoryController::resetStats() {
	for (size_t i = 0; i < NUM_RANKS; i++) {
		for (size_t j = 0; j < NUM_BANKS; j++) {
			//XXX: this means the bank list won't be printed for partial epochs
			grandTotalBankAccesses[SEQUENTIAL(i,j)] +=
					totalReadsPerBank[SEQUENTIAL(i,j)]
							+ totalWritesPerBank[SEQUENTIAL(i,j)];
			totalReadsPerBank[SEQUENTIAL(i,j)] = 0;
			totalWritesPerBank[SEQUENTIAL(i,j)] = 0;
			totalEpochLatency[SEQUENTIAL(i,j)] = 0;
			totalActPerBank[SEQUENTIAL(i,j)] = 0;
			totalPrePerBank[SEQUENTIAL(i,j)] = 0;
		}

		burstEnergy[i] = 0;
		actpreEnergy[i] = 0;
		refreshEnergy[i] = 0;
		backgroundEnergy[i] = 0;
		totalReadsPerRank[i] = 0;
		totalWritesPerRank[i] = 0;
	}
}
//prints statistics at the end of an epoch or  simulation
void MemoryController::printStats(bool finalStats) {
	if (currentClockCycle == 0)
		return;
	unsigned myChannel = parentMemorySystem->systemID;

//if we are not at the end of the epoch, make sure to adjust for the actual number of cycles elapsed

	uint64_t cyclesElapsed =
			(currentClockCycle % EPOCH_LENGTH == 0) ?
					EPOCH_LENGTH : currentClockCycle % EPOCH_LENGTH;
	unsigned bytesPerTransaction = (JEDEC_DATA_BUS_BITS * BL) / 8;
	uint64_t totalBytesTransferred = totalTransactions * bytesPerTransaction;
	double secondsThisEpoch = (double) cyclesElapsed * tCK * 1E-9;

// only per rank
	vector<double> backgroundPower = vector<double>(NUM_RANKS, 0.0);
	vector<double> burstPower = vector<double>(NUM_RANKS, 0.0);
	vector<double> refreshPower = vector<double>(NUM_RANKS, 0.0);
	vector<double> actprePower = vector<double>(NUM_RANKS, 0.0);
	vector<double> averagePower = vector<double>(NUM_RANKS, 0.0);

	vector < uint64_t > canceledwriteperRank = vector < uint64_t
			> (NUM_RANKS, 0);

// per bank variables
	vector<double> averageLatency = vector<double>(NUM_RANKS * NUM_BANKS, 0.0);

	vector<double> readEnergy = vector<double>(NUM_RANKS, 0.0);
	vector<double> writeEnergy = vector<double>(NUM_RANKS, 0.0);
	vector<double> readPower = vector<double>(NUM_RANKS, 0.0);
	vector<double> writePower = vector<double>(NUM_RANKS, 0.0);

	vector<double> readEnergyperBank = vector<double>(NUM_RANKS * NUM_BANKS,
			0.0);

	vector<double> bandwidth = vector<double>(NUM_RANKS * NUM_BANKS, 0.0);

	double totalBandwidth = 0.0;
	for (size_t i = 0; i < NUM_RANKS; i++) {
		for (size_t j = 0; j < NUM_BANKS; j++) {
			bandwidth[SEQUENTIAL(i,j)] =
					(((double) (totalReadsPerBank[SEQUENTIAL(i,j)]
							+ totalWritesPerBank[SEQUENTIAL(i,j)])
							* (double) bytesPerTransaction)
							/ (1024.0 * 1024.0 * 1024.0)) / secondsThisEpoch;
			averageLatency[SEQUENTIAL(i,j)] =
					((float) totalEpochLatency[SEQUENTIAL(i,j)]
							/ (float) (totalReadsPerBank[SEQUENTIAL(i,j)]))
							* tCK;
			totalBandwidth += bandwidth[SEQUENTIAL(i,j)];
			totalReadsPerRank[i] += totalReadsPerBank[SEQUENTIAL(i,j)];
			totalWritesPerRank[i] += totalWritesPerBank[SEQUENTIAL(i,j)];

			totalActPerRank[i] += totalActPerBank[SEQUENTIAL(i,j)];
			totalPrePerRank[i] += totalPrePerBank[SEQUENTIAL(i,j)];

			canceledwriteperRank[i] += cancelwriteQueue.coutcanceledwrite[i][j];
			readEnergyperBank[SEQUENTIAL(i,j)] =
					(((double) totalReadsPerBank[SEQUENTIAL(i,j)]
							* (double) bytesPerTransaction * 8))
							* readEnergyperCell;
			/*			writeEnergyperBank[SEQUENTIAL(i,j)] =
			 ((double) totalWritesPerBank[SEQUENTIAL(i,j)] * set_energy)
			 * (double) bytesPerTransaction * 8;*/
			readEnergy[i] += readEnergyperBank[SEQUENTIAL(i,j)];
			writeEnergy[i] += writeEnergyperBank[SEQUENTIAL(i,j)];
		}
	}
#ifdef LOG_OUTPUT
	dramsim_log.precision(3);
	dramsim_log.setf(ios::fixed,ios::floatfield);
#else
	cout.precision(3);
	cout.setf(ios::fixed, ios::floatfield);
#endif
//	commandQueue.print();
	PRINT(" =======================================================");
	PRINT(" READ TRANS "<< addedRdTrans <<" WRITE TRANS " << addedWrTrans);

	PRINT(
			" ============== Printing Statistics [id:"<<parentMemorySystem->systemID<<"]==============");
	PRINTN("   Total Return Transactions : " << totalTransactions);
	PRINT(
			" ("<<totalBytesTransferred <<" bytes) aggregate average bandwidth "<<totalBandwidth<<"GB/s");

	double totalAggregateBandwidth = 0.0;
	double totalAverageLatency = 0.0;
	for (size_t r = 0; r < NUM_RANKS; r++) {

		PRINT("      -Rank   "<<r<<" : ");
		PRINTN("        -Reads  : " << totalReadsPerRank[r]);
		PRINT(" ("<<totalReadsPerRank[r] * bytesPerTransaction<<" bytes)");
		PRINTN("        -Writes : " << totalWritesPerRank[r]);
		PRINT(" ("<<totalWritesPerRank[r] * bytesPerTransaction<<" bytes)");
		PRINTN("        -Active  : " << totalActPerRank[r]);
		PRINTN("        -Precharge  : " << totalPrePerRank[r]);
		PRINT("        -CanceledWrites : " << canceledwriteperRank[r]);

		for (size_t j = 0; j < NUM_BANKS; j++) {
			PRINT(
					"        -Bandwidth / Latency  (Bank " <<j<<"): " <<bandwidth[SEQUENTIAL(r,j)] << " GB/s\t\t" <<averageLatency[SEQUENTIAL(r,j)] << " ns");
			totalAverageLatency=totalAverageLatency+averageLatency[SEQUENTIAL(r,j)];

		}
		/*		for (size_t b = 0; b < NUM_BANKS; b++) {
		 printAccesscount(r, b);
		 }*/
		// factor of 1000 at the end is to account for the fact that totalEnergy is accumulated in mJ since IDD values are given in mA
		backgroundPower[r] = ((double) backgroundEnergy[r]
				/ (double) (cyclesElapsed)) * Vdd / 1000.0;
		burstPower[r] = ((double) burstEnergy[r] / (double) (cyclesElapsed))
				* Vdd / 1000.0;
		refreshPower[r] = ((double) refreshEnergy[r] / (double) (cyclesElapsed))
				* Vdd / 1000.0;
		actprePower[r] = ((double) actpreEnergy[r] / (double) (cyclesElapsed))
				* Vdd / 1000.0;
		readPower[r] = ((double) readEnergy[r] / (double) (secondsThisEpoch))
				/ 1.0e9; //(mW)
		writePower[r] = ((double) writeEnergy[r] / (double) (secondsThisEpoch))
				/ 1.0e9; //(mW)
		averagePower[r] = ((backgroundEnergy[r] + burstEnergy[r]
				+ refreshEnergy[r] + actpreEnergy[r]) / (double) cyclesElapsed)
				* Vdd / 1000.0;

		if ((*parentMemorySystem->ReportPower) != NULL) {
			(*parentMemorySystem->ReportPower)(backgroundPower[r],
					burstPower[r], refreshPower[r], actprePower[r]);
		}

		PRINT(" == Power Data for Rank        " << r);
		PRINT("   Average Power (watts)     : " << averagePower[r]);
		PRINT("     -Read   (mwatts)     : " << readPower[r]);
		PRINT("     -Write  (mwatts)     : " << writePower[r]);
		PRINT("     -Background (watts)     : " << backgroundPower[r]);
		PRINT("     -Act/Pre    (watts)     : " << actprePower[r]);
		PRINT("     -Burst      (watts)     : " << burstPower[r]);
		PRINT("     -Refresh    (watts)     : " << refreshPower[r]);

		PRINT("     -ReadEnergy   (pJ)     : " << readEnergy[r]);
		PRINT("     -WriteEnergy  (pJ)     : " << writeEnergy[r]);
		if (VIS_FILE_OUTPUT) {
			//	cout << "c="<<myChannel<< " r="<<r<<"writing to csv out on cycle "<< currentClockCycle<<endl;
			// write the vis file output
			csvOut << CSVWriter::IndexedName("Background_Power", myChannel, r)
					<< backgroundPower[r];
			csvOut << CSVWriter::IndexedName("ACT_PRE_Power", myChannel, r)
					<< actprePower[r];
			csvOut << CSVWriter::IndexedName("Burst_Power", myChannel, r)
					<< burstPower[r];
			csvOut << CSVWriter::IndexedName("Refresh_Power", myChannel, r)
					<< refreshPower[r];
			double totalRankBandwidth = 0.0;
			for (size_t b = 0; b < NUM_BANKS; b++) {
				csvOut << CSVWriter::IndexedName("Bandwidth", myChannel, r, b)
						<< bandwidth[SEQUENTIAL(r,b)];
				totalRankBandwidth += bandwidth[SEQUENTIAL(r,b)];
				totalAggregateBandwidth += bandwidth[SEQUENTIAL(r,b)];
				csvOut
						<< CSVWriter::IndexedName("Average_Latency", myChannel,
								r, b) << averageLatency[SEQUENTIAL(r,b)];
			}
			csvOut
					<< CSVWriter::IndexedName("Rank_Aggregate_Bandwidth",
							myChannel, r) << totalRankBandwidth;
			csvOut
					<< CSVWriter::IndexedName("Rank_Average_Bandwidth",
							myChannel, r) << totalRankBandwidth / NUM_RANKS;
		}
	}
	PRINT("     Total averageLatency : "<<totalAverageLatency/(NUM_RANKS*NUM_BANKS));
	if (VIS_FILE_OUTPUT) {
		csvOut << CSVWriter::IndexedName("Aggregate_Bandwidth", myChannel)
				<< totalAggregateBandwidth;
		csvOut << CSVWriter::IndexedName("Average_Bandwidth", myChannel)
				<< totalAggregateBandwidth / (NUM_RANKS * NUM_BANKS);
	}

// only print the latency histogram at the end of the simulation since it clogs the output too much to print every epoch
	if (finalStats) {
//		PRINT(" ---  Latency list ("<<latencies.size()<<")");
//		PRINT("       [lat] : #");
//		if (VIS_FILE_OUTPUT) {
//			csvOut.getOutputStream() << "!!HISTOGRAM_DATA" << endl;
//		}
//
//		map<unsigned, unsigned>::iterator it; //
//		for (it = latencies.begin(); it != latencies.end(); it++) {
//			PRINT(
//					"       ["<< it->first <<"-"<<it->first+(HISTOGRAM_BIN_SIZE-1)<<"] : "<< it->second);
//			if (VIS_FILE_OUTPUT) {
//				csvOut.getOutputStream() << it->first << "=" << it->second
//						<< endl;
//			}
//		}
		if (currentClockCycle % EPOCH_LENGTH == 0) {
			PRINT(" --- Grand Total Bank usage list");
			for (size_t i = 0; i < NUM_RANKS; i++) {
				PRINT("Rank "<<i<<":");
				for (size_t j = 0; j < NUM_BANKS; j++) {
					PRINT(
							"  b"<<j<<": "<<grandTotalBankAccesses[SEQUENTIAL(i,j)]);
				}
			}
		}

	}
	PRINT(
			endl<< " == Pending Transactions : "<<pendingReadTransactions.size()<<" ("<<currentClockCycle<<")==");
#ifdef LOG_OUTPUT
	dramsim_log.flush();
#endif
	resetStats();
}
MemoryController::~MemoryController() {
//ERROR("MEMORY CONTROLLER DESTRUCTOR");
//abort();
	for (size_t i = 0; i < pendingReadTransactions.size(); i++) {
		delete pendingReadTransactions[i];
	}
	for (size_t i = 0; i < returnTransaction.size(); i++) {
		delete returnTransaction[i];
	}
	for (size_t r = 0; r < NUM_RANKS; r++) {
		for (size_t b = 0; b < NUM_BANKS; b++) {
			for (size_t k = 0; k < ripaccessPerBank[SEQUENTIAL(r,b)].size();
					k++) {
				delete ripaccessPerBank[SEQUENTIAL(r,b)][k];
				//accessPerBank[SEQUENTIAL(rank,bank)].clear();
			}

			for (size_t m = 0; m < addraccessPerBank[SEQUENTIAL(r,b)].size();
					m++) {
				delete addraccessPerBank[SEQUENTIAL(r,b)][m];
				//accessPerBank[SEQUENTIAL(rank,bank)].clear();
			}
		}
	}
}
//inserts a latency into the latency histogram
void MemoryController::insertHistogram(unsigned latencyValue, unsigned rank,
		unsigned bank) {
	totalEpochLatency[SEQUENTIAL(rank,bank)] += latencyValue;
//poor man's way to bin things.
	latencies[(latencyValue / HISTOGRAM_BIN_SIZE) * HISTOGRAM_BIN_SIZE]++;
}
void MemoryController::accessCount(unsigned rank, unsigned bank, uint64_t rip,
		uint64_t addr, unsigned type) { //type0=write;type1=read
	vector<RipAccesscount*> &ripaccessBank =
			ripaccessPerBank[SEQUENTIAL(rank,bank)];
	vector<RipAccesscount*> &addraccessBank =
			addraccessPerBank[SEQUENTIAL(rank,bank)];
	bool found = false;
	for (size_t i = 0; i != ripaccessBank.size(); i++) {
		if (ripaccessBank[i]->RIP == rip) {
			found = true;
			if (type == 1) {
				ripaccessBank[i]->readcount++;
			} else {
				ripaccessBank[i]->writecount++;
			}
			break;
		}
	}
	if (found == false && type == 0) {
		ripaccessBank.push_back(new RipAccesscount(rip));
	}
	bool found_addr = false;
	for (size_t j = 0; j != addraccessBank.size(); j++) {
		if (addraccessBank[j]->RIP == addr) {
			found_addr = true;
			addraccessBank[j]->accesstime.push_back(currentClockCycle);
			if (type == 1) {
				addraccessBank[j]->readcount++;
			} else {
				addraccessBank[j]->writecount++;
			}
			break;
		}
	}
	if (found_addr == false && type == 0) {
//		RipAccesscount* addrentry = new RipAccesscount(addr);
//		addrentry->accesstime.push_back(currentClockCycle);
		addraccessBank.push_back(new RipAccesscount(addr));
		addraccessBank[addraccessBank.size() - 1]->accesstime.push_back(
				currentClockCycle);
	}
	return;
}
void MemoryController::printAccesscount(unsigned rank, unsigned bank) { //type0=write;type1=read
	vector<RipAccesscount*> &ripaccessBank =
			ripaccessPerBank[SEQUENTIAL(rank,bank)];
	vector<RipAccesscount*> &addraccessBank =
			addraccessPerBank[SEQUENTIAL(rank,bank)];
	PRINT("Rank "<<rank<<"Bank "<<bank<<" : ");
	for (size_t i = 0; i < ripaccessBank.size(); i++) {
		PRINT(
				" no. "<<i<<" RIP "<<hex<<ripaccessBank[i]->RIP<<dec<<" readCount : "<<ripaccessBank[i]->readcount<<" writeCount: "<<ripaccessBank[i]->writecount);
	}
	for (size_t j = 0; j < addraccessBank.size(); j++) {
		PRINTN(
				" no. "<<j<<" Addr: "<<hex<<addraccessBank[j]->RIP<<dec<<" Access interval: ");
		for (size_t t = 1; t < addraccessBank[j]->accesstime.size(); t++) {
			PRINTN(
					addraccessBank[j]->accesstime[t]-addraccessBank[j]->accesstime[t-1]<<":");
		}
		PRINT(
				"\treadCount : "<<addraccessBank[j]->readcount<<" writeCount: "<<addraccessBank[j]->writecount);
	}
	return;
}
