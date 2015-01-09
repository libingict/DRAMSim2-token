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








//BusPacket.cpp
//
//Class file for bus packet object
//

#include "BusPacket.h"

using namespace DRAMSim;
using namespace std;

BusPacket::BusPacket(BusPacketType packtype, uint64_t physicalAddr, 
		unsigned col, unsigned rw, unsigned r, unsigned b, DataPacket* data,
		ostream &dramsim_log_,uint64_t rip,uint64_t laten,double energy_) :
	dramsim_log(dramsim_log_),
	busPacketType(packtype),
	column(col),
	row(rw),
	bank(b),
	rank(r),
	physicalAddress(physicalAddr),
	RIP(rip),
	latency(laten),
	energy(energy_){
	if(data!=NULL)
	{
		dataPacket=new DataPacket(data->getData(),data->getoldData());
	}
	else{
		dataPacket=new DataPacket(0,0);
	}
}
BusPacket::~BusPacket(){
	if (dataPacket!=NULL)
			{
				free(dataPacket);
			}
		}
void BusPacket::print(uint64_t currentClockCycle, bool dataStart)
{
	if (this == NULL)
	{
		return;
	}

	if (VERIFICATION_OUTPUT)
	{
		switch (busPacketType)
		{
		case READ:
			cmd_verify_out << currentClockCycle << ": read ("<<rank<<","<<bank<<","<<column<<",0);"<<endl;
			break;
		case READ_P:
			cmd_verify_out << currentClockCycle << ": read ("<<rank<<","<<bank<<","<<column<<",1);"<<endl;
			break;
		case WRITE:
			cmd_verify_out << currentClockCycle << ": write ("<<rank<<","<<bank<<","<<column<<",0 , 0, 'h0);"<<dataPacket<<endl;
			break;
		case WRITE_P:
			cmd_verify_out << currentClockCycle << ": write ("<<rank<<","<<bank<<","<<column<<",1, 0, 'h0);"<<endl;
			break;
		case ACTIVATE:
			cmd_verify_out << currentClockCycle <<": activate (" << rank << "," << bank << "," << row <<");"<<endl;
			break;
		case PRECHARGE:
			cmd_verify_out << currentClockCycle <<": precharge (" << rank << "," << bank << "," << row <<");"<<endl;
			break;
		case REFRESH:
			cmd_verify_out << currentClockCycle <<": refresh (" << rank << ");"<<endl;
			break;
		case DATA:
			//TODO: data verification?
			break;
		default:
			ERROR("Trying to print unknown kind of bus packet");
			exit(-1);
		}
	}
}
void BusPacket::print()
{
	if (this == NULL) //pointer use makes this a necessary precaution
	{
		return;
	}
	else
	{
		switch (busPacketType)
		{
		case READ:
			PRINT("BP [READ] pa[0x"<<hex<<physicalAddress<<dec<<"] r["<<rank<<"] b["<<bank<<"] row["<<row<<"] col["<<column<<"] rip["<<hex<<RIP<<dec<<"] data[0x"<<*dataPacket<<dec<<"]");
			break;
		case READ_P:
			PRINT("BP [READ_P] pa[0x"<<hex<<physicalAddress<<dec<<"] r["<<rank<<"] b["<<bank<<"] row["<<row<<"] col["<<column<<"] rip["<<hex<<RIP<<dec<<"] data[0x"<<*dataPacket<<dec<<"]");
			break;
		case PREACTWR:
			PRINT("BP [PREACTWR] pa[0x"<<hex<<physicalAddress<<dec<<"] r["<<rank<<"] b["<<bank<<"] row["<<row<<"] col["<<column<<"] rip["<<hex<<RIP<<"] data[0x"<<*dataPacket<<dec<<"]");
			break;
		case ACTWR:
			PRINT("BP [ACTWR] pa[0x"<<hex<<physicalAddress<<dec<<"] r["<<rank<<"] b["<<bank<<"] row["<<row<<"] col["<<column<<"] rip["<<hex<<RIP<<dec<<"]");
			break;
		case READ_PREHIT:
			PRINT("BP [READ_PREHIT] pa[0x"<<hex<<physicalAddress<<dec<<"] r["<<rank<<"] b["<<bank<<"] row["<<row<<"] col["<<column<<"] rip["<<hex<<RIP<<"] data[0x"<<*dataPacket<<dec<<"]");
			break;
		case READ_PREMISS:
			PRINT("BP [READ_PREMISS] pa[0x"<<hex<<physicalAddress<<dec<<"] r["<<rank<<"] b["<<bank<<"] row["<<row<<"] col["<<column<<"] rip["<<hex<<RIP<<dec<<"]");
			break;
		case WRITE:
			PRINT("BP [WRITE] pa[0x"<<hex<<physicalAddress<<dec<<"] r["<<rank<<"] b["<<bank<<"] row["<<row<<"] col["<<column<<"] rip["<<hex<<RIP<<dec<<"] data[0x"<<*dataPacket<<dec<<"]");
			break;
		case WRITE_P:
			PRINT("BP [WRITE_P] pa[0x"<<hex<<physicalAddress<<dec<<"] r["<<rank<<"] b["<<bank<<"] row["<<row<<"] col["<<column<<"] rip["<<hex<<RIP<<dec<<"] data[0x"<<*dataPacket<<dec<<"]");
			break;
		case ACTIVATE:
			PRINT("BP [ACT] pa[0x"<<hex<<physicalAddress<<dec<<"] r["<<rank<<"] b["<<bank<<"] row["<<row<<"] col["<<column<<"] rip["<<hex<<RIP<<"] data[0x"<<*dataPacket<<dec<<"]");
			break;
		case PRECHARGE:
			PRINT("BP [PRE] pa[0x"<<hex<<physicalAddress<<dec<<"] r["<<rank<<"] b["<<bank<<"] row["<<row<<"] col["<<column<<"] rip["<<hex<<RIP<<dec<<"]");
			break;
		case REFRESH:
			PRINT("BP [REF] pa[0x"<<hex<<physicalAddress<<dec<<"] r["<<rank<<"] b["<<bank<<"] row["<<row<<"] col["<<column<<"]");
			break;
		case DATA:
			PRINTN("BP [DATA] pa[0x"<<hex<<physicalAddress<<dec<<"] r["<<rank<<"] b["<<bank<<"] row["<<row<<"] col["<<column<<"] data[=");
			printData(dataPacket);
			PRINT("");
			break;
		default:
			ERROR("Trying to print unknown kind of bus packet");
			exit(-1);
		}
	}
}

void BusPacket::printData(const DataPacket *data)
{
	if (data == NULL)
	{
		PRINTN("NO DATA");
		return;
	}
	PRINTN(data);
}
