/*
 * Entry.h
 *
 *  Created on: 2014-2-26
 *      Author: libing
 */

#ifndef ENTRY_H_
#define ENTRY_H_

#include"BusPacket.h"
using namespace DRAMSim;
/*enum PartialPacketType
{
	PartialSET, //Write with short WR
	FullSET   //Write with long WR
};*/
class Entry{
public:
	BusPacket *busPacket;
	unsigned elapsedTime;
	Entry(BusPacket *bspacket);
	virtual ~Entry();
//	Entry(const Entry&);
};




#endif /* ENTRY_H_ */
