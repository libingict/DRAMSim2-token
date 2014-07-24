/*
 * RipAccesscount.h
 *
 *  Created on: 2014-7-15
 *      Author: libing
 */

#ifndef RIPACCESSCOUNT_H_
#define RIPACCESSCOUNT_H_

class RipAccesscount{
public:
	uint64_t RIP;
	uint64_t rowAddress;
	vector<uint64_t> accesstime;
	uint64_t readcount;
	uint64_t writecount;
	RipAccesscount() :
			RIP(0), readcount(0),writecount(1){
		vector<uint64_t> accesstime = vector<uint64_t>();
	}
	RipAccesscount(uint64_t rip){
		RIP=rip;
		readcount=0;
		writecount=1;
	}
};



#endif /* RIPACCESSCOUNT_H_ */
