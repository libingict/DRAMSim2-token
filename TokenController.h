/*
 * TokenController.h
 *
 *  Created on: 2014-9-22
 *      Author: libing
 */

#ifndef TOKENCONTROLLER_H_
#define TOKENCONTROLLER_H_

#include "SimulatorObject.h"
#include "SystemConfiguration.h"
#include"BusPacket.h"

/*
 #define RESETLatency (WL+BL/2+150/tCK)
 #define SETLatency (WL+BL/2+250/tCK)
 #define RESETToken 2
 #define SETToken 1
 */
using namespace std;
namespace DRAMSim {
class DataCounts {
public:
	vector<unsigned> resetCounts;
	vector<unsigned> setCounts;
	vector<unsigned> partsetCounts;
	vector<unsigned> partresetCounts;
	DataCounts() :
			resetCounts(NUM_DEVICES, 0), setCounts(NUM_DEVICES, 0), partsetCounts(
					NUM_DEVICES, 0), partresetCounts(NUM_DEVICES, 0) {
	}
};
class TokenEntry {
public:
	unsigned startCycle;
	uint64_t physicalAddress;
	bool valid;
	DataCounts* dataCounts;	//per chip
	vector<uint64_t> requestToken;
	uint64_t latency;
	double energy;
	TokenEntry();
	TokenEntry(unsigned currentClock, uint64_t physicaladdr, bool valid_,
			DataCounts* datacounts,uint64_t latency_,double energy_) :
			startCycle(currentClock), physicalAddress(physicaladdr), valid(
					valid_),latency(latency),energy(energy_){
		dataCounts = new DataCounts();
		requestToken = vector < uint64_t > (NUM_DEVICES, 0);
		if (datacounts != NULL) {
			for (size_t i = 0; i < NUM_DEVICES; i++) {
			dataCounts->resetCounts[i]=datacounts->resetCounts[i];
			dataCounts->setCounts[i]=datacounts->setCounts[i];
			dataCounts->partresetCounts[i]=datacounts->partresetCounts[i];
			dataCounts->partsetCounts[i]=datacounts->partsetCounts[i];
		}
	}
//	friend ostream &operator<<(ostream &os, const TokenEntry &t) {
//		os << "Token [0x" << hex << t.physicalAddress << "] startCycle" << dec << t.startCycle << "] latency[" << t.latency<< "] energy[" << t.energy <<"] ";
//		for (size_t i = 0; i < NUM_DEVICES; i++) {
//			os << " chip["<<i<<"] requestToken["<<t.requestToken[i]<<"] ";
//		}
//		os<<endl;
//		return os;
//	}
/*		~TokenEntry()
		{
			if (dataCounts != NULL)
				delete dataCounts;
		}*/
}
};
class TokenController: public SimulatorObject {
private:
//	vector<bool> existed;	//per bank
	ostream &dramsim_log;

public:
	TokenController(ostream &dramsim_log_);
	void initial(BusPacket *bspacket);
	void print();
	vector<vector<uint64_t> > latency;
	vector<vector<double> > energy;
	vector<uint64_t> tokenPool;		//per chip
	vector<DataCounts*>  dataCounts;	//per chip
	vector<vector<TokenEntry*> > tokenQueue;
//	void set_RankBank(unsigned rankid_,unsigned bankid_){
//		rank=rankid_;
//		bank=bankid_;
//	}

	unsigned getiterNumber(unsigned datalevel);
	bool powerAllowable(BusPacket *buspacket);
	virtual ~TokenController();
	void update();

};
}

#endif /* TOKENCONTROLLER_H_ */
