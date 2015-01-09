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
	ostream &dramsim_log;
public:
	unsigned startCycle;
//	uint64_t physicalAddress;
	bool valid;						//写请求是否正在执行
	bool done;						//写请求是否完成
	DataCounts* dataCounts;			//per chip上的数据分布
	BusPacket* packet;
	vector<uint64_t> requestToken;	//写请求所需的power token,迭代完成时更新
	uint64_t latency;				//写请求的延迟
	double energy;					//写请求消耗的能量
	TokenEntry();
	TokenEntry(unsigned startCycle_, BusPacket* packet_, bool valid_,bool done_,
			DataCounts* datacounts, uint64_t latency_, double energy_,vector<uint64_t> token_, ostream &dramsim_log_) :
			startCycle(startCycle_), valid(valid_), done(done_),latency(latency_), energy(
					energy_), dramsim_log(dramsim_log_){
		dataCounts = new DataCounts();
		packet = new BusPacket(packet_->busPacketType, packet_->physicalAddress,
				packet_->column, packet_->row, packet_->rank, packet_->bank,
				packet_->dataPacket, dramsim_log, packet_->RIP,latency,energy);
		requestToken = vector < uint64_t > (NUM_DEVICES, 0);
		if (datacounts != NULL) {
			for (size_t i = 0; i < NUM_DEVICES; i++) {
				dataCounts->resetCounts[i] = datacounts->resetCounts[i];
				dataCounts->setCounts[i] = datacounts->setCounts[i];
				dataCounts->partresetCounts[i] = datacounts->partresetCounts[i];
				dataCounts->partsetCounts[i] = datacounts->partsetCounts[i];
				requestToken[i]=token_[i];
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
	void updateReclaim(TokenEntry* tokenEntry);
	void updateReallocate(TokenEntry* tokenEntry);
	unsigned ratio;
public:
	TokenController(vector<vector<TokenEntry*> > &writequeue,
			ostream &dramsim_log_);
	bool addwriteRequest(vector<TokenEntry*>& writeBank, BusPacket *bspacket,bool &found);
	void print();
	vector<vector<uint64_t> > latency;
	vector<vector<double> > energy;
	vector<double> tokenPool;		//per chip
	vector<DataCounts*> dataCounts;	//per chip
	vector<vector<TokenEntry*> > &tokenQueue;
//	void set_RankBank(unsigned rankid_,unsigned bankid_){
//		rank=rankid_;
//		bank=bankid_;
//	}

	unsigned getiterNumber(unsigned datalevel);
	bool issueWrite(BusPacket *buspacket);
	bool powerAllowable(TokenEntry*& writerequest);
	virtual ~TokenController();
	void update();
	void update_Naive();
	void update_FPB();
	void update_SPA(); //SET scheme;
	bool release(size_t rank, size_t bank, uint64_t &addr);

//	bool release(unsigned rank, unsigned bank,); //delete the finished write

};
}

#endif /* TOKENCONTROLLER_H_ */
