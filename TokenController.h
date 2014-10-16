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
namespace DRAMSim
{
class TokenController: public SimulatorObject {
private:
	vector<unsigned> startCycle; //per bank
	vector<bool> valid;	//per bank
	vector<bool> existed;	//per bank
	ostream &dramsim_log;

public:
	TokenController(ostream &dramsim_log_) ;
	void initial(BusPacket *bspacket);
	void print();
	enum DataLevel
	{
		RESET,
		PARTRESET,
		PARTSET,
		SET
	};
	vector < uint64_t > latency;
	vector < double > energy;
	vector<uint64_t> writtendata;
	vector<vector<uint64_t> > requestToken; //per cell
	vector<uint64_t> tokenPool;		//per chip
	vector<vector<unsigned> > resetCounts;	//per chip
	vector<vector<unsigned> > partresetCounts;	//per chip
	vector<vector<unsigned> >setCounts;			//per chip
	vector<vector<unsigned> > partsetCounts;		//per chip

//	void set_RankBank(unsigned rankid_,unsigned bankid_){
//		rank=rankid_;
//		bank=bankid_;
//	}
//	double getEnergy(unsigned rank, unsigned bank);

	unsigned getiterNumber(unsigned datalevel);
	bool powerAllowable(BusPacket *buspacket);
	virtual ~TokenController();
	void update();



};
}



#endif /* TOKENCONTROLLER_H_ */
