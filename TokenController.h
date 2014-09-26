/*
 * TokenController.h
 *
 *  Created on: 2014-9-22
 *      Author: libing
 */

#ifndef TOKENCONTROLLER_H_
#define TOKENCONTROLLER_H_

#include "SimulatorObject.h"
//#include "SystemConfiguration.h"
#include"BusPacket.h"

#define RESETLatency 150/tCK
#define SETLatency 250/tCK
#define RESETToken 2
#define SETToken 1
using namespace std;
namespace DRAMSim
{
class TokenController: public SimulatorObject {
private:
	unsigned rank;
	vector<unsigned> startCycle; //per bank
	vector<bool> valid;	//per bank
	ostream &dramsim_log;
public:
	TokenController(unsigned r, ostream &dramsim_log_) ;
	void initial(BusPacket *bspacket);
	enum DataLevel
	{
		RESET,
		PARTRESET,
		PARTSET,
		SET
	};

	uint64_t writeLatency;
	vector<unsigned> iterNumber; //per cell
	vector<uint64_t> demandedToken; //per cell
	vector<uint64_t> tokenPool;		//per chip
	vector<vector<unsigned> > resetCounts;	//per chip
	vector<vector<unsigned> > partresetCounts;	//per chip
	vector<vector<unsigned> >setCounts;			//per chip
	vector<vector<unsigned> > partsetCounts;		//per chip

	void set_RankBank(unsigned rankid_,unsigned bankid_){
		rank=rankid_;
		bank=bankid_;
	}
	void intial(BusPacket *buspacket);
	uint64_t releaseToken(BusPacket *buspacket);
	unsigned getiterNumber(unsigned datalevel);
	bool powerAllowable(BusPacket *buspacket);
	virtual ~TokenController();
	void update();



};
}



#endif /* TOKENCONTROLLER_H_ */
