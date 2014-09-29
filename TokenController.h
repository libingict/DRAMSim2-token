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

#define RESETLatency (WL+BL/2+150/tCK)
#define SETLatency (WL+BL/2+250/tCK)
#define RESETToken 2
#define SETToken 1
using namespace std;
namespace DRAMSim
{
class TokenController: public SimulatorObject {
private:
	vector<unsigned> startCycle; //per bank
	vector<bool> valid;	//per bank
	ostream &dramsim_log;
	bool hasSame(unsigned rank, unsigned bank);
public:
	TokenController(ostream &dramsim_log_) ;
	void initial(BusPacket *bspacket);
	enum DataLevel
	{
		RESET,
		PARTRESET,
		PARTSET,
		SET
	};

	vector<vector<unsigned> >  iterNumber; //per cell
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
	void intial(BusPacket *buspacket);
	uint64_t getLatency(unsigned rank, unsigned bank);
	double getEnergy(unsigned rank, unsigned bank);

	unsigned getiterNumber(unsigned datalevel);
	bool powerAllowable(BusPacket *buspacket);
	virtual ~TokenController();
	void update();



};
}



#endif /* TOKENCONTROLLER_H_ */
