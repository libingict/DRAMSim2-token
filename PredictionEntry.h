/*
 * PredictionEntry.h
 *
 *  Created on: 2014-4-15
 *      Author: libing
 */

#ifndef PREDICTTABLE_H_
#define PREDICTTABLE_H_
class PredictionEntry {
	//PredictionEntry();
public:
	uint64_t RIP;
	uint64_t address;
	uint64_t timeAccess;
	vector<uint64_t> idleInterval;
	PredictionEntry() :
			RIP(0), address(0), timeAccess(0) {
//		idleInterval();
	}
	PredictionEntry(uint64_t rip, uint64_t addr, uint64_t clock) :
			RIP(rip), address(addr), timeAccess(clock) {
//		idleInterval();
	}
	PredictionEntry(const PredictionEntry &p) :
			RIP(p.RIP), address(p.address), timeAccess(p.timeAccess), idleInterval(
					p.idleInterval) {
//		idleInterval();
	}
	/*	static bool compare(PredictTable newins, PredictTable oldins) {
	 return (newins.timeAccess > oldins.timeAccess);
	 }*/
};

#endif /* PREDICTTABLE_H_ */
