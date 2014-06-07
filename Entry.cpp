/*
 * Entry.cpp
 *
 *  Created on: 2014-2-28
 *      Author: libing
 */
#include "Entry.h"
Entry::Entry(BusPacket *buspacket) :
		busPacket(buspacket), elapsedTime(0) {

}
Entry::~Entry() {
	if (buspacket != NULL) {
		delete buspacket;
		buspacket = NULL;
	}
}

