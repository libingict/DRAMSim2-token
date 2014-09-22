#include "DataPacket.h"
#include <stdio.h>

namespace DRAMSim 
{

	ostream &operator<<(ostream &os, const DataPacket &dp)
	{

		os << "DATA: ("<<hex<< dp._data << ") '";
		os<<  "OLDATA: ("<< dp._oldata<<") "<<dec<<std::endl;

		return os;
	}

}
