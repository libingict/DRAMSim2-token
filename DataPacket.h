#ifndef _DATA_PACKET_H_
#define _DATA_PACKET_H_

#include <string.h> //memcpy 
#include <stdint.h>
#include <stdlib.h> //free
#include <iostream>

using std::ostream; 
using std::dec;
using std::hex;

namespace DRAMSim {
	typedef unsigned char byte;

	class DataPacket {
	/*
	 * A very thin wrapper around a data that is sent and received in DRAMSim2
	 *
	 */
	private:
		// Disable copying for a datapacket
		DataPacket(const DataPacket &other);
		DataPacket &operator =(const DataPacket &other); 

	public: 
		/**
		 * Constructor to be used if we are using NO_STORAGE
		 *
		 */
		DataPacket() : _data(0), _oldata(0)
		{}

		/**
		 * @param data pointer to a buffer of data; DRAMSim will take ownership of the buffer and will be responsible for freeing it 
		 * @param numBytes number of bytes in the data buffer
		 * @param unalignedAddr DRAMSim will typically kill the bottom bits to align them to the DRAM bus width, but if an access is unaligned (and smaller than the transaction size, the raw address will need to be known to properly execute the read/write)
		 *
		 */
		DataPacket(uint64_t data, uint64_t oldata) :
			_data(data), _oldata(oldata)
		{}
/*		virtual ~DataPacket()
		{
		if (_data) {
			free(_data);
		}
		if (_oldata) {
			free (_oldata);
		}
	}*/

		// accessors
		uint64_t getData() const
		{
			return _data;
		}
		uint64_t getoldData() const{
			return _oldata;
		}
		void setData(const uint64_t data, uint64_t oldata)
		{
//			_data = (uint64_t *)calloc(size,sizeof(uint64_t));
//			memcpy(_data, data, uint64_t);
			_data=data;
			_oldata=oldata;
		}
		bool hasNoData() const
		{
			return (_data == 0 && _oldata==0);
		}

		friend ostream &operator<<(ostream &os, const DataPacket &dp);

	private:
		uint64_t _data;
		uint64_t _oldata;
	};


} // namespace DRAMSim

#endif // _DATA_PACKET_H_
