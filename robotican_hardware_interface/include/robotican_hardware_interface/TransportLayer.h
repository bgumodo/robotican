//
// Created by tom on 05/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_TRANSPORTLAYER_H
#define ROBOTICAN_HARDWARE_INTERFACE_TRANSPORTLAYER_H

#include <robotican_hardware_interface/Protocol.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

typedef uint16_t crc;

#define POLYNOMIAL 0xD8  /* 11011 followed by 0's */
#define WIDTH  (8 * sizeof(crc))
#define TOPBIT (1 << (WIDTH - 1))
#define MAX_TABLE 256


class TransportLayer {
private:
    crc _crcTable[MAX_TABLE];
    boost::asio::io_service _ioService;
    boost::asio::serial_port _serial;
    bool read(byte buff[], byte buffLength);

    void crcInit();

public:
    TransportLayer(std::string port, unsigned int baudrate);
    ~TransportLayer();

    bool tryToRead(byte *buff, byte buffLength);

    void write(byte buff[], byte buffLength);

    crc calcChecksum(uint8_t const message[], int nBytes);
};

#endif //ROBOTICAN_HARDWARE_INTERFACE_TRANSPORTLAYER_H
