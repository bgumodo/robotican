//
// Created by tom on 05/05/16.
//

#include "robotican_hardware_interface/TransportLayer.h"

TransportLayer::TransportLayer(std::string port, unsigned int baudrate) : _ioService(), _serial(_ioService, port) {
    crcInit();
    _serial.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
//    _serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
//    _serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
//    _serial.set_option(boost::asio::serial_port_base::character_size(boost::asio::serial_port_base::character_size(8)));
//    _serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
//
}

bool TransportLayer::read(byte *buff, byte buffLength) {
    byte size[1] = {0};
    byte message[128] = {0};

    boost::asio::read(_serial, boost::asio::buffer(size, 1));

    if(size[0] < buffLength) {
        boost::asio::read(_serial, boost::asio::buffer(message, size[0] - 1));
        buff[0] = size[0];
        for(int i = 1; i <= size[0] - 1; ++i) {
            buff[i] = message[i-1];
        }
        return true;
    }
    return false;
}

void TransportLayer::crcInit() {

    crc  remainder;
    /*
    * Compute the remainder of each possible dividend.
    */
    for (int dividend = 0; dividend < 256; ++dividend)
    {
        /*
         * Start with the dividend followed by zeros.
         */
        remainder = dividend << (WIDTH - 8);

        /*
         * Perform modulo-2 division, a bit at a time.
         */
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            /*
             * Try to divide the current data bit.
             */
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }

        /*
         * Store the result into the table.
         */
        _crcTable[dividend] = remainder;
    }

}

bool TransportLayer::tryToRead(byte *buff, byte buffLength) {
    byte headerSignal[1] = {0};
    boost::asio::read(_serial, boost::asio::buffer(headerSignal, 1));
    if(headerSignal[0] == HEADER_SIGNAL) {
        return read(buff, buffLength);
    }
    return false;
}

void TransportLayer::write(byte *buff, byte buffLength) {
    byte headerSignal[1];
    headerSignal[0] = HEADER_SIGNAL;
    boost::asio::write(_serial, boost::asio::buffer(headerSignal, 1));
    boost::asio::write(_serial, boost::asio::buffer(buff, buffLength));
}

crc TransportLayer::calcChecksum(uint8_t const message[], int nBytes) {
    uint8_t data;
    crc remainder = 0;

    /*
     * Divide the message by the polynomial, a byte at a time.
     */
    for (int byte = 0; byte < nBytes; ++byte)
    {
        data = message[byte] ^ (remainder >> (WIDTH - 8));
        remainder = _crcTable[data] ^ (remainder << 8);
    }

    /*
     * The final remainder is the CRC.
     */
    return (remainder);
}

TransportLayer::~TransportLayer() {
    _serial.close();

}
