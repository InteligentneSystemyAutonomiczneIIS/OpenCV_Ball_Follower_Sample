#pragma once

#include <string>
#include <map>
#include "rs232.h"

class ArduinoSerialCommunicator
{
public:

	//Constructor for specifying com port by name
	ArduinoSerialCommunicator(std::string port, int baudRate = 9600, int dataBits = 8, char parity = 'N', int stopBits = 1)
	{
		int portNumber;
		try
		{
			portNumber = COMPortMapping.at(port);
		}
		catch (const std::out_of_range)
		{
			std::cout << "COM PORT NOT FOUND OR UNSUPPORTED" << std::endl;
			this->isComPortOpenedProperly = false; //just to be sure
			return;
		}

		init(portNumber, baudRate, dataBits, parity, stopBits);


	}

//	ArduinoSerialCommunicator(int portNumber, int baudRate = 9600, int dataBits = 8, char parity = 'N', int stopBits = 1)
//	{

//		throw std::logic_error("Not Implemented");

//	}



	//Check if com port has been opened properly
	bool isOpened()
	{
		return isComPortOpenedProperly;
	}

	//Send a string over Serial port
	void sendString(std::string stringToSend)
	{
		RS232_cputs(this->portNumber, stringToSend.c_str());
		
	}

	//Send a single byte over Serial port.
	//Returns status C-style: 0 -> everything went fine, !=0 -> error
	int sendByte(unsigned char byteToSend)
	{
		int status = RS232_SendByte(this->portNumber, byteToSend);
        return status;

	}


	std::string pollSerialPortForData()
	{
		std::string dataReceived;
		const int BUFF_SIZE = 128;
		unsigned char buffer[BUFF_SIZE];


		int n = RS232_PollComport(this->portNumber, buffer, (int)BUFF_SIZE);
		if (n > 0) {
			buffer[n] = 0;
			/* always put a "null" at the end of a string! */
			printf("Received %i bytes: '%s'\n", n, (char *)buffer);
			dataReceived = std::string(reinterpret_cast<const char*> (buffer));
		}

		return dataReceived;

	}


	~ArduinoSerialCommunicator()
	{
		if (this->isOpened())
		{
			RS232_CloseComport(this->portNumber);
		}

	}




private:

	char mode[4];
	int portNumber;
	int baudRate;

	std::map<std::string, int> COMPortMapping = 
	{
		{"/dev/ttyS0",0}, {"/dev/ttyS1",1}, {"/dev/ttyS2", 2}, {"/dev/ttyS3",3}, {"/dev/ttyS4",4}, {"/dev/ttyS5",5},
		{"/dev/ttyS6",6}, {"/dev/ttyS7", 7}, {"/dev/ttyS8",8}, {"/dev/ttyS9",9} , {"/dev/ttyS10",10} , {"/dev/ttyS11",11},
		{"/dev/ttyS12",12 }, {"/dev/ttyS13",13}, {"/dev/ttyS14",14}, {"/dev/ttyS15",15}, {"/dev/ttyUSB0",16}, 
		{"/dev/ttyUSB1",17}, {"/dev/ttyUSB2",18}, {"/dev/ttyUSB3",19}, {"/dev/ttyUSB4",20}, {"/dev/ttyUSB5",21},
		{"/dev/ttyAMA0",22}, {"/dev/ttyAMA1",23}, {"/dev/ttyACM0",24}, {"/dev/ttyACM1",25}, {"/dev/rfcomm0",26}, 
		{"/dev/rfcomm1",27}, {"/dev/ircomm0",28}, {"/dev/ircomm1",29}, {"/dev/cuau0",30}, {"/dev/cuau1",31}, 
		{"/dev/cuau2",32}, {"/dev/cuau3",33}, {"/dev/cuaU0",34}, {"/dev/cuaU1",35}, {"/dev/cuaU2",36}, {"/dev/cuaU3",37},
		{"COM1",0}, {"COM2", 1}, {"COM3", 2}, {"COM4", 3}, {"COM5",4}, {"COM6", 5}, {"COM7", 6}, {"COM8", 7},
		{"COM9",8 },{ "COM10", 9 },{ "COM11", 10 },{ "COM12", 11 },{ "COM13",12 },{ "COM14", 13 },{ "COM15", 14 },{ "COM16", 15 }

	};

	bool isComPortOpenedProperly = false;


	void init(int portNumber, int baudRate, char dataBits, char parity, int stopBits)
	{

		this->portNumber = portNumber;
		this->baudRate = baudRate;
		//set up COM mode
		this->mode[0] = '0' + (char)dataBits;
		this->mode[1] = parity;
		this->mode[2] = '0' + (char)stopBits;
		this->mode[3] = 0;

		if (RS232_OpenComport(this->portNumber, this->baudRate, this->mode))
		{
			std::cout << "COM PORT CANNOT BE OPENED" << std::endl;
			this->isComPortOpenedProperly = false;
			return;

		}

		this->isComPortOpenedProperly = true;
	}

};
