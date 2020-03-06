/**
*  Copyright (c) 2018, Marten Junga (Github.com/Maju-Ketchup)
* All Rights Reserved.
*
* The file is part of the modm library and is released under the 3-clause BSD
* license. See the file `LICENSE` for the full license governing this code.
*
*
* The headder contains the class implementation of the IEEE standart 802.15.4-2011 Frame
* current max size is 255 bytes
* Set always control first
*
*/
#ifndef RANGING_IMPL_HPP
#define RANGING_IMPL_HPP


#endif // RANGING_IMPL_HPP


template < typename ComDevice >
uint8_t
modm::Ranging<ComDevice>::buffer[256] ={0};

template < typename ComDevice >
modm::Frame802154
modm::Ranging<ComDevice>::sendframe = modm::Frame802154();





template < typename ComDevice >
float
modm::Ranging<ComDevice>::computeDsTwrDistance(float distance,modm::Frame802154 receiveframe)
{
	receiveframe.getPayload(6,buffer);
	//receiveframe.debugToString();
	if (buffer[0] == DSTWR_RESP1)
	{
		float tof;
		tof = getTof(buffer);
		//MODM_LOG_DEBUG.printf("foreign tof %f \n",tof);
		tof = tof*ComDevice::TIME_UNIT_TO_S;

		return ((computeDistance(tof,SPEED_OF_LIGHT)+ distance)/2);
	}
	else
	{
		return 0;
	}
}

template < typename ComDevice >
void
modm::Ranging<ComDevice>::sendSSTWRinit()
{

	uint8_t payload = SSTWR_INIT;

	sendframe.setControl(control); // 16Bit Addresses short PAN DATA
	sendframe.setSequenceNumber(ComDevice::frame_seq_nb); //Set SequenceNumber
	sendframe.setDestinationPANAddress(0xDECA); //Set PANAddress
	sendframe.setDestinationAddress16(frame802154::broadcast16bitAddress);	//Set destinationaddress
	sendframe.setSourceAddress16(ComDevice::hostaddress);	//Set Sourceaddress to own address
	sendframe.setPayload(1,&payload);
	//sendframe.getFrame(buffer);
	ComDevice::writeSendBuffer(sendframe.length,sendframe.returnbufferpointer());
	ComDevice::starttx(ComDevice::RESPONSE_EXPECTED);
	ComDevice::frame_seq_nb++;
}

template < typename ComDevice >
void
modm::Ranging<ComDevice>::sendSSTWRinitAt(uint16_t address)
{

	uint8_t payload = SSTWR_INIT;

	sendframe.setControl(control); // 16Bit Addresses short PAN DATA
	sendframe.setSequenceNumber(ComDevice::frame_seq_nb); //Set SequenceNumber
	sendframe.setDestinationPANAddress(0xDECA); //Set PANAddress
	sendframe.setDestinationAddress16(address);	//Set destinationaddress
	sendframe.setSourceAddress16(ComDevice::hostaddress);	//Set Sourceaddress to own address
	sendframe.setPayload(1,&payload);
	//sendframe.getFrame(buffer);
	ComDevice::writeSendBuffer(sendframe.length,sendframe.returnbufferpointer());
	ComDevice::starttx(ComDevice::RESPONSE_EXPECTED);
	ComDevice::frame_seq_nb++;
}

template < typename ComDevice >
void
modm::Ranging<ComDevice>::sendDSTWRinit()
{

	uint8_t payload = DSTWR_INIT0;
	sendframe.setControl(control); // 16Bit Addresses short PAN DATA
	sendframe.setSequenceNumber(ComDevice::frame_seq_nb); //Set SequenceNumber
	sendframe.setDestinationPANAddress(0xDECA); //Set PANAddress
	sendframe.setDestinationAddress16(frame802154::broadcast16bitAddress);	//Set destinationaddress
	sendframe.setSourceAddress16(ComDevice::hostaddress);	//Set Sourceaddress to own address
    sendframe.setPayload(1,&payload);
    //ComDevice::send(sendframe.length,sendframe.returnbufferpointer());
    ComDevice::writeSendBuffer(sendframe.length,sendframe.returnbufferpointer());
    ComDevice::starttx(ComDevice::RESPONSE_EXPECTED);
	ComDevice::frame_seq_nb++;
}



template < typename ComDevice >
void
modm::Ranging<ComDevice>::sendDSTWRinitAt(uint16_t address)
{

	uint8_t payload = DSTWR_INIT0;

	sendframe.setControl(control); // 16Bit Addresses short PAN DATA
	sendframe.setSequenceNumber(ComDevice::frame_seq_nb); //Set SequenceNumber
	sendframe.setDestinationPANAddress(0xDECA); //Set PANAddress
	sendframe.setDestinationAddress16(address);	//Set destinationaddress
	sendframe.setSourceAddress16(ComDevice::hostaddress);	//Set Sourceaddress to own address
	sendframe.setPayload(1,&payload);
	//sendframe.getFrame(buffer);
    //ComDevice::send(sendframe.length, sendframe.returnbufferpointer());
    ComDevice::writeSendBuffer(sendframe.length,sendframe.returnbufferpointer());
    ComDevice::starttx(ComDevice::RESPONSE_EXPECTED);
    //sendframe.debugToString();
	ComDevice::frame_seq_nb++;
}

template < typename ComDevice >
bool
modm::Ranging<ComDevice>::sendAnswer(modm::Frame802154 receiveframe)
{

	receiveframe.getPayload(1,buffer);
	if (buffer[0] == SSTWR_INIT){
		return(sendtimestamps(SSTWR_RESP,receiveframe));
	}
	else if (buffer[0] == DSTWR_INIT0){
		return(sendtimestamps(DSTWR_RESP0,receiveframe));
	}
	else if (buffer[0] == DSTWR_RESP0){
		return(sendtimestamps(DSTWR_INIT1,receiveframe));
	}
	else if (buffer[0] == DSTWR_INIT1){
		sendtof(receiveframe);
		return true;
	}
	return false;
}

template < typename ComDevice >
bool
modm::Ranging<ComDevice>::IsRangingFrame(modm::Frame802154 receiveframe)
{
	receiveframe.getPayload(1,buffer);
	return (buffer[0] == SSTWR_INIT || buffer[0] == SSTWR_RESP || buffer[0] == DSTWR_INIT0 || buffer[0] == DSTWR_RESP0 || buffer[0] == DSTWR_INIT1 || buffer[0] == DSTWR_RESP1);
}


template < typename ComDevice >
float
modm::Ranging<ComDevice>::computeSsTwrDistance(modm::Frame802154 receiveframe)
{

	uint64_t owntx,ownrx;
	uint32_t responserx = 0;
	uint32_t responsetx = 0;
	float tof;
	float distance = 0;

	receiveframe.getPayload(receiveframe.payloadlength,buffer);
	if (buffer[0] == SSTWR_RESP || buffer[0] == DSTWR_RESP0)
	{
		gettimestamps(responserx,responsetx,buffer);
		owntx = ComDevice::readTXTimestamp64();
		ownrx = ComDevice::readRXTimestamp64();
		tof = (float)computeSsTwrTof(ownrx,owntx,responsetx,responserx)*ComDevice::TIME_UNIT_TO_S;
		//MODM_LOG_INFO.printf ("TOF is: %d \n",computeSsTwrTof(ownrx,owntx,responsetx,responserx));
		//MODM_LOG_INFO.printf ("TOF float is: %.9lf \n",tof);
		distance = computeDistance(tof,SPEED_OF_LIGHT);
		ComDevice::frame_seq_nb = receiveframe.getSequenceNumber() + 1 ;

	}
	else
	{
		MODM_LOG_ERROR << "EXPECTET RANGING FRAME GOT:" << modm::endl ;
		receiveframe.debugToString();
		MODM_LOG_ERROR << modm::endl << modm::endl ;

	}
	return (distance);
}

template < typename ComDevice >
int
modm::Ranging<ComDevice>::computeSsTwrTof(int init_rx, int init_tx, int resp_tx , int resp_rx)
{
	int returnv = (((init_rx-init_tx)-(resp_tx-resp_rx))/2);
	return (returnv);
}

template < typename ComDevice >
float
modm::Ranging<ComDevice>::computeDistance(float tof, int travelspeed)
{
	return (tof*travelspeed);
}

template < typename ComDevice >
void
modm::Ranging<ComDevice>::sendtof(modm::Frame802154 receiveframe){

	receiveframe.getPayload(1,buffer);
	if (buffer[0] == DSTWR_INIT1)
	{
		int tof = 0;
		tof= getTof(receiveframe);
		//SET ANSWER
		sendframe.setControl(control); // 16Bit Addresses; one PAN; DATA
		sendframe.setDestinationPANAddress(receiveframe.getDestinationPANAddress()); //Set PANAddress
		sendframe.setSourceAddress16(uint16_t(ComDevice::hostaddress));	//Set Sourceaddress to own address
		setanswerpayload_tof(buffer,tof);
		ComDevice::frame_seq_nb = receiveframe.getSequenceNumber() + 1 ;
		sendframe.setSequenceNumber(ComDevice::frame_seq_nb);
		sendframe.setDestinationAddress16(receiveframe.getSourceAddress16());
		sendframe.setPayload(6,buffer);
		//sendframe.getFrame(buffer);
		//SEND ANSWER
		ComDevice::writeSendBuffer(sendframe.length,sendframe.returnbufferpointer());
		ComDevice::starttx(ComDevice::RESPONSE_EXPECTED);
		//MODM_LOG_DEBUG.printf("owntx = %llu,ownrx = %llu,responserx = %lu, responsetx= %lu, tof: %d \n", owntx, ownrx,responserx, responsetx, tof);
		//receiveframe.debugToString();
		//send.debugToString();
	}
}

template < typename ComDevice >
bool
modm::Ranging<ComDevice>::sendtimestamps(uint8_t flag, modm::Frame802154 receiveframe)
{
	uint32_t sendtime;
	uint64_t owntx,ownrx;
	//PREPAIR ANSWER
	sendframe.setControl(control); // 16Bit Addresses; one PAN; DATA
	sendframe.setDestinationPANAddress(receiveframe.getDestinationPANAddress()); //Set PANAddress
	sendframe.setSourceAddress16(uint16_t(ComDevice::hostaddress));	//Set Sourceaddress to own address
	ownrx = ComDevice::readRXTimestamp64();
	sendtime = (ownrx + (RESP_RX_TIMEOUT_UUS * (ComDevice::UUS_TO_TIME_UNIT))) >> 8;
	owntx = ((uint64_t)((uint64_t)sendtime & 0xFFFFFFFEUL) << 8) + ComDevice::TX_ANT_DLY;
	// set starttime
	setanswerpayload(buffer, flag, ownrx, owntx);
	(ComDevice::frame_seq_nb) = receiveframe.getSequenceNumber() + 1 ;
	//SET ANSWER
	sendframe.setSequenceNumber(ComDevice::frame_seq_nb);
	sendframe.setDestinationAddress16(receiveframe.getSourceAddress16());
	sendframe.setPayload(9,buffer);
	//sendframe.getFrame(buffer);
	//SEND ANSWER
	ComDevice::frame_seq_nb ++;
    ComDevice::writeSendBuffer(sendframe.length,sendframe.returnbufferpointer());
    ComDevice::setdelayedtrxtime(sendtime);
    return(ComDevice::starttx(ComDevice::DELAYED_TX_WITH_RESPONSE));
	//return(ComDevice::sendAt(sendframe.length,sendframe.returnbufferpointer(),sendtime));
}

template < typename ComDevice >
void
modm::Ranging<ComDevice>::gettimestamps(uint32_t &timestamp1,uint32_t &timestamp2 , uint8_t buffer[])
{
	int i;
	timestamp1 = 0;
	timestamp2 = 0;
	for (i=4;i>0;i--)
	{
		timestamp1 = (timestamp1<<8)|buffer[i];
	}
	for (i=9;i>4;i--)
	{
		timestamp2 = (timestamp2<<8)|buffer[i];
	}
	//MODM_LOG_DEBUG.printf("timestamp1: %lu , timestamp2: %lu \n",timestamp1,timestamp2);
	return;
}

template < typename ComDevice >
int
modm::Ranging<ComDevice>::getTof(uint8_t buffer[])
{

	int i;
	int tof = 0;
	for (i=1;i<5;i++){
		tof |= (uint32_t)buffer[i]<<(i-1)*8;}
	if (buffer[5] == 0xFF){
		tof = 0 - tof;	}
	return tof;
}

template < typename ComDevice >
void
modm::Ranging<ComDevice>::setanswerpayload(uint8_t buffer [],uint8_t flag, uint64_t ownrx, uint64_t owntx)
{
	buffer[0] = flag;
	int i;
	for (i=1;i<5;i++)
	{
		buffer[i] = ownrx >> ((i-1)*8);
	}
	for (i=5;i<9;i++)
	{
		buffer[i] = owntx >> ((i-5)*8);
	}
}

template < typename ComDevice >
void
modm::Ranging<ComDevice>::setanswerpayload_tof(uint8_t buffer[], int tof)
{
	buffer[0] = DSTWR_RESP1;
	if (tof < 0){
		buffer[5] = 0xFF;
	}
	else{
		buffer[5] = 0x00;
	}
	if (tof < 0) {
		tof = tof - (2*tof);
	}

	uint32_t tof2 = tof;
	int i;
	for (i=1;i<5;i++)
	{
		buffer[i] = tof2 >> ((i-1)*8);
	}
}

template < typename ComDevice >
int
modm::Ranging<ComDevice>:: getTof(modm::Frame802154 receiveframe)
{

	uint64_t owntx,ownrx;
	uint32_t responserx;
	uint32_t responsetx = 0;
	int tof;

	receiveframe.getPayload(receiveframe.payloadlength,buffer);
	if (buffer[0] == DSTWR_RESP0 ||buffer[0] == DSTWR_INIT1)
	{
		gettimestamps(responserx,responsetx,buffer);
		owntx = ComDevice::readTXTimestamp64();
		ownrx = ComDevice::readRXTimestamp64();
		tof = computeSsTwrTof(ownrx,owntx,responsetx,responserx);

	}
	else if (buffer[0] == DSTWR_RESP1)
	{
		tof = getTof(buffer);
		MODM_LOG_DEBUG.printf("foreign tof %d \n",tof);
	}
	else
	{
		MODM_LOG_ERROR << "EXPECTET RANGING FRAME WITH HEADER E3|E4|E5 GOT:" << modm::endl ;
		receiveframe.debugToString();
		MODM_LOG_ERROR << modm::endl << modm::endl ;
		tof = 0;

	}
	return tof;
}



