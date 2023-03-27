/**************************************************************************/
/*!
    @file     emulatefelica.cpp
    @author   Shohei Okubo
    @license  BSD
*/
/**************************************************************************/

#include "emulatefelica.h"
#include "PN532_debug.h"

#include <string.h>

#define MAX_TGREAD

bool EmulateFelica::init(){
  return pn532.begin();
}

void EmulateFelica::setIdm(uint8_t* idm){
  idmPtr = idm;
}

void EmulateFelica::setPmm(uint8_t* pmm){
  pmmPtr = pmm;
}

void EmulateFelica::setSys(uint8_t* sys){
  sysPtr = sys;
}

bool EmulateFelica::emulate(const uint16_t tgInitAsTargetTimeout){

  uint8_t command[] = {
        PN532_COMMAND_TGINITASTARGET,
        5,                  // MODE: PICC only, Passive only

        0x04, 0x00,         // SENS_RES
        0x00, 0x00, 0x00,   // NFCID1
        0x20,               // SEL_RES

        0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,   // FeliCaParams
        0,0,

        0,0,0,0,0,0,0,0,0,0, // NFCID3t

        0, // length of general bytes
        0  // length of historical bytes
  };

  if(idmPtr != 0){  // if uid is set copy 3 bytes to nfcid1
    memcpy(command + 8, idmPtr, 8);
    memcpy(command + 26, idmPtr, 8);
  }

  if(pmmPtr != 0){  // if uid is set copy 3 bytes to nfcid1
    memcpy(command + 16, pmmPtr, 8);
  }

  if(sysPtr != 0){  // if uid is set copy 3 bytes to nfcid1
    memcpy(command + 24, sysPtr, 2);
  }

  if(1 != pn532.tgInitAsTarget(command,sizeof(command), tgInitAsTargetTimeout)){
    DMSG("tgInitAsTarget failed or timed out!");
    return false;
  }

  uint8_t rwbuf[128];
  uint8_t sendlen;
  int16_t status;
  bool runLoop = true;

  while(runLoop){
    status = pn532.tgGet(rwbuf, sizeof(rwbuf));
    if(status < 0){
      DMSG("tgGetData failed!\n");
      pn532.inRelease();
      return true;
    }

    uint8_t p1 = rwbuf[C_APDU_P1];
    uint8_t p2 = rwbuf[C_APDU_P2];
    uint8_t lc = rwbuf[C_APDU_LC];
    uint16_t p1p2_length = ((int16_t) p1 << 8) + p2;

    switch(rwbuf[C_APDU_INS]){
    case ISO7816_SELECT_FILE:
      switch(p1){
      case C_APDU_P1_SELECT_BY_ID:
	if(p2 != 0x0c){
	  DMSG("C_APDU_P2 != 0x0c\n");
	  setResponse(COMMAND_COMPLETE, rwbuf, &sendlen);
	} else if(lc == 2 && rwbuf[C_APDU_DATA] == 0xE1 && (rwbuf[C_APDU_DATA+1] == 0x03 || rwbuf[C_APDU_DATA+1] == 0x04)){
	  setResponse(COMMAND_COMPLETE, rwbuf, &sendlen);
	  if(rwbuf[C_APDU_DATA+1] == 0x03){
	    currentFile = CC;
	  } else if(rwbuf[C_APDU_DATA+1] == 0x04){
	    currentFile = NDEF;
	  }
	} else {
	  setResponse(TAG_NOT_FOUND, rwbuf, &sendlen);
	}
	break;
      case C_APDU_P1_SELECT_BY_NAME: 
        const uint8_t ndef_tag_application_name_v2[] = {0, 0x7, 0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01 };
	if(0 == memcmp(ndef_tag_application_name_v2, rwbuf + C_APDU_P2, sizeof(ndef_tag_application_name_v2))){
	  setResponse(COMMAND_COMPLETE, rwbuf, &sendlen);
	} else{
	  DMSG("function not supported\n");
	  setResponse(FUNCTION_NOT_SUPPORTED, rwbuf, &sendlen);
	} 
	break;
      }
      break;
    case ISO7816_READ_BINARY:
      switch(currentFile){
      case NONE:
	setResponse(TAG_NOT_FOUND, rwbuf, &sendlen);
	break;
      case CC:
	if( p1p2_length > NDEF_MAX_LENGTH){
	  setResponse(END_OF_FILE_BEFORE_REACHED_LE_BYTES, rwbuf, &sendlen);
	}else {
	  memcpy(rwbuf,compatibility_container + p1p2_length, lc);
	  setResponse(COMMAND_COMPLETE, rwbuf + lc, &sendlen, lc);
	}
	break;
      case NDEF:
	if( p1p2_length > NDEF_MAX_LENGTH){
	  setResponse(END_OF_FILE_BEFORE_REACHED_LE_BYTES, rwbuf, &sendlen);
	}else {
	  memcpy(rwbuf, ndef_file + p1p2_length, lc);
	  setResponse(COMMAND_COMPLETE, rwbuf + lc, &sendlen, lc);
	}
	break;
      }
      break;    
    case ISO7816_UPDATE_BINARY:
      if(!tagWriteable){
	  setResponse(FUNCTION_NOT_SUPPORTED, rwbuf, &sendlen);
      } else{      
	if( p1p2_length > NDEF_MAX_LENGTH){
	  setResponse(MEMORY_FAILURE, rwbuf, &sendlen);
	}
	else{
	  memcpy(ndef_file + p1p2_length, rwbuf + C_APDU_DATA, lc);
	  setResponse(COMMAND_COMPLETE, rwbuf, &sendlen);
	  tagWrittenByInitiator = true;
      
      uint16_t ndef_length = (ndef_file[0] << 8) + ndef_file[1];
      if ((ndef_length > 0) && (updateNdefCallback != 0)) {
        updateNdefCallback(ndef_file + 2, ndef_length);
      }
	}
      }
      break;
    default:
      DMSG("Command not supported!");
      DMSG_HEX(rwbuf[C_APDU_INS]);
      DMSG("\n");
      setResponse(FUNCTION_NOT_SUPPORTED, rwbuf, &sendlen);
    }
    status = pn532.tgSetData(rwbuf, sendlen);
    if(status < 0){
      DMSG("tgSetData failed\n!");
      pn532.inRelease();
      return true;
    }
  }
  pn532.inRelease();
  return true;
}

void EmulateFelica::setResponse(responseCommand cmd, uint8_t* buf, uint8_t* sendlen, uint8_t sendlenOffset){
  switch(cmd){
  case COMMAND_COMPLETE:
    buf[0] = R_APDU_SW1_COMMAND_COMPLETE;
    buf[1] = R_APDU_SW2_COMMAND_COMPLETE;
    *sendlen = 2 + sendlenOffset;
    break;
  case TAG_NOT_FOUND:
    buf[0] = R_APDU_SW1_NDEF_TAG_NOT_FOUND;
    buf[1] = R_APDU_SW2_NDEF_TAG_NOT_FOUND;
    *sendlen = 2;    
    break;
  case FUNCTION_NOT_SUPPORTED:
    buf[0] = R_APDU_SW1_FUNCTION_NOT_SUPPORTED;
    buf[1] = R_APDU_SW2_FUNCTION_NOT_SUPPORTED;
    *sendlen = 2; 
    break;
  case MEMORY_FAILURE:
    buf[0] = R_APDU_SW1_MEMORY_FAILURE;
    buf[1] = R_APDU_SW2_MEMORY_FAILURE;
    *sendlen = 2;
    break;
  case END_OF_FILE_BEFORE_REACHED_LE_BYTES:
    buf[0] = R_APDU_SW1_END_OF_FILE_BEFORE_REACHED_LE_BYTES;
    buf[1] = R_APDU_SW2_END_OF_FILE_BEFORE_REACHED_LE_BYTES;
    *sendlen= 2;
    break;
  }
}

void EmulateFelica::setIdm(uint8_t *uid)
{
}
