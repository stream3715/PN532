/**************************************************************************/
/*!
    @file     emulatefelica.h
    @author   Shohei Okubo
    @license  BSD

    Implemented using NFC forum documents & library of libnfc
*/
/**************************************************************************/

#ifndef __EMULATEFELICA_H__
#define __EMULATEFELICA_H__

#include "PN532.h"

#define NDEF_MAX_LENGTH 128  // altough ndef can handle up to 0xfffe in size, arduino cannot.
typedef enum {COMMAND_COMPLETE, TAG_NOT_FOUND, FUNCTION_NOT_SUPPORTED, MEMORY_FAILURE, END_OF_FILE_BEFORE_REACHED_LE_BYTES} responseCommand;

class EmulateFelica{

public:
  EmulateFelica(PN532Interface &interface) : pn532(interface), idmPtr(0), pmmPtr(0), sysPtr(0) {}

  void init();

  bool emulate(const uint16_t tgInitAsTargetTimeout = 0);

  /*
   * @param uid pointer to byte array of length 3 (uid is 4 bytes - first byte is fixed) or zero for uid 
   */
  void setIdm(uint8_t* uid = 0);
  void setPmm(uint8_t* uid = 0);
  void setSys(uint8_t* uid = 0);

private:
  PN532 pn532;
  uint8_t* idmPtr;
  uint8_t* pmmPtr;
  uint8_t* sysPtr;

  void setResponse(responseCommand cmd, uint8_t* buf, uint8_t* sendlen, uint8_t sendlenOffset = 0);
};

#endif
