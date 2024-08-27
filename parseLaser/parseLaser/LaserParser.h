/*
 * Laser parser library for Arduino
 *
 * Simple and compact Laser parser designed for Arduino
 *
 * Author : Glinnes Hulden, Modified by A.Capelli to support longer Laser messages
 *
 * This work is distributed under license CC0.
 * Check https://creativecommons.org/publicdomain/zero/1.0/deed.en
 *
 * No Copyright
 *
 * The person who associated a work with this deed has dedicated the work
 * to the public domain by waiving all of his or her rights to the work
 * worldwide under copyright law, including all related and neighboring rights,
 * to the extent allowed by law.
 *
 * You can copy, modify, distribute and perform the work, even for commercial
 * purposes, all without asking permission. See Other Information below.
 *
 * Other Information
 *
 * In no way are the patent or trademark rights of any person affected by CC0,
 * nor are the rights that other persons may have in the work or in how the
 * work is used, such as publicity or privacy rights.
 * Unless expressly stated otherwise, the person who associated a work with
 * this deed makes no warranties about the work, and disclaims liability for
 * all uses of the work, to the fullest extent permitted by applicable law.
 * When using or citing the work, you should not imply endorsement by the
 * author or the affirmer.
 */

#ifndef __LaserParser_h__
#define __LaserParser_h__

#ifdef __amd64__
/* To use on my development platform */
#include <stddef.h>
#include <stdint.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#else
#include <Arduino.h>
#endif

namespace Laser {
  /*
   * Error codes
   */
  typedef enum {
      NO_ERROR,
      UNEXPECTED_CHAR,
      BUFFER_FULL,
      TYPE_TOO_LONG,
      INTERNAL_ERROR
  } ErrorCode;
}

/*
 * The library consists of a single template: LaserParser.
 */
template <size_t S> class LaserParser {

private:
  typedef void (*LaserErrorHandler)(void);
  typedef void (*LaserHandler)(void);
  typedef enum { INIT, ARG } State;

public:
  /*
   * maximum sentence size is 64 
   */
  static const uint8_t kSentenceMaxSize = 64;

private:
  /*
   * buffer to store the Laser sentence 
   */
  char mBuffer[kSentenceMaxSize];

  /*
   * Current index to store a char of the sentence
   */
  uint8_t mIndex;

  /*
   * Current index to store the index of an argument
   */
  uint8_t argCount;

  /*
   * A handler to notify a malformed sentence
   */
  LaserHandler mLaserHandler;

  /*
   * A handler to notify a malformed sentence
   */
  LaserErrorHandler mErrorHandler;

  /*
   * A handler for well formed but unrecognized sentences
   */
  // LaserDefaultHandler mDefaultHandler;



  /*
   * The current number of mHandlers
   */
  uint8_t mHandlerCount;

  /*
   * Parsing automaton variable
   */
  State mState;

  /*
   * mError
   */
  Laser::ErrorCode mError;


  /*
   * Temperature
   */
  float  Temp;

  /*
   * Distance
   */
  float  Dist;

  /*
   * Signal Quality
   */
  float  SigQual;
  
   /*
   * Signal Quality
   */
  int LaserError;
  

  /*
   * Call the Laser String Handler
   */
  void callLaserHandler(void)
  {
    if (mLaserHandler != NULL) {
      mLaserHandler();
    }
  }

  /*
   * Call the error handler if defined
   */
  void callErrorHandler(void)
  {
    if (mErrorHandler != NULL) {
      mErrorHandler();
    }
  }

  /*
   * Reset the parser
   */
  void reset() {
    mState = INIT;
    mIndex = 0;
    argCount = 0;
    mError = Laser::NO_ERROR;
	LaserError=0;
  }

  /*
   * Called when the parser encounter a char that should not be there
   */
  void unexpectedChar()
  {
    mError = Laser::UNEXPECTED_CHAR;
    callErrorHandler();
    reset();
  }

  /*
   * Called when the buffer is full because of a malformed sentence
   */
  void bufferFull()
  {
    mError = Laser::BUFFER_FULL;
    callErrorHandler();
    reset();
  }


  /*
   * Called when the state of the parser is not ok
   */
  void internalError()
  {
    mError = Laser::INTERNAL_ERROR;
    callErrorHandler();
    reset();
  }

  /*
   * retuns true is there is at least one byte left in the buffer
   */
  bool spaceAvail()
  {
    return (mIndex < kSentenceMaxSize);
  }



  /*
   * When all the sentence has been parsed, process it by calling the handler
   */
  void processSentence()
  {
    //D 0003.814 17.7  25.4
    if (LaserError==-1){
		LaserParserStringify stfy(this, 4);
		LaserError = atoi(&mBuffer[2]);
		
	} else{
		// for(int i=1;i<=argCount;i++){
			// // Serial.print("i");
			// // Serial.print(i);
			// // Serial.print(": ");
			// switch(i){
				// case 1:{
					// // Dist=convertToFloat(2,9);
					// LaserParserStringify stfy(this, 9);
					// Dist = atof(&mBuffer[2]);
					// // Serial.print(&mBuffer[2]);
					// // Serial.print(", Dist: ");
					// // Serial.println(Dist);
					// break;
				// }
				// case 2:
					// {   // SigQual=convertToFloat(10,13);
					// LaserParserStringify stfy(this, 14);
					// SigQual = atof(&mBuffer[10]);
					// // Serial.print(&mBuffer[10]);
					// break;
				// }
				// case 4:
					// {// Temp=convertToFloat(15,18);
					// LaserParserStringify stfy(this, 20);
					// Temp = atof(&mBuffer[16]);
					// // Serial.print(&mBuffer[16]);
					// break;
				// }
			// }
		// }
	}
	callLaserHandler();
  }

		
		
// // Function to convert a part of a char array to a float
// float convertToFloat(uint8_t  start, uint8_t end) {
    // // Null-terminate the substring
    // char temp[16];
    // uint8_t  length = end - start;
    // if (length >= sizeof(temp)) {
        // length = sizeof(temp) - 1;
    // }

    // for(uint8_t i=0;i<=length;i++){
		// temp[i]=mBuffer[start+i];
	// }
	
    // temp[length] = '\0'; // Null-terminate the substring

    // // Convert the substring to float
    // return std::atof(temp);
// }


  /*
   * Return true if inArgNum corresponds to an actual argument
   */
  bool validArgNum(uint8_t inArgNum)
  {
    return inArgNum <= argCount;
  }


public:
  /*
   * Constructor initialize the parser.
   */
  LaserParser() :
    mErrorHandler(NULL),
	mLaserHandler(NULL),
    mHandlerCount(0),
    mError(Laser::NO_ERROR)    // mDefaultHandler(NULL),
  {
    reset();
  }


  /*
   * Set the Laser handler which is called when a sentence is malformed
   */
  void setLaserHandler(LaserHandler inHandler)
  {
    mLaserHandler = inHandler;
  }


  /*
   * Set the error handler which is called when a sentence is malformed
   */
  void setErrorHandler(LaserErrorHandler inHandler)
  {
    mErrorHandler = inHandler;
  }

  /*
   * Set the default handler which is called when a sentence is well formed
   * but has no handler associated to
   */
  // void setDefaultHandler(LaserHandler inHandler)
  // {
    // mDefaultHandler = inHandler;
  // }

  /*
   * Give a character to the parser
   */
  void operator<<(char inChar)
  {

    switch (mState) {

      /* Waiting for the starting $ character */
      case INIT:
        mError = Laser::NO_ERROR;
        if (inChar == 'D') {
          mState = ARG;
        }
        else unexpectedChar();
        break;

      case ARG:
        if (spaceAvail()) {
          switch(inChar) {
            case ' ' :
			  mBuffer[mIndex++] = inChar;
			  argCount++;
              break;
            case 'E' :
			  mBuffer[mIndex++] = inChar;
              LaserError=-1;
			  break;
			case '\n' :
			  mBuffer[mIndex++] = inChar;
              processSentence();
			  reset();
            default :
              mBuffer[mIndex++] = inChar;
              break;
          }
        }
        else bufferFull();
        break;

      default:
        internalError();
        break;
    }
  }




  /*
   * Returns one of the arguments. 
   */
  float getTemp()
  {
    if (!LaserError & argCount>3) {
		
		LaserParserStringify stfy(this, 20);
		return atof(&mBuffer[16]);
    }
    else return 999;
  }


  float getDist( )
  {
    if (!LaserError) {
      LaserParserStringify stfy(this, 9);
	  Dist = atof(&mBuffer[2]);
      return Dist;
    }
    else return -1;
  }
  
  
   float getSigQual()
  {
    if (!LaserError & argCount>1) {
      LaserParserStringify stfy(this, 14);
	  SigQual = atof(&mBuffer[10]);
      return SigQual;
    }
    else return -1;
  }

   int getLaserError( )
  {
    return LaserError;
    }



    Laser::ErrorCode error() {
    return mError;
  }
  
  
  /*
   * LaserParserStringify is used internally to temporarely replace a char
   * in the buffer by a '\0' so that libc string functions may be used.
   * Instantiating a LaserParserStringify object in a pair of {} defines
   * a section in which the 'stringification' is done : the constructor
   * does that according to the arguments and se destructor restore the buffer.
   */
  class LaserParserStringify {
    uint8_t       mPos;
    char          mTmp;
    LaserParser<S> *mParent;
  public:
    LaserParserStringify(LaserParser<S> *inParent, uint8_t inPos) :
      mPos(inPos),
      mParent(inParent)
    {
      mTmp = mParent->mBuffer[mPos];
      mParent->mBuffer[mPos] = '\0';
    }
    ~LaserParserStringify()
    {
      mParent->mBuffer[mPos] = mTmp;
    }
  };
  
  
  
  
  
#ifdef __amd64__
  void printBuffer()
  {
    {
      LaserParserStringify stfy(this, startArgPos(0));
      printf("%s\n", mBuffer);
    }
    for (uint8_t i = 0; i < argCount(); i++) {
      uint8_t startPos = startArgPos(i);
      uint8_t endPos = endArgPos(i);
      {
        LaserParserStringify stfy(this, endPos);
        printf("%s\n", &mBuffer[startPos]);
      }
    }
  }
#endif
};

#endif
