/*
	logger.h
	
	Declarations for logging
	
	2017-01-06  JDW  Created.
*/

#ifndef __PCG_LOG_H__
#define __PCG_LOG_H__

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <chrono> // C++11
#include <ctime> // Bits that C++11 is missing
#include "json.hpp"
using json = nlohmann::json;
using namespace std;

typedef enum LogVerbosityTag {
	LOG_VERB_SILENT  = 0,
	LOG_VERB_ERROR   = 1,
	LOG_VERB_WARNING = 2,
	LOG_VERB_INFO    = 3,
	LOG_VERB_DEBUG   = 4,
} LogVerbosity;

class Logger {
private:
	int statusLoggingLevel = LogVerbosity::LOG_VERB_INFO;
	ostream * statusLogStream;
	ofstream statusLogFile;
	bool haveStatusstatusLogStream = false;

	string getTimeString();
	string timestampPattern = "";
	void log(string message);
	
public:
	void init(json options);
	void log(string message, LogVerbosity level);
	void logError   (string message);
	void logWarning (string message);
	void logInfo    (string message);
	void logDebug   (string message);
};


#endif // __PCG_LOG_H__
