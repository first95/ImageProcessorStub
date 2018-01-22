/*
	logger.cpp
	
	Declarations for image acquisition
	
	2017-01-06  JDW  Created.
*/
#include <logger.h>
using json = nlohmann::json;
using namespace std;

string Logger::getTimeString() {
	chrono::time_point<chrono::system_clock> now = chrono::system_clock::now();
	time_t tt = chrono::system_clock::to_time_t(now);
	stringstream ss;
	ss << put_time(localtime(&tt), timestampPattern.c_str());
	return ss.str();
}

void Logger::log(string message) {
	if(haveStatusstatusLogStream) {
		(*statusLogStream) << getTimeString() << message << endl;
		statusLogStream->flush();
	} else {
		cout << getTimeString() << message << endl;
	}
}
void Logger::log(string message, LogVerbosity level) {
	if(statusLoggingLevel >= level) {
		log(message);
	}
}

void Logger::init(json options) {
	string cur_key = "";
	try {
		string status_stream_name = "";
		cur_key = "verbosity";        statusLoggingLevel = options[cur_key];
		cur_key = "stream";           status_stream_name = options[cur_key];
		cur_key = "timestampPattern"; timestampPattern   = options[cur_key];
		if(status_stream_name == "cout") {
			statusLogStream = &cout;
			haveStatusstatusLogStream = true;
		} else if(status_stream_name == "cerr") {
			statusLogStream = &cerr;
			haveStatusstatusLogStream = true;
		} else  if(status_stream_name == "file") {
			string file_stream_name;
			string file_stream_path;
			cur_key = "fileName"; file_stream_name = options[cur_key];
			cur_key = "path";     file_stream_path = options[cur_key];
			string file_path = file_stream_path + file_stream_name;
			statusLogFile.open(file_path, ofstream::out | ofstream::app);
			statusLogStream = &statusLogFile;
			haveStatusstatusLogStream = true;
		} else {
			throw runtime_error("Logging stream must be one of: \"cout\", \"cerr\", \"file\"");
		}
	} catch (domain_error e) {
		cerr << "JSON field missing or corrupted.  Please see example file in config directory."
			 << endl << "While reading key \"" << cur_key << "\" in logging section: "
			 << e.what() << endl;
		throw(e);
		return;
	}
}

void Logger::logError  (string message) { log("E " + message, LogVerbosity::LOG_VERB_ERROR  ); }
void Logger::logWarning(string message) { log("W " + message, LogVerbosity::LOG_VERB_WARNING); }
void Logger::logInfo   (string message) { log("I " + message, LogVerbosity::LOG_VERB_INFO   ); }
void Logger::logDebug  (string message) { log("D " + message, LogVerbosity::LOG_VERB_DEBUG  ); }
