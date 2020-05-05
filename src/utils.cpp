/*
 * utils.c
 *
 *  Created on: Feb 17, 2019
 *      Author: kict
 */

#include "client_interface/utils.h"
#include "client_interface/client_interface.h"

#include <iostream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <assert.h>

int Gap(const struct tm &tm1, const struct tm &tm2);

using namespace std;

extern "C" {

void print_time(char *str) {
	struct timeval tv;
	struct timezone tz;
	struct tm *tm;
	gettimeofday(&tv, &tz);
	tm=localtime(&tv.tv_sec);
	sprintf(str, "%d:%02d:%02d_%d", tm->tm_hour, tm->tm_min,
		  tm->tm_sec, tv.tv_usec);
}

}


int StrToTime(const char *text, struct tm * tm) {


	/*
	for (vector<string>::const_iterator it = results.begin(); it != results.end(); ++it) {
		cout << *it << endl;
	}
	*/

	string str(text);

	//cout << "str is " << str << endl;
	vector<std::string> results;
	try {


		boost::split(results, str, [](char c){return c == ':';});

		assert(results.size() == 3);
		tm->tm_hour = stoi(results[0]);
		tm->tm_min = stoi(results[1]);
		boost::split(results, results[2], [](char c){return c == '_';});
		assert(results.size() == 2);
		tm->tm_sec = stoi(results[0]);

 	}
	catch (const exception & e) {
		cout << "Error occured " << e.what() << endl;
		return -1;
	}

	//cout << "str: " << text << " sec " << tm->tm_sec << endl;
	return 0;
	//cout << curTime << endl;

}

int Gap(const char * str1, const char * str2) {
	struct tm tm1, tm2;
	if(StrToTime(str1, &tm1) == 0 && StrToTime(str2, &tm2) == 0) {
		return Gap(tm1, tm2);
	}
	else
		THROW(RobotException, "Error in getting Time gap");
}

int Gap(const struct tm &tm1, const struct tm &tm2) {
	//int sec1 = tm1.tm_hour*3600+tm1.tm_min*60+tm1.tm
	auto secs = [](const struct tm &t) {return (t.tm_hour*3600+t.tm_min*60+t.tm_sec); };
	//cout << "tm2 is " << secs(tm2) << " and tm1 is " << secs(tm1) << endl;
	return secs(tm1) - secs(tm2);

}
