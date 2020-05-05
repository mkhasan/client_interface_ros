/*
 * robot_exception.h
 *
 *  Created on: Oct 22, 2019
 *      Author: kict
 */

#ifndef CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_ROBOT_EXCEPTION_H_
#define CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_ROBOT_EXCEPTION_H_

#include <sstream>

#define THROW(exceptionClass, message) throw exceptionClass(__FILE__, \
__LINE__, (message) )


class RobotException : public std::exception
{
  // Disable copy constructors
	RobotException& operator=(const RobotException&);
	std::string file_;
	int line_;
	std::string e_what_;
	int errno_;
public:
	explicit RobotException (std::string file, int line, int errnum)
    : file_(file), line_(line), errno_(errnum) {
      std::stringstream ss;
#if defined(_WIN32) && !defined(__MINGW32__)
      char error_str [1024];
      strerror_s(error_str, 1024, errnum);
#else
      char * error_str = strerror(errnum);
#endif
      ss << "Robot Exception (" << errno_ << "): " << error_str;
      ss << ", file " << file_ << ", line " << line_ << ".";
      e_what_ = ss.str();
	}
	explicit RobotException (std::string file, int line, const char * description)
		: file_(file), line_(line), errno_(0) {
		std::stringstream ss;
		ss << "Robot Exception: " << description;
		ss << ", file " << file_ << ", line " << line_ << ".";
		e_what_ = ss.str();
	}
	virtual ~RobotException() throw() {}
	RobotException (const RobotException& other) : line_(other.line_), e_what_(other.e_what_), errno_(other.errno_) {}

	int getErrorNumber () const { return errno_; }

	virtual const char* what () const throw () {
		return e_what_.c_str();
	}

};






#endif /* CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_ROBOT_EXCEPTION_H_ */
