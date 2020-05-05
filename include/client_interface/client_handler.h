/*
 * client_handler.h
 *
 *  Created on: Feb 24, 2019
 *      Author: kict
 */

#ifndef CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_CLIENT_HANDLER_H_
#define CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_CLIENT_HANDLER_H_



#include "ace/Svc_Handler.h"
#include "client_interface/queue.h"

#if !defined (ACE_LACKS_PRAGMA_ONCE)
# pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */

#include "ace/SOCK_Stream.h"

#include "client_interface/speedo.h"

namespace client_interface {

class CmdProcessor;
class Connector;

class Client_Handler : public ACE_Svc_Handler <ACE_SOCK_STREAM, ACE_NULL_SYNCH>
{
	enum{MAX_CMD_LEN = 20};
	queue *q;
	//CmdProcessor *cmdProcessor;
	queue *cmd_q;
	Connector * _connector;
public:

  Client_Handler (void);

  void destroy (void);

  int open (void *acceptor);

  int close (u_long flags = 0);

  
  virtual int handle_close (ACE_HANDLE handle = ACE_INVALID_HANDLE,
                            ACE_Reactor_Mask mask = ACE_Event_Handler::ALL_EVENTS_MASK);

protected:

  int svc (void);
  
  int handle_input (ACE_HANDLE handle);

  int process (char *rdbuf, int rdbuf_len);

  ~Client_Handler (void);

  int ProcessCmd(const char * p, int dataLen, char *cmdStr);
};

}




#endif /* CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_CLIENT_HANDLER_H_ */
