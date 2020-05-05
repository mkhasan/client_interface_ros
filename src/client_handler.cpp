// client_handler.cpp,v 1.9 2000/03/19 20:09:22 jcej Exp





#include "client_interface/client_acceptor.h"
#include "client_interface/client_handler.h"

#include "ros/ros.h"


#include <iostream>
#include <vector>

using namespace std;

namespace client_interface {


Client_Handler::Client_Handler (void): q(NULL), cmd_q(NULL), _connector(NULL)
{
}

Client_Handler::~Client_Handler (void)
{
  this->peer ().close ();
}

void
Client_Handler::destroy (void)
{
  this->reactor ()->remove_handler (this,
                                    ACE_Event_Handler:: READ_MASK | ACE_Event_Handler::DONT_CALL);

  delete this;
}

int
Client_Handler::open (void *_acceptor)
{
	Connector *acceptor = (Connector *) _acceptor;

	_connector = acceptor;



	q = acceptor->q;
	cmd_q = acceptor->cmd_q;

	//  return this->activate (THR_DETACHED);

	this->reactor (acceptor->reactor ());

	ACE_INET_Addr addr;

	if (this->peer ().get_remote_addr (addr) == -1)
		return -1;

	if (this->reactor ()->register_handler (this,
										  ACE_Event_Handler::READ_MASK) == -1)
	ACE_ERROR_RETURN ((LM_ERROR,
					   "(%P|%t) can't register with reactor\n"),
					  -1);

	ACE_DEBUG ((LM_DEBUG,
			  "(%P|%t) connected with %s\n",
			  addr.get_host_name ()));

	/* Always return zero on success.  */

	char addrStr[255];
	addr.addr_to_string(addrStr, sizeof(addrStr));
	_connector->AddToRemoteList(addrStr);
	ROS_DEBUG("Got a connection %s", addrStr);
	vector<string> list = _connector->GetRemoteList();
	for(vector<string>::const_iterator it = list.begin(); it != list.end(); ++it)
		ROS_ERROR("remote member: %s", it->c_str());
	return 0;

}

int
Client_Handler::handle_input (ACE_HANDLE handle)
{
  ACE_UNUSED_ARG (handle);

  char buf[BUFSIZ];

  return this->process (buf, sizeof (buf));
}

int
Client_Handler::handle_close (ACE_HANDLE handle,
                              ACE_Reactor_Mask mask)
{
  ACE_UNUSED_ARG (handle);
  ACE_UNUSED_ARG (mask);


  ACE_INET_Addr addr;

  if (this->peer ().get_remote_addr (addr) == -1)
  	return -1;

	char addrStr[255];
	addr.addr_to_string(addrStr, sizeof(addrStr));

	ROS_DEBUG("Connection gone %s", addrStr);
	if (_connector->RemoveFromRemoteList(addrStr) != 0) {
		ROS_ERROR("Client_Handler::handle_close: remote addr not found in the connector");
	}
	delete this;
	return 0;
}

int Client_Handler::ProcessCmd(const char * data, int dataLen,  char *cmdStr) {
	for(int i=0; i<MAX_CMD_LEN && i<dataLen; ++i) {
		cmdStr[i] = data[i];
		if(cmdStr[i] == NULL)
			return i;
		if(i>1 && cmdStr[i] == '\n' && cmdStr[i-1] == '\r') {
			cmdStr[i+1] = NULL;
			return i+1;
		}

		if(i == dataLen) {
			cmdStr[dataLen] = NULL;
			return dataLen;
		}
		cmdStr[MAX_CMD_LEN] = NULL;
		return MAX_CMD_LEN;

	}

}

int
Client_Handler::process (char *rdbuf,
                         int rdbuf_len)
{
    ssize_t bytes_read = -1;
    static string lastCmd = "";


    switch ( (bytes_read = this->peer ().recv (rdbuf, rdbuf_len)) )
    {
    case -1: // Complain and leave
      ACE_ERROR_RETURN ((LM_ERROR,
                         "(%P|%t) %p bad read\n",
                         "client"),
                        -1);
    case 0: // Complain and leave
      ACE_ERROR_RETURN ((LM_ERROR,
                         "(%P|%t) closing daemon (fd = %d)\n",
                         this->get_handle ()),
                        -1);
    default: // Show the data
        // NULL-terminate the string before printing it.
      rdbuf[bytes_read] = 0;
      lastCmd = rdbuf;
      ROS_DEBUG(rdbuf);

      if(_connector->speedo_connected)
    	  cmd_q->add(rdbuf, bytes_read);
      /*
      ACE_DEBUG ((LM_DEBUG,
                  "(%P|%t) from client: %s",
                  rdbuf));
                  */


      //int sent = this->peer().send_n("hello", strlen("hello"));

#if 0
      if(0 && lastCmd.length() && lastCmd != string("PING\r\n")) {
    	  ROS_DEBUG("Last cmd %s", lastCmd.c_str());
    	  int ret = cmdProcessor->GetCmd(lastCmd);
    	  if(ret != 0) {
    		  ROS_WARN("Sending cmd %s too quickly", lastCmd.c_str());
    	  }
      }
#endif

    }


  return 0;
}


int
Client_Handler::svc(void)
{

	char buf[BUFSIZ];


	while( 1 )
	{
		ROS_WARN("iN SVC");

		const int SIZE = q->get_size();
		static vector<char> data;
		data.resize(SIZE);


		ROS_DEBUG("Trying to get data");
		int len = q->remove(&data[0]);

		ROS_DEBUG("data got %d bytes", len);
		if(len > 0) {
		  int sent = this->peer().send_n(&data[0], len);
		  if(sent != len)
			  ROS_ERROR("Client_Handler::process: Error in sending data to client");

		  if(sent < 0)
			  return -1;

		}


		usleep(2000000);
	}

	return 0;
}

int
Client_Handler::close(u_long flags)
{
  ACE_UNUSED_ARG (flags);

  this->destroy ();

  return 0;
}

}
