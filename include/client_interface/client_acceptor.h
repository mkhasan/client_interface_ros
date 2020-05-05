/*
 * client_acceptor.h
 *
 *  Created on: Feb 24, 2019
 *      Author: kict
 */

#ifndef CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_CLIENT_ACCEPTOR_H_
#define CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_CLIENT_ACCEPTOR_H_

#include "ace/Acceptor.h"

#include <boost/algorithm/string.hpp>

#if !defined (ACE_LACKS_PRAGMA_ONCE)
# pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */

#include "ace/SOCK_Acceptor.h"

#include "client_interface/client_handler.h"
#include "client_interface/queue.h"
#include "client_interface/speedo.h"

#include <vector>

typedef ACE_Acceptor <client_interface::Client_Handler, ACE_SOCK_ACCEPTOR> Client_Acceptor;


class Client_Handler;

namespace client_interface {
class Connector : public Client_Acceptor {
	friend class Client_Handler;
	friend class Speedo;

	queue *q;
	queue *cmd_q;

	std::vector<std::string> remoteList;

	bool speedo_connected;

public:
	Connector(queue *_q, queue *_cmd_q);
	~Connector();

	std::vector<std::string> GetRemoteList() const;

	void AddToRemoteList(const std::string & addr);

	int RemoveFromRemoteList(const std::string & addr);
	int RemoteConnectionCount() const;
/*
	int
	handle_input (ACE_HANDLE handle)
	{
		return -1;
	}
*/

};
}

#endif /* CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_CLIENT_ACCEPTOR_H_ */
