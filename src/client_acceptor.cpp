/*
 * client_acceptor.cpp
 *
 *  Created on: Mar 21, 2019
 *      Author: kict
 */


#include "client_interface/client_acceptor.h"

using namespace std;
using namespace client_interface;

static pthread_mutex_t lock;

Connector::Connector(queue *_q, queue *_cmd_q): ACE_Acceptor(), q(_q), cmd_q(_cmd_q), speedo_connected(false) {
	pthread_mutex_init(&lock, NULL);
}

Connector::~Connector() {
	pthread_mutex_destroy(&lock);
}

vector<string> Connector::GetRemoteList() const {

	vector<string> temp;

	pthread_mutex_lock(&lock);
	temp = remoteList;
	pthread_mutex_unlock(&lock);


	return temp;

}

void Connector::AddToRemoteList(const std::string & addr) {
	std::vector<std::string>::iterator it;

	pthread_mutex_lock(&lock);
	bool found = false;
	for (it=remoteList.begin(); it!=remoteList.end(); ++it) {
		std::vector<std::string> results;
		boost::split(results, *it, [](char c){return c == ':';});
		if(results.size() >= 2) {
			auto is_prefix = [](const std::string & p, const std::string &q)
					{ int k=0;
					  for(k=0; k<p.length(); k++)
						   if(q.at(k) != p.at(k))
							   break;
						return k == p.length() ? true : false;
					};


			if(is_prefix(results[0], addr)) {		// results[0] is a prefix of addr
				*it = addr;
				found = true;
				break;
			}
		}
	}
	if(!found)
		remoteList.push_back(addr);

	pthread_mutex_unlock(&lock);
}

int Connector::RemoveFromRemoteList(const std::string & addr) {
	std::vector<std::string>::const_iterator it;

	pthread_mutex_lock(&lock);

	std::vector<std::string> temp;
	bool flag = false;
	for(it=remoteList.begin(); it!=remoteList.end(); ++it) {
		if(*it == addr) {
			flag = true;
		}
		else
			temp.push_back(*it);

	}


	if(flag)		// no need to replace becuase they are same
		remoteList = temp;

	pthread_mutex_unlock(&lock);
	return (flag ? 0 : -1);


}

int Connector::RemoteConnectionCount() const {

	pthread_mutex_lock(&lock);
	int size = remoteList.size();
	pthread_mutex_unlock(&lock);
	return size;
}

