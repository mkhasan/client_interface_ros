/*
 * queue.h
 *
 *  Created on: Feb 23, 2019
 *      Author: hasan
 */

#ifndef SRC_QUEUE_H_
#define SRC_QUEUE_H_

#include <pthread.h>
#include <string.h>

namespace client_interface {


class Speedo;

class queue {

	friend class Speedo;

	char name[255];
	enum{BUFF_SIZE = 513};
	int head;
	int tail;
	int currSize;
	bool usable;

	static char buffer[BUFF_SIZE];

	pthread_mutex_t lock;
	pthread_cond_t  cond;

	queue();
	~queue();

	/*
	void set_name(const char *_name ) {
		strcpy(name, _name);
	}
	*/


public:

	void initialize();
	void finalize();
	void add(const char *data, int len);
	int remove(char *data);
	int get_size() {
		return (BUFF_SIZE-1);
	}


};

}

#endif /* SRC_QUEUE_H_ */
