/*
 * Thread_Pool1.h
 *
 *  Created on: 2019-9-18
 *      Author: siasunhebo
 */

#ifndef THREAD_POOL1_H_
#define THREAD_POOL1_H_


void pool_init (int max_thread_num);
int pool_add_worker (void *(*process) (void *arg), void *arg);
int pool_destroy ();
void * thread_routine (void *arg);
void * myprocess (void *arg);
int Thread_Pool (int argc, char **argv);
void Pthread_Analy();
void * myprocess0 (void *arg);
void * myprocess1 (void *arg);
void * myprocess2 (void *arg);
void * myprocess3 (void *arg);
void * myprocess4 (void *arg);

#endif /* THREAD_POOL1_H_ */
