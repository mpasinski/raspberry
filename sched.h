#include <sys/epoll.h>
#include <map>
#include <errno.h>
#include <queue>
#include <functional>
#include <algorithm>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <iostream>

#pragma once

class epollScheduler;

enum epollEvents {epollRead, epollWrite, epollError, epollTime};

class epollItem
{
    public:
       int fd;
       unsigned time;
       void *data;
       epollEvents event;
       void *callbackFunction(void*);
       void *(*callback)(int fd, epollEvents event, void* data);

       friend class epollScheduler;
       
};

class epollScheduler
{
  private:
    int epollFd;
    const unsigned int epollSize;
    std::multimap<long, epollItem> reqList;  //we could have multiple events registered on the same descriptor
    std::priority_queue<unsigned long, std::vector<unsigned long>, std::less<unsigned long> > timeSlices;

    void epollHandleEvents(struct epoll_event* events, int num);
    int runOnce(unsigned int time);
    long calculateNextBreak();

  public:
    epollScheduler(unsigned int);
    ~epollScheduler();
    
    bool addItem(epollItem &item);
    bool delItem(epollItem &item);
    bool addTimer(unsigned int time);
    bool removeTimer();

    void run();
   
    
};
