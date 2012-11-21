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

#include "sched.h"

void *epollItem::callbackFunction(void *arg)
{
    return this->callback(this->fd, this->event, this->data);
};




epollScheduler::epollScheduler(unsigned int size) : epollSize(size)
{
    epollFd = epoll_create(epollSize);

    //TODO: if (epollFd)
};

epollScheduler::~epollScheduler()
{
    close(epollFd);
    reqList.clear();
};

bool epollScheduler::addItem(epollItem &item)
{
    //TODO: check if already have this fd for e.g. READ and then only modify --> EPOLL_CTL_MOD
    struct epoll_event ev = {0};

    if (item.fd < 0)
        std::cout << "wrong file descriptor while adding to esys" << std::endl;

    if (item.event == epollRead)
    {
        ev.events = EPOLLIN|EPOLLPRI|EPOLLET;
    }
    else if (item.event == epollWrite)
    {
        ev.events = EPOLLOUT|EPOLLET;
    }
    else if (item.event == epollTime)
    {
        reqList.insert(std::pair<long,epollItem>(item.time, item));
        return true;
    }
    else
    {
        return false;
    }

    ev.data.fd = item.fd;

    if (epoll_ctl(epollFd, EPOLL_CTL_ADD, item.fd, &ev) != 0)
    {
        std::cout << "some error occured while adding descriptor " << item.fd << " to epoll!" << std::endl;
        return false;
    }
    reqList.insert(std::pair<int,epollItem>(item.fd, item));
    return true;
};

bool epollScheduler::delItem(epollItem &item)
{
    using namespace std;
    //TODO: check if already have this fd for e.g. READ and then only modify --> EPOLL_CTL_MOD
    struct epoll_event ev = {0};

    if (epoll_ctl(epollFd, EPOLL_CTL_DEL, item.fd, &ev) < 0)
    {
        std::cout << "descriptor " << item.fd << " removed from epoll" << std::endl;
    }

    pair<multimap<long, epollItem>::iterator, multimap<long, epollItem>::iterator> ret;
    multimap<long, epollItem>::iterator it;

    ret = reqList.equal_range(item.fd);
    for (it = ret.first; it != ret.second; it++)
    {
        if (((*it).second).event == item.event)
        {
            reqList.erase(it);
            cout << "element correctly removed from list" << endl;
            break;
        }
    }

    return true;
};

bool epollScheduler::addTimer(unsigned int time)
{
    timeSlices.push(time);
}

bool epollScheduler::removeTimer()
{
}

long epollScheduler::calculateNextBreak()
{
    if (this->timeSlices.empty())
        return -1;
    
    long next_time = timeSlices.top();
    timeSlices.pop();
    //TODO: recalculate other values

    return next_time;
};

void epollScheduler::run()
{
    if (this->reqList.empty() && this->timeSlices.empty())
    {
        std::cout << "epoll empty; not waiting" << std::endl;
        return;
    }

    int ret = 0;

    do
    {
        ret = this->runOnce(this->calculateNextBreak());
        
    } while (ret > 0 /*&& errno == EINTR*/);

    std::cout << "epoll returned: " << ret << " with error: " << errno << std::endl;
};

int epollScheduler::runOnce(unsigned int time)
{
    struct epoll_event events[epollSize];
    int ret;

    ret = epoll_wait(epollFd, events, epollSize, time);

    if (ret < 0)
    {
        std::cout << "epoll returned: " << ret << " with error: " << errno << std::endl;
        return -1;
    }

    else if (ret > 0)
    {
        epollHandleEvents(events, ret);
    }
    
    else
    {
        std::cout << "0 returned -> timeout" << std::cout;
    }
};

void epollScheduler::epollHandleEvents(struct epoll_event* events, int num)
{
    using namespace std;

    if (!events || num < 1)
    {
        return;
    }
    for (int i = 0; i < num; i++)
    {
        int fd = events[i].data.fd;

        pair<multimap<long, epollItem>::iterator, multimap<long, epollItem>::iterator> ret;
        multimap<long, epollItem>::iterator it;

        ret = reqList.equal_range(fd);
        
        if (events[i].events & EPOLLOUT)
        {
            for (it = ret.first; it != ret.second; it++)
            {
               if (((*it).second).event == epollWrite)
               {
                   ((*it).second).callbackFunction(NULL);
                   break;
               }
            }  
        }

        if ((events[i].events & EPOLLIN) || (events[i].events & EPOLLPRI))
        {
            for (it = ret.first; it != ret.second; it++)
            {
               if (((*it).second).event == epollRead)
               {
                   ((*it).second).callbackFunction(NULL);
                   break;
               }
            }  
        }

        //some error occured
        if ((events[i].events & EPOLLERR) || (events[i].events & EPOLLHUP))
        {
            //TODO:
        }
    }
};
