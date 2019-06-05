/**
 * filename:    Task.h
 * brief:       Headers for Task class
 * desc:        Task class is our representation of a task
 */

#ifndef TASK_H_
#define TASK_H_

#include <iostream>
#include "TaskProtocol.h"

class Task 
{
    protected:
        TaskProtocol protocol;
    public:
        Task(void);

        int type;
        std::string type_str;
        int src;
        std::string src_str;
        int dest;
        std::string dest_str;
        int obj;
        std::string obj_str;
        int container;
        std::string container_str;
        bool picked;
        bool placed;
        
        void setType(int type);
        void setSrc(int src);
        void setDest(int dest);
        void setObj(int obj);
        void setContainer(int container);

        bool isEqualTo(Task* t);
        bool isDestSameAsSrc(void);
        void printTaskData(void);
};

#endif