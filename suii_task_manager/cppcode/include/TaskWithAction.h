
#ifndef TASK_WITH_ACTION_H
#define TASK_WITH_ACTION_H

#include "Task.h"
#include "TaskProtocol.h"

class TaskWithAction : public Task
{
    public:
        TaskWithAction(void);
        
        int action;
        std::string action_str;
        void copyFromTask(Task* t);
        void setAction(int id);
        void printTaskData(void);
};

#endif