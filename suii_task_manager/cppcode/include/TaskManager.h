#ifndef TASK_MANAGER_H_
#define TASK_MANAGER_H_

#include "TaskList.h"
#include "TaskWithAction.h"
#include <vector>

#define MAX_HOLDING_CAPACITY    3

class TaskManager {
    private:
        TaskProtocol protocol;
    public:
        void formatDrive (int dest, std::vector<TaskWithAction*>& result);
        void formatPick (TaskList& taskList, std::vector<TaskWithAction*>& result);
        void formatPickTask (Task* task, std::vector<TaskWithAction*>& result);
        void formatPlace (TaskList& taskList, std::vector<TaskWithAction*>& result);
        void formatPlaceTask (Task* task, std::vector<TaskWithAction*>& result);
        
        void formatOneSource (TaskList& taskList, std::vector<TaskWithAction*>& result);
        void formatSetOfTasks (TaskList& taskList, std::vector<TaskWithAction*>& result);
        bool optimizeList(TaskList& taskList, std::vector<TaskWithAction*>& result);
};

#endif