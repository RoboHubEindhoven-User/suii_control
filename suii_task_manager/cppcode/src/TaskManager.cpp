#include "TaskManager.h"
#include <map>


void TaskManager::formatDrive (int dest, std::vector<TaskWithAction*>& result)
{
    TaskWithAction* twa = new TaskWithAction();
    twa->setDest(dest);
    twa->setAction(protocol.lookUpTaskActionString("DRIVE"));
    result.push_back(twa);
}

void TaskManager::formatPickTask (Task* task, std::vector<TaskWithAction*>& result)
{
    TaskWithAction* twa = new TaskWithAction();
    twa->copyFromTask(task);
    twa->setAction(protocol.lookUpTaskActionString("PICK"));
    std::cout << "Picking: "; twa->printTaskData();
    result.push_back(twa);
}

void TaskManager::formatPick (TaskList& taskList, std::vector<TaskWithAction*>& result)
{
    for (unsigned int i = 0; i < taskList.taskList.size(); i++)
    {
        this->formatPickTask(taskList.taskList[i], result);
    }
}

void TaskManager::formatPlaceTask (Task* task, std::vector<TaskWithAction*>& result)
{
    TaskWithAction* twa = new TaskWithAction();
    twa->copyFromTask(task);
    twa->setAction(protocol.lookUpTaskActionString("PLACE"));
    std::cout << "Placing: "; twa->printTaskData();
    result.push_back(twa);
}

void TaskManager::formatPlace (TaskList& taskList, std::vector<TaskWithAction*>& result)
{
    for (unsigned int i = 0; i < taskList.taskList.size(); i++)
    {
        this->formatPlaceTask(taskList.taskList[i], result);
    }
}


void TaskManager::formatSetOfTasks (TaskList& taskList, std::vector<TaskWithAction*>& result)
{
    std::map<int, std::string> uniqueSrc = taskList.getUniqueSrc();

    if (uniqueSrc.size() > 0)
    {
        std::cout << "\nPicking objects from " << uniqueSrc.size() << " source(s)\n";
        std::map<int, std::string>::iterator it = uniqueSrc.begin();
        while (it != uniqueSrc.end())
        {
            std::cout << "\nPicking objects from " << it->second << "...\n";
            formatDrive(it->first, result);                         // drive to first location
            TaskList pickUp = taskList.getTasksBySrc(it->first);    // get all the tasks with that source

            // Pick up selectively
            for (unsigned int i = 0; i < pickUp.taskList.size(); i++)
            {
                if (pickUp.taskList[i]->isDestSameAsSrc())
                {
                    std::cout << "Object has the same destination as source!\n";
                    this->formatPickTask(pickUp.taskList[i], result);
                    this->formatPlaceTask(pickUp.taskList[i], result);
                    taskList.removeTask(pickUp.taskList[i]);
                }
                else 
                {
                    this->formatPickTask(pickUp.taskList[i], result);
                }
            }
            it++;
        }
    }
    
    std::map<int, std::string> uniqueDest = taskList.getUniqueDest();

    if (uniqueDest.size() > 0)
    {
        std::cout << "\nPlacing objects at " << uniqueSrc.size() << " destination(s)\n";
        std::map<int, std::string>::iterator it = uniqueDest.begin();
        while (it != uniqueDest.end())
        {
            std::cout << "\nPlacing objects at " << it->second << "...\n";
            formatDrive(it->first, result);                          // drive to first location
            TaskList dropOff = taskList.getTasksByDest(it->first);   // get all the tasks with that dest
            formatPlace(dropOff, result);                            // place them
            it++;
        }
    }
}

bool TaskManager::optimizeList(TaskList& taskList, std::vector<TaskWithAction*>& result)
{
    TaskList holdingList(MAX_HOLDING_CAPACITY);
    bool formatNow = false;
    std::cout << "Sorting task list by source and destination...\n";
    taskList.sortBySrcAndDest();
    taskList.printTaskList();
    
    while (!taskList.isEmpty())
    {
        if (!holdingList.isFull())
        {
            Task task;
            int index;
            if (!taskList.getNextObjToPick(task, index)) break; // picked everything

            if (holdingList.isEmpty())
            {
                holdingList.addTask(task); taskList.taskList[index]->picked = true;
            }
            else 
            {
                if (task.src == holdingList.taskList[0]->src)
                {
                    holdingList.addTask(task); taskList.taskList[index]->picked = true;
                }
                else formatNow = true;
            }
        }
        else formatNow = true;


        if (formatNow) 
        {
            formatSetOfTasks(holdingList, result);

            for (unsigned int i = 0; i < holdingList.taskList.size(); i++)
            {
                taskList.removeTask(holdingList.taskList[i]);
            }
            holdingList.clearTask();
            formatNow = false;
        }
    }

    if (!holdingList.isEmpty())
    {
        formatSetOfTasks(holdingList, result);

    for (unsigned int i = 0; i < holdingList.taskList.size(); i++)
    {
        taskList.removeTask(holdingList.taskList[i]);
    }
    holdingList.clearTask();
    }

    return true;
}