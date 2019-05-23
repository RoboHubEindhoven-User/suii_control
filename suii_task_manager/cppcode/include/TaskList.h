/**
 * filename:    Task.h
 * brief:       TaskList serves as a manager for many Tasks
 */

#ifndef TASK_LIST_H_
#define TASK_LIST_H_

#include "TaskProtocol.h"
#include "Task.h"

#include <vector>
#include <algorithm>
#include <iostream>

#define DEFAULT_CAPACITY    20 

class TaskList{
    private:
        unsigned int capacity;
        TaskProtocol protocol;
        static bool sortBySourceAndDest(Task* t1, Task* t2)
        {
            return (t1->src < t2->src) ||                           // sort by src 
                   (t1->src == t2->src && (t1->dest < t2->dest));   // if the same src, sort by desc
        }
    public:
        std::vector<Task*> taskList;

        TaskList();
        TaskList(int capacity);
        ~TaskList();

        bool isEmpty(void);
        bool isFull(void);

        int indexTask (int type, int src, int dest, int obj, int container);
        bool getNextObjToPick(Task& t, int& index);

        void clearTask (void);
        bool addTask (Task t);
        bool addTask (Task* t);
        bool removeTask (Task t);
        bool removeTask (Task* t);
        
        TaskList getTasksBySrc (int src);
        TaskList getTasksByDest (int dest);
        std::map<int, std::string> getUniqueDest (void);
        std::map<int, std::string> getUniqueSrc (void);

        void sortBySrcAndDest(void);
        void printTaskList(void);
};

#endif