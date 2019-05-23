#include "TaskList.h"

TaskList::TaskList()
{
    this->capacity = DEFAULT_CAPACITY;
}

TaskList::TaskList(int capacity)
{
    this->capacity = capacity;
}

TaskList::~TaskList()
{
    // free task list memory
    for (unsigned int i = 0; i < this->taskList.size(); i++)
    {
        delete this->taskList[i];
    }
}

int TaskList::indexTask(int type, int src, int dest, int obj, int container)
{
    Task t;
    t.setType(type);
    t.setSrc(src);
    t.setDest(dest);
    t.setObj(obj);
    t.setContainer(container);
    for (unsigned int i = 0; i < this->taskList.size(); i++)
    {
        if (this->taskList[i]->isEqualTo(&t))
            return i;
    }
    return -1;
}

bool TaskList::isEmpty(void)
{
    return this->taskList.size() == 0;
}

bool TaskList::isFull(void)
{
    return this->taskList.size() == this->capacity;
}

void TaskList::clearTask(void)
{
    this->taskList.clear();
}

bool TaskList::addTask (Task t)
{
    if (!this->isFull())
    {
        Task* task;
        task = new Task;
        task->setType(t.type);
        task->setSrc(t.src);
        task->setDest(t.dest);
        task->setObj(t.obj);
        task->setContainer(t.container);
        this->taskList.push_back(task);
        return true;
    }
    return false;
}

bool TaskList::addTask (Task* t)
{
    if (!this->isFull())
    {
        Task* task;
        task = new Task;
        task->setType(t->type);
        task->setSrc(t->src);
        task->setDest(t->dest);
        task->setObj(t->obj);
        task->setContainer(t->container);
        this->taskList.push_back(task);
        return true;
    }
    return false;
}

bool TaskList::removeTask(Task t)
{
    int index = indexTask(t.type, t.src, t.dest, t.obj, t.container);
    if (index == -1)
        return false;
    this->taskList.erase(this->taskList.begin() + index);
    return true;
}

bool TaskList::removeTask(Task* t)
{
    int index = indexTask(t->type, t->src, t->dest, t->obj, t->container);
    if (index == -1)
        return false;
    this->taskList.erase(this->taskList.begin() + index);
    return true;
}

bool TaskList::getNextObjToPick(Task& t, int& index)
{
    for (unsigned int i = 0; i < this->taskList.size(); i++)
    {
        if (!this->taskList[i]->picked)
        {
            t.setType(this->taskList[i]->type);
            t.setSrc(this->taskList[i]->src);
            t.setDest(this->taskList[i]->dest);
            t.setObj(this->taskList[i]->obj);
            t.setContainer(this->taskList[i]->container);
            index = i;
            return true;
        }
    }
    return false;
}

TaskList TaskList::getTasksBySrc (int src)
{
    TaskList list;
    for (unsigned int i = 0; i < this->taskList.size(); i++)
    {
        if (this->taskList[i]->src == src)
        {
            list.addTask(this->taskList[i]);
        }
    }
    return list;
}

TaskList TaskList::getTasksByDest (int dest)
{
    TaskList list;
    for (unsigned int i = 0; i < this->taskList.size(); i++)
    {
        if (this->taskList[i]->dest == dest)
        {
            list.addTask(this->taskList[i]);
        }
    }
    return list;
}

std::map<int, std::string> TaskList::getUniqueDest(void)
{
    std::map<int, std::string> uniqueDest;
    for (unsigned int i = 0; i < this->taskList.size(); i++)
    {
        uniqueDest.insert(std::make_pair(this->taskList[i]->dest, this->taskList[i]->dest_str));
    }
    return uniqueDest;
}

std::map<int, std::string> TaskList::getUniqueSrc(void)
{
    std::map<int, std::string> uniqueSrc;
    for (unsigned int i = 0; i < this->taskList.size(); i++)
    {
        uniqueSrc.insert(std::make_pair(this->taskList[i]->src, this->taskList[i]->src_str));
    }
    return uniqueSrc;
}

void TaskList::sortBySrcAndDest(void)
{
    std::sort(this->taskList.begin(), this->taskList.end(), this->sortBySourceAndDest);
}

void TaskList::printTaskList(void)
{
    for (unsigned int i = 0; i < this->taskList.size(); i++)
    {
        this->taskList[i]->printTaskData();
    }
}