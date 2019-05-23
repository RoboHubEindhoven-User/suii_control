#include "TaskWithAction.h"

TaskWithAction::TaskWithAction(void) : Task()
{
    this->setAction(-1);
}

void TaskWithAction::copyFromTask(Task* t)
{
    this->setType(t->type);
    this->setSrc(t->src);
    this->setDest(t->dest);
    this->setObj(t->obj);
    this->setContainer(t->container);
}

void TaskWithAction::setAction(int id)
{
    this->action = id;
    this->action_str = protocol.lookUpTaskActionID(id);
}

void TaskWithAction::printTaskData(void)
{
    std::cout << "Action: " << this->action_str << "; "; Task::printTaskData();
}