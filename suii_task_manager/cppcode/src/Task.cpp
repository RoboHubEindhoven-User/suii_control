#include "Task.h"

Task::Task()
{
    this->setType(-1);
    this->setSrc(-1);
    this->setDest(-1);
    this->setObj(-1);
    this->setContainer(-1);
    this->picked = false;
    this->placed = false;
}

void Task::setType(int type)
{
    this->type = type;
    this->type_str = protocol.lookUpTaskTypeID(type);
}

void Task::setSrc(int src)
{
    this->src = src;
    this->src_str = protocol.lookUpLocationID(src);
}

void Task::setDest(int dest)
{
    this->dest = dest;
    this->dest_str = protocol.lookUpLocationID(dest);
}

void Task::setObj(int obj)
{
    this->obj = obj;
    this->obj_str = protocol.lookUpObjectID(obj);
}

void Task::setContainer(int container)
{
    this->container = container;
    this->container_str = protocol.lookUpContainerID(container);
}

bool Task::isDestSameAsSrc(void)
{
    return dest == src;
}

bool Task::isEqualTo(Task* t)
{
    return t->type == this->type && t->src == this->src && t->dest == this->dest && t->obj == this->obj;
}

void Task::printTaskData(void)
{
    // Print oneline
    std::cout << "Type: " << this->type_str << "; Src: " << this->src_str << "; Dest: " << this->dest_str << "; Obj: " << this->obj_str << "; Container: " << this->container_str << "\n";
    // Printing with a lot of newlines
    // std::cout << "--- Task Data --- \n";
    // std::cout << "Source ID: " << this->src << "\n";
    // std::cout << "Source String: " << this->src_str << "\n";
    // std::cout << "Dest ID: " << this->dest << "\n";
    // std::cout << "Dest String: " << this->dest_str << "\n";
    // std::cout << "Obj ID: " << this->obj << "\n";
    // std::cout << "Obj String: " << this->obj_str << "\n";
    // std::cout << "Destination same as source? " << this->isDestSameAsSrc() << "\n";
}