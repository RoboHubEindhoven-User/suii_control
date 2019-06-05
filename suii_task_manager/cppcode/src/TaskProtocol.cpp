/**
 * filename:    TaskProtocol.cpp
 * brief:       Implement dictionary for protocols
 */

#include "TaskProtocol.h"

// Populate dictionary data here
TaskProtocol::TaskProtocol()
{
    // --- Populating Task Type Dict --- //
    this->taskTypeDict.insert(std::make_pair(1, "TRANSPORTATION"));
    this->taskTypeDict.insert(std::make_pair(2, "NAVIGATION"));

    // --- Populating Task Action Dict --- //
    this->taskActionDict.insert(std::make_pair(1, "PICK"));
    this->taskActionDict.insert(std::make_pair(2, "PICK_FROM_ROBOT"));
    this->taskActionDict.insert(std::make_pair(3, "PLACE"));
    this->taskActionDict.insert(std::make_pair(4, "PLACE_FROM_ROBOT"));
    this->taskActionDict.insert(std::make_pair(5, "DRIVE"));

    // --- Populating Location Dict --- //
    this->locationDict.insert(std::make_pair(1, "Workstation 1"));
    this->locationDict.insert(std::make_pair(2, "Workstation 2"));
    this->locationDict.insert(std::make_pair(3, "Workstation 3"));
    this->locationDict.insert(std::make_pair(4, "Workstation 4"));
    this->locationDict.insert(std::make_pair(5, "Workstation 5"));
    this->locationDict.insert(std::make_pair(6, "Workstation 6"));

    // --- Populating Object Dict --- //
    this->objectDict.insert(std::make_pair(1, "Large Black Alu. Profile"));
    this->objectDict.insert(std::make_pair(2, "Large Nut"));
    this->objectDict.insert(std::make_pair(3, "Small Nut"));
    this->objectDict.insert(std::make_pair(4, "Motor"));
    this->objectDict.insert(std::make_pair(5, "Plastic Tube"));
    this->objectDict.insert(std::make_pair(6, "Small Grey Alu. Profile"));
    this->objectDict.insert(std::make_pair(7, "Distance Tube"));
    this->objectDict.insert(std::make_pair(8, "Bearing"));
    this->objectDict.insert(std::make_pair(9, "Bolt"));
    this->objectDict.insert(std::make_pair(10, "Small Black Alu. Profile"));
    this->objectDict.insert(std::make_pair(11, "Bearing Box"));
    this->objectDict.insert(std::make_pair(12, "Axis"));
    this->objectDict.insert(std::make_pair(13, "Large Grey Alu. Profile"));
    

    // --- Populating Container Dict --- //
    this->containerDict.insert(std::make_pair(1, "CONTAINER_B"));
    this->containerDict.insert(std::make_pair(2, "CONTAINER_R"));
}

// Printing 
void TaskProtocol::printLocationProtocol(void)
{
    std::cout << "--- Location protocol ---\n";
    this->printProtocol(this->locationDict);
}

void TaskProtocol::printObjectProtocol(void)
{
    std::cout << "--- Object protocol ---\n";
    this->printProtocol(this->objectDict);
}

void TaskProtocol::printTaskTypeProtocol(void)
{
    std::cout << "--- Task type protocol ---\n";
    this->printProtocol(this->taskTypeDict);
}

void TaskProtocol::printTaskActionProtocol(void)
{
    std::cout << "--- Task action protocol ---\n";
    this->printProtocol(this->taskActionDict);
}

void TaskProtocol::printContainerProtocol(void)
{
    std::cout << "--- Container protocol ---\n";
    this->printProtocol(this->containerDict);
}

// Task type lookup
void TaskProtocol::addTaskType(int id, std::string str)
{
    this->addPair(this->taskTypeDict, id, str);
}

void TaskProtocol::removeTaskType(int id)
{
    this->removePair(this->taskTypeDict, id);
}

int TaskProtocol::lookUpTaskTypeString(std::string str)
{
    return this->lookUpValue(this->taskTypeDict, str);
}

std::string TaskProtocol::lookUpTaskTypeID(int id)
{
    return this->lookUpKey(this->taskTypeDict, id);
}

// Location lookup
void TaskProtocol::addLocation(int id, std::string str)
{
    this->addPair(this->locationDict, id, str);
}

void TaskProtocol::removeLocation(int id)
{
    this->removePair(this->locationDict, id);
}

int TaskProtocol::lookUpLocationString(std::string str)
{
    return this->lookUpValue(this->locationDict, str);
}

std::string TaskProtocol::lookUpLocationID(int id)
{
    return this->lookUpKey(this->locationDict, id);
}

// Object lookup
void TaskProtocol::addObject(int id, std::string str)
{
    this->addPair(this->objectDict, id, str);
}

void TaskProtocol::removeObject(int id)
{
    this->removePair(this->objectDict, id);
}

int TaskProtocol::lookUpObjectString(std::string str)
{
    return this->lookUpValue(this->objectDict, str);
}

std::string TaskProtocol::lookUpObjectID(int id)
{
    return this->lookUpKey(this->objectDict, id);
}

// Container lookup
void TaskProtocol::addContainer(int id, std::string str)
{
    this->addPair(this->containerDict, id, str);
}

void TaskProtocol::removeContainer(int id)
{
    this->removePair(this->containerDict, id);
}

int TaskProtocol::lookUpContainerString(std::string str)
{
    return this->lookUpValue(this->containerDict, str);
}

std::string TaskProtocol::lookUpContainerID(int id)
{
    return this->lookUpKey(this->containerDict, id);
}

// Action lookup
void TaskProtocol::addTaskAction(int id, std::string str)
{
    this->addPair(this->taskActionDict, id, str);
}

void TaskProtocol::removeTaskAction(int id)
{
    this->removePair(this->taskActionDict, id);
}

int TaskProtocol::lookUpTaskActionString(std::string str)
{
    return this->lookUpValue(this->taskActionDict, str);
}

std::string TaskProtocol::lookUpTaskActionID(int id)
{
    return this->lookUpKey(this->taskActionDict, id);
}
