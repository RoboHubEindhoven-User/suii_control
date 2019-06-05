/**
 * filename:    TaskProtocol.h
 * brief:       Task Protocol contains the protocol for which ID corresponds with which location/object
 */

#ifndef TASK_PROTOCOL_H
#define TASK_PROTOCOL_H

#include <iostream>
#include <map>

#define ERROR_STRING    "NOT_FOUND"

class TaskProtocol {
    private:
        std::map<int, std::string> taskTypeDict;        
        std::map<int, std::string> taskActionDict;
        std::map<int, std::string> locationDict;
        std::map<int, std::string> objectDict;
        std::map<int, std::string> containerDict;

        void printProtocol(std::map<int, std::string> dict)
        {
            std::map<int, std::string>::iterator it = dict.begin();
            while (it != dict.end())
            {
                std::cout << "Key: " << it->first;
                std::cout << "; Value: " << it->second << "\n";
                it++;
            }
        }

        std::string lookUpKey(std::map<int, std::string> dict, int ID)
        {
            std::map<int, std::string>::iterator it = dict.begin();
            while (it != dict.end())
            {
                if (it->first == ID)
                {
                    return it->second;
                }
                it++;
            }
            return ERROR_STRING;
        }

        int lookUpValue(std::map<int, std::string> dict, std::string val)
        {
            std::map<int, std::string>::iterator it = dict.begin();
            while (it != dict.end())
            {
                if (it->second == val)
                {
                    return it->first;
                }
                it++;
            }
            return -1;
        }

        void addPair(std::map<int, std::string> dict, int key, std::string val)
        {
            dict.insert(std::make_pair(key, val));
        }

        void removePair(std::map<int, std::string> dict, int key)
        {
            dict.erase(key);
        }

    public:
        TaskProtocol(void);

        void printTaskTypeProtocol(void);
        int lookUpTaskTypeString(std::string str);
        std::string lookUpTaskTypeID (int id);
        void addTaskType(int id, std::string str);
        void removeTaskType(int id);

        void printTaskActionProtocol(void);
        int lookUpTaskActionString(std::string str);
        std::string lookUpTaskActionID (int id);
        void addTaskAction(int id, std::string str);
        void removeTaskAction(int id);
        
        void printLocationProtocol(void);
        int lookUpLocationString(std::string str);
        std::string lookUpLocationID (int id);
        void addLocation(int id, std::string str);
        void removeLocation(int id);
        
        void printObjectProtocol(void);
        int lookUpObjectString(std::string str);
        std::string lookUpObjectID (int id);
        void addObject(int id, std::string str);
        void removeObject(int id);

        void printContainerProtocol(void);
        int lookUpContainerString(std::string str);
        std::string lookUpContainerID (int id);
        void addContainer(int id, std::string str);
        void removeContainer(int id);
};

#endif