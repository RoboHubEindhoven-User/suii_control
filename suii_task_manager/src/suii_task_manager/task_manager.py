#! /usr/bin/env python
from protocol.enum_task_action import TaskActionType
from suii_task_manager.task_protocol import TaskProtocol
from suii_task_manager.task_with_action import TaskWithAction

class TaskManager:
    MAX_HOLDING_CAPACITY = 3

    def __init__(self):
        self.protocol = TaskProtocol()
        self.holding_capacity = self.MAX_HOLDING_CAPACITY
        
    def format_drive (self, destination,  result):
        twa = TaskWithAction()
        twa.set_destination(dest)
        twa.set_action(protocol.look_up_key(TaskActionType.DRIVE))
        result.append(twa)

    def format_pick_task(self, task,  result):
        twa = TaskWithAction()
        twa.copy_from_task(task)
        twa.set_action(protocol.look_up_key(TaskActionType.PICK))
        print("Picking: " + twa.print_task_data())
        result.append(twa)

    def format_pick (self, task_list, result):
        for task in task_list.task_list:
            self.format_pick_task(task_list.task_list[i], result)

    def format_place_task (self, task, result):
        twa = TaskWithAction()
        twa.copy_from_task(task)
        twa.set_action(self.protocol.look_up_key(TaskActionType.PLACE))
        print("Placing: " + twa.print_task_data())
        result.append(twa)

    def formatPlace (self, task_list, result):
        for task in task_list.task_list:
            self.format_place_task(task_list.task_list[i], result)
        
    def format_set_of_tasks (self, task_list, result):
        unique_task_list = task_list.get_unique_source()

    if (len(task_list) > 0):
        print("\nPicking objects from " + str(len(unique_task_list)) + " source(s)\n")
            
        std::map<int, std::string>::iterator it = uniqueSrc.begin()
        while (it != uniqueSrc.end())
        {
            print("\nPicking objects from " + it.second + "...\n"
            formatDrive(it.first, result)                         // drive to first location
            TaskList pickUp = task_list.getTasksBySrc(it.first)    // get all the tasks with that source

            // Pick up selectively
            for (unsigned int i = 0 i < pickUp.task_list.size() i++)
            {
                if (pickUp.task_list[i].isDestSameAsSrc())
                {
                    print("Object has the same destination as source!\n"
                    self.formatPickTask(pickUp.task_list[i], result)
                    self.formatPlaceTask(pickUp.task_list[i], result)
                    task_list.removeTask(pickUp.task_list[i])
                }
                else 
                {
                    self.formatPickTask(pickUp.task_list[i], result)
                }
            }
            it++
        }
    }
    
    std::map<int, std::string> uniqueDest = task_list.getUniqueDest()

    if (uniqueDest.size() > 0)
    {
        print("\nPlacing objects at " + uniqueSrc.size() + " destination(s)\n"
        std::map<int, std::string>::iterator it = uniqueDest.begin()
        while (it != uniqueDest.end())
        {
            print("\nPlacing objects at " + it.second + "...\n"
            formatDrive(it.first, result)                          // drive to first location
            TaskList dropOff = task_list.getTasksByDest(it.first)   // get all the tasks with that dest
            formatPlace(dropOff, result)                            // place them
            it++
        }
    }
}

    def optimize_list(self, task_list):
        holding_list TaskList(self.holding_capacity)
        result
        format_now = False
        print("Sorting task list by source and destination...\n")
        task_list.sort_by_src_and_dest()
        task_list.print_task_list()
        
        while not task_list.is_empty():
            if not holding_list.is_full():
                task  = Task()
                index = 0
                if not task_list.get_next_obj_to_pick(task, index): # picked everything
                    break 

                if holding_list.is_empty():
                    holding_list.add_task(task) 
                    task_list.task_list[index].picked = True
                else: 
                    if (task.src == holding_list.task_list[0].src):
                        holding_list.add_task(task) 
                        task_list.task_list[index].picked = True
                    else:
                        format_now = True
            else: 
                format_now = True


            if (format_now):
                self.format_set_of_tasks(holding_list, result)

                for task in holding_list.task_list:
                    task_list.remove_task(task)
                holding_list.clear_task()
                format_now = False

        if not holding_list.is_empty():
            format_set_of_tasks(holding_list, result)

        for task in holding_list.task_list:
            task_list.remove_task(task)
        holding_list.clear_task()

        return True, result