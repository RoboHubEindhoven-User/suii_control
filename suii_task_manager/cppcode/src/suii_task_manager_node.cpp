
#include "suii_task_manager.h"
#include "TaskProtocol.h"
#include "TaskManager.h"
#include <vector>

TaskManager taskManager;
TaskProtocol protocol;
TaskList taskList;

bool addTransportTask (atwork_ros_msgs::Task rosTask)
{
  int src = protocol.lookUpLocationString(rosTask.transportation_task.source.description.data);
  if (src == -1) { ROS_ERROR("Look up failed for source"); std::cout << "Source: " << rosTask.transportation_task.source.description.data << "\n"; return false; }
  int dest = protocol.lookUpLocationString(rosTask.transportation_task.destination.description.data);
  if (dest == -1)  { ROS_ERROR("Destination look-up failed"); std::cout << "Destination: " << rosTask.transportation_task.destination.description.data << "\n"; return false; }
  int obj = protocol.lookUpObjectString(rosTask.transportation_task.object.description.data);
  if (obj == -1)  { ROS_ERROR("Object look-up failed"); std::cout << "Object: " << rosTask.transportation_task.object.description.data << "\n"; return false; }
      
  int container = -1;
  if (rosTask.transportation_task.container.description.data != "")
  {
    container = protocol.lookUpContainerString(rosTask.transportation_task.container.description.data);
    if (container == -1)  { ROS_ERROR("Container look-up failed"); std::cout << "Container: " << rosTask.transportation_task.container.description.data << "\n"; return false; }
  }
  Task temp; temp.setType(protocol.lookUpTaskTypeString("TRANSPORTATION")); temp.setSrc(src); if (src == -1) temp.setDest(src); else temp.setDest(dest); temp.setObj(obj); temp.setContainer(container);
  taskList.addTask(temp);
  return true;
}

bool addNavTask (atwork_ros_msgs::Task rosTask)
{
  int dest = protocol.lookUpLocationString(rosTask.transportation_task.destination.description.data);
  if (dest == -1)  { ROS_ERROR("Destination look-up failed"); return false; }
  Task temp; temp.setType(protocol.lookUpTaskTypeString("NAVIGATION")); temp.setSrc(-1); temp.setDest(dest); temp.setObj(-1); temp.setContainer(-1);
  taskList.addTask(temp);
  return true;
}

bool processOneTask (atwork_ros_msgs::Task rosTask)
{
  
  switch (rosTask.type.data)
  {
    case 1: // TRANSPORTATION
      ROS_INFO("Transportation task received");
      return addTransportTask(rosTask);
      break;
    case 2: // NAVIGATION
      ROS_INFO("Navigation task received");
      return addNavTask(rosTask);
      break;
  }
  return false;
}

void taskCallback(const atwork_ros_msgs::TaskInfo::ConstPtr& msg)
{
  taskList.clearTask();
  atwork_ros_msgs::TaskInfo t;
  std::vector<atwork_ros_msgs::Task> tasks = msg->tasks;
  for (unsigned int i = 0; i < tasks.size(); i++)
  {
    if (processOneTask(tasks[i]))
    {
      ROS_INFO("Task processed");
    }
    else return;
  }
  std::vector<TaskWithAction*> result;
  if (taskManager.optimizeList(taskList, result)) ROS_INFO("Finished optimizing");

  ROS_INFO("Final task list:");
  for (unsigned int i = 0; i < result.size(); i++)
  {
    result[i]->printTaskData();
  }
}

int main(int argc, char **argv)
{
  ROS_ERROR("@THANH UN-BUG THE CODE!!!!!!!!!!!!!!!!!!!!!!");
  ros::init(argc, argv, "suii_task_manager");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(TASK_TOPIC, 1000, taskCallback);
  ros::spin();
  return 0;
}