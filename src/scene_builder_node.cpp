#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <memory>
#include <std_srvs/Trigger.h>

#include <ros/callback_queue.h>

void addBOX(moveit::planning_interface::PlanningSceneInterface &planning_scene,
            double dim_x, double dim_y, double dim_z,
            geometry_msgs::PoseStamped pose, const std::string &obj_id)
{
  std::cout << "\n#####\nAdding box [" << dim_x << ", " << dim_y << ", "
            << dim_z << "]\n pose:\n"
            << pose << "\n#####\n";

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = pose.header.frame_id;
  collision_object.id = obj_id;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = dim_x;
  primitive.dimensions[primitive.BOX_Y] = dim_y;
  primitive.dimensions[primitive.BOX_Z] = dim_z;

  // pose.pose.position.z += dim_z / 2.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose.pose);
  collision_object.operation = collision_object.ADD;

  planning_scene.applyCollisionObject(collision_object);
  ros::Duration(1).sleep();
}

std::unique_ptr<moveit::planning_interface::PlanningSceneInterface>
    planning_scene_interface;

std::unique_ptr<moveit::planning_interface::MoveGroupInterface>
    move_group_interface;

void clearScene()
{
  // RIMUOVO TUTTO GLI OGGETTI PRESENTI
  for (auto const &element : planning_scene_interface->getAttachedObjects())
  {
    move_group_interface->detachObject(element.first);
    ros::Duration(1).sleep();
  }
  auto objects_map = planning_scene_interface->getObjects();
  std::vector<std::string> obj_keys;
  for (auto const &element : objects_map)
  {
    obj_keys.push_back(element.first);
  }
  planning_scene_interface->removeCollisionObjects(obj_keys);
  ros::Duration(1.0).sleep();
}

void buildScene()
{

  clearScene();

  // COSTRUISCO LA SCENA
  // dimensioni tavolo yaskawa
  double Table_DX = 0.65;
  double Table_DY = 0.85;
  double Table_DZ = 0.80;

  // dimensioni cesta frutta
  double container_spessore_max = 0.025;
  double container_spessore_min = 0.01;
  double container_larghezza = 0.385;
  double container_altezza = 0.14;
  double container_lunghezza = 0.30;
  double x1 = 0.15; // distanza tra il bordo del tavolo e la cesta

  // dimensioni tavolo place
  double table_place_altezza = 0.76;
  double table_place_lunghezza = 0.62;
  double table_place_larghezza = 0.33;

  // table yaskawa
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.orientation.w = 1.0;
    pose.pose.position.x = 0;
    pose.pose.position.y = -Table_DY / 4 - 0.03; // 0
    pose.pose.position.z = -Table_DZ / 2;
    addBOX(*planning_scene_interface, Table_DX, Table_DY, Table_DZ, pose,
           "obst_1");
  }

  // table place
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.orientation.w = 1.0;
    pose.pose.position.x = Table_DX / 2 + 0.11 + table_place_larghezza / 2;
    pose.pose.position.y = -table_place_lunghezza / 4; // 0
    pose.pose.position.z = -table_place_altezza / 2 - 0.08;
    addBOX(*planning_scene_interface, table_place_larghezza, table_place_lunghezza, table_place_altezza, pose,
           "obst_10");
  }

  // // object contaier
  // {
  //   geometry_msgs::PoseStamped pose;
  //   pose.header.frame_id = "world";
  //   pose.pose.orientation.w = 1.0;
  //   pose.pose.position.x = 0.125;
  //   pose.pose.position.y = -Table_DY + 0.162 + container_spessore_max;
  //   pose.pose.position.z = container_altezza / 2;
  //   addBOX(*planning_scene_interface, container_larghezza, container_spessore_min, container_altezza, pose,
  //          "obst_2");
  // }

  // {
  //   geometry_msgs::PoseStamped pose;
  //   pose.header.frame_id = "world";
  //   pose.pose.orientation.w = 1.0;
  //   pose.pose.position.x = 0.125;
  //   pose.pose.position.y = -(Table_DY - 0.172 - container_lunghezza - container_spessore_max);
  //   pose.pose.position.z = container_altezza / 2;
  //   addBOX(*planning_scene_interface, container_larghezza, container_spessore_min, container_altezza, pose,
  //          "obst_3");
  // }

  // {
  //   geometry_msgs::PoseStamped pose;
  //   pose.header.frame_id = "world";
  //   pose.pose.orientation.w = 1.0;
  //   pose.pose.position.x = container_larghezza / 2 - container_spessore_min + 0.125;
  //   pose.pose.position.y = -(Table_DY) + container_larghezza - 2*container_spessore_max;
  //   pose.pose.position.z = container_altezza / 2;
  //   addBOX(*planning_scene_interface, container_spessore_max, container_lunghezza + container_spessore_max, container_altezza, pose,
  //          "obst_4");
  // }

  // {
  //   geometry_msgs::PoseStamped pose;
  //   pose.header.frame_id = "world";
  //   pose.pose.orientation.w = 1.0;
  //   pose.pose.position.x = -(container_larghezza / 2) + container_spessore_min + 0.125;
  //   pose.pose.position.y = -(Table_DY) + container_larghezza - 2*container_spessore_max; //- container_larghezza / 2);
  //   pose.pose.position.z = container_altezza / 2;
  //   addBOX(*planning_scene_interface, container_spessore_max, container_lunghezza + container_spessore_max, container_altezza, pose,
  //          "obst_5");
  // }
}

bool build_scene_srv_cb(std_srvs::Trigger::Request &req,
                        std_srvs::Trigger::Response &res)
{

  buildScene();

  res.success = true;

  return true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "scene_builder");
  ros::NodeHandle node_handle;

  static const std::string PLANNING_GROUP = "yaskawa_arm";
  // static const std::string EE_LINK = "panda_hand_tcp";

  move_group_interface =
      std::unique_ptr<moveit::planning_interface::MoveGroupInterface>(
          new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));

  planning_scene_interface =
      std::unique_ptr<moveit::planning_interface::PlanningSceneInterface>(
          new moveit::planning_interface::PlanningSceneInterface);

  // ros::NodeHandle node_handle_srv;
  // node_handle_srv.setCallbackQueue(callbk_q);

  ros::ServiceServer service =
      node_handle.advertiseService("build_scene", build_scene_srv_cb);

  ros::spin();

  return 0;
}
