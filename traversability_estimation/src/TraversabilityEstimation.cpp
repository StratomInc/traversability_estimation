/*
 * TraversabilityEstimation.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "traversability_estimation/TraversabilityEstimation.hpp"

#include <traversability_msgs/msg/traversability_result.hpp>

// ROS
#include <geometry_msgs/msg/pose.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

namespace traversability_estimation
{
TraversabilityEstimation::TraversabilityEstimation(const rclcpp::NodeOptions& options)
  : rclcpp::Node("traversability_estimation", options)
  , acceptGridMapToInitTraversabilityMap_(false)
  , traversabilityType_("traversability")
  , slopeType_("traversability_slope")
  , stepType_("traversability_step")
  , roughnessType_("traversability_roughness")
  , robotSlopeType_("robot_slope")
  , getImageCallback_(false)
  , useRawMap_(false)
{
  RCLCPP_DEBUG(this->get_logger(), "Traversability estimation node started.");
  initTimer_ = this->create_wall_timer(std::chrono::duration<double>(0.1),
                                       std::bind(&TraversabilityEstimation::init, this));
}

TraversabilityEstimation::~TraversabilityEstimation()
{
  if (!updateTimer_)
  {
    updateTimer_->cancel();
  }
}

void TraversabilityEstimation::init()
{
  initTimer_->cancel();

  traversabilityMap_ = std::make_unique<TraversabilityMap>(this->shared_from_this());
  gridMap_ = std::make_shared<grid_map::GridMap>();

  callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, this->get_node_base_interface());

  readParameters();
  traversabilityMap_->createLayers(useRawMap_);
  submapClient_ = this->create_client<grid_map_msgs::srv::GetGridMap>(
      submapServiceName_, rmw_qos_profile_services_default, callback_group_);

  if (dt_ > 0)
  {
    updateTimer_ =
        this->create_wall_timer(std::chrono::duration<double>(dt_),
                                std::bind(&TraversabilityEstimation::updateTimerCallback, this));
  }
  else
  {
    RCLCPP_WARN(this->get_logger(),
                "Update rate is zero. No traversability map will be published.");
  }

  loadElevationMapService_ = this->create_service<grid_map_msgs::srv::ProcessFile>(
      "load_elevation_map",
      std::bind(&TraversabilityEstimation::loadElevationMap, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
  updateTraversabilityService_ = this->create_service<grid_map_msgs::srv::GetGridMapInfo>(
      "update_traversability",
      std::bind(&TraversabilityEstimation::updateServiceCallback, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
  getTraversabilityService_ = this->create_service<grid_map_msgs::srv::GetGridMap>(
      "get_traversability",
      std::bind(&TraversabilityEstimation::getTraversabilityMap, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
  footprintPathService_ = this->create_service<traversability_msgs::srv::CheckFootprintPath>(
      "check_footprint_path",
      std::bind(&TraversabilityEstimation::checkFootprintPath, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
  updateParameters_ = this->create_service<std_srvs::srv::Empty>(
      "update_parameters",
      std::bind(&TraversabilityEstimation::updateParameter, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
  traversabilityFootprint_ = this->create_service<std_srvs::srv::Empty>(
      "traversability_footprint",
      std::bind(&TraversabilityEstimation::traversabilityFootprint, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
  saveToBagService_ = this->create_service<grid_map_msgs::srv::ProcessFile>(
      "save_traversability_map_to_bag",
      std::bind(&TraversabilityEstimation::saveToBag, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
  imageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      imageTopic_, 1,
      std::bind(&TraversabilityEstimation::imageCallback, this, std::placeholders::_1));

  if (!useServiceRequest_ || acceptGridMapToInitTraversabilityMap_)
  {
    gridMapSubscriber_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        gridMapToInitTraversabilityMapTopic_, 1,
        std::bind(&TraversabilityEstimation::gridMapToInitTraversabilityMapCallback, this,
                  std::placeholders::_1));
  }

  elevationMapLayers_.push_back("elevation");
  if (!useRawMap_)
  {
    elevationMapLayers_.push_back("upper_bound");
    elevationMapLayers_.push_back("lower_bound");
  }
  else
  {
    elevationMapLayers_.push_back("variance");
    elevationMapLayers_.push_back("horizontal_variance_x");
    elevationMapLayers_.push_back("horizontal_variance_y");
    elevationMapLayers_.push_back("horizontal_variance_xy");
    elevationMapLayers_.push_back("time");
  }

  transformBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transformListener_ = std::make_unique<tf2_ros::TransformListener>(*transformBuffer_);

  spinner_ = std::make_unique<std::thread>([this]() { callback_group_executor_.spin(); });
}

void TraversabilityEstimation::readParameters()
{
  // Read boolean to switch between raw and fused map.
  useRawMap_ = this->declare_parameter("use_raw_map", false);

  submapServiceName_ = this->declare_parameter("submap_service", "/get_grid_map");

  double updateRate;
  updateRate = this->declare_parameter("min_update_rate", 1.0);
  if (updateRate != 0.0)
  {
    dt_ = 1.0 / updateRate;
  }
  // Read parameters for image subscriber.
  imageTopic_ = this->declare_parameter("image_topic", "/image_elevation");
  imageResolution_ = this->declare_parameter("resolution", 0.03);
  imageMinHeight_ = this->declare_parameter("min_height", 0.0);
  imageMaxHeight_ = this->declare_parameter("max_height", 1.0);
  imagePosition_.x() = this->declare_parameter("image_position_x", 0.0);
  imagePosition_.y() = this->declare_parameter("image_position_y", 0.0);

  robotFrameId_ = this->declare_parameter("robot_frame_id", "robot");
  robot_ = this->declare_parameter("robot", "robot");
  package_ = this->declare_parameter("package", "traversability_estimation");

  grid_map::Position mapCenter;
  mapCenter.x() = this->declare_parameter("map_center_x", 0.0);
  mapCenter.y() = this->declare_parameter("map_center_y", 0.0);

  submapPoint_.header.frame_id = robotFrameId_;
  submapPoint_.point.x = mapCenter.x();

  submapPoint_.point.y = mapCenter.y();

  submapPoint_.point.z = 0.0;
  mapLength_.x() = this->declare_parameter("map_length_x", 5.0);
  mapLength_.y() = this->declare_parameter("map_length_y", 5.0);
  footprintYaw_ = this->declare_parameter("footprint_yaw", M_PI_2);
  useServiceRequest_ = this->declare_parameter("use_service_request", true);

  // Grid map to initialize elevation layer
  acceptGridMapToInitTraversabilityMap_ =
      this->declare_parameter("grid_map_to_initialize_traversability_map.enable", false);
  gridMapToInitTraversabilityMapTopic_ =
      this->declare_parameter("grid_map_topic_name", "elevation_map");
}

bool TraversabilityEstimation::loadElevationMap(
    const grid_map_msgs::srv::ProcessFile::Request::SharedPtr request,
    grid_map_msgs::srv::ProcessFile::Response::SharedPtr response)
{
  RCLCPP_INFO(this->get_logger(), "TraversabilityEstimation: loadElevationMap");
  if (request->file_path.empty() || request->topic_name.empty())
  {
    RCLCPP_WARN(this->get_logger(),
                "Fields 'file_path' and 'topic_name' in service request must be filled in.");
    response->success = static_cast<unsigned char>(false);
    return true;
  }

  grid_map::GridMap map;
  if (!grid_map::GridMapRosConverter::loadFromBag(request->file_path, request->topic_name, map))
  {
    RCLCPP_ERROR(
        this->get_logger(),
        "TraversabilityEstimation: Cannot find bag '%s' or topic '%s' of the elevation map!",
        request->file_path.c_str(), request->topic_name.c_str());
    response->success = static_cast<unsigned char>(false);
  }
  else
  {
    map.setTimestamp(this->now().nanoseconds());
    if (!initializeTraversabilityMapFromGridMap(map))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "TraversabilityEstimation: loadElevationMap: it was not possible to load "
                   "elevation map from bag with path '%s' and topic '%s'.",
                   request->file_path.c_str(), request->topic_name.c_str());
      response->success = static_cast<unsigned char>(false);
    }
    else
    {
      response->success = static_cast<unsigned char>(true);
    }
  }

  return true;
}

void TraversabilityEstimation::imageCallback(const sensor_msgs::msg::Image::SharedPtr image)
{
  if (!getImageCallback_)
  {
    grid_map::GridMapRosConverter::initializeFromImage(*image, imageResolution_, imageGridMap_,
                                                       imagePosition_);
    RCLCPP_INFO(this->get_logger(), "Initialized map with size %f x %f m (%i x %i cells).",
                imageGridMap_.getLength().x(), imageGridMap_.getLength().y(),
                imageGridMap_.getSize()(0), imageGridMap_.getSize()(1));
    imageGridMap_.add("upper_bound", 0.0);  // TODO: Add value for layers.
    imageGridMap_.add("lower_bound", 0.0);
    imageGridMap_.add("uncertainty_range",
                      imageGridMap_.get("upper_bound") - imageGridMap_.get("lower_bound"));
    getImageCallback_ = true;
  }
  grid_map::GridMapRosConverter::addLayerFromImage(*image, "elevation", imageGridMap_,
                                                   imageMinHeight_, imageMaxHeight_);
  auto elevationMap = grid_map::GridMapRosConverter::toMessage(imageGridMap_);
  traversabilityMap_->setElevationMap(*elevationMap);
}

void TraversabilityEstimation::updateTimerCallback()
{
  updateTraversability();
}

bool TraversabilityEstimation::updateServiceCallback(
    const grid_map_msgs::srv::GetGridMapInfo::Request::SharedPtr,
    grid_map_msgs::srv::GetGridMapInfo::Response::SharedPtr response)
{
  if (!dt_)
  {
    if (!updateTraversability())
    {
      RCLCPP_ERROR(this->get_logger(), "Traversability Estimation: Cannot update traversability!");
      return false;
    }
  }
  // Wait until traversability map is computed.
  while (!traversabilityMap_->traversabilityMapInitialized())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  grid_map_msgs::msg::GridMap msg;
  grid_map::GridMap traversabilityMap = traversabilityMap_->getTraversabilityMap();

  response->info.resolution = traversabilityMap.getResolution();
  response->info.length_x = traversabilityMap.getLength()[0];
  response->info.length_y = traversabilityMap.getLength()[1];
  geometry_msgs::msg::Pose pose;
  grid_map::Position position = traversabilityMap.getPosition();
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.orientation.w = 1.0;
  response->info.pose = pose;

  return true;
}

bool TraversabilityEstimation::updateTraversability()
{
  grid_map_msgs::msg::GridMap elevationMap;
  if (!getImageCallback_)
  {
    if (!requestElevationMap(elevationMap))
    {
      RCLCPP_WARN(this->get_logger(), "Failed to retrieve elevation grid map.");
      return false;
    }

    traversabilityMap_->setElevationMap(elevationMap);
    if (!traversabilityMap_->computeTraversability())
      return false;
  }
  else
  {
    if (!traversabilityMap_->computeTraversability())
      return false;
  }
  return true;
}

bool TraversabilityEstimation::updateParameter(const std_srvs::srv::Empty::Request::SharedPtr,
                                               std_srvs::srv::Empty::Response::SharedPtr)
{
  if (!traversabilityMap_->updateFilter())
    return false;

  return true;
}

bool TraversabilityEstimation::requestElevationMap(grid_map_msgs::msg::GridMap& map)
{
  submapPoint_.header.stamp = this->now();
  geometry_msgs::msg::PointStamped submapPointTransformed;

  try
  {
    auto transform = transformBuffer_->lookupTransform(
        traversabilityMap_->getMapFrameId(), submapPoint_.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(submapPoint_, submapPointTransformed, transform);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    return false;
  }

  if (useServiceRequest_)
  {
    if (!submapClient_->wait_for_service(std::chrono::seconds(2)))
      return false;

    auto submapService = std::make_shared<grid_map_msgs::srv::GetGridMap::Request>();
    submapService->position_x = submapPointTransformed.point.x;
    submapService->position_y = submapPointTransformed.point.y;
    submapService->length_x = mapLength_.x();
    submapService->length_y = mapLength_.y();
    submapService->layers = elevationMapLayers_;

    RCLCPP_DEBUG(this->get_logger(), "Sending request to %s.", submapServiceName_.c_str());
    auto fut = submapClient_->async_send_request(submapService);
    const auto status = fut.wait_for(std::chrono::seconds(2));
    if (std::future_status::ready != status)
      return false;

    auto result = fut.get();
    map = result->map;
  }
  else
  {
    if (!gridMap_)
      return false;

    grid_map::Position pos(submapPointTransformed.point.x, submapPointTransformed.point.y);
    grid_map::Length len(mapLength_.x(), mapLength_.y());
    bool success = false;
    auto subMap = gridMap_->getSubmap(pos, len, success);
    if (!success)
      return false;

    map = *grid_map::GridMapRosConverter::toMessage(subMap);
  }
  return true;
}

bool TraversabilityEstimation::traversabilityFootprint(
    const std_srvs::srv::Empty::Request::SharedPtr /*request*/,
    std_srvs::srv::Empty::Response::SharedPtr /*response*/)
{
  if (!traversabilityMap_->traversabilityFootprint(footprintYaw_))
    return false;

  return true;
}

bool TraversabilityEstimation::checkFootprintPath(
    const traversability_msgs::srv::CheckFootprintPath::Request::SharedPtr request,
    traversability_msgs::srv::CheckFootprintPath::Response::SharedPtr response)
{
  const int nPaths = request->path.size();
  if (nPaths == 0)
  {
    RCLCPP_WARN(this->get_logger(), "No footprint path available to check!");
    return false;
  }

  for (int j = 0; j < nPaths; j++)
  {
    auto result = std::make_shared<traversability_msgs::msg::TraversabilityResult>();
    auto path = std::make_shared<traversability_msgs::msg::FootprintPath>(request->path[j]);
    if (!traversabilityMap_->checkFootprintPath(path, result, true))
      return false;

    response->result.push_back(*result);
  }

  return true;
}

bool TraversabilityEstimation::getTraversabilityMap(
    const grid_map_msgs::srv::GetGridMap::Request::SharedPtr request,
    grid_map_msgs::srv::GetGridMap::Response::SharedPtr response)
{
  grid_map::Position requestedSubmapPosition(request->position_x, request->position_y);
  grid_map::Length requestedSubmapLength(request->length_x, request->length_y);
  grid_map_msgs::msg::GridMap msg;
  grid_map::GridMap map, subMap;
  map = traversabilityMap_->getTraversabilityMap();
  bool isSuccess;
  subMap = map.getSubmap(requestedSubmapPosition, requestedSubmapLength, isSuccess);
  if (request->layers.empty())
  {
    auto resultMap = grid_map::GridMapRosConverter::toMessage(subMap);
    response->map = *resultMap;
  }
  else
  {
    vector<string> layers;
    for (const auto& layer : request->layers)
    {
      layers.push_back(layer);
    }
    auto resultMap = grid_map::GridMapRosConverter::toMessage(subMap, layers);
    response->map = *resultMap;
  }
  return isSuccess;
}

bool TraversabilityEstimation::saveToBag(
    const grid_map_msgs::srv::ProcessFile::Request::SharedPtr request,
    grid_map_msgs::srv::ProcessFile::Response::SharedPtr response)
{
  RCLCPP_INFO(this->get_logger(), "Save to bag.");
  if (request->file_path.empty() || request->topic_name.empty())
  {
    RCLCPP_WARN(this->get_logger(),
                "Fields 'file_path' and 'topic_name' in service request must be filled in.");
    response->success = static_cast<unsigned char>(false);
    return true;
  }

  response->success = static_cast<unsigned char>(grid_map::GridMapRosConverter::saveToBag(
      traversabilityMap_->getTraversabilityMap(), request->file_path, request->topic_name));
  return true;
}

bool TraversabilityEstimation::initializeTraversabilityMapFromGridMap(
    const grid_map::GridMap& gridMap)
{
  if (traversabilityMap_->traversabilityMapInitialized())
  {
    RCLCPP_WARN(this->get_logger(),
                "[TraversabilityEstimation::gridMapToInitTraversabilityMapCallback]: received grid "
                "map message cannot be used to initialize"
                " the traversability map, because current traversability map has been already "
                "initialized.");
    return false;
  }

  grid_map::GridMap mapWithCheckedLayers = gridMap;
  for (const auto& layer : elevationMapLayers_)
  {
    if (!mapWithCheckedLayers.exists(layer))
    {
      mapWithCheckedLayers.add(layer, 0.0);
      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "[TraversabilityEstimation::initializeTraversabilityMapFromGridMap]: Added layer '"
              << layer << "'.");
    }
  }
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Map frame id: " << mapWithCheckedLayers.getFrameId());
  for (const auto& layer : mapWithCheckedLayers.getLayers())
  {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Map layers: " << layer);
  }
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Map size: " << mapWithCheckedLayers.getLength());
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Map position: " << mapWithCheckedLayers.getPosition());
  RCLCPP_DEBUG_STREAM(this->get_logger(),
                      "Map resolution: " << mapWithCheckedLayers.getResolution());

  auto message = grid_map::GridMapRosConverter::toMessage(mapWithCheckedLayers);
  traversabilityMap_->setElevationMap(*message);
  if (!traversabilityMap_->computeTraversability())
  {
    RCLCPP_WARN(this->get_logger(),
                "TraversabilityEstimation: initializeTraversabilityMapFromGridMap: cannot compute "
                "traversability.");
    return false;
  }
  return true;
}

void TraversabilityEstimation::gridMapToInitTraversabilityMapCallback(
    const grid_map_msgs::msg::GridMap::SharedPtr message)
{
  grid_map::GridMapRosConverter::fromMessage(*message, *gridMap_);
  if (acceptGridMapToInitTraversabilityMap_)
  {
    if (!initializeTraversabilityMapFromGridMap(*gridMap_))
    {
      RCLCPP_ERROR(
          this->get_logger(),
          "[TraversabilityEstimation::gridMapToInitTraversabilityMapCallback]: "
          "It was not possible to use received grid map message to initialize traversability map.");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(),
                  "[TraversabilityEstimation::gridMapToInitTraversabilityMapCallback]: "
                  "Traversability Map initialized using received grid map on topic '%s'.",
                  gridMapToInitTraversabilityMapTopic_.c_str());
    }
  }
}

}  // namespace traversability_estimation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(traversability_estimation::TraversabilityEstimation)
