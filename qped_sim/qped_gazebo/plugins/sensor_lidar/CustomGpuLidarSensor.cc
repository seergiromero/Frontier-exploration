#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/Utility.hh>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/transport/Node.hh>

#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/sensors/RenderingEvents.hh>
#include <gz/rendering/RenderingIface.hh>
#include "CustomGpuLidarSensor.hh"

/// \brief Private data for the GpuLidar class
class custom::CustomGpuLidarSensorPrivate
{
  /// \brief Fill the point cloud packed message
  /// \param[in] _laserBuffer Lidar data buffer.
  public: void FillPointCloudMsg(const float *_laserBuffer, const std::chrono::steady_clock::duration &timeDiff);

  /// \brief Rendering camera
  public: gz::rendering::GpuRaysPtr gpuRays;

  /// \brief Connection to the Manager's scene change event.
  public: gz::common::ConnectionPtr sceneChangeConnection;

  /// \brief Event that is used to trigger callbacks when a new
  /// lidar frame is available
  public: gz::common::EventT<
          void(const float *_scan, unsigned int _width,
               unsigned int _height, unsigned int _channels,
               const std::string &_format)> lidarEvent;

  /// \brief Callback when new lidar frame is received
  public: void OnNewLidarFrame(const float *_scan, unsigned int _width,
               unsigned int _height, unsigned int _channels,
               const std::string &_format);

  /// \brief Connection to gpuRays new lidar frame event
  public: gz::common::ConnectionPtr lidarFrameConnection;

  /// \brief The point cloud message.
  public: gz::msgs::PointCloudPacked pointMsg;

  /// \brief Transport node.
  public: gz::transport::Node node;

  /// \brief Publisher for the publish point cloud message.
  public: gz::transport::Node::Publisher pointPub;

  /// \brief Previous timestamp
  public: std::chrono::steady_clock::duration prevTimestamp;
};

//////////////////////////////////////////////////
custom::CustomGpuLidarSensor::CustomGpuLidarSensor()
  : dataPtr(new CustomGpuLidarSensorPrivate())
{
}

//////////////////////////////////////////////////
custom::CustomGpuLidarSensor::~CustomGpuLidarSensor()
{
  this->RemoveGpuRays(this->Scene());

  this->dataPtr->sceneChangeConnection.reset();

  if (this->laserBuffer)
  {
    delete [] this->laserBuffer;
    this->laserBuffer = nullptr;
  }
}

/////////////////////////////////////////////////
void custom::CustomGpuLidarSensor::SetScene(gz::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->lidarMutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    this->RemoveGpuRays(this->Scene());
    RenderingSensor::SetScene(_scene);

    if (this->initialized)
      this->CreateLidar();
  }
}

//////////////////////////////////////////////////
void custom::CustomGpuLidarSensor::RemoveGpuRays(
    gz::rendering::ScenePtr _scene)
{
  if (_scene)
  {
    _scene->DestroySensor(this->dataPtr->gpuRays);
  }
  this->dataPtr->gpuRays.reset();
  this->dataPtr->gpuRays = nullptr;
}

//////////////////////////////////////////////////
void custom::CustomGpuLidarSensor::FindScene()
{
  auto loadedEngNames = gz::rendering::loadedEngines();
  if (loadedEngNames.empty())
  {
    gzdbg << "No rendering engine is loaded yet" << std::endl;
    return;
  }
 
  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    gzdbg << "More than one engine is available. "
      << "Using engine [" << engineName << "]" << std::endl;
  }
  auto engine = gz::rendering::engine(engineName);
  if (!engine)
  {
    gzerr << "Internal error: failed to load engine [" << engineName
      << "]. Grid plugin won't work." << std::endl;
    return;
  }
 
  if (engine->SceneCount() == 0)
  {
    gzdbg << "No scene has been created yet" << std::endl;
    return;
  }
 
  // Get first scene
  auto scenePtr = engine->SceneByIndex(0);
  if (nullptr == scenePtr)
  {
    gzerr << "Internal error: scene is null." << std::endl;
    return;
  }
 
  if (engine->SceneCount() > 1)
  {
    gzdbg << "More than one scene is available. "
      << "Using scene [" << scenePtr->Name() << "]" << std::endl;
  }
 
  if (!scenePtr->IsInitialized() || nullptr == scenePtr->RootVisual())
  {
    return;
  }
  this->SetScene(scenePtr);
}



//////////////////////////////////////////////////
bool custom::CustomGpuLidarSensor::Load(const sdf::Sensor &_sdf)
{

  // Modify type to GPU_lidar for initialization
  if (!this->Lidar::Load(_sdf))
  {
    return false;
  }
  std::cout<<"AAAAAAAAAAAAAAAAAAAAAA"<<std::endl;
  std::cout<<this->Scene()<<std::endl;
  std::cout<<this->Scene()<<std::endl;
  std::cout<<"BBBBBBBBBBBBBBBBBBBBBB"<<std::endl;

  // // Accessing parameters from the SDF file
  // std::string topic = _sdf.Topic();

  this->RangeMin = _sdf.Element()->GetElement("ray")->GetElement("range")->GetElement("min")->Get<double>();
  this->RangeMax = _sdf.Element()->GetElement("ray")->GetElement("range")->GetElement("max")->Get<double>();
  this->RangeRes = _sdf.Element()->GetElement("ray")->GetElement("range")->GetElement("resolution")->Get<double>();
  
  this->horizontal_samples = _sdf.Element()->GetElement("ray")->GetElement("scan")->GetElement("horizontal")->GetElement("samples")->Get<int>();
  this->horizontal_resolution = _sdf.Element()->GetElement("ray")->GetElement("scan")->GetElement("horizontal")->GetElement("resolution")->Get<int>();
  this->horizontal_minAngle = _sdf.Element()->GetElement("ray")->GetElement("scan")->GetElement("horizontal")->GetElement("min_angle")->Get<double>();
  this->horizontal_maxAngle = _sdf.Element()->GetElement("ray")->GetElement("scan")->GetElement("horizontal")->GetElement("max_angle")->Get<double>();

  this->vertical_samples = _sdf.Element()->GetElement("ray")->GetElement("scan")->GetElement("vertical")->GetElement("samples")->Get<int>();
  this->vertical_resolution = _sdf.Element()->GetElement("ray")->GetElement("scan")->GetElement("vertical")->GetElement("resolution")->Get<int>();
  this->vertical_minAngle = _sdf.Element()->GetElement("ray")->GetElement("scan")->GetElement("vertical")->GetElement("min_angle")->Get<double>();
  this->vertical_maxAngle = _sdf.Element()->GetElement("ray")->GetElement("scan")->GetElement("vertical")->GetElement("max_angle")->Get<double>();

  // Initialize the point message.
  // \todo(anyone) The true value in the following function call forces
  // the xyz and rgb fields to be aligned to memory boundaries. This is need
  // by ROS1: https://github.com/ros/common_msgs/pull/77. Ideally, memory
  // alignment should be configured. This same problem is in the
  // RgbdCameraSensor.
  gz::msgs::InitPointCloudPacked(this->dataPtr->pointMsg, this->FrameId(), true,
      {{"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32},
       {"intensity", gz::msgs::PointCloudPacked::Field::FLOAT32},
       {"ring", gz::msgs::PointCloudPacked::Field::UINT16},
       {"time", gz::msgs::PointCloudPacked::Field::FLOAT32}});

  if (this->Scene())
    this->CreateLidar();

  this->dataPtr->sceneChangeConnection =
    gz::sensors::RenderingEvents::ConnectSceneChangeCallback(
        std::bind(&CustomGpuLidarSensor::SetScene, this, std::placeholders::_1));

  // Create the point cloud publisher
  this->SetTopic(this->Topic() + "/points");

  this->dataPtr->pointPub =
      this->dataPtr->node.Advertise<gz::msgs::PointCloudPacked>(
          this->Topic());

  if (!this->dataPtr->pointPub)
  {
    gzerr << "Unable to create publisher on topic["
      << this->Topic() << "].\n";
    return false;
  }

  gzdbg << "Lidar points for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

  this->initialized = true;

  return true;
}

//////////////////////////////////////////////////
bool custom::CustomGpuLidarSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool custom::CustomGpuLidarSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool custom::CustomGpuLidarSensor::CreateLidar()
{
  this->dataPtr->gpuRays = this->Scene()->CreateGpuRays(
      this->Name());

  if (!this->dataPtr->gpuRays)
  {
    gzerr << "Unable to create gpu laser sensor\n";
    return false;
  }

  this->dataPtr->gpuRays->SetWorldPosition(this->Pose().Pos());
  this->dataPtr->gpuRays->SetWorldRotation(this->Pose().Rot());

  this->dataPtr->gpuRays->SetNearClipPlane(this->RangeMin);
  this->dataPtr->gpuRays->SetFarClipPlane(this->RangeMax);
  // this->dataPtr->gpuRays->SetRangeResolution(this->RangeRes);

  // Mask ranges outside of min/max to +/- inf, as per REP 117
  this->dataPtr->gpuRays->SetClamp(false);

  this->dataPtr->gpuRays->SetAngleMin(this->horizontal_minAngle);
  this->dataPtr->gpuRays->SetAngleMax(this->horizontal_maxAngle);
  this->dataPtr->gpuRays->SetHorizontalResolution(this->horizontal_resolution);

  this->dataPtr->gpuRays->SetVerticalAngleMin(
      this->vertical_minAngle);
  this->dataPtr->gpuRays->SetVerticalAngleMax(
      this->vertical_maxAngle);
  this->dataPtr->gpuRays->SetVerticalResolution(
      this->vertical_resolution);

  this->dataPtr->gpuRays->SetRayCount(this->horizontal_samples);
  this->dataPtr->gpuRays->SetVerticalRayCount(
      this->vertical_samples);

  this->Scene()->RootVisual()->AddChild(
      this->dataPtr->gpuRays);

  // Set the values on the point message.
  this->dataPtr->pointMsg.set_width(this->dataPtr->gpuRays->RangeCount());
  this->dataPtr->pointMsg.set_height(
      this->dataPtr->gpuRays->VerticalRangeCount());
  this->dataPtr->pointMsg.set_row_step(
      this->dataPtr->pointMsg.point_step() *
      this->dataPtr->pointMsg.width());
  this->dataPtr->gpuRays->SetVisibilityMask(this->VisibilityMask());

  this->dataPtr->lidarFrameConnection =
      this->dataPtr->gpuRays->ConnectNewGpuRaysFrame(
      std::bind(&CustomGpuLidarSensor::OnNewLidarFrame, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));

  this->AddSensor(this->dataPtr->gpuRays);

  return true;
}

/////////////////////////////////////////////////
void custom::CustomGpuLidarSensor::OnNewLidarFrame(const float *_data,
    unsigned int _width, unsigned int _height, unsigned int _channels,
    const std::string &_format)
{
  std::lock_guard<std::mutex> lock(this->lidarMutex);

  if (_width==0) _width = this->dataPtr->gpuRays->RangeCount();
  if (_height==0) _height = this->dataPtr->gpuRays->VerticalRangeCount();
  if (_channels==0) _channels = this->dataPtr->gpuRays->Channels();

  unsigned int samples = _width * _height * _channels;
  unsigned int lidarBufferSize = samples * sizeof(float);

  if (!this->laserBuffer)
    this->laserBuffer = new float[samples];

  memcpy(this->laserBuffer, _data, lidarBufferSize);

  if (this->dataPtr->lidarEvent.ConnectionCount() > 0)
  {
    this->dataPtr->lidarEvent(_data, _width, _height, _channels, _format);
  }
}

//////////////////////////////////////////////////
bool custom::CustomGpuLidarSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("GpuLidarSensor::Update");
  if (!this->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->gpuRays)
  {
    gzerr << "GpuRays doesn't exist.\n";
    this->FindScene();
    return false;
  }

  this->Render();

  // Apply noise before publishing the data.
  this->ApplyNoise();

  this->PublishLidarScan(_now);

  if (this->dataPtr->pointPub.HasConnections())
  {
    // Set the time stamp
    *this->dataPtr->pointMsg.mutable_header()->mutable_stamp() =
      gz::msgs::Convert(_now);

    // Calculate the time difference between scans
    auto timeDiff = _now - this->dataPtr->prevTimestamp;
    this->dataPtr->prevTimestamp = _now;

    // Set frame_id
    for (auto i = 0;
         i < this->dataPtr->pointMsg.mutable_header()->data_size();
         ++i)
    {
      if (this->dataPtr->pointMsg.mutable_header()->data(i).key() == "frame_id"
          && this->dataPtr->pointMsg.mutable_header()->data(i).value_size() > 0)
      {
        this->dataPtr->pointMsg.mutable_header()->mutable_data(i)->set_value(
              0,
              this->FrameId());
      }
    }

    this->dataPtr->FillPointCloudMsg(this->laserBuffer, timeDiff);

    {
      this->AddSequence(this->dataPtr->pointMsg.mutable_header());
      GZ_PROFILE("GpuLidarSensor::Update Publish point cloud");
      this->dataPtr->pointPub.Publish(this->dataPtr->pointMsg);
    }
  }
  return true;
}

/////////////////////////////////////////////////
gz::common::ConnectionPtr custom::CustomGpuLidarSensor::ConnectNewLidarFrame(
          std::function<void(const float *_scan, unsigned int _width,
                  unsigned int _height, unsigned int _channels,
                  const std::string &/*_format*/)> _subscriber)
{
  return this->dataPtr->lidarEvent.Connect(_subscriber);
}

/////////////////////////////////////////////////
gz::rendering::GpuRaysPtr custom::CustomGpuLidarSensor::GpuRays() const
{
  return this->dataPtr->gpuRays;
}

//////////////////////////////////////////////////
bool custom::CustomGpuLidarSensor::IsHorizontal() const
{
  return this->dataPtr->gpuRays->IsHorizontal();
}

//////////////////////////////////////////////////
gz::math::Angle custom::CustomGpuLidarSensor::HFOV() const
{
  return this->dataPtr->gpuRays->HFOV();
}

//////////////////////////////////////////////////
gz::math::Angle custom::CustomGpuLidarSensor::VFOV() const
{
  return this->dataPtr->gpuRays->VFOV();
}

//////////////////////////////////////////////////
bool custom::CustomGpuLidarSensor::HasConnections() const
{
  return Lidar::HasConnections() ||
     (this->dataPtr->pointPub && this->dataPtr->pointPub.HasConnections()) ||
     this->dataPtr->lidarEvent.ConnectionCount() > 0u;
}

//////////////////////////////////////////////////
void custom::CustomGpuLidarSensorPrivate::FillPointCloudMsg(const float *_laserBuffer, const std::chrono::steady_clock::duration &timeDiff)
{
  GZ_PROFILE("CustomGpuLidarSensorPrivate::FillPointCloudMsg");
  uint32_t width = this->pointMsg.width();
  uint32_t height = this->pointMsg.height();
  unsigned int channels = 3;

  float angleStep =
    (this->gpuRays->AngleMax() - this->gpuRays->AngleMin()).Radian() /
    (this->gpuRays->RangeCount()-1);

  float verticleAngleStep = (this->gpuRays->VerticalAngleMax() -
      this->gpuRays->VerticalAngleMin()).Radian() /
    (this->gpuRays->VerticalRangeCount()-1);

  // Angles of ray currently processing, azimuth is horizontal, inclination
  // is vertical
  float inclination = this->gpuRays->VerticalAngleMin().Radian();

  std::string *msgBuffer = this->pointMsg.mutable_data();
  msgBuffer->resize(this->pointMsg.row_step() *
      this->pointMsg.height());
  char *msgBufferIndex = msgBuffer->data();
  // Set Pointcloud as dense. Change if invalid points are found.
  bool isDense { true };
  // Iterate over scan and populate point cloud
  for (uint32_t j = 0; j < height; ++j)
  {
    float azimuth = this->gpuRays->AngleMin().Radian();

    for (uint32_t i = 0; i < width; ++i)
    {
      // Index of current point, and the depth value at that point
      auto index = j * width * channels + i * channels;
      float depth = _laserBuffer[index];
      // Validate Depth/Radius and update pointcloud density flag
      if (isDense)
        isDense = !(gz::math::isnan(depth) || std::isinf(depth));

      float intensity = _laserBuffer[index + 1];
      uint16_t ring = j;
      // Calculate the incremental time for each point
      // float time = static_cast<float>(std::chrono::duration_cast<std::chrono::nanoseconds>(timeDiff).count()) / 1e9f;
      float time = static_cast<float>(i) / static_cast<float>(width) *0.1f;

      int fieldIndex = 0;

      // Convert spherical coordinates to Cartesian for pointcloud
      // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
      *reinterpret_cast<float *>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::cos(inclination) * std::cos(azimuth);

      *reinterpret_cast<float *>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::cos(inclination) * std::sin(azimuth);

      *reinterpret_cast<float *>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::sin(inclination);

      // Intensity
      *reinterpret_cast<float *>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) = intensity;

      // Ring
      *reinterpret_cast<uint16_t *>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) = ring;

      // Time
      *reinterpret_cast<float *>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) = time;

      // Move the index to the next point.
      msgBufferIndex += this->pointMsg.point_step();

      azimuth += angleStep;
    }
    inclination += verticleAngleStep;
  }
  this->pointMsg.set_is_dense(isDense);
}

GZ_ADD_PLUGIN(
  custom::CustomGpuLidarSensor,
  gz::sensors::Lidar)