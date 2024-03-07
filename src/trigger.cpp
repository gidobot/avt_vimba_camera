#include "avt_vimba_camera/trigger.h"

namespace trigger
{
Trigger::Trigger() : vimba_system_(AVT::VmbAPI::VimbaSystem::GetInstance()), pnh_("~")
{
}

Trigger::~Trigger()
{
  VmbErrorType lError = VmbErrorSuccess;
  lError = interface_ptr_->Close();
  if( VmbErrorSuccess != lError )
  {
      std::cout << "Could not close interface. Reason: " << lError << std::endl;
  }
  vimba_system_.Shutdown();
}

void Trigger::Init()
{
  VmbErrorType return_value = vimba_system_.Startup();

  if (return_value != VmbErrorSuccess)
  {
    ROS_ERROR_STREAM("Failed to start Vimba system, vimba error code: " << return_value);
    ros::shutdown();
  }

  LoadParams();
  // InitializeInterface();
  // InitializeAddress();

  if (trigger_src_ == "timer")
  {
    trigger_timer_ = nh_.createTimer(ros::Duration(timer_period_), &Trigger::TimerCb, this);
  }
  else if (trigger_src_ == "subscriber")
  {
    trigger_sub_ = nh_.subscribe("trigger_input", 10, &Trigger::TriggerCb, this);
  }
  else
  {
    ROS_ERROR("Unknown trigger_src %s", trigger_src_.c_str());
    ros::shutdown();
  }
}

void Trigger::LoadParams()
{
  std::string destination_ip;
  pnh_.param<std::string>("destination_ip", destination_ip, "");
  pnh_.param<std::string>("destination_interface", destination_interface_);
  pnh_.param<std::string>("trigger_src", trigger_src_, "timer");
  pnh_.param<float>("timer_period", timer_period_, 0.2);
  pnh_.param<int>("action_device_key", action_device_key_, 1);
  pnh_.param<int>("action_group_key", action_group_key_, 1);
  pnh_.param<int>("action_group_mask", action_group_mask_, 1);

  // if (inet_aton(destination_ip.c_str(), &destination_ip_) == 0)
  // {
  //   ROS_ERROR("Unable to parse desination_ip: %s", destination_ip.c_str());
  //   ros::shutdown();
  // }
}

void Trigger::InitializeAddress()
{
  VmbErrorType return_value = VmbErrorSuccess;

  if (!SetIntFeatureValue("GevActionDestinationIPAddress", destination_ip_.s_addr))
  {
    ROS_ERROR("Could not set destination address");
  }

  ROS_INFO("Destination address set");
}

void Trigger::InitializeInterface()
{
  VmbErrorType return_value = VmbErrorSuccess;

  // if (!SetIntFeatureValue("GevActionDestinationIPAddress", destination_ip_.s_addr))
  // {
  //   ROS_ERROR("Could not set destination address");
  // }

  // ROS_INFO("Destination address set");

  // get available interfaces
  AVT::VmbAPI::InterfacePtrVector lInterfaces;
  return_value = vimba_system_.GetInterfaces(lInterfaces);
  if ( (return_value != VmbErrorSuccess) || (lInterfaces.size() == 0) )
  {
    ROS_ERROR("No interfaces found");
    ros::shutdown();
    return;
  }

  // print interface list
  bool lFound = false;
  int lIndex = 0;
  std::cout << "List of network interfaces: " << std::endl;
  for( int i=0; i<lInterfaces.size(); ++i )
  {
      AVT::VmbAPI::InterfacePtr lInterface = lInterfaces.at(i);
      std::string lInterfaceID = "";
      return_value = lInterface->GetID( lInterfaceID );
      if( VmbErrorSuccess == return_value )
      {
          std::cout << ".........[" << i << "] " << lInterfaceID << std::endl;

          // compare given interface ID with current one
          if( 0 == lInterfaceID.compare( destination_interface_ ) )
          {
              // if interface ID matches, keep index
              lFound = true;
              lIndex = i;
          }
      }
  }

  // get interface pointer
  interface_ptr_ = lInterfaces.at( lIndex );
  if( true == SP_ISNULL(interface_ptr_) )
  {
      std::cout << "[F]...No valid interface pointer with given index found" << std::endl;
      ros::shutdown();
      return;
  }

  // check interface type
  VmbInterfaceType lInterfaceType = VmbInterfaceUnknown;
  return_value = interface_ptr_->GetType( lInterfaceType );
  if( (VmbErrorSuccess != return_value) || (VmbInterfaceEthernet != lInterfaceType) )
  {
      printf( "[F]...Selected interface is non-GigE interface!\n" );
      ros::shutdown();
      return;
  }

  // open interface
  return_value = interface_ptr_->Open();
  if( VmbErrorSuccess != return_value )
  {
      std::cout << "[F]...Could not open interface" << std::endl;
      ros::shutdown();
      return;
  }

  std::cout << "......Interface has been opened." << std::endl;

}

bool Trigger::PrepareActionCommand()
{
  return (SetIntFeatureValue("ActionDeviceKey", action_device_key_) && SetIntFeatureValue("ActionGroupKey", action_group_key_) &&
          SetIntFeatureValue("ActionGroupMask", action_group_mask_));
}

// Sets an integer feature value on the vimba system
bool Trigger::SetIntFeatureValue(const std::string& name, int64_t value)
{
  VmbErrorType return_value = VmbErrorSuccess;

  AVT::VmbAPI::FeaturePtr feature_ptr;
  return_value = vimba_system_.GetFeatureByName(name.c_str(), feature_ptr);

  if (return_value != VmbErrorSuccess)
  {
    ROS_ERROR_STREAM("Failed to get feature, vimba error code: " << return_value);
    return false;
  }
  else
  {
    return_value = feature_ptr->SetValue((VmbInt64_t)value);
  }

  return (return_value == VmbErrorSuccess);
}

void Trigger::TimerCb(const ros::TimerEvent& event)
{
  SendActionCommand();
}

void Trigger::TriggerCb(const std_msgs::Bool::ConstPtr& msg)
{
  SendActionCommand();
}

void Trigger::SendActionCommand()
{
  if (!PrepareActionCommand())
  {
    ROS_ERROR_THROTTLE(1.0, "Failed to prepare action command");
    return;
  }

  VmbErrorType return_value = VmbErrorSuccess;

  AVT::VmbAPI::FeaturePtr lFeature;
  return_value = vimba_system_.GetFeatureByName("ActionCommand", lFeature);

  if (return_value == VmbErrorSuccess)
  {
    return_value = lFeature->RunCommand();
  }

  if (return_value == VmbErrorSuccess)
  {
    ROS_DEBUG("Action command sent");
  }
  else
  {
    ROS_ERROR_THROTTLE(1.0, "Failed to send action command");
  }
}

}  // namespace trigger
