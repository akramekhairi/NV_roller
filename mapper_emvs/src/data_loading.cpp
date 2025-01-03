#include <mapper_emvs/data_loading.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <glog/logging.h>

namespace data_loading {

void parse_rosbag(const std::string &rosbag,
                  std::vector<dvs_msgs::Event>& events_,
                  sensor_msgs::CameraInfo& camera_info_msg,
                  const std::string& event_topic,
                  const std::string& camera_info_topic,
                  const double tmin,
                  const double tmax)
{
  std::vector<std::string> topics;
  topics.push_back(event_topic);
  topics.push_back(camera_info_topic);

  events_.clear();

  rosbag::Bag bag(rosbag, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool continue_looping_through_bag = true;
  bool got_initial_stamp = false;
  ros::Time initial_timestamp;

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if(!continue_looping_through_bag)
    {
      break;
    }

    const std::string& topic_name = m.getTopic();
    VLOG(2) << topic_name;

    // Events
    if (topic_name == topics[0])
    {
      dvs_msgs::EventArray::ConstPtr msg = m.instantiate<dvs_msgs::EventArray>();
      if (msg != NULL)
      {
        if(msg->events.empty())
        {
          continue;
        }
        const ros::Time& stamp = msg->events[0].ts;

        if(!got_initial_stamp)
        {
          initial_timestamp = stamp;
          LOG(INFO) << "initial stamp: " << stamp;
          got_initial_stamp = true;
        }

        for (size_t i = 0; i < msg->events.size(); ++i)
        {
          const double rel_stamp = (msg->events[i].ts - initial_timestamp).toSec();
          if(rel_stamp < tmin)
          {
            continue;
          }
          if(rel_stamp > tmax)
          {
            continue_looping_through_bag = false;
          }

          dvs_msgs::Event ev_modified(msg->events[i]);
          ev_modified.ts = ros::Time(ev_modified.ts.toSec() - initial_timestamp.toSec());
          events_.push_back(ev_modified);
        }
      }
    }

    // Camera Info
    if (topic_name == topics[1])
    {
      camera_info_msg = *(m.instantiate<sensor_msgs::CameraInfo>());
    }
  }

  // Sort events by increasing timestamps
  std::sort(events_.begin(), events_.end(),
            [](const dvs_msgs::Event& a, const dvs_msgs::Event& b) -> bool
  {
    return a.ts < b.ts;
  });
}

} // namespace data_loading

