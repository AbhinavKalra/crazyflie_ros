#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "pid.hpp"
#include "crazyflie_driver/Position.h"
#include <geometry_msgs/PointStamped.h>


double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}

class Controller
{
public:

    Controller(
        const std::string& worldFrame,
        const std::string& frame,
        const ros::NodeHandle& n)
        : m_worldFrame(worldFrame)
        , m_frame(frame)
        , m_pubNav()
        , m_pubNeighbourPosition()
        , m_pubCurrentPosition()
        , m_listener()
        , m_state(Idle)
        , m_goal()
        , m_subscribeGoal()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_thrust(0)
        , m_startZ(0)
        , m_startY(0)
        , m_startX(0)

    {
        ros::NodeHandle nh;
        m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0)); 
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
        m_pubCurrentPosition = nh.advertise<geometry_msgs::PointStamped>("external_position",1);
        m_pubNeighbourPosition = nh.advertise<crazyflie_driver::Position>("cmd_neighbour_position",1);
        m_subscribeGoal = nh.subscribe("goal", 1, &Controller::goalChanged, this);
        m_serviceTakeoff = nh.advertiseService("takeoff", &Controller::takeoff, this);
        m_serviceLand = nh.advertiseService("land", &Controller::land, this);
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        ros::spin();
    }

private:
    void goalChanged(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        m_goal = *msg;
    }

    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;

        tf::StampedTransform transform;
        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
        m_startZ = transform.getOrigin().z();
        m_startX = transform.getOrigin().x();
        m_startY = transform.getOrigin().y();

        return true;
    }

    bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;

        return true;
    }

    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
    }

    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch(m_state)
        {
        case TakingOff:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() > m_startZ + 0.05 || m_thrust > 50000)    
                {
                    m_state = Automatic;
                    m_thrust = 0;
                }
                else
                {
                    m_thrust += 10000 * dt;                             ////////
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    msg.linear.y = 0;
                    msg.linear.x = 0;
                    msg.angular.z = 0;
                    m_pubNav.publish(msg);
                }

            }
            break;
        case Landing:
            {
                m_goal.pose.position.z = m_startZ + 0.05;
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() <= m_startZ + 0.05) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    m_pubNav.publish(msg);
                }
            }
            // intentional fall-thru
        case Automatic:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                geometry_msgs::PointStamped current_position_msg;
                crazyflie_driver::Position neighbour_position_msg;
                current_position_msg.point.x = transform.getOrigin().x();
                current_position_msg.point.y = transform.getOrigin().y();
                current_position_msg.point.z = transform.getOrigin().z();
                m_pubCurrentPosition.publish(current_position_msg);                 // current position published
                
                m_listener.lookupTransform(m_worldFrame, "crazyflie0", ros::Time(0), transform);
                neighbour_position_msg.x = transform.getOrigin().x();
                neighbour_position_msg.y = transform.getOrigin().y();
                neighbour_position_msg.z = transform.getOrigin().z();
                neighbour_position_msg.yaw=0;
                m_pubNeighbourPosition.publish(neighbour_position_msg);
            }
            break;
        case Idle:
            {
                geometry_msgs::Twist msg;
                m_pubNav.publish(msg);
            }
            break;
        }
    }

private:

    enum State
    {
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing = 3,
    };

private:
    std::string m_worldFrame;
    std::string m_frame;
    ros::Publisher m_pubNav;
    ros::Publisher m_pubNeighbourPosition;
    ros::Publisher m_pubCurrentPosition;
    tf::TransformListener m_listener;
    State m_state;
    geometry_msgs::PoseStamped m_goal;
    ros::Subscriber m_subscribeGoal;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    float m_thrust;
    float m_startZ;
    float m_startX;
    float m_startY;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "followerController");

  // Read parameters
  ros::NodeHandle n("~");
  std::string worldFrame;
  n.param<std::string>("worldFrame", worldFrame, "/world");
  std::string frame;
  n.getParam("frame", frame);
  double frequency;
  n.param("frequency", frequency, 50.0);

  Controller controller(worldFrame, frame, n);
  controller.run(frequency);

  return 0;
}
