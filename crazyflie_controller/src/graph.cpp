#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "pid.hpp"
//#include <Eigen/Geometry>
// #include <Eigen/Dense>
#include <cmath>

using namespace std;

struct Position
{
	float x,y,z;
	float oX,oY,oZ;
	Position ()
	{
		x=y=z=oX=oY=0;
		oZ=0;
	}
	Position subtract (Position Q)
	{
		Position P;
		P.x= x-Q.x;
		P.y= y-Q.y;
		P.z= z-Q.z;
		return P;
	}
	void add (Position Q)
	{
		x= x+Q.x;
		y= y+Q.y;
		z= z+Q.z;
	}
		void divide (int i)
	{
		x= x/i;
		y= y/i;
		z= z/i;
	}
};

class Graph {
	private:
	    enum State {
	        Idle = 0,
	        Automatic = 1,
	        TakingOff = 2,
	        Landing = 3,
	    };

		vector<Position> poseList;
		vector <vector<int> > edge;
		vector<vector<Position> > formation;
		vector<geometry_msgs::PoseStamped> goals;
		vector<ros::Publisher> publish_goals;
		int count;
		std::string m_worldFrame;
    	vector<std::string> m_frame;
		tf::TransformListener m_listener;	


		void updatePosition() {
			for(int i=0;i<count;i++) {
				tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame.at(i), ros::Time(0), transform);
                poseList.at(i).x=transform.getOrigin().x();
                poseList.at(i).y=transform.getOrigin().y();
                poseList.at(i).z=transform.getOrigin().z();
			}
		}

	 	void iteration(const ros::TimerEvent& e){
	        computeNewPosition();
	        // cout<<"-------------- NO ERROR YET --------------"<<endl;
	    }

		void insertNode(int i) {
			//Henneberg construction 
			//count++;
			Position P;
			//vector<int> (edge.at(i))();										/////////////////////
			poseList.push_back(P);	
			if(i==1)
				edge.at(1).push_back(0);
			else if(i<=5) {
				edge.at(i).push_back(i-2);
				edge.at(i).push_back(i-1);
			}
			else {
				edge.at(i).push_back(i-3);
				edge.at(i).push_back(i-2);
				edge.at(i).push_back(i-1);
			}
		}
		
		void getFormation(string filename) {
			string line;
			ifstream myfile("/home/abhinav/crazyflie_ws/src/crazyflie_ros/crazyflie_controller/src/formation.txt");
			if(myfile.is_open()) {
				formation.resize(count);
				for(int i=0;i<count;i++) {
				 	formation.at(i).resize(count);
				 	for(int j=0;j<count;j++) {
				 		float f;
				 		myfile>>f;
				 		// cout<<f<<endl;
				 		formation.at(i).at(j).x=f;
				 	}			
				}
				for(int i=0;i<count;i++) {
				 	for(int j=0;j<count;j++) {
				 		float f;
				 		myfile>>f;
				 		// cout<<f<<endl;
				 		formation.at(i).at(j).y=f;
				 	}			
				}
				for(int i=0;i<count;i++) {
				 	for(int j=0;j<count;j++) {
				 		float f;
				 		myfile>>f;
				 		// cout<<f<<endl;
				 		formation.at(i).at(j).z=f;
				 	}			
				}
				myfile.close();
			}
		}

		void computeNewPosition() {
			updatePosition();
			for(int i=1;i<count;i++) {
				Position current=getPosition(i);
				Position desired;
				for(int j=0;j<edge.at(i).size();j++) {
					int k= edge.at(i).at(j);
					Position p=getPosition(k);
					desired.add(p.subtract(formation.at(i).at(j))); 
 				}
 				desired.divide(edge.at(i).size());
				goals.at(i).header.seq += 1;
				goals.at(i).header.stamp = ros::Time::now();
				goals.at(i).pose.position.x = desired.x;
				goals.at(i).pose.position.y = desired.y;
				goals.at(i).pose.position.z = desired.z;

				cout<<"No error yet"<<endl;

				tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
				q=q.normalized();
				for(int i=0;i<4;i++)
				std::cout<<q[i]<<endl;

				goals.at(i).pose.orientation.x = 0;
				goals.at(i).pose.orientation.y = 0;//q[1];
				goals.at(i).pose.orientation.z = 0;//q[2];
				goals.at(i).pose.orientation.w = 1;//q[3];

	            publish_goals.at(i).publish(goals.at(i));
                ////////////// Give desired position to each node somehow?????
			}
		}

	public:
		Graph(){}
		Graph(std::string worldFrame,int frame) {
			count=frame;
			m_worldFrame=worldFrame;
			m_frame.resize(count);
			ros::NodeHandle nh;
			//m_listener.resize(count); /////////////////////////////////////////
			edge.resize(count);
			vector<vector<int> > edge(count,vector<int>());   /////////////////////////
			goals.resize(count);
			publish_goals.resize(count);
			for(int i=0;i<count;i++) {
				m_frame.at(i)="crazyflie"+std::to_string(i);
				m_listener.waitForTransform(m_worldFrame,m_frame.at(i),ros::Time(0),ros::Duration(10.0));
				insertNode(i);
				goals.at(i).header.seq=0;
				goals.at(i).header.frame_id=m_worldFrame;
				if(i != 0){
					publish_goals.at(i)=nh.advertise<geometry_msgs::PoseStamped>(m_frame.at(i)+"/goal",1); 			// m_frame.at(i)+"/goal"					
				}
			}
			this->getFormation("formation.txt");
		}
		
		Position getPosition(int i) {
			return poseList.at(i);
		}

	    void run(double frequency) {
	        ros::NodeHandle node;
	        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Graph::iteration, this);
	        ros::spin();
	    }
};

int main(int argc, char **argv)
{
	cout<<"-------------------------------------Graph started running-------------------------------------"<<endl<<endl;	
	ros::init(argc, argv, "graph");

	// Read parameters
	ros::NodeHandle n("~");
	std::string worldFrame;
	n.param<std::string>("worldFrame", worldFrame, "/world");
	int frame;
	n.getParam("frame", frame);
	double frequency;
	n.param("frequency", frequency, 50.0);
	Graph graph(worldFrame,frame);
	graph.run(frequency);

	return 0;
}
