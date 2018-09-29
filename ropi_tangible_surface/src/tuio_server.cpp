#include <ros/ros.h>
#include <ros/console.h>

#include <thread>
#include <mutex>
#include <list>
#include <cmath>
#include <string>

#include "ropi_msgs/MultiTouch.h"
#include "ropi_msgs/SingleTouch.h"
#include "ropi_msgs/DeleteSelection.h"

#include "TUIO/TuioServer.h"
#include "TUIO/TuioCursor.h"
#include "oscpack/osc/OscTypes.h"

// #include "TUIO/TcpSender.h"
// #include "TUIO/WebSockSender.h"

namespace RoPI
{
using namespace TUIO;
class TouchSender
{
  public:
	TouchSender(TuioServer *server);
	~TouchSender();

	TuioServer *tuio_server;
	std::list<std::pair<std::string, TuioCursor *>> cursor_list;
	void touchCallback(const ropi_msgs::MultiTouch::ConstPtr &touches);

  private:
	std::mutex mutex;
	ros::ServiceServer delete_service_;
	ros::Subscriber touch_sub_;
	int width, height;
	// int screen_width, screen_height;
	// int window_width, window_height;
	TuioTime frame_time;

	static const uint8_t CURSOR_PRESSED = 0;
	static const uint8_t CURSOR_DRAGGED = 1;
	static const uint8_t CURSOR_RELEASED = 2;
	static const uint8_t CURSOR_UNDETERMINED = 3;

    bool deleteCursor(ropi_msgs::DeleteSelection::Request &req, ropi_msgs::DeleteSelection::Response &res);
	void addCursor(const ropi_msgs::SingleTouch &cursor_msg);
	void releaseCursor(std::string id);
	void processCursors(const std::vector<ropi_msgs::SingleTouch> &touches);
};

void TouchSender::addCursor(const ropi_msgs::SingleTouch &cursor_msg)
{
	std::lock_guard<std::mutex> guard(mutex);
	std::list<std::pair<std::string, TuioCursor *>>::iterator cur = std::find_if(this->cursor_list.begin(), this->cursor_list.end(), [&](const std::pair<std::string, TuioCursor *> &cur) { return cur.first == cursor_msg.id; });
	if (cur != this->cursor_list.end())
	{
		ROS_INFO_STREAM("Pressed ---> Dragged.");
		if (cur->second->getTuioTime() == frame_time)
			return;
		ROS_INFO_STREAM(" Update: " << float((cursor_msg.cursor.x - 1) / width) << " " << float((cursor_msg.cursor.y - 1) / height));
		this->tuio_server->updateTuioCursor(cur->second, float((cursor_msg.cursor.x - 1) / width), float((cursor_msg.cursor.y - 1) / height));
	}
	else
	{
		ROS_INFO_STREAM(" Add: " << float((cursor_msg.cursor.x - 1) / width) << " " << float((cursor_msg.cursor.y - 1) / height));
		TuioCursor *cursor = tuio_server->addTuioCursor(float((cursor_msg.cursor.x - 1) / width), float((cursor_msg.cursor.y - 1) / height));
		cursor->addPositionFilter(1.0f, 0.005f);
		this->cursor_list.push_back(std::make_pair(cursor_msg.id, cursor));
	}
}

void TouchSender::releaseCursor(std::string id)
{
	std::lock_guard<std::mutex> guard(mutex);
	std::list<std::pair<std::string, TuioCursor *>> released_cursor_list = this->cursor_list;
	for (auto &cur : this->cursor_list)
	{
		if (cur.first == id)
		{
			ROS_INFO_STREAM("Release cursor: " << id << " " << cur.first);
			this->tuio_server->removeTuioCursor(cur.second);
			released_cursor_list.remove(cur);
		}
	}
	this->cursor_list = released_cursor_list;
}

void TouchSender::processCursors(const std::vector<ropi_msgs::SingleTouch> &touches)
{
	for (auto &cur : this->cursor_list)
	{
		std::cout << "id: " << cur.first << " " << std::endl;
	}
	for (auto &cursor_msg : touches)
	{
		ROS_INFO_STREAM("State: " << int(cursor_msg.state));
		if (cursor_msg.state == this->CURSOR_RELEASED)
		{
			ROS_INFO_STREAM("Released.");
			this->releaseCursor(cursor_msg.id);
		}
		else if (cursor_msg.state == this->CURSOR_UNDETERMINED)
		{
			ROS_INFO_STREAM("Releasing.");
			// this->releaseCursor(cursor_msg.id);
		}
		else if (cursor_msg.state == this->CURSOR_DRAGGED)
		{
			ROS_INFO_STREAM("Dragged.");
			bool drag_matched = false;
			for (auto &cur : this->cursor_list)
			{
				if (cursor_msg.id == cur.first)
				{	
					drag_matched = true;
					if (cur.second->getTuioTime() == frame_time)
						break;
					std::cout << " Update: " << float((cursor_msg.cursor.x - 1) / width) << " " << float((cursor_msg.cursor.y - 1) / height) << std::endl;
					this->tuio_server->updateTuioCursor(cur.second, float((cursor_msg.cursor.x - 1) / width), float((cursor_msg.cursor.y - 1) / height));
					break;
				}
			}
			if (!drag_matched)
			{
				ROS_INFO_STREAM("Unmatched drag");
				this->addCursor(cursor_msg);
			}
		}
		else if (cursor_msg.state == this->CURSOR_PRESSED)
		{
			ROS_INFO_STREAM("Pressed.");
			this->addCursor(cursor_msg);
			// bool matched = false;
			// for (auto &cur : this->cursor_list)
			// {
			// 	if (!matched && cursor_msg.id == cur.first)
			// 	{
			// 		matched = true;
			// 		this->tuio_server->updateTuioCursor(cur.second, cursor_msg.cursor.x, cursor_msg.cursor.y);
			// 		break;
			// 	}
			// }
			// if (!matched)
			// {
			// 	this->addCursor(cursor_msg);
			// }
		}
		else
		{
			ROS_ERROR("State Error!");
		}
	}
}

void TouchSender::touchCallback(const ropi_msgs::MultiTouch::ConstPtr &msg)
{
	ROS_INFO_STREAM("Callback " << msg->cursors.size());
	frame_time = TuioTime::getSessionTime();
	this->tuio_server->initFrame(frame_time);
	this->width = msg->width;
	this->height = msg->height;
	if (msg->cursors.size() != 0)
	{
		std::vector<ropi_msgs::SingleTouch> touches = msg->cursors;
		this->processCursors(touches);
	}
	tuio_server->stopUntouchedMovingCursors();
	tuio_server->commitFrame();
}
bool TouchSender::deleteCursor(ropi_msgs::DeleteSelection::Request &req, 
								ropi_msgs::DeleteSelection::Response &res)
{
	ROS_INFO_STREAM("release");
	this->releaseCursor(req.guid);
	res.success = true;
	res.message = "delete";
	return true;
}

TouchSender::TouchSender(TuioServer *server)
	//: screen_width(1024), screen_height(768), window_width(320), window_height(200)
{
	ros::NodeHandle nh;
	TuioTime::initSession();
	frame_time = TuioTime::getSessionTime();

	tuio_server = server;
	tuio_server->setSourceName("ROS");
	tuio_server->setVerbose(true);
	tuio_server->setInversion(true, true, false);
	tuio_server->enableObjectProfile(false);
	tuio_server->enableBlobProfile(false);
	delete_service_ = nh.advertiseService("delete_cursor", &TouchSender::deleteCursor, this);
	touch_sub_ = nh.subscribe("touch", 100, &TouchSender::touchCallback, this);
	ros::Rate r(150); // 50 hz
	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
}

} // namespace RoPI

using namespace RoPI;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "tuio_server");
	TuioServer *server = NULL;
	if (argc == 3)
	{
		server = new TuioServer(argv[1], atoi(argv[2]));
	}
	else
		server = new TuioServer(); // default is UDP port 3333 on localhost

	TouchSender *ts = new TouchSender(server);
}