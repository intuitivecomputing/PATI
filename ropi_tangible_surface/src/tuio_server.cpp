#include <ros/ros.h>
#include "ropi_msgs/MultiTouch.h"
#include "ropi_msgs/SingleTouch.h"

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
	std::list<std::pair<int, TuioCursor *>> cursor_list;
	void touchCallback(const ropi_msgs::MultiTouch::ConstPtr &touches);

  private:
	ros::Subscriber touch_sub_;
	int width, height;
	int screen_width, screen_height;
	int window_width, window_height;
	TuioTime frame_time;

	static const uint8_t CURSOR_PRESSED = 0;
	static const uint8_t CURSOR_DRAGGED = 1;
	static const uint8_t CURSOR_RELEASED = 2;

	
	void addCursor(const ropi_msgs::SingleTouch &cursor_msg);
	void releaseCursor(int id);
	void processCursors(const std::vector<ropi_msgs::SingleTouch> &touches);
};

void TouchSender::addCursor(const ropi_msgs::SingleTouch &cursor_msg)
{
	TuioCursor *cursor = tuio_server->addTuioCursor(cursor_msg.cursor.x, cursor_msg.cursor.y);
	this->cursor_list.push_back(std::make_pair(cursor_msg.id, cursor));
}

void TouchSender::releaseCursor(int id)
{
	for (auto &cur : this->cursor_list)
	{
		if (cur.first == id)
		{
			this->cursor_list.remove(cur);
			this->tuio_server->removeTuioCursor(cur.second);
		}
	}
}

void TouchSender::processCursors(const std::vector<ropi_msgs::SingleTouch> &touches)
{
	for (auto &cursor_msg : touches)
	{
		if (cursor_msg.state == this->CURSOR_RELEASED)
		{
			this->releaseCursor(cursor_msg.id);
		}
		else
		{
			bool matched = false;
			for (auto &cur : this->cursor_list)
			{
				if (!matched && cursor_msg.id == cur.first)
				{
					matched = true;
					this->tuio_server->updateTuioCursor(cur.second, cursor_msg.cursor.x, cursor_msg.cursor.y);
					break;
				}
			}
			if (!matched)
			{
				this->addCursor(cursor_msg);
			}
		}
	}
}

void TouchSender::touchCallback(const ropi_msgs::MultiTouch::ConstPtr &msg)
{
	frame_time = TuioTime::getSessionTime();
	this->tuio_server->initFrame(frame_time);
	if (msg->cursors.size() == 0)
		return;

	std::vector<ropi_msgs::SingleTouch> touches = msg->cursors;
	this->processCursors(touches);

	tuio_server->stopUntouchedMovingCursors();
	tuio_server->commitFrame();
}

TouchSender::TouchSender(TuioServer *server)
	: screen_width(1024), screen_height(768), window_width(640), window_height(480)
{
	ros::NodeHandle nh;
	TuioTime::initSession();
	frame_time = TuioTime::getSessionTime();

	tuio_server = server;
	tuio_server->setSourceName("ROS");
	tuio_server->enableObjectProfile(false);
	tuio_server->enableBlobProfile(false);
	touch_sub_ = nh.subscribe("touch", 100, &TouchSender::touchCallback, this);
	ros::Rate r(50); // 50 hz
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
	TuioServer *server = new TuioServer();;

	TouchSender *ts = new TouchSender(server);
}