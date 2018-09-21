#include <boost/python.hpp>

#include <string>

#include <ros/serialization.h>
#include <std_msgs/Int64.h>
#include "robotiq_control/gripper_ur_control.h"


// /* Read a ROS message from a serialized string.
//   */
// template <typename M>
// M from_python(const std::string str_msg)
// {
//   size_t serial_size = str_msg.size();
//   boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
//   for (size_t i = 0; i < serial_size; ++i)
//   {
//     buffer[i] = str_msg[i];
//   }
//   ros::serialization::IStream stream(buffer.get(), serial_size);
//   M msg;
//   ros::serialization::Serializer<M>::read(stream, msg);
//   return msg;
// }

// /* Write a ROS message into a serialized string.
// */
// template <typename M>
// std::string to_python(const M& msg)
// {
//   size_t serial_size = ros::serialization::serializationLength(msg);
//   boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
//   ros::serialization::OStream stream(buffer.get(), serial_size);
//   ros::serialization::serialize(stream, msg);
//   std::string str_msg;
//   str_msg.reserve(serial_size);
//   for (size_t i = 0; i < serial_size; ++i)
//   {
//     str_msg.push_back(buffer[i]);
//   }
//   return str_msg;
// }

// template<class T>
// struct vector_to_python
// {
//   static PyObject* convert(const std::vector<T>& vec)
//   {
//     boost::python::list* l = new boost::python::list();
//     for(std::size_t i = 0; i < vec.size(); i++)
//       (*l).append(vec[i]);

//     return l->ptr();
//   }
// };

class RobotiqGripper : public GripperUR
{
    public:
        RobotiqGripper(): GripperUR() 
        {
          // GripperUR();
        }

        void init()
        {
          GripperUR::init();
        }

};

BOOST_PYTHON_MODULE(_robotiq_gripper_wrapper_cpp)
{
  boost::python::class_<RobotiqGripper>("RobotiqGripper", boost::python::init<>())
    .def("init", &RobotiqGripper::init)
    // .def("open", &RobotiqGripper::open)
    // .def("close", &RobotiqGripper::close)
    // .def("moveto", &RobotiqGripper::moveto)
    // .def("setSpeed", &RobotiqGripper::setSpeed)
    // .def("setForce", &RobotiqGripper::setForce)
    // .def("setPoseTolerance", &RobotiqGripper::setPoseTolerance)
    // .def("setCheckpointAddress", &RobotiqGripper::setCheckpointAddress)
    // .def("getCheckpointAddress", &RobotiqGripper::getCheckpointAddress)
    ;
    
}