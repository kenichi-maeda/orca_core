#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <cstring>

#include "dynamixel_native.h"

namespace py = pybind11;

PYBIND11_MODULE(dynamixel_native, m)
{
    py::class_<DynamixelNative>(m, "Client")

        .def(py::init<const std::string&, int, const std::vector<int>&>(),
             py::arg("port"),
             py::arg("baudrate"),
             py::arg("ids"))

        .def("connect", &DynamixelNative::connect)
        .def("disconnect", &DynamixelNative::disconnect)

        .def("enable_torque",
             &DynamixelNative::enable_torque,
             py::arg("ids"),
             py::arg("enable"),
             py::arg("retries") = -1,
             py::arg("retry_interval_sec") = 0.25)

        .def("set_operating_mode",
             &DynamixelNative::set_operating_mode,
             py::arg("ids"),
             py::arg("mode"))

        .def("set_goal_position",
             &DynamixelNative::set_goal_position,
             py::arg("ids"),
             py::arg("ticks"))

        .def("set_goal_current",
             &DynamixelNative::set_goal_current,
             py::arg("ids"),
             py::arg("ticks"))

        .def("set_profile_velocity",
             &DynamixelNative::set_profile_velocity,
             py::arg("ids"),
             py::arg("ticks"))

        .def("sync_write",
             &DynamixelNative::sync_write,
             py::arg("ids"),
             py::arg("values"),
             py::arg("address"),
             py::arg("size"))


        .def("read_pos_vel_cur",
             [](DynamixelNative& self)
             {
                 auto result = self.read_pos_vel_cur();

                 const auto& pos = std::get<0>(result);
                 const auto& vel = std::get<1>(result);
                 const auto& cur = std::get<2>(result);

                 py::array_t<float> pos_arr(pos.size());
                 py::array_t<float> vel_arr(vel.size());
                 py::array_t<float> cur_arr(cur.size());

                 std::memcpy(pos_arr.mutable_data(), pos.data(), pos.size() * sizeof(float));
                 std::memcpy(vel_arr.mutable_data(), vel.data(), vel.size() * sizeof(float));
                 std::memcpy(cur_arr.mutable_data(), cur.data(), cur.size() * sizeof(float));

                 return py::make_tuple(pos_arr, vel_arr, cur_arr);
             })

        .def("read_temperature",
             &DynamixelNative::read_temperature)

        .def("read_moving_status",
             &DynamixelNative::read_moving_status);
}
