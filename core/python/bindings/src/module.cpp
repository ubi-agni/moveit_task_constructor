/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <pybind11/smart_holder.h>

namespace moveit {
namespace python {

void export_properties(pybind11::module& m);
void export_solvers(pybind11::module& m);
void export_core(pybind11::module& m);
void export_stages(pybind11::module& m);

}  // namespace python
}  // namespace moveit

PYBIND11_MODULE(pymoveit_mtc, m) {
	// disable function signatures in generated documentation
	pybind11::options options;
	options.disable_function_signatures();

	auto msub = m.def_submodule("core", "MoveIt Task Contructor Core");
	msub.doc() = R"pbdoc(
		This python package contains
		core components such as
		base types of stage-
		and planner classes.
	)pbdoc";

	moveit::python::export_properties(msub);
	moveit::python::export_solvers(msub);
	moveit::python::export_core(msub);

	msub = m.def_submodule("stages", "MoveIt Task Constructor Stages");
	msub.doc() = R"pbdoc(
		This python package contains
		all stages that
		are available to the user.
		To use a stage, create an instance, then
		add it to the task hierarchy at the
		desired spot.
 		The arrangement of stages in the hierarchy
		define the task to be carried out.
	)pbdoc";
	moveit::python::export_stages(msub);
}
