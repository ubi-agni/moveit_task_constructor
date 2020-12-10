#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/task.h>
#include <pybind11/pybind11.h>

namespace moveit {
namespace task_constructor {

// Trampoline classes to allow inheritance in Python (overriding virtual functions)
template <class T = Stage>
class PyStage : public T
{
public:
	using T::T;

	void init(const moveit::core::RobotModelConstPtr& robot_model) override {
		PYBIND11_OVERRIDE(void, T, init, robot_model);
	}
	void reset() override { PYBIND11_OVERRIDE(void, T, reset, ); }
};

template <class T = Generator>
class PyGenerator : public PyStage<T>
{
public:
	using PyStage<T>::PyStage;
	bool canCompute() const override { PYBIND11_OVERRIDE_PURE(bool, T, canCompute, ); }
	void compute() override { PYBIND11_OVERRIDE_PURE(void, T, compute, ); }
};

template <class T = MonitoringGenerator>
class PyMonitoringGenerator : public PyGenerator<T>
{
public:
	using PyGenerator<T>::PyGenerator;
	void onNewSolution(const SolutionBase& s) override { PYBIND11_OVERRIDE_PURE(void, T, onNewSolution, s); }
};

class PubMonitoringGenerator : public MonitoringGenerator
{
public:
	using MonitoringGenerator::onNewSolution;
};

template <class T = PropagatingEitherWay>
class PyPropagatingEitherWay : public PyStage<T>
{
public:
	using PyStage<T>::PyStage;
	void computeForward(const InterfaceState& from) override { PYBIND11_OVERRIDE_PURE(void, T, computeForward, from); }
	void computeBackward(const InterfaceState& to) override { PYBIND11_OVERRIDE_PURE(void, T, computeBackward, to); }
};

template <class T = Connecting>
class PyConnecting : public PyStage<T>
{
public:
	using PyStage<T>::PyStage;
	void compute(const InterfaceState& from, const InterfaceState& to) override {
		PYBIND11_OVERRIDE_PURE(void, T, compute, from, to);
	}
	bool compatible(const InterfaceState& from_state, const InterfaceState& to_state) const override {
		PYBIND11_OVERRIDE(bool, T, compatible, from_state, to_state);
	}
};

class PubConnecting : public Connecting
{
public:
	using Connecting::compatible;
};

template <class T = ContainerBase>
class PyContainerBase : public PyStage<T>
{
public:
	using PyStage<T>::PyStage;

	bool insert(Stage::pointer&& stage, int before = -1) override { PYBIND11_OVERRIDE(bool, T, insert, stage, before); }
	Stage::pointer remove(int pos) override { PYBIND11_OVERRIDE(Stage::pointer, T, remove, pos); }
	Stage::pointer remove(Stage* child) override { PYBIND11_OVERRIDE(Stage::pointer, T, remove, child); }
	void clear() override { PYBIND11_OVERRIDE(void, T, clear, ); }

	bool canCompute() const override { PYBIND11_OVERRIDE_PURE(bool, T, canCompute, ); }
	void compute() override { PYBIND11_OVERRIDE_PURE(void, T, compute, ); }
	void onNewSolution(const SolutionBase& s) override { PYBIND11_OVERRIDE_PURE(void, T, onNewSolution, s); }
};

}  // namespace task_constructor
}  // namespace moveit
