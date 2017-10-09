#include "subtask_p.h"
#include "container_p.h"
#include <iostream>
#include <iomanip>
#include <ros/console.h>

namespace moveit { namespace task_constructor {

SubTask::SubTask(SubTaskPrivate *impl)
   : pimpl_(impl)
{
}

SubTask::~SubTask()
{
	delete pimpl_;
}

const std::string& SubTask::getName() const {
	return pimpl_->name_;
}

std::ostream& operator<<(std::ostream &os, const SubTask& stage) {
	os << *stage.pimpl_func();
	return os;
}

template<SubTaskPrivate::InterfaceFlag own, SubTaskPrivate::InterfaceFlag other>
const char* direction(const SubTaskPrivate& stage) {
	SubTaskPrivate::InterfaceFlags f = stage.deducedFlags();
	bool own_if = f & own;
	bool other_if = f & other;
	bool reverse = own & SubTaskPrivate::INPUT_IF_MASK;
	if (own_if && other_if) return "<>";
	if (!own_if && !other_if) return "--";
	if (other_if ^ reverse) return "->";
	return "<-";
};

std::ostream& operator<<(std::ostream &os, const SubTaskPrivate& stage) {
	// inputs
	for (const Interface* i : {stage.prev_output_, stage.input_.get()}) {
		os << std::setw(3);
		if (i) os << i->size();
		else os << "-";
	}
	// trajectories
	os << std::setw(5) << direction<SubTaskPrivate::READS_INPUT, SubTaskPrivate::WRITES_PREV_OUTPUT>(stage)
	   << std::setw(3) << stage.trajectories_.size()
	   << std::setw(5) << direction<SubTaskPrivate::READS_OUTPUT, SubTaskPrivate::WRITES_NEXT_INPUT>(stage);
	// outputs
	for (const Interface* i : {stage.output_.get(), stage.next_input_}) {
		os << std::setw(3);
		if (i) os << i->size();
		else os << "-";
	}
	// name
	os << " / " << stage.name_;
	return os;
}


SubTaskPrivate::SubTaskPrivate(SubTask *me, const std::__cxx11::string &name)
   : me_(me), name_(name), parent_(nullptr), prev_output_(nullptr), next_input_(nullptr)
{}

SubTrajectory& SubTaskPrivate::addTrajectory(const robot_trajectory::RobotTrajectoryPtr& trajectory, double cost){
	trajectories_.emplace_back(trajectory);
	SubTrajectory& back = trajectories_.back();
	return back;
}

SubTaskPrivate::InterfaceFlags SubTaskPrivate::interfaceFlags() const
{
	InterfaceFlags result = announcedFlags();
	result &= ~InterfaceFlags(OWN_IF_MASK);
	result |= deducedFlags();
	return result;
}

// return the interface flags that can be deduced from the interface
inline SubTaskPrivate::InterfaceFlags SubTaskPrivate::deducedFlags() const
{
	InterfaceFlags f;
	if (input_)  f |= READS_INPUT;
	if (output_) f |= READS_OUTPUT;
	if (prevOutput()) f |= WRITES_PREV_OUTPUT;
	if (nextInput())  f |= WRITES_NEXT_INPUT;
	return f;
}


PropagatingAnyWayPrivate::PropagatingAnyWayPrivate(PropagatingAnyWay *me, PropagatingAnyWay::Direction dir, const std::__cxx11::string &name)
   : SubTaskPrivate(me, name), dir(dir)
{
}

SubTaskPrivate::InterfaceFlags PropagatingAnyWayPrivate::announcedFlags() const {
	InterfaceFlags f;
	if (dir & PropagatingAnyWay::FORWARD)
		f |= InterfaceFlags({READS_INPUT, WRITES_NEXT_INPUT});
	if (dir & PropagatingAnyWay::BACKWARD)
		f |= InterfaceFlags({READS_OUTPUT, WRITES_PREV_OUTPUT});
	return f;
}

inline bool PropagatingAnyWayPrivate::hasStartState() const{
	return next_input_state_ != input_->end();
}

const InterfaceState& PropagatingAnyWayPrivate::fetchStartState(){
	if (!hasStartState())
		throw std::runtime_error("no new state for beginning available");

	const InterfaceState& state= *next_input_state_;
	++next_input_state_;

	return state;
}

inline bool PropagatingAnyWayPrivate::hasEndState() const{
	return next_output_state_ != output_->end();
}

const InterfaceState& PropagatingAnyWayPrivate::fetchEndState(){
	if(!hasEndState())
		throw std::runtime_error("no new state for ending available");

	const InterfaceState& state= *next_output_state_;
	++next_output_state_;

	return state;
}

bool PropagatingAnyWayPrivate::canCompute() const
{
	if ((dir & PropagatingAnyWay::FORWARD) && hasStartState())
		return true;
	if ((dir & PropagatingAnyWay::BACKWARD) && hasEndState())
		return true;
	return false;
}

bool PropagatingAnyWayPrivate::compute()
{
	PropagatingAnyWay* me = static_cast<PropagatingAnyWay*>(me_);

	bool result = false;
	planning_scene::PlanningScenePtr ps;
	robot_trajectory::RobotTrajectoryPtr trajectory;
	double cost;
	if ((dir & PropagatingAnyWay::FORWARD) && hasStartState()) {
		if (me->computeForward(fetchStartState()))
			result |= true;
	}
	if ((dir & PropagatingAnyWay::BACKWARD) && hasEndState()) {
		if (me->computeBackward(fetchEndState()))
			result |= true;
	}
	return result;
}

void PropagatingAnyWayPrivate::newInputState(const Interface::iterator &it)
{
	// we just appended a state to the list, but the iterator doesn't see it anymore
	// so let's point it at the new one
	if(next_input_state_ == input_->end())
		--next_input_state_;
}

void PropagatingAnyWayPrivate::newOutputState(const Interface::iterator &it)
{
	// we just appended a state to the list, but the iterator doesn't see it anymore
	// so let's point it at the new one
	if(next_output_state_ == output_->end())
		--next_output_state_;
}


PropagatingAnyWay::PropagatingAnyWay(const std::string &name)
   : PropagatingAnyWay(new PropagatingAnyWayPrivate(this, ANYWAY, name))
{
}

PropagatingAnyWay::PropagatingAnyWay(PropagatingAnyWayPrivate *impl)
   : SubTask(impl)
{
	initInterface();
}

void PropagatingAnyWay::initInterface()
{
	IMPL(PropagatingAnyWay)
	if (impl->dir & PropagatingAnyWay::FORWARD) {
		if (!impl->input_) { // keep existing interface if possible
			impl->input_.reset(new Interface([impl](const Interface::iterator& it) { impl->newInputState(it); }));
			impl->next_input_state_ = impl->input_->begin();
		}
	} else {
		impl->input_.reset();
		impl->next_input_state_ = Interface::iterator();
	}

	if (impl->dir & PropagatingAnyWay::BACKWARD) {
		if (!impl->output_) { // keep existing interface if possible
			impl->output_.reset(new Interface([impl](const Interface::iterator& it) { impl->newOutputState(it); }));
			impl->next_output_state_ = impl->output_->end();
		}
	} else {
		impl->output_.reset();
		impl->next_output_state_ = Interface::iterator();
	}
}

void PropagatingAnyWay::restrictDirection(PropagatingAnyWay::Direction dir)
{
	IMPL(PropagatingAnyWay);
	if (impl->dir == dir) return;
	if (impl->isConnected())
		throw std::runtime_error("Cannot change direction after being connected");
	impl->dir = dir;
	initInterface();
}

void PropagatingAnyWay::sendForward(const InterfaceState& from,
                                           const planning_scene::PlanningSceneConstPtr& to,
                                           const robot_trajectory::RobotTrajectoryPtr& t,
                                           double cost){
	IMPL(PropagatingAnyWay)
	std::cout << "sending state forward" << std::endl;
	SubTrajectory &trajectory = impl->addTrajectory(t, cost);
	trajectory.setStartState(from);
	impl->nextInput()->add(to, &trajectory, NULL);
}

void PropagatingAnyWay::sendBackward(const planning_scene::PlanningSceneConstPtr& from,
                                            const InterfaceState& to,
                                            const robot_trajectory::RobotTrajectoryPtr& t,
                                            double cost){
	IMPL(PropagatingAnyWay)
	std::cout << "sending state backward" << std::endl;
	SubTrajectory& trajectory = impl->addTrajectory(t, cost);
	trajectory.setEndState(to);
	impl->prevOutput()->add(from, NULL, &trajectory);
}


PropagatingForwardPrivate::PropagatingForwardPrivate(PropagatingForward *me, const std::__cxx11::string &name)
   : PropagatingAnyWayPrivate(me, PropagatingAnyWay::FORWARD, name)
{
	// indicate, that we don't accept new states from output interface
	output_.reset();
	next_output_state_ = Interface::iterator();
}


PropagatingForward::PropagatingForward(const std::string& name)
   : PropagatingAnyWay(new PropagatingForwardPrivate(this, name))
{}


PropagatingBackwardPrivate::PropagatingBackwardPrivate(PropagatingBackward *me, const std::__cxx11::string &name)
   : PropagatingAnyWayPrivate(me, PropagatingAnyWay::BACKWARD, name)
{
	// indicate, that we don't accept new states from input interface
	input_.reset();
	next_input_state_ = Interface::iterator();
}


PropagatingBackward::PropagatingBackward(const std::string &name)
   : PropagatingAnyWay(new PropagatingBackwardPrivate(this, name))
{}


GeneratorPrivate::GeneratorPrivate(Generator *me, const std::__cxx11::string &name)
   : SubTaskPrivate(me, name)
{}

SubTaskPrivate::InterfaceFlags GeneratorPrivate::announcedFlags() const {
	return InterfaceFlags({WRITES_NEXT_INPUT,WRITES_PREV_OUTPUT});
}

bool GeneratorPrivate::canCompute() const {
	return static_cast<Generator*>(me_)->canCompute();
}

bool GeneratorPrivate::compute() {
	return static_cast<Generator*>(me_)->compute();
}


Generator::Generator(const std::string &name)
   : SubTask(new GeneratorPrivate(this, name))
{}

void Generator::spawn(const planning_scene::PlanningSceneConstPtr& ps, double cost)
{
	std::cout << "spawning state forwards and backwards" << std::endl;
	IMPL(Generator)
	// empty trajectory ref -> this node only produces states
	robot_trajectory::RobotTrajectoryPtr dummy;
	SubTrajectory& trajectory = impl->addTrajectory(dummy, cost);
	impl->prevOutput()->add(ps, NULL, &trajectory);
	impl->nextInput()->add(ps, &trajectory, NULL);
}


ConnectingPrivate::ConnectingPrivate(Connecting *me, const std::__cxx11::string &name)
   : SubTaskPrivate(me, name)
{
	input_.reset(new Interface([this](const Interface::iterator& it) { this->newInputState(it); }));
	output_.reset(new Interface([this](const Interface::iterator& it) { this->newOutputState(it); }));
	it_pairs_ = std::make_pair(input_->begin(), output_->begin());
}

SubTaskPrivate::InterfaceFlags ConnectingPrivate::announcedFlags() const {
	return InterfaceFlags({READS_INPUT, READS_OUTPUT});
}

void ConnectingPrivate::newInputState(const Interface::iterator& it)
{
	// TODO: need to handle the pairs iterator
	if(it_pairs_.first == input_->end())
		--it_pairs_.first;
}

void ConnectingPrivate::newOutputState(const Interface::iterator& it)
{
	// TODO: need to handle the pairs iterator properly
	if(it_pairs_.second == output_->end())
		--it_pairs_.second;
}

bool ConnectingPrivate::canCompute() const{
	// TODO: implement this properly
	return it_pairs_.first != input_->end() &&
	       it_pairs_.second != output_->end();
}

bool ConnectingPrivate::compute() {
	// TODO: implement this properly
	const InterfaceState& from = *it_pairs_.first;
	const InterfaceState& to = *(it_pairs_.second++);
	return static_cast<Connecting*>(me_)->compute(from, to);
}


Connecting::Connecting(const std::string &name)
   : SubTask(new ConnectingPrivate(this, name))
{
}

void Connecting::connect(const InterfaceState& from, const InterfaceState& to,
                         const robot_trajectory::RobotTrajectoryPtr& t, double cost) {
	IMPL(Connecting)
	SubTrajectory& trajectory = impl->addTrajectory(t, cost);
	trajectory.setStartState(from);
	trajectory.setEndState(to);
}

} }
