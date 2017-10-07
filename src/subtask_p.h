// copyright Robert Haschke @ 2017

#pragma once

#include <moveit_task_constructor/subtask.h>
#include <moveit_task_constructor/storage.h>

namespace moveit { namespace task_constructor {

class ContainerBasePrivate;
class SubTaskPrivate {
	friend class SubTask;
	friend class BaseTest; // allow access for unit tests
	friend class ContainerBasePrivate; // allow to set parent_ and it_
	friend std::ostream& operator<<(std::ostream &os, const SubTaskPrivate& stage);

public:
	typedef std::list<SubTask::pointer> array_type;

	SubTaskPrivate(SubTask* me, const std::string& name);

	enum InterfaceFlag {
		READS_INPUT        = 0x01,
		READS_OUTPUT       = 0x02,
		WRITES_NEXT_INPUT  = 0x04,
		WRITES_PREV_OUTPUT = 0x08,

		OWN_IF_MASK        = READS_INPUT | READS_OUTPUT,
		EXT_IF_MASK        = WRITES_NEXT_INPUT | WRITES_PREV_OUTPUT,
		INPUT_IF_MASK      = READS_INPUT | WRITES_PREV_OUTPUT,
		OUTPUT_IF_MASK     = READS_OUTPUT | WRITES_NEXT_INPUT,
	};
	typedef Flags<InterfaceFlag> InterfaceFlags;

	InterfaceFlags interfaceFlags() const;
	InterfaceFlags deducedFlags() const;
	virtual InterfaceFlags announcedFlags() const = 0;
	std::list<SubTrajectory>& trajectories() { return trajectories_; }

public:
	SubTask* const me_; // associated/owning SubTask instance
	const std::string name_;

	planning_scene::PlanningSceneConstPtr scene_;
	planning_pipeline::PlanningPipelinePtr planner_;

	InterfacePtr input_;
	InterfacePtr output_;

	inline const Interface* prevOutput() const { return prev_output_; }
	inline const Interface* nextInput() const { return next_input_; }
	inline bool isConnected() const { return prev_output_ || next_input_; }

protected:
	std::list<SubTrajectory> trajectories_;
	SubTrajectory& addTrajectory(const robot_trajectory::RobotTrajectoryPtr &, double cost);

private:
	// !! items accessed only by ContainerBasePrivate to maintain hierarchy !!
	ContainerBasePrivate* parent_; // owning parent
	array_type::iterator it_; // iterator into parent's children_ list referring to this
	// caching the pointers to the output_ / input_ interface of previous / next stage
	mutable Interface *prev_output_; // interface to be used for sendBackward()
	mutable Interface *next_input_;  // interface to be use for sendForward()
};
std::ostream& operator<<(std::ostream &os, const SubTaskPrivate& stage);


class PropagatingAnyWayPrivate : public SubTaskPrivate {
	friend class PropagatingAnyWay;

public:
	PropagatingAnyWay::Direction dir;

	inline PropagatingAnyWayPrivate(PropagatingAnyWay *me, PropagatingAnyWay::Direction dir,
	                                const std::string &name);
	InterfaceFlags announcedFlags() const override;

	bool hasStartState() const;
	const InterfaceState &fetchStartState();
	void sendForward(const robot_trajectory::RobotTrajectoryPtr& trajectory,
	                 const InterfaceState& from,
	                 const planning_scene::PlanningSceneConstPtr& to,
	                 double cost = 0);

	bool hasEndState() const;
	const InterfaceState &fetchEndState();
	void sendBackward(const robot_trajectory::RobotTrajectoryPtr& trajectory,
	                  const planning_scene::PlanningSceneConstPtr& from,
	                  const InterfaceState& to,
	                  double cost = 0);

protected:
	// get informed when new input or output state becomes available
	void newInputState(const std::list<InterfaceState>::iterator& it);
	void newOutputState(const std::list<InterfaceState>::iterator& it);

	Interface::const_iterator next_input_state_;
	Interface::const_iterator next_output_state_;
};


class PropagatingForwardPrivate : public PropagatingAnyWayPrivate {
public:
	inline PropagatingForwardPrivate(PropagatingForward *me, const std::string &name);

private:
	// restrict access to backward methods to provide compile-time check
	using PropagatingAnyWayPrivate::hasEndState;
	using PropagatingAnyWayPrivate::fetchEndState;
	using PropagatingAnyWayPrivate::sendBackward;
};


class PropagatingBackwardPrivate : public PropagatingAnyWayPrivate {
public:
	inline PropagatingBackwardPrivate(PropagatingBackward *me, const std::string &name);

private:
	// restrict access to forward method to provide compile-time check
	using PropagatingAnyWayPrivate::hasStartState;
	using PropagatingAnyWayPrivate::fetchStartState;
	using PropagatingAnyWayPrivate::sendForward;
};


class GeneratorPrivate : public SubTaskPrivate {
public:
	inline GeneratorPrivate(Generator *me, const std::string &name);
	InterfaceFlags announcedFlags() const override;
	bool spawn(const planning_scene::PlanningSceneConstPtr& ps, double cost);
};


class ConnectingPrivate : public SubTaskPrivate {
	friend class Connecting;

public:
	inline ConnectingPrivate(Connecting *me, const std::string &name);
	InterfaceFlags announcedFlags() const override;
	void connect(const robot_trajectory::RobotTrajectoryPtr& t,
	             const InterfaceStatePair& state_pair, double cost);

private:
	// get informed when new input or output state becomes available
	void newInputState(const std::list<InterfaceState>::iterator& it);
	void newOutputState(const std::list<InterfaceState>::iterator& it);

	std::pair<Interface::const_iterator, Interface::const_iterator> it_pairs_;
};

} }


// get correctly casted private impl pointer
#define IMPL(Class) Class##Private * const impl = static_cast<Class##Private*>(pimpl_func());
