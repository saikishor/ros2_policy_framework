Extending the framework
=======================

Every piece that touches data is a plugin, loaded by its class name from the parameters. New
behaviour is added by writing a plugin in your own package; the framework itself does not
change.

Which plugin to write
---------------------

.. graphviz::

   digraph which_plugin {
     rankdir=TB;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     q1 [label="What do you need?", shape=diamond, fillcolor="#fff6d9"];
     source [label="InputSource"];
     term [label="ObservationTerm\n(or TopicTerm)"];
     command [label="CommandTerm"];
     pre [label="Preprocessor\n+ Postprocessor"];
     inf [label="Inference"];
     exec [label="Executor\n(node)"];
     iexec [label="InterfaceExecutor\n(controller)"];
     check [label="SafetyCheck"];
     q1 -> source [label="A sensor topic as\nnamed state entries"];
     q1 -> term [label="A new observation\nterm for mjlab policies"];
     q1 -> command [label="A user command\nfor mjlab policies"];
     q1 -> pre [label="Policies from another\ntraining framework"];
     q1 -> inf [label="Another model format\nor runtime"];
     q1 -> exec [label="Commands on\nanother topic type"];
     q1 -> iexec [label="Another control law\nor command interface"];
     q1 -> check [label="A state the robot\nmust not be in"];
   }

.. list-table::
   :header-rows: 1
   :widths: 22 30 48

   * - Base class
     - It turns
     - Examples in the framework
   * - ``InputSource``
     - a topic into state entries
     - ``JointStateSource``, ``ImuSource``, ``TwistSource``, ``OdometrySource``
   * - ``Preprocessor``
     - state entries into the model's input tensors
     - ``MjlabPreprocessor``, ``ConcatPreprocessor``
   * - ``ObservationTerm``
     - state entries (or a topic) into one block of the observation
     - ``TwistCommandTerm``
   * - ``Inference``
     - input tensors into output tensors
     - ``OnnxInference``
   * - ``Postprocessor``
     - output tensors into command entries
     - ``MjlabPostprocessor``, ``SplitPostprocessor``
   * - ``Executor``
     - command entries into topics (node)
     - ``MultiDofCommandExecutor``
   * - ``InterfaceExecutor``
     - command entries into command interface values (controller)
     - ``PdEffortExecutor``, ``ReferenceInterfaceExecutor``
   * - ``SafetyCheck``
     - state entries (and the kinematics) into a ``Verdict``: safe or not, why, and
       optionally the reaction to a fault
     - ``StateBoundsCheck``, ``JumpCheck``, ``MockSafetyCheck``

Both hosts run the ``SafetyCheck`` plugins (:doc:`safety`).

The base classes are in ``policy_framework/plugin_interfaces.hpp`` and
``policy_framework/observation_term.hpp``, each method documented there.

The life of a plugin
--------------------

.. graphviz::

   digraph plugin_life {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     configure [label="configure()\nread parameters,\nregister entries,\nallocate", fillcolor="#fff6d9"];
     cycle [label="Control cycle\ncompute() / read() / write() / check()\nindices only, no allocation", fillcolor="#e8f5e9"];
     reset [label="reset() / clear()\nforget per-run state"];
     configure -> reset [label="host activated"];
     reset -> cycle;
     cycle -> cycle [label="every step\nor update"];
     cycle -> reset [label="policy activated,\nhost reactivated", style=dashed];
   }

Two phases, with different rules:

**Configuring** happens once, outside the control loop. Here a plugin reads its parameters,
checks the model, *registers* every store entry it will read or write (keeping the returned
index), creates subscriptions and allocates its buffers. Anything wrong is reported by
throwing an exception: the host then fails to configure, with the message.

**The control cycle** runs at the control rate, in the controller inside the real-time loop.
Here a plugin only uses the indices it kept. It must not allocate memory, wait on a lock,
look anything up by name, or log on every call.

Parameters
~~~~~~~~~~

A plugin receives a ``PluginContext``: the node, its own name, and the kinematics if
enabled. Its parameters live under its name, so two policies can use the same plugin class
with different settings:

.. code-block:: yaml

   loco:                       # name of the policy = prefix of its stages' parameters
     preprocessor: my_package/MyPreprocessor
     my_parameter: 3.0

Example: an observation term from a topic
-----------------------------------------

``TopicTerm`` owns the subscription and hands the latest message to the control cycle
safely. A term only says how big it is and how to write its value. Its ``max_age`` parameter
makes the host fault when the topic stops (:doc:`safety`):

.. code-block:: cpp

   #include "policy_framework/observation_term.hpp"

   using namespace policy_framework;

   class TargetTerm : public TopicTerm<geometry_msgs::msg::PointStamped>
   {
   public:
     TargetTerm() : TopicTerm("target") {}   // default topic

     double compute(const DataStore &, const TensorMap &, float * out) override
     {
       const auto in = latest();             // ready() is true once a message arrived
       out[0] = static_cast<float>(in.msg->point.x);
       out[1] = static_cast<float>(in.msg->point.y);
       out[2] = static_cast<float>(in.msg->point.z);
       // When the value was produced: the header stamp, else now.
       const auto & h = in.msg->header.stamp;
       return h.sec == 0 && h.nanosec == 0 ? now() : rclcpp::Time(h).seconds();
     }

   protected:
     size_t configure_term(const PluginContext &, const ModelInfo &) override {return 3;}
   };

   PLUGINLIB_EXPORT_CLASS(TargetTerm, policy_framework::ObservationTerm)

Declare it in a ``plugins.xml`` and register that file against ``policy_framework``:

.. code-block:: cmake

   pluginlib_export_plugin_description_file(policy_framework plugins.xml)

Then use it for the term of that name in a policy:

.. code-block:: yaml

   loco:
     terms:
       target:
         type: my_package/TargetTerm
         topic: /target

Example: a user command term
----------------------------

A term that feeds a user command, such as a velocity command or a motion reference, derives
from ``CommandTerm``. It only says how to write the command and the neutral command; the base
does what keeps a command safe, so that no command term can forget it:

- the neutral command before the first message, once the last one was received longer ago
  than ``timeout`` (above zero), while ``usable()`` refuses the message, and while the
  ``CommandGate`` is closed (the safety entry is 0, or the ``NEUTRAL_COMMANDS`` reaction is
  active);
- the stamp: the header stamp when the message type has one and it is set, else the receive
  time, so that the input age of the command is how long ago it arrived (the neutral command
  is stamped now);
- the user command flag, so that its stamp does not count for the observation stamp
  (:doc:`data_flow`).

.. code-block:: cpp

   #include "policy_framework/observation_term.hpp"
   #include "std_msgs/msg/float64.hpp"

   using namespace policy_framework;

   class HeadingCommand : public CommandTerm<std_msgs::msg::Float64>
   {
   public:
     HeadingCommand() : CommandTerm("heading", 0.5) {}   // default topic, default timeout (s)

   protected:
     size_t configure_command(const PluginContext &, const ModelInfo &) override {return 1;}
     void write_command(const std_msgs::msg::Float64 & msg, const DataStore &, float * out)
     override
     {
       out[0] = static_cast<float>(msg.data);
     }
     void write_neutral(const DataStore &, float * out) override {out[0] = 0.0F;}
   };

   PLUGINLIB_EXPORT_CLASS(HeadingCommand, policy_framework::ObservationTerm)

On every cycle exactly one of ``write_command()`` and ``write_neutral()`` is called, so
``write_neutral()`` runs on every cycle while the command is neutral: a term with state resets
it there. Both run in the control
cycle, like ``usable()``, which sees the same message again on every cycle until a new one
arrives: cache what is costly to check, and throttle any log. ``ready()`` is true by default;
a term may override it to wait for something else than the command.

``TwistCommandTerm`` is built this way.

Example: a term from the state and the kinematics
-------------------------------------------------

A term that reads sensors available as state interfaces should read them from the state
store, so the controller gets them in real time:

.. code-block:: cpp

   class HandPositionTerm : public ObservationTerm
   {
   public:
     size_t configure(const PluginContext & ctx, const ModelInfo &, DataStore &) override
     {
       if (!ctx.kinematics) {throw std::runtime_error("needs kinematics.enabled");}
       kinematics_ = ctx.kinematics;
       hand_ = kinematics_->frame("hand_link");      // name lookup: only while configuring
       return 3;
     }
     bool ready(const DataStore &) override {return kinematics_->ready();}
     double compute(const DataStore &, const TensorMap &, float * out) override
     {
       const Eigen::Vector3d p = kinematics_->transform(hand_).translation();
       for (int i = 0; i < 3; ++i) {out[i] = static_cast<float>(p[i]);}
       return kinematics_->stamp();
     }

   private:
     std::shared_ptr<Kinematics> kinematics_;
     size_t hand_ = 0;
   };

Example: a safety check
-----------------------

A ``SafetyCheck`` registers what it reads, and returns a ``Verdict`` on every cycle. This one
fails on any leg actuator error, and asks for ``ERROR`` when both legs have one; with a single
leg it leaves the reaction to the configured ``on_fault``:

.. code-block:: cpp

   #include <algorithm>
   #include <cmath>

   #include "policy_framework/plugin_interfaces.hpp"

   using namespace policy_framework;

   class LegActuatorsCheck : public SafetyCheck
   {
   public:
     void configure(const PluginContext & ctx, const DataStore &, DataStore & state) override
     {
       using Names = std::vector<std::string>;
       for (const auto & j : get_parameter<Names>(ctx, "left", {})) {
         left_.push_back(state.add(j + "/error_code"));   // the controller claims it
       }
       for (const auto & j : get_parameter<Names>(ctx, "right", {})) {
         right_.push_back(state.add(j + "/error_code"));
       }
     }

     // Every reaction a Verdict may ask for, known once configured: checked at startup.
     std::vector<FaultReaction> possible_reactions() const override
     {
       return {FaultReaction::ERROR};
     }

     Verdict check(const DataStore & state, double, std::string & reason) override
     {
       const bool left = any_error(state, left_);
       const bool right = any_error(state, right_);
       if (!left && !right) {return {};}                // safe: nothing allocated
       if (left && right) {
         reason = "actuator errors on both legs";
         return {false, FaultReaction::ERROR};
       }
       reason = left ? "an actuator error on the left leg" : "an actuator error on the right leg";
       return {false, std::nullopt};                    // the configured on_fault
     }

   private:
     static bool any_error(const DataStore & state, const std::vector<size_t> & entries)
     {
       return std::any_of(
         entries.begin(), entries.end(),
         // An integer code stored as a double; written so that NaN is an error too.
         [&](size_t i) {return state.received(i) && !(std::abs(state.get(i)) < 0.5);});
     }

     std::vector<size_t> left_, right_;
   };

   PLUGINLIB_EXPORT_CLASS(LegActuatorsCheck, policy_framework::SafetyCheck)

The ``Verdict`` carries the whole answer of a cycle, so a check keeps no state between
``check()`` and its host besides its own filters and counters (forgotten in ``reset()``). Its
configuration:

.. code-block:: yaml

   safety_checks: [legs]
   legs:
     type: my_package/LegActuatorsCheck
     left: [leg_left_1_joint, leg_left_2_joint]
     right: [leg_right_1_joint, leg_right_2_joint]
     on_fault: neutral_commands   # one leg: stop walking; both legs: ERROR from the Verdict

Hooks beyond the basic cycle
----------------------------

Most plugins only need ``configure()`` and one cycle method. A few optional hooks exist for
the situations described in the other pages:

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - Hook
     - When it is called, and what for
   * - ``InputSource::receive_time()``
     - Required: when the latest data arrived (0 before any), checked against the source's
       ``max_age`` on every cycle (:doc:`safety`). ``TopicSource`` implements it.
   * - ``ObservationTerm::receive_time()``
     - When the latest message of a term with its own subscription arrived, checked against
       its ``max_age`` after each policy step. ``TopicTerm`` implements it (with the
       ``max_age`` and ``max_age_on_fault`` parameters); a term reading the state store has
       none.
   * - ``Preprocessor::clear()``
     - Before the first activation and when the host restarts: forget everything.
   * - ``Preprocessor::reset()``, ``Postprocessor::reset()``, ``ObservationTerm::reset()``
     - Whenever the policy becomes active: restart per-activation state.
   * - ``Preprocessor::observe()``
     - Every step while another policy is active: keep a history current
       (:doc:`policies`).
   * - ``Postprocessor::initial_commands()``
     - When the host starts: write what is known before the first step, such as gains.
   * - ``InterfaceExecutor::hold()``
     - Every update until the policy's first commands: keep the robot where it is
       (:doc:`safety`).
   * - ``InterfaceExecutor::safe_values()``
     - On deactivation and failure: the values that stop commanding.
   * - ``InterfaceExecutor::fault()``
     - The values of a fault reaction (``HOLD``, ``DAMP`` or ``SAFE``, :doc:`safety`), on every
       update while faulted. Default: the safe values. ``supports_fault_reaction()`` tells
       which reactions the configuration allows; ``PolicyController`` refuses to configure
       with one an executor does not support.
   * - ``SafetyCheck::possible_reactions()``
     - Once configured: the reactions a ``Verdict`` may ask for besides ``on_fault``. The
       host refuses to configure when an executor cannot apply one. Default: none.
   * - ``SafetyCheck::reset()``
     - When the host is reactivated, and when a check disabled by the previous policy
       applies again (:ref:`checks-per-policy`): forget filters and counters.
   * - ``CommandGate::open()``
     - False while the safety entry is 0 or the ``NEUTRAL_COMMANDS`` fault reaction is active
       (:doc:`safety`). Any plugin that feeds user commands checks it and writes a neutral
       command while it is false, and also after a timeout above zero, measured on the
       receive time. A ``CommandTerm`` does both for a term.

Testing a plugin
----------------

Plugins are plain classes: they can be constructed and called directly in a unit test, with
a ``DataStore`` filled by hand, without a robot, a simulator or a controller manager. The
tests of both packages are written that way and are the best examples to start from.
