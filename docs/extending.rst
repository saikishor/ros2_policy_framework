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
     pre [label="Preprocessor\n+ Postprocessor"];
     inf [label="Inference"];
     exec [label="Executor\n(node)"];
     iexec [label="InterfaceExecutor\n(controller)"];
     q1 -> source [label="A sensor topic as\nnamed state entries"];
     q1 -> term [label="A new observation\nterm for mjlab policies"];
     q1 -> pre [label="Policies from another\ntraining framework"];
     q1 -> inf [label="Another model format\nor runtime"];
     q1 -> exec [label="Commands on\nanother topic type"];
     q1 -> iexec [label="Another control law\nor command interface"];
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
     - ``TwistCommandTerm``, ``MotionGoalTerm``
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
     - state entries (and the kinematics) into a verdict: safe or not, and why
     - ``StateBoundsCheck``, ``JumpCheck``

The hosts do not run ``SafetyCheck`` plugins yet (:doc:`safety`).

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
     cycle [label="Control cycle\ncompute() / read() / write()\nindices only, no allocation", fillcolor="#e8f5e9"];
     configure -> cycle [label="host activated"];
     cycle -> cycle [label="every step\nor update"];
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
safely. A term only says how big it is and how to write its value:

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
       return rclcpp::Time(in.msg->header.stamp).seconds();   // when the value was produced
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

Hooks beyond the basic cycle
----------------------------

Most plugins only need ``configure()`` and one cycle method. A few optional hooks exist for
the situations described in the other pages:

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - Hook
     - When it is called, and what for
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
   * - ``CommandGate::open()``
     - Check it in any plugin that feeds user commands, and use a neutral command while it
       is false (:doc:`safety`).

Testing a plugin
----------------

Plugins are plain classes: they can be constructed and called directly in a unit test, with
a ``DataStore`` filled by hand, without a robot, a simulator or a controller manager. The
tests of both packages are written that way and are the best examples to start from.
