using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.Net.Http.Headers;
using System.Text;

namespace Box2DOpenTK
{
    /// This is prototype for a Box2D task. Your task system is expected to invoke the Box2D task with these arguments.
    /// The task spans a range of the parallel-for: [startIndex, endIndex)
    /// The worker index must correctly identify each worker in the user thread pool, expected in [0, workerCount).
    ///	A worker must only exist on only one thread at a time and is analogous to the thread index.
    /// The task context is the context pointer sent from Box2D when it is enqueued.
    ///	The startIndex and endIndex are expected in the range [0, itemCount) where itemCount is the argument to EnqueueTaskCallback
    /// below. Box2D expects startIndex < endIndex and will execute a loop like this:
    ///
    ///	@code{.c}
    /// for (int i = startIndex; i < endIndex; ++i)
    ///	{
    ///		DoWork();
    ///	}
    ///	@endcode
    ///	@ingroup world
    public delegate void TaskCallback(int startIndex, int endIndex, uint workerIndex, IntPtr taskContext);

    /// These functions can be provided to Box2D to invoke a task system. These are designed to work well with enkiTS.
    /// Returns a pointer to the user's task object. May be nullptr. A nullptr indicates to Box2D that the work was executed
    ///	serially within the callback and there is no need to call FinishTaskCallback.
    ///	The itemCount is the number of Box2D work items that are to be partitioned among workers by the user's task system.
    ///	This is essentially a parallel-for. The minRange parameter is a suggestion of the minimum number of items to assign
    ///	per worker to reduce overhead. For example, suppose the task is small and that itemCount is 16. A minRange of 8 suggests
    ///	that your task system should split the work items among just two workers, even if you have more available.
    ///	In general the range [startIndex, endIndex) send to TaskCallback should obey:
    ///	endIndex - startIndex >= minRange
    ///	The exception of course is when itemCount < minRange.
    ///	@ingroup world
    public delegate IntPtr EnqueueTaskCallback(TaskCallback task, int itemCount, int minRange, IntPtr taskContext,
                                     IntPtr userContext);

    /// Finishes a user task object that wraps a Box2D task.
    ///	@ingroup world
    public delegate void FinishTaskCallback(IntPtr userTask, IntPtr userContext);

    /// 2D rotation
    /// This is similar to using a complex number for rotation
    public struct Rot
    {
        /// cosine and sine
        float c, s;
    }

    /// A 2D rigid transform
    public struct Transform
    {
        Vector2 p;
        Rot q;
    }

    public struct RayResult
    {
        public ShapeId shapeId;
        public Vector2 point;
        public Vector2 normal;
        public float fraction;
        public bool hit;
    };

    /// World definition used to create a simulation world.
    /// Must be initialized using DefaultWorldDef().
    /// @ingroup world
    public struct WorldDef
    {
        /// Gravity vector. Box2D has no up-vector defined.
        public Vector2 gravity;

        /// Restitution velocity threshold, usually in m/s. Collisions above this
        /// speed have restitution applied (will bounce).
        public float restitutionThreshold;

        /// This parameter controls how fast overlap is resolved and has units of meters per second
        public float contactPushoutVelocity;

        /// Threshold velocity for hit events. Usually meters per second.
        public float hitEventThreshold;

        /// Contact stiffness. Cycles per second.
        public float contactHertz;

        /// Contact bounciness. Non-dimensional.
        public float contactDampingRatio;

        /// Joint stiffness. Cycles per second.
        public float jointHertz;

        /// Joint bounciness. Non-dimensional.
        public float jointDampingRatio;

        /// Maximum linear velocity. Usually meters per second.
        public float maximumLinearVelocity;

        /// Can bodies go to sleep to improve performance
        public bool enableSleep;

        /// Enable continuous collision
        public bool enableContinous;

        /// Number of workers to use with the provided task system. Box2D performs best when using only
        ///	performance cores and accessing a single L2 cache. Efficiency cores and hyper-threading provide
        ///	little benefit and may even harm performance.
        public int workerCount;

        /// Function to spawn tasks
        public EnqueueTaskCallback enqueueTask;

        /// Function to finish a task
        public FinishTaskCallback finishTask;

        /// User context that is provided to enqueueTask and finishTask
        public IntPtr userTaskContext;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int internalValue;
    }

    /// The body simulation type.
    /// Each body is one of these three types. The type determines how the body behaves in the simulation.
    /// @ingroup body
    public enum BodyType
    {
        /// zero mass, zero velocity, may be manually moved
        staticBody = 0,

        /// zero mass, velocity set by user, moved by solver
        kinematicBody = 1,

        /// positive mass, velocity determined by forces, moved by solver
        dynamicBody = 2,

        /// number of body types
        bodyTypeCount,
    }

    /// A body definition holds all the data needed to construct a rigid body.
    /// You can safely re-use body definitions. Shapes are added to a body after construction.
    ///	Body definitions are temporary objects used to bundle creation parameters.
    /// Must be initialized using DefaultBodyDef().
    /// @ingroup body
    public struct BodyDef
    {
        /// The body type: static, kinematic, or dynamic.
        public BodyType type;

        /// The initial world position of the body. Bodies should be created with the desired position.
        /// @note Creating bodies at the origin and then moving them nearly doubles the cost of body creation, especially
        ///	if the body is moved after shapes have been added.
        public Vector2 position;

        /// The initial world rotation of the body. Use MakeRot() if you have an angle.
        public Rot rotation;

        /// The initial linear velocity of the body's origin. Typically in meters per second.
        public Vector2 linearVelocity;

        /// The initial angular velocity of the body. Radians per second.
        public float angularVelocity;

        /// Linear damping is use to reduce the linear velocity. The damping parameter
        /// can be larger than 1 but the damping effect becomes sensitive to the
        /// time step when the damping parameter is large.
        ///	Generally linear damping is undesirable because it makes objects move slowly
        ///	as if they are floating.
        public float linearDamping;

        /// Angular damping is use to reduce the angular velocity. The damping parameter
        /// can be larger than 1.0f but the damping effect becomes sensitive to the
        /// time step when the damping parameter is large.
        ///	Angular damping can be use slow down rotating bodies.
        public float angularDamping;

        /// Scale the gravity applied to this body. Non-dimensional.
        public float gravityScale;

        /// Sleep velocity threshold, default is 0.05 meter per second
        public float sleepThreshold;

        /// Use this to store application specific body data.
        public IntPtr userData;

        /// Set this flag to false if this body should never fall asleep.
        public bool enableSleep;

        /// Is this body initially awake or sleeping?
        public bool isAwake;

        /// Should this body be prevented from rotating? Useful for characters.
        public bool fixedRotation;

        /// Treat this body as high speed object that performs continuous collision detection
        /// against dynamic and kinematic bodies, but not other bullet bodies.
        ///	@warning Bullets should be used sparingly. They are not a solution for general dynamic-versus-dynamic
        ///	continuous collision. They may interfere with joint constraints.
        public bool isBullet;

        /// Used to disable a body. A disabled body does not move or collide.
        public bool isEnabled;

        /// Automatically compute mass and related properties on this body from shapes.
        /// Triggers whenever a shape is add/removed/changed. Default is true.
        public bool automaticMass;

        /// This allows this body to bypass rotational speed limits. Should only be used
        ///	for circular objects, like wheels.
        public bool allowFastRotation;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int internalValue;
    }

    /// This is used to filter collision on shapes. It affects shape-vs-shape collision
    ///	and shape-versus-query collision (such as World_CastRay).
    /// @ingroup shape
    public struct Filter
    {
        /// The collision category bits. Normally you would just set one bit. The category bits should
        ///	represent your application object types. For example:
        ///	@code{.cpp}
        ///	enum MyCategories
        ///	{
        ///	   Static  = 0x00000001,
        ///	   Dynamic = 0x00000002,
        ///	   Debris  = 0x00000004,
        ///	   Player  = 0x00000008,
        ///	   // etc
        /// };
        ///	@endcode
        public uint categoryBits;

        /// The collision mask bits. This states the categories that this
        /// shape would accept for collision.
        ///	For example, you may want your player to only collide with static objects
        ///	and other players.
        ///	@code{.c}
        ///	maskBits = Static | Player;
        ///	@endcode
        public uint maskBits;

        /// Collision groups allow a certain group of objects to never collide (negative)
        /// or always collide (positive). A group index of zero has no effect. Non-zero group filtering
        /// always wins against the mask bits.
        ///	For example, you may want ragdolls to collide with other ragdolls but you don't want
        ///	ragdoll self-collision. In this case you would give each ragdoll a unique negative group index
        ///	and apply that group index to all shapes on the ragdoll.
        public int groupIndex;
    }

    /// The query filter is used to filter collisions between queries and shapes. For example,
    ///	you may want a ray-cast representing a projectile to hit players and the static environment
    ///	but not debris.
    /// @ingroup shape
    public struct QueryFilter
    {
        /// The collision category bits of this query. Normally you would just set one bit.
        public uint categoryBits;

        /// The collision mask bits. This states the shape categories that this
        /// query would accept for collision.
        public uint maskBits;
    }

    /// Shape type
    /// @ingroup shape
    public enum ShapeType
    {
        /// A circle with an offset
        circleShape,

        /// A capsule is an extruded circle
        capsuleShape,

        /// A line segment
        segmentShape,

        /// A convex polygon
        polygonShape,

        /// A smooth segment owned by a chain shape
        smoothSegmentShape,

        /// The number of shape types
        shapeTypeCount
    }

    /// Used to create a shape.
    /// This is a temporary object used to bundle shape creation parameters. You may use
    ///	the same shape definition to create multiple shapes.
    /// Must be initialized using DefaultShapeDef().
    /// @ingroup shape
    public struct ShapeDef
    {
        /// Use this to store application specific shape data.
        public IntPtr userData;

        /// The Coulomb (dry) friction coefficient, usually in the range [0,1].
        public float friction;

        /// The restitution (bounce) usually in the range [0,1].
        public float restitution;

        /// The density, usually in kg/m^2.
        public float density;

        /// Collision filtering data.
        public Filter filter;

        /// Custom debug draw color.
        public uint customColor;

        /// A sensor shape generates overlap events but never generates a collision response.
        public bool isSensor;

        /// Enable sensor events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
        public bool enableSensorEvents;

        /// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
        public bool enableContactEvents;

        /// Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
        public bool enableHitEvents;

        /// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
        ///	and must be carefully handled due to threading. Ignored for sensors.
        public bool enablePreSolveEvents;

        /// Normally shapes on static bodies don't invoke contact creation when they are added to the world. This overrides
        ///	that behavior and causes contact creation. This significantly slows down static body creation which can be important
        ///	when there are many static shapes.
        /// This is implicitly always true for sensors.
        public bool forceContactCreation;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int internalValue;
    }

    /// Used to create a chain of edges. This is designed to eliminate ghost collisions with some limitations.
    ///	- chains are one-sided
    ///	- chains have no mass and should be used on static bodies
    ///	- chains have a counter-clockwise winding order
    ///	- chains are either a loop or open
    /// - a chain must have at least 4 points
    ///	- the distance between any two points must be greater than _linearSlop
    ///	- a chain shape should not self intersect (this is not validated)
    ///	- an open chain shape has NO COLLISION on the first and final edge
    ///	- you may overlap two open chains on their first three and/or last three points to get smooth collision
    ///	- a chain shape creates multiple smooth edges shapes on the body
    /// https://en.wikipedia.org/wiki/Polygonal_chain
    /// Must be initialized using DefaultChainDef().
    ///	@warning Do not use chain shapes unless you understand the limitations. This is an advanced feature.
    /// @ingroup shape
    public struct ChainDef
    {
        /// Use this to store application specific shape data.
        public IntPtr userData;

        /// An array of at least 4 points. These are cloned and may be temporary.
        public Vector2[] points;

        /// The point count, must be 4 or more.
        public int count;

        /// The friction coefficient, usually in the range [0,1].
        public float friction;

        /// The restitution (elasticity) usually in the range [0,1].
        public float restitution;

        /// Contact filtering data.
        public Filter filter;

        /// Indicates a closed chain formed by connecting the first and last points
        public bool isLoop;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int internalValue;
    }

    //! @cond
    /// Profiling data. Times are in milliseconds.
    public struct Profile
    {
        public float step;
        public float pairs;
        public float collide;
        public float solve;
        public float buildIslands;
        public float solveConstraints;
        public float prepareTasks;
        public float solverTasks;
        public float prepareConstraints;
        public float integrateVelocities;
        public float warmStart;
        public float solveVelocities;
        public float integratePositions;
        public float relaxVelocities;
        public float applyRestitution;
        public float storeImpulses;
        public float finalizeBodies;
        public float splitIslands;
        public float sleepIslands;
        public float hitEvents;
        public float broadphase;
        public float continuous;
    }

    /// Counters that give details of the simulation size.
    public struct Counters
    {
        public static Counters GetCounter()
        {
            Counters counter = new Counters();
            counter.colorCounts = new int[12];
            return counter;
        }

        public int staticBodyCount;
        public int bodyCount;
        public int shapeCount;
        public int contactCount;
        public int jointCount;
        public int islandCount;
        public int stackUsed;
        public int staticTreeHeight;
        public int treeHeight;
        public int byteCount;
        public int taskCount;
        public int[] colorCounts;
    }
    //! @endcond

    /// Joint type enumeration
    ///
    /// This is useful because all joint types use JointId and sometimes you
    /// want to get the type of a joint.
    /// @ingroup joint
    public enum JointType
    {
        distanceJoint,
        motorJoint,
        mouseJoint,
        prismaticJoint,
        revoluteJoint,
        weldJoint,
        wheelJoint,
    }

    /// Distance joint definition
    ///
    /// This requires defining an anchor point on both
    /// bodies and the non-zero distance of the distance joint. The definition uses
    /// local anchor points so that the initial configuration can violate the
    /// constraint slightly. This helps when saving and loading a game.
    /// @ingroup distance_joint
    public struct DistanceJointDef
    {
        /// The first attached body
        public BodyId bodyIdA;

        /// The second attached body
        public BodyId bodyIdB;

        /// The local anchor point relative to bodyA's origin
        public Vector2 localAnchorA;

        /// The local anchor point relative to bodyB's origin
        public Vector2 localAnchorB;

        /// The rest length of this joint. Clamped to a stable minimum value.
        public float length;

        /// Enable the distance constraint to behave like a spring. If false
        ///	then the distance joint will be rigid, overriding the limit and motor.
        public bool enableSpring;

        /// The spring linear stiffness Hertz, cycles per second
        public float hertz;

        /// The spring linear damping ratio, non-dimensional
        public float dampingRatio;

        /// Enable/disable the joint limit
        public bool enableLimit;

        /// Minimum length. Clamped to a stable minimum value.
        public float minLength;

        /// Maximum length. Must be greater than or equal to the minimum length.
        public float maxLength;

        /// Enable/disable the joint motor
        public bool enableMotor;

        /// The maximum motor force, usually in newtons
        public float maxMotorForce;

        /// The desired motor speed, usually in meters per second
        public float motorSpeed;

        /// Set this flag to true if the attached bodies should collide
        public bool collideConnected;

        /// User data pointer
        public IntPtr userData;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int internalValue;
    }

    /// A motor joint is used to control the relative motion between two bodies
    ///
    /// A typical usage is to control the movement of a dynamic body with respect to the ground.
    /// @ingroup motor_joint
    public struct MotorJointDef
    {
        /// The first attached body
        public BodyId bodyIdA;

        /// The second attached body
        public BodyId bodyIdB;

        /// Position of bodyB minus the position of bodyA, in bodyA's frame
        public Vector2 linearOffset;

        /// The bodyB angle minus bodyA angle in radians
        public float angularOffset;

        /// The maximum motor force in newtons
        public float maxForce;

        /// The maximum motor torque in newton-meters
        public float maxTorque;

        /// Position correction factor in the range [0,1]
        public float correctionFactor;

        /// Set this flag to true if the attached bodies should collide
        public bool collideConnected;

        /// User data pointer
        public IntPtr userData;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int internalValue;
    }

    /// A mouse joint is used to make a point on a body track a specified world point.
    ///
    /// This a soft constraint and allows the constraint to stretch without
    /// applying huge forces. This also applies rotation constraint heuristic to improve control.
    /// @ingroup mouse_joint
    public struct MouseJointDef
    {
        /// The first attached body.
        public BodyId bodyIdA;

        /// The second attached body.
        public BodyId bodyIdB;

        /// The initial target point in world space
        public Vector2 target;

        /// Stiffness in hertz
        public float hertz;

        /// Damping ratio, non-dimensional
        public float dampingRatio;

        /// Maximum force, typically in newtons
        public float maxForce;

        /// Set this flag to true if the attached bodies should collide.
        public bool collideConnected;

        /// User data pointer
        public IntPtr userData;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int internalValue;
    }

    /// Prismatic joint definition
    ///
    /// This requires defining a line of motion using an axis and an anchor point.
    /// The definition uses local anchor points and a local axis so that the initial
    /// configuration can violate the constraint slightly. The joint translation is zero
    /// when the local anchor points coincide in world space.
    /// @ingroup prismatic_joint
    public struct PrismaticJointDef
    {
        /// The first attached body
        public BodyId bodyIdA;

        /// The second attached body
        public BodyId bodyIdB;

        /// The local anchor point relative to bodyA's origin
        public Vector2 localAnchorA;

        /// The local anchor point relative to bodyB's origin
        public Vector2 localAnchorB;

        /// The local translation unit axis in bodyA
        public Vector2 localAxisA;

        /// The constrained angle between the bodies: bodyB_angle - bodyA_angle
        public float referenceAngle;

        /// Enable a linear spring along the prismatic joint axis
        public bool enableSpring;

        /// The spring stiffness Hertz, cycles per second
        public float hertz;

        /// The spring damping ratio, non-dimensional
        public float dampingRatio;

        /// Enable/disable the joint limit
        public bool enableLimit;

        /// The lower translation limit
        public float lowerTranslation;

        /// The upper translation limit
        public float upperTranslation;

        /// Enable/disable the joint motor
        public bool enableMotor;

        /// The maximum motor force, typically in newtons
        public float maxMotorForce;

        /// The desired motor speed, typically in meters per second
        public float motorSpeed;

        /// Set this flag to true if the attached bodies should collide
        public bool collideConnected;

        /// User data pointer
        public IntPtr userData;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int internalValue;
    }

    /// Revolute joint definition
    ///
    /// This requires defining an anchor point where the bodies are joined.
    /// The definition uses local anchor points so that the
    /// initial configuration can violate the constraint slightly. You also need to
    /// specify the initial relative angle for joint limits. This helps when saving
    /// and loading a game.
    /// The local anchor points are measured from the body's origin
    /// rather than the center of mass because:
    /// 1. you might not know where the center of mass will be
    /// 2. if you add/remove shapes from a body and recompute the mass, the joints will be broken
    /// @ingroup revolute_joint
    public struct RevoluteJointDef
    {
        /// The first attached body
        public BodyId bodyIdA;

        /// The second attached body
        public BodyId bodyIdB;

        /// The local anchor point relative to bodyA's origin
        public Vector2 localAnchorA;

        /// The local anchor point relative to bodyB's origin
        public Vector2 localAnchorB;

        /// The bodyB angle minus bodyA angle in the reference state (radians).
        /// This defines the zero angle for the joint limit.
        public float referenceAngle;

        /// Enable a rotational spring on the revolute hinge axis
        public bool enableSpring;

        /// The spring stiffness Hertz, cycles per second
        public float hertz;

        /// The spring damping ratio, non-dimensional
        public float dampingRatio;

        /// A flag to enable joint limits
        public bool enableLimit;

        /// The lower angle for the joint limit in radians
        public float lowerAngle;

        /// The upper angle for the joint limit in radians
        public float upperAngle;

        /// A flag to enable the joint motor
        public bool enableMotor;

        /// The maximum motor torque, typically in newton-meters
        public float maxMotorTorque;

        /// The desired motor speed in radians per second
        public float motorSpeed;

        /// Scale the debug draw
        public float drawSize;

        /// Set this flag to true if the attached bodies should collide
        public bool collideConnected;

        /// User data pointer
        public IntPtr userData;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int internalValue;
    }

    /// Weld joint definition
    ///
    /// A weld joint connect to bodies together rigidly. This constraint provides springs to mimic
    ///	soft-body simulation.
    /// @note The approximate solver in Box2D cannot hold many bodies together rigidly
    /// @ingroup weld_joint
    public struct WeldJointDef
    {
        /// The first attached body
        public BodyId bodyIdA;

        /// The second attached body
        public BodyId bodyIdB;

        /// The local anchor point relative to bodyA's origin
        public Vector2 localAnchorA;

        /// The local anchor point relative to bodyB's origin
        public Vector2 localAnchorB;

        /// The bodyB angle minus bodyA angle in the reference state (radians)
        public float referenceAngle;

        /// Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
        public float linearHertz;

        /// Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
        public float angularHertz;

        /// Linear damping ratio, non-dimensional. Use 1 for critical damping.
        public float linearDampingRatio;

        /// Linear damping ratio, non-dimensional. Use 1 for critical damping.
        public float angularDampingRatio;

        /// Set this flag to true if the attached bodies should collide
        public bool collideConnected;

        /// User data pointer
        public IntPtr userData;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int internalValue;
    }

    /// Wheel joint definition
    ///
    /// This requires defining a line of motion using an axis and an anchor point.
    /// The definition uses local  anchor points and a local axis so that the initial
    /// configuration can violate the constraint slightly. The joint translation is zero
    /// when the local anchor points coincide in world space.
    /// @ingroup wheel_joint
    public struct WheelJointDef
    {
        /// The first attached body
        public BodyId bodyIdA;

        /// The second attached body
        public BodyId bodyIdB;

        /// The local anchor point relative to bodyA's origin
        public Vector2 localAnchorA;

        /// The local anchor point relative to bodyB's origin
        public Vector2 localAnchorB;

        /// The local translation unit axis in bodyA
        public Vector2 localAxisA;

        /// Enable a linear spring along the local axis
        public bool enableSpring;

        /// Spring stiffness in Hertz
        public float hertz;

        /// Spring damping ratio, non-dimensional
        public float dampingRatio;

        /// Enable/disable the joint linear limit
        public bool enableLimit;

        /// The lower translation limit
        public float lowerTranslation;

        /// The upper translation limit
        public float upperTranslation;

        /// Enable/disable the joint rotational motor
        public bool enableMotor;

        /// The maximum motor torque, typically in newton-meters
        public float maxMotorTorque;

        /// The desired motor speed in radians per second
        public float motorSpeed;

        /// Set this flag to true if the attached bodies should collide
        public bool collideConnected;

        /// User data pointer
        public IntPtr userData;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int internalValue;
    }

    /**
     * @defgroup events Events
     * World event types.
     *
     * Events are used to collect events that occur during the world time step. These events
     * are then available to query after the time step is complete. This is preferable to callbacks
     * because Box2D uses multithreaded simulation.
     *
     * Also when events occur in the simulation step it may be problematic to modify the world, which is
     * often what applications want to do when events occur.
     *
     * With event arrays, you can scan the events in a loop and modify the world. However, you need to be careful
     * that some event data may become invalid. There are several samples that show how to do this safely.
     *
     * @{
     */

    /// A begin touch event is generated when a shape starts to overlap a sensor shape.
    public struct SensorBeginTouchEvent
    {
        /// The id of the sensor shape
        public ShapeId sensorShapeId;

        /// The id of the dynamic shape that began touching the sensor shape
        public ShapeId visitorShapeId;
    }

    /// An end touch event is generated when a shape stops overlapping a sensor shape.
    public struct SensorEndTouchEvent
    {
        /// The id of the sensor shape
        public ShapeId sensorShapeId;

        /// The id of the dynamic shape that stopped touching the sensor shape
        public ShapeId visitorShapeId;
    }

    /// Sensor events are buffered in the Box2D world and are available
    ///	as begin/end overlap event arrays after the time step is complete.
    ///	Note: these may become invalid if bodies and/or shapes are destroyed
    public struct SensorEvents
    {
        /// Array of sensor begin touch events
        public SensorBeginTouchEvent[] beginEvents;

        /// Array of sensor end touch events
        public SensorEndTouchEvent[] endEvents;

        /// The number of begin touch events
        public int beginCount;

        /// The number of end touch events
        public int endCount;
    }

    /// A begin touch event is generated when two shapes begin touching.
    public struct ContactBeginTouchEvent
    {
        /// Id of the first shape
        public ShapeId shapeIdA;

        /// Id of the second shape
        public ShapeId shapeIdB;
    }

    /// An end touch event is generated when two shapes stop touching.
    public struct ContactEndTouchEvent
    {
        /// Id of the first shape
        public ShapeId shapeIdA;

        /// Id of the second shape
        public ShapeId shapeIdB;
    }

    /// A hit touch event is generated when two shapes collide with a speed faster than the hit speed threshold.
    public struct ContactHitEvent
    {
        /// Id of the first shape
        public ShapeId shapeIdA;

        /// Id of the second shape
        public ShapeId shapeIdB;

        /// Point where the shapes hit
        public Vector2 point;

        /// Normal vector pointing from shape A to shape B
        public Vector2 normal;

        /// The speed the shapes are approaching. Always positive. Typically in meters per second.
        public float approachSpeed;
    }

    /// Contact events are buffered in the Box2D world and are available
    ///	as event arrays after the time step is complete.
    ///	Note: these may become invalid if bodies and/or shapes are destroyed
    public struct ContactEvents
    {
        /// Array of begin touch events
        public ContactBeginTouchEvent[] beginEvents;

        /// Array of end touch events
        public ContactEndTouchEvent[] endEvents;

        /// Array of hit events
        public ContactHitEvent[] hitEvents;

        /// Number of begin touch events
        public int beginCount;

        /// Number of end touch events
        public int endCount;

        /// Number of hit events
        public int hitCount;
    }

    /// Body move events triggered when a body moves.
    /// Triggered when a body moves due to simulation. Not reported for bodies moved by the user.
    /// This also has a flag to indicate that the body went to sleep so the application can also
    /// sleep that actor/entity/object associated with the body.
    /// On the other hand if the flag does not indicate the body went to sleep then the application
    /// can treat the actor/entity/object associated with the body as awake.
    ///	This is an efficient way for an application to update game object transforms rather than
    ///	calling functions such as Body_GetTransform() because this data is delivered as a contiguous array
    ///	and it is only populated with bodies that have moved.
    ///	@note If sleeping is disabled all dynamic and kinematic bodies will trigger move events.
    public struct BodyMoveEvent
    {
        public Transform transform;
        public BodyId bodyId;
        public IntPtr userData;
        public bool fellAsleep;
    }

    /// Body events are buffered in the Box2D world and are available
    ///	as event arrays after the time step is complete.
    ///	Note: this data becomes invalid if bodies are destroyed
    public struct BodyEvents
    {
        /// Array of move events
        public BodyMoveEvent[] moveEvents;

        /// Number of move events
        public int moveCount;
    }

    /// The contact data for two shapes. By convention the manifold normal points
    ///	from shape A to shape B.
    ///	@see Shape_GetContactData() and Body_GetContactData()
    public struct ContactData
    {
        public ShapeId shapeIdA;
        public ShapeId shapeIdB;
        public Manifold manifold;
    }

    /**@}*/

    /// Prototype for a contact filter callback.
    /// This is called when a contact pair is considered for collision. This allows you to
    ///	perform custom logic to prevent collision between shapes. This is only called if
    ///	one of the two shapes has custom filtering enabled. @see ShapeDef.
    /// Notes:
    ///	- this function must be thread-safe
    ///	- this is only called if one of the two shapes has enabled custom filtering
    /// - this is called only for awake dynamic bodies
    ///	Return false if you want to disable the collision
    ///	@warning Do not attempt to modify the world inside this callback
    ///	@ingroup world
    public delegate bool CustomFilterFcn(ShapeId shapeIdA, ShapeId shapeIdB, IntPtr context);

    /// Prototype for a pre-solve callback.
    /// This is called after a contact is updated. This allows you to inspect a
    /// contact before it goes to the solver. If you are careful, you can modify the
    /// contact manifold (e.g. modify the normal).
    /// Notes:
    ///	- this function must be thread-safe
    ///	- this is only called if the shape has enabled pre-solve events
    /// - this is called only for awake dynamic bodies
    /// - this is not called for sensors
    /// - the supplied manifold has impulse values from the previous step
    ///	Return false if you want to disable the contact this step
    ///	@warning Do not attempt to modify the world inside this callback
    ///	@ingroup world
    public delegate bool PreSolveFcn(ShapeId shapeIdA, ShapeId shapeIdB, Manifold* manifold, IntPtr context);

    /// Prototype callback for overlap queries.
    /// Called for each shape found in the query.
    /// @see World_QueryAABB
    /// @return false to terminate the query.
    ///	@ingroup world
    public delegate bool OverlapResultFcn(ShapeId shapeId, IntPtr context);

    /// Prototype callback for ray casts.
    /// Called for each shape found in the query. You control how the ray cast
    /// proceeds by returning a float:
    /// return -1: ignore this shape and continue
    /// return 0: terminate the ray cast
    /// return fraction: clip the ray to this point
    /// return 1: don't clip the ray and continue
    /// @param shapeId the shape hit by the ray
    /// @param point the point of initial intersection
    /// @param normal the normal vector at the point of intersection
    /// @param fraction the fraction along the ray at the point of intersection
    ///	@param context the user context
    /// @return -1 to filter, 0 to terminate, fraction to clip the ray for closest hit, 1 to continue
    /// @see World_CastRay
    ///	@ingroup world
    public delegate float CastResultFcn(ShapeId shapeId, Vector2 point, Vector2 normal, float fraction, IntPtr context);

    /// These colors are used for debug draw.
    ///	See https://www.rapidtables.com/web/color/index.html
    public enum HexColor
    {
        colorAliceBlue = 0xf0f8ff,
        colorAntiqueWhite = 0xfaebd7,
        colorAqua = 0x00ffff,
        colorAquamarine = 0x7fffd4,
        colorAzure = 0xf0ffff,
        colorBeige = 0xf5f5dc,
        colorBisque = 0xffe4c4,
        colorBlack = 0x000000,
        colorBlanchedAlmond = 0xffebcd,
        colorBlue = 0x0000ff,
        colorBlueViolet = 0x8a2be2,
        colorBrown = 0xa52a2a,
        colorBurlywood = 0xdeb887,
        colorCadetBlue = 0x5f9ea0,
        colorChartreuse = 0x7fff00,
        colorChocolate = 0xd2691e,
        colorCoral = 0xff7f50,
        colorCornflowerBlue = 0x6495ed,
        colorCornsilk = 0xfff8dc,
        colorCrimson = 0xdc143c,
        colorCyan = 0x00ffff,
        colorDarkBlue = 0x00008b,
        colorDarkCyan = 0x008b8b,
        colorDarkGoldenrod = 0xb8860b,
        colorDarkGray = 0xa9a9a9,
        colorDarkGreen = 0x006400,
        colorDarkKhaki = 0xbdb76b,
        colorDarkMagenta = 0x8b008b,
        colorDarkOliveGreen = 0x556f,
        colorDarkOrange = 0xff8c00,
        colorDarkOrchid = 0x9932cc,
        colorDarkRed = 0x8b0000,
        colorDarkSalmon = 0xe9967a,
        colorDarkSeaGreen = 0x8fbc8f,
        colorDarkSlateBlue = 0x483d8b,
        colorDarkSlateGray = 0x2f4f4f,
        colorDarkTurquoise = 0x00ced1,
        colorDarkViolet = 0x9400d3,
        colorDeepPink = 0xff1493,
        colorDeepSkyBlue = 0x00bfff,
        colorDimGray = 0x696969,
        colorDodgerBlue = 0x1e90ff,
        colorFirebrick = 0x2222,
        colorFloralWhite = 0xfffaf0,
        colorForestGreen = 0x2282,
        colorFuchsia = 0xff00ff,
        colorGainsboro = 0xdcdcdc,
        colorGhostWhite = 0xf8f8ff,
        colorGold = 0xffd700,
        colorGoldenrod = 0xdaa520,
        colorGray = 0xbebebe,
        colorGray1 = 0x1a1a1a,
        colorGray2 = 0x333333,
        colorGray3 = 0x4d4d4d,
        colorGray4 = 0x666666,
        colorGray5 = 0x7f7f7f,
        colorGray6 = 0x999999,
        colorGray7 = 0xb3b3b3,
        colorGray8 = 0xcccccc,
        colorGray9 = 0xe5e5e5,
        colorGreen = 0x00ff00,
        colorGreenYellow = 0xadff2f,
        colorHoneydew = 0xf0fff0,
        colorHotPink = 0xff69b4,
        colorIndianRed = 0xcd5c5c,
        colorIndigo = 0x4b0082,
        colorIvory = 0xfffff0,
        colorKhaki = 0xf0e68c,
        colorLavender = 0xe6e6fa,
        colorLavenderBlush = 0xfff0f5,
        colorLawnGreen = 0x7cfc00,
        colorLemonChiffon = 0xfffacd,
        colorLightBlue = 0xadd8e6,
        colorLightCoral = 0xf08080,
        colorLightCyan = 0xe0ffff,
        colorLightGoldenrod = 0xeedd82,
        colorLightGoldenrodYellow = 0xfafad2,
        colorLightGray = 0xd3d3d3,
        colorLightGreen = 0x90ee90,
        colorLightPink = 0xffb6c1,
        colorLightSalmon = 0xffa07a,
        colorLightSeaGreen = 0x20aa,
        colorLightSkyBlue = 0x87cefa,
        colorLightSlateBlue = 0x8470ff,
        colorLightSlateGray = 0x778899,
        colorLightSteelBlue = 0xb0c4de,
        colorLightYellow = 0xffffe0,
        colorLime = 0x00ff00,
        colorLimeGreen = 0x32cd32,
        colorLinen = 0xfaf0e6,
        colorMagenta = 0xff00ff,
        colorMaroon = 0xb03060,
        colorMediumAquamarine = 0x66cdaa,
        colorMediumBlue = 0x0000cd,
        colorMediumOrchid = 0xba55d3,
        colorMediumPurple = 0x9370db,
        colorMediumSeaGreen = 0x3cb371,
        colorMediumSlateBlue = 0x7b68ee,
        colorMediumSpringGreen = 0x00fa9a,
        colorMediumTurquoise = 0x48d1cc,
        colorMediumVioletRed = 0xc71585,
        colorMidnightBlue = 0x191970,
        colorMintCream = 0xf5fffa,
        colorMistyRose = 0xffe4e1,
        colorMoccasin = 0xffe4b5,
        colorNavajoWhite = 0xffdead,
        colorNavy = 0x000080,
        colorNavyBlue = 0x000080,
        colorOldLace = 0xfdf5e6,
        colorOlive = 0x808000,
        colorOliveDrab = 0x6b8e23,
        colorOrange = 0xffa500,
        colorOrangeRed = 0xff4500,
        colorOrchid = 0xda70d6,
        colorPaleGoldenrod = 0xeee8aa,
        colorPaleGreen = 0x98fb98,
        colorPaleTurquoise = 0xafeeee,
        colorPaleVioletRed = 0xdb7093,
        colorPapayaWhip = 0xffefd5,
        colorPeachPuff = 0xffdab9,
        colorPeru = 0xcd853f,
        colorPink = 0xffc0cb,
        colorPlum = 0xdda0dd,
        colorPowderBlue = 0xb0e0e6,
        colorPurple = 0xa020f0,
        colorRebeccaPurple = 0x663399,
        colorRed = 0xff0000,
        colorRosyBrown = 0xbc8f8f,
        colorRoyalBlue = 0x4169e1,
        colorSaddleBrown = 0x8b4513,
        colorSalmon = 0xfa8072,
        colorSandyBrown = 0xf4a460,
        colorSeaGreen = 0x2e8b57,
        colorSeashell = 0xfff5ee,
        colorSienna = 0xa0522d,
        colorSilver = 0xc0c0c0,
        colorSkyBlue = 0x87ceeb,
        colorSlateBlue = 0x6a5acd,
        colorSlateGray = 0x708090,
        colorSnow = 0xfffafa,
        colorSpringGreen = 0x00ff7f,
        colorSteelBlue = 0x4682b4,
        colorTan = 0xd2b48c,
        colorTeal = 0x008080,
        colorThistle = 0xd8bfd8,
        colorTomato = 0xff6347,
        colorTurquoise = 0x40e0d0,
        colorViolet = 0xee82ee,
        colorVioletRed = 0xd02090,
        colorWheat = 0xf5deb3,
        colorWhite = 0xffffff,
        colorWhiteSmoke = 0xf5f5f5,
        colorYellow = 0xffff00,
        colorYellowGreen = 0x9acd32,
        colorBox2DRed = 0xdc3132,
        colorBox2DBlue = 0x30aebf,
        colorBox2DGreen = 0x8cc924,
        colorBox2DYellow = 0xffee8c
    }

    /// This struct holds callbacks you can implement to draw a Box2D world.
    ///	@ingroup world
    public struct DebugDraw
    {
        /// Draw a closed polygon provided in CCW order.
        public delegate void DrawPoligon(Vector2[] vertices, int vertexCount, HexColor color, IntPtr context);

        /// Draw a solid closed polygon provided in CCW order.
        public delegate void DrawSolidPolygon(Transform transform, Vector2[] vertices, int vertexCount, float radius, HexColor color, IntPtr context);

        /// Draw a circle.
        public delegate void DrawCircle(Vector2 center, float radius, HexColor color, IntPtr context);

        /// Draw a solid circle.
        public delegate void DrawSolidCircle(Transform transform, float radius, HexColor color, IntPtr context);

        /// Draw a capsule.
        public delegate void DrawCapsule(Vector2 p1, Vector2 p2, float radius, HexColor color, IntPtr context);

        /// Draw a solid capsule.
        public delegate void DrawSolidCapsule(Vector2 p1, Vector2 p2, float radius, HexColor color, IntPtr context);

        /// Draw a line segment.
        public delegate void DrawSegment(Vector2 p1, Vector2 p2, HexColor color, IntPtr context);

        /// Draw a transform. Choose your own length scale.
        public delegate void DrawTransform(Transform transform, IntPtr context);

        /// Draw a point.
        public delegate void DrawPoint(Vector2 p, float size, HexColor color, IntPtr context);

        /// Draw a string.
        public delegate void DrawString(Vector2 p, string s, IntPtr context);

        /// Bounds to use if restricting drawing to a rectangular region
        public Box2 drawingBounds;

        /// Option to restrict drawing to a rectangular region. May suffer from unstable depth sorting.
        public bool useDrawingBounds;

        /// Option to draw shapes
        public bool drawShapes;

        /// Option to draw joints
        public bool drawJoints;

        /// Option to draw additional information for joints
        public bool drawJointExtras;

        /// Option to draw the bounding boxes for shapes
        public bool drawAABBs;

        /// Option to draw the mass and center of mass of dynamic bodies
        public bool drawMass;

        /// Option to draw contact points
        public bool drawContacts;

        /// Option to visualize the graph coloring used for contacts and joints
        public bool drawGraphColors;

        /// Option to draw contact normals
        public bool drawContactNormals;

        /// Option to draw contact normal impulses
        public bool drawContactImpulses;

        /// Option to draw contact friction impulses
        public bool drawFrictionImpulses;

        /// User context that is passed as an argument to drawing callback functions
        public IntPtr context;
    }

    public struct BitSet
    {
        public ulong[] bits;
        public uint blockCapacity;
        public uint blockCount;

        public static BitSet CreateBitSet(uint bitCapacity)
        {
            BitSet bitSet = new BitSet();

            bitSet.blockCapacity = (bitCapacity + sizeof(ulong) * 8 - 1) / (sizeof(ulong) * 8);
            bitSet.blockCount = 0;
            bitSet.bits = new ulong[bitSet.blockCapacity * sizeof(ulong)];
            return bitSet;
        }

        public static void SetBitCountAndClear(ref BitSet bitSet, uint bitCount)
        {
            uint blockCount = (bitCount + sizeof(ulong) * 8 - 1) / (sizeof(ulong) * 8);
            if (bitSet.blockCapacity < blockCount)
            {
                bitSet.bits = new ulong[bitSet.blockCapacity * sizeof(ulong)];
            }

            bitSet.blockCount = blockCount;
        }

        public static void GrowBitSet(ref BitSet bitSet, uint blockCount)
        {
            if (blockCount < bitSet.blockCount) throw new Exception("blockCount < bitSet.blockCount");

            if (blockCount > bitSet.blockCapacity)
            {
                if (bitSet.bits == null) throw new Exception("blockCount == null");
                uint oldCapacity = bitSet.blockCapacity;
                bitSet.blockCapacity = blockCount + blockCount / 2;
                ulong[] newBits = new ulong[bitSet.blockCapacity * sizeof(ulong)];
                Array.Copy(bitSet.bits, newBits, oldCapacity);
                bitSet.bits = newBits;
            }

            bitSet.blockCount = blockCount;
        }

        public static void InPlaceUnion(ref BitSet setA, ref BitSet setB)
        {
            if (setA.blockCount != setB.blockCount) throw new Exception("setA.blockCount != setB.blockCount");
            uint blockCount = setA.blockCount;
            for (uint i = 0; i < blockCount; ++i)
            {
                setA.bits[i] |= setB.bits[i];
            }
        }

        public static void SetBit(ref BitSet bitSet, int bitIndex)
        {
            uint blockIndex = (uint)(bitIndex / 64);
            if (blockIndex > bitSet.blockCount) throw new Exception("blockIndex > bitSet.blockCount");
            bitSet.bits[blockIndex] |= ((ulong)1 << bitIndex % 64);
        }

        public static void SetBitGrow(ref BitSet bitSet, int bitIndex)
        {
            uint blockIndex = (uint)(bitIndex / 64);
            if (blockIndex >= bitSet.blockCount)
            {
                GrowBitSet(ref bitSet, blockIndex + 1);
            }
            bitSet.bits[blockIndex] |= ((ulong)1 << bitIndex % 64);
        }

        public static void ClearBit(ref BitSet bitSet, int bitIndex)
        {
            uint blockIndex = (uint)(bitIndex / 64);
            if (blockIndex >= bitSet.blockCount)
            {
                return;
            }
            bitSet.bits[blockIndex] &= ~((ulong)1 << bitIndex % 64);
        }

        public static bool GetBit(ref BitSet bitSet, int bitIndex)
        {
            uint blockIndex = (uint)(bitIndex / 64);
            if (blockIndex >= bitSet.blockCount)
            {
                return false;
            }
            return (bitSet.bits[blockIndex] & ((ulong)1 << bitIndex % 64)) != 0;
        }

        public static uint GetBitSetBytes(ref BitSet bitSet)
        {
            return bitSet.blockCapacity * sizeof(ulong);
        }
    }
}
