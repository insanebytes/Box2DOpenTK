using OpenTK.Mathematics;
using System;

namespace Box2DOpenTK
{

    public enum SetType
    {
        staticSet = 0,
        disabledSet = 1,
        awakeSet = 2,
        firstSleepingSet = 3,
    };

    // Per thread task storage
    public struct b2TaskContext
    {
        // These bits align with the b2ConstraintGraph::contactBlocks and signal a change in contact status
        BitSet contactStateBitSet;

        // Used to track bodies with shapes that have enlarged AABBs. This avoids having a bit array
        // that is very large when there are many static shapes.
        BitSet enlargedSimBitSet;

        // Used to put islands to sleep
        BitSet awakeIslandBitSet;

        // Per worker split island candidate
        float splitSleepTime;
        int splitIslandId;

    }

    public struct World
    {
        public static float lengthUnitsPerMeter;

        const int MAX_WORLDS = 128;

        public StackAllocator stackAllocator;
        public BroadPhase broadPhase;
        public ConstraintGraph constraintGraph;

        // The body id pool is used to allocate and recycle body ids. Body ids
        // provide a stable identifier for users, but incur caches misses when used
        // to access body data. Aligns with b2Body.
        public IdPool bodyIdPool;

        // This is a sparse array that maps body ids to the body data
        // stored in solver sets. As sims move within a set or across set.
        // Indices come from id pool.
        public Body[] bodyArray;

        // Provides free list for solver sets.
        public IdPool solverSetIdPool;

        // Solvers sets allow sims to be stored in contiguous arrays. The first
        // set is all static sims. The second set is active sims. The third set is disabled
        // sims. The remaining sets are sleeping islands.
        public SolverSet[] solverSetArray;

        // Used to create stable ids for joints
        public IdPool jointIdPool;

        // This is a sparse array that maps joint ids to the joint data stored in the constraint graph
        // or in the solver sets.
        public Joint[] jointArray;

        // Used to create stable ids for contacts
        public IdPool contactIdPool;

        // This is a sparse array that maps contact ids to the contact data stored in the constraint graph
        // or in the solver sets.
        public Contact[] contactArray;

        // Used to create stable ids for islands
        public IdPool islandIdPool;

        // This is a sparse array that maps island ids to the island data stored in the solver sets.
        public Island[] islandArray;

        public IdPool shapeIdPool;
        public IdPool chainIdPool;

        // These are sparse arrays that point into the pools above
        public Shape[] shapeArray;
        public ChainShape[] chainArray;

        // Per thread storage
        public TaskContext[] taskContextArray;

        public BodyMoveEvent[] bodyMoveEventArray;
        public SensorBeginTouchEvent[] sensorBeginEventArray;
        public SensorEndTouchEvent[] sensorEndEventArray;
        public ContactBeginTouchEvent[] contactBeginArray;
        public ContactEndTouchEvent[] contactEndArray;
        public ContactHitEvent[] contactHitArray;

        // Used to track debug draw
        public BitSet debugBodySet;
        public BitSet debugJointSet;
        public BitSet debugContactSet;

        // Id that is incremented every time step
        public ulong stepIndex;

        // Identify islands for splitting as follows:
        // - I want to split islands so smaller islands can sleep
        // - when a body comes to rest and its sleep timer trips, I can look at the island and flag it for splitting
        //   if it has removed constraints
        // - islands that have removed constraints must be put split first because I don't want to wake bodies incorrectly
        // - otherwise I can use the awake islands that have bodies wanting to sleep as the splitting candidates
        // - if no bodies want to sleep then there is no reason to perform island splitting
        public int splitIslandId;

        public Vector2 gravity;
        public float hitEventThreshold;
        public float restitutionThreshold;
        public float maxLinearVelocity;
        public float contactPushoutVelocity;
        public float contactHertz;
        public float contactDampingRatio;
        public float jointHertz;
        public float jointDampingRatio;

        public ushort revision;

        public Profile profile;

        public PreSolveFcn[] preSolveFcn;
        public IntPtr preSolveContext;

        public CustomFilterFcn[] customFilterFcn;
        public IntPtr customFilterContext;

        public int workerCount;
        public EnqueueTaskCallback[] enqueueTaskFcn;
        public FinishTaskCallback[] finishTaskFcn;
        public IntPtr userTaskContext;
        public IntPtr userTreeTask;

        // Remember type step used for reporting forces and torques
        public float inv_h;

        public int activeTaskCount;
        public int taskCount;

        public ushort worldId;

        public bool enableSleep;
        public bool locked;
        public bool enableWarmStarting;
        public bool enableContinuous;
        public bool inUse;
    }
}
