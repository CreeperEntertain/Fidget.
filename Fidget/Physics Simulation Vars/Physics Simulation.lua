local physicsSim = {}
--setting up the physics simulation vars
physicsSim.physicsIterations = 2 --physics steps per tick [int]
physicsSim.dt = (1/20)/physicsSim.physicsIterations --delta time controls how fast the simulation runs if its lower then the simulation runs slower [float]
physicsSim.step = 0 --step count(how many steps happened since the physics started) used for cache  [int]
physicsSim.velocityIterations = 4 -- improves stability of the simulation has a lesser impact than physics iterations but keeping it <5 is recommended [int]
physicsSim.baumgarteMultiplier = 0.2 -- controls how much objects are pushed out of each other, keep this below 0.2 and above 0.05 with >0.3 there might be jitter and <0.05 the objects will clip into each other more
physicsSim.slop = 0.000 --unused tolerance value for collision detection [float]
physicsSim.relaxation = 1 --mightTM improve stability and convergence(but from my testing it didnt lmao) [float]
physicsSim.cacheMultiplier = 0.9--How much of the cached impulse come to the next physics step [float]
physicsSim.normalSnappingThreshold = 0.01--controls how matching the normals have to be to get snapped to world axis, improves stacking stability [float]
physicsSim.sleeping = false-- if true rigidbodies can sleep aka get immobilized after not moving for a bit its kinda janky for now
physicsSim.sleepTimeThreshold = 0.1 --how long does a body need to be still to sleep in seconds [float]
physicsSim.sleepVelocityThreshold = 0.1 -- How fast does a body need to move to sleep [float]
physicsSim.sleepRotVelocityThreshold = 0.02 -- How slow does the body need to rotate to sleep [float]
physicsSim.waterDensity = 1   -- controls buoyancy [float]
physicsSim.waterDamping = 0.92 -- how much is the velocity and ang velocity slown down in water [float]
physicsSim.contactPointMergingThreshold = 0.03--controls how close 2 contact points need to be to get merged into one [float]
physicsSim.solver = "regularPGS" --either regularPGS or scuffedSplitImpulse the 1st one has more accurate friction but is less stable with stacking, the 2nd is more stable but has a few "bugs" with friction sometimes. 
physicsSim.performFullSAT = false-- if false only performs sat on 6 axis. This gives about 10% performance increase. It may cause bugs but i havent encounterd any in testing
physicsSim.simulationRunning = true -- if false simulation stops running
physicsSim.spriteTaskLightDirection = vec(1,1,0):normalize() -- the light direction used for calculating inllumination of sprite tasks



physicsSim.debug = {
worldColliders = false,  --shows the world colliders
contacts = false,   --shows the contact points with impulses applied
axis = false, --shows axis of the body at local origin of the body
broadphaseAABB = false, --shows aabbs used in broadphase collision
collisionNormals = false, --show the normals at collisions(the the local space origin of one of the bodies)
waterVolumes = false, --shows the volumes used for buoyancy
joints = false, --shows the joints
}

function physicsSim.changeQuality(setting)
    if setting == "high" then
        physicsSim.physicsIterations = 3
        physicsSim.velocityIterations = 4
        physicsSim.dt = (1/20)/physicsSim.physicsIterations
        physicsSim.performFullSAT = true
    end
    if setting == "medium" then
        physicsSim.physicsIterations = 2
        physicsSim.velocityIterations = 4
        physicsSim.dt = (1/20)/physicsSim.physicsIterations
        physicsSim.performFullSAT = true
    end
    if setting == "low" then
        physicsSim.physicsIterations = 1
        physicsSim.velocityIterations = 3
        physicsSim.dt = (1/20)/physicsSim.physicsIterations
        physicsSim.performFullSAT = false
    end
    if setting == "lowest" then
        physicsSim.physicsIterations = 1
        physicsSim.velocityIterations = 1
        physicsSim.dt = (1/20)/physicsSim.physicsIterations
        physicsSim.performFullSAT = false
    end
end
  
function physicsSim.pauseSimulation()
physicsSim.simulationRunning = false
end

function physicsSim.resumeSimulation()
physicsSim.simulationRunning = true
end
return physicsSim





