local rigidbodyMT = {}
local rigidbodies = {}
local particles = {}
local copyStorage = models:newPart("copyStorage", "WORLD")
particles.spriteTasksToRender = {}
rigidbodies.allRigidbodies = {}
rigidbodyMT.__index = {}
models.Fidget.Model_Placeholders.placeholders.cube:setVisible(false)
local mtIndex = rigidbodyMT.__index
local vec3 = vectors.vec3
local length = vec3().length

function mtIndex.remove(self)
  self.model:getParent():removeChild(self.model)
  self.model:remove()
  local index = self.index

  for i, joint in pairs(Fidget.joints.allJoints) do
    if joint.rigidbody1.index == index then
      Fidget.joints.allJoints[i] = nil
    end
    if joint.rigidbody2.index == index then
      Fidget.joints.allJoints[i] = nil
    end
  end
  if self.spriteTasksRendered then
    for i, spriteTaskId in pairs(self.spriteTasksRendered) do
      local sprite = Fidget.particles.spriteTasksToRender[spriteTaskId]
      if sprite then
        sprite[5]:remove()
      end
      Fidget.particles.spriteTasksToRender[spriteTaskId] = nil
    end
  end
  rigidbodies.allRigidbodies[self.index] = nil
end

function mtIndex.addForce(self, force)
  self.forceAccum = self.forceAccum + force
end

function mtIndex.addForceAtPoint(self, point, force)
  local newPoint = point -self.pos
  --transformWorldToLocal(point,PhysicsObjects[objectIndex].rotMat,PhysicsObjects[objectIndex].position)
  self.torqueAccum = self.torqueAccum + newPoint:crossed(force)
  self.forceAccum = self.forceAccum + force
end



function mtIndex.setPos(self, pos)
  self.pos = pos
end

function mtIndex.getPos(self)
  return self.pos
end

function mtIndex.setVel(self, vel)
  self.vel = vel
end

function mtIndex.getVel(self)
  return self.vel
end

function mtIndex.setRot(self, rot)
  self.rot = rot
end

function mtIndex.getRot(self)
  return self.rot
end

function mtIndex.setRotVel(self, rotVel)
  self.rotVel = rotVel
end

function mtIndex.getRotVel(self)
  return self.rotVel
end

function mtIndex.setrotVel(self, rotVel)
  self.rotVel = rotVel
end

function mtIndex.getrotVel(self)
  return self.rotVel
end

function mtIndex.setDimensions(self, dims)
  self.dimensions = dims
end

function mtIndex.getDimensions(self)
  return self.dimensions
end

function mtIndex.setGravity(self, gravity)
  self.gravity = gravity
end

function mtIndex.getGravity(self)
  return self.gravity
end

function mtIndex.setFriction(self, friction)
  self.friction = friction
end

function mtIndex.getFriction(self)
  return self.friction
end

function mtIndex.setMass(self, mass)
  self.invMass = 1/mass
end

function mtIndex.getMass(self)
  return 1/self.invMass
end

function mtIndex.setLinearMovement(self, yn)
  self.linearMovement = yn
end

function mtIndex.getLinearMovement(self)
  return self.linearMovement
end

function mtIndex.setlinearMovement(self, yn)
  self.linearMovement = yn
end

function mtIndex.getlinearMovement(self)
  return self.linearMovement
end

function mtIndex.setWorldCollision(self, yn)
  self.worldCollision = yn
end

function mtIndex.getWorldCollision(self)
  return self.worldCollision
end

function mtIndex.setworldCollision(self, yn)
  self.worldCollision = yn
end

function mtIndex.getworldCollision(self)
  return self.worldCollision
end

function mtIndex.setIsSleeping(self, yn)
  self.isSleeping = yn
end

function mtIndex.getIsSleeping(self)
  return self.isSleeping
end

function mtIndex.setisSleeping(self, yn)
  self.isSleeping = yn
end

function mtIndex.getisSleeping(self)
  return self.isSleeping
end

function mtIndex.setModelScale(self, yn)
  self.modelScale = yn
end

function mtIndex.getModelScale(self)
  return self.modelScale
end

function mtIndex.setmodelScale(self, yn)
  self.modelScale = yn
end

function mtIndex.getmodelScale(self)
  return self.modelScale
end

function mtIndex.setDamping(self, yn)
  self.damping = yn
end

function mtIndex.getDamping(self)
  return self.damping
end

function mtIndex.setRotataionDamping(self, yn)
  self.rotationDamping = yn
end

function mtIndex.getRotataionDamping(self)
  return self.rotationDamping
end

function mtIndex.setBodyCollision(self, yn)
  self.bodyCollsion = yn
end

function mtIndex.getBodyCollision(self)
  return self.bodyCollsion
end











function mtIndex.getRotationMatrix(self)
  return self.rotMat
end

function mtIndex.dirToWorldSpace(self, dir)
  return dir * self.rotMat
end

function mtIndex.dirToLocalSpace(self, dir)
  return dir * self.rotMat:transposed()
end

function mtIndex.posToWorldSpace(self, pos)
  return (pos * self.rotMat) + self.pos
end

function mtIndex.posToLocalSpace(self, pos)
  return (self.pos - pos) * self.rotMat
end
function mtIndex.getModel(self)
  return self.model
end
function mtIndex.setUpdatePrevPos(self,bool)
  self.updatePrevPos = bool
end
function mtIndex.setUpdatePrevRot(self,bool)
  self.updatePrevRot = bool
end
function mtIndex.getUpdatePrevPos(self)
  return self.updatePrevPos
end
function mtIndex.getUpdatePrevRot(self)
  return self.updatePrevRot
end

function rigidbodies.raycast(startPos, endPos)
  local AABBsHit = {}
  local n = 1

  for i, rigidbody in pairs(rigidbodies.allRigidbodies) do
    local aabb1 = { {
      -(rigidbody.halfDimensions),
      (rigidbody.halfDimensions),
    } }

    local aabb, hitPos, side, aabbHitIndex = raycast:aabb((startPos - rigidbody.pos) * rigidbody.rotMat:transposed(),
      (endPos - rigidbody.pos) * rigidbody.rotMat:transposed(), aabb1)
    if aabbHitIndex then
      local worldHitPos = (hitPos * rigidbody.rotMat) + rigidbody.pos
      AABBsHit[n] = { id = i, hitPos = worldHitPos, side = side, distance = length(startPos - worldHitPos) }
      n = n + 1
    end
  end
  table.sort(AABBsHit, function(a, b) return a.distance < b.distance end)
  return AABBsHit
end

function rigidbodies.raycastBoundingAABB(startPos, endPos)
  local AABBsHit = {}
  local n = 1

  for i, rigidbody in pairs(rigidbodies.allRigidbodies) do
    local aabb1 = { {
      -(rigidbody.halfDimensions) + rigidbody.pos,
      (rigidbody.halfDimensions) + rigidbody.pos,
    } }

    local aabb, hitPos, side, aabbHitIndex = raycast:aabb(startPos, endPos, aabb1)
    if aabbHitIndex then
      local worldHitPos = (hitPos * rigidbody.rotMat) + rigidbody.pos
      AABBsHit[n] = { id = i, side = side, hitPos = worldHitPos, distance = length(startPos - worldHitPos) }
      n = n + 1
    end
  end
  table.sort(AABBsHit, function(a, b) return a.distance < b.distance end)
  return AABBsHit
end



--[[Example:

rigidbodyParams = {

mass = 100,
gravity = vec(0,0,0),
pos = player:getPos(),
vel = vec(0,1,0),
model = models.beerBottle,
friction = 0.3
}





]]
local vec3 = vectors.vec3
local vec4 = vectors.vec4
local unpack3 = vec3().unpack
--Ill always be using these since they are faster than plain old vec() and you can omit arguments in them to get a zero vector so vec3() == vec(0,0,0) this saves time and instructions for sending arguments to the function


local rigidbodyCopyStorage = models:newPart("copyStorage", "WORLD")
local function createCopy(modelpart)
  local copy = modelpart:copy("name"):setParentType("WORLD"):setVisible(true)
  rigidbodyCopyStorage:addChild(copy)
  return copy
end

local function getPlaceholderModel(type)
  if type == "cube" then
    --why can figura just not take fucking folders into account when indexxing modelparts bruhhhhh
    return models.Fidget.Model_Placeholders.placeholders.cube
  end
  if type == "particle" then
    return models.Fidget.Model_Placeholders.placeholders.cube
  end
end

local nullVec = vec3(0, 0, 0)
local function getInverseInertiaTensor(params)
  if params.noRot then
    return matrices.mat3() * 0
  end
  if params.type == "cube" then
    local a, b, c = unpack3(params.dimensions or vec3(1, 1, 1))
    return matrices.mat3(
      vec3((1 / 12) * params.mass * (b * b + c * c)),
      vec3(0, (1 / 12) * params.mass * (a * a + c * c)),
      vec3(0, 0, (1 / 12) * params.mass * (a * a + b * b))
    ):inverted()
  elseif params.type == "sphere" then
    local r = params.radius or 1
    local inertia = (2 / 5) * params.mass * r * r
    return matrices.mat3(
      vec3(inertia, 0, 0),
      vec3(0, inertia, 0),
      vec3(0, 0, inertia)
    ):inverted()
  elseif params.type == "particle" then
    return matrices.mat3(nullVec, nullVec, nullVec)
  end
end

local function getBoundingSphere(type, dimensions)
  if type == "cube" then
    return (dimensions or vec3(1, 1, 1)):length() / 2
  end
end



function rigidbodies.createRigidbody(params)
  if type(params.pos) ~= "Vector3" then
    error("§4§nRigidbody Creation: Position(vec3) expected, got: " .. tostring(params.pos) .. "§r")
  end
  if not params.type then
    params.type = "cube" --default type is cube
  end
  if not params.modelScale then
    params.modelScale = vec3() + 1 --default model scale is 1,1,1
    if params.type == "particle" then
      params.modelScale = vec3() + 0.1
    end
  end
  if not params.dimensions then
    params.dimensions = vec3() + 1 --default dimensions for cube is 1,1,1
  end
  if params.linearMovement == nil then
    params.linearMovement = true --by default rigidbodies can move linearly
  end
  if not params.mass then
    params.mass = 1
  end

  if params.mass <= 0 then
    error("§4§nRigidbody Creation: Mass must be a positive number, got: " .. tostring(params.mass) .. "§r")
  end

  if params.worldCollision == nil then
    params.worldCollision = true
  end
  if params.bodyCollision == nil then
    params.bodyCollision = true
  end
  local rigidbody = {

    --general attributes
    invMass = (1 / params.mass),
    invInertiaTensorLOCAL = getInverseInertiaTensor(params),
    invInertiaTensorWORLD = getInverseInertiaTensor(params),
    gravity = params.gravity or vec3(0, -100, 0),
    friction = params.friction or 0.5,
    rotMat = matrices.mat3(),

    --rigidbody movement
    pos = params.pos,
    vel = params.vel or vec3(),
    rot = params.rot or vec4(0, 0, -1, 0),
    rotVel = params.rotVel or vec3(),
    forceAccum = vec3(),
    torqueAccum = vec3(),
    linearMovement = params.linearMovement,
    damping = params.damping or 1,
    rotationDamping = params.rotationDamping or 1,

    --render stuff
    model = createCopy(params.model or getPlaceholderModel(params.type)), --set the model to the one given by user or to a placeholder
    prevPos = params.pos or vec3(),
    prevRot = params.rot or vec4(0, 0, -1, 0),
    modelScale = params.modelScale or vec3(1, 1, 1),

    --stuff specific to certain rigidbodies
    --**cuboid**--
    dimensions = params.dimensions,
    halfDimensions = params.dimensions:copy() * 0.5,
    --**sphere**--
    --radius = params.radius or 1,

    --**mesh**--
    --mesh = params.mesh
    --meshTriangles =


    --engine stuff
    boundingAABB = vec(0.5, 0.5, 0.5),
    vertices = {},
    type = params.type,
    cache = {},                             --for storing impulses cause it kinda converges really badly without this apparently
    index = #rigidbodies.allRigidbodies + 1,
    worldCollision = params.worldCollision, --only supported with aabb broadphase
    isSleeping = params.isSleeping or false,
    sleepTimer = -1,
    bodyCollision = params.bodyCollision,
    spriteTasksRendered = {},
    updatePrevPos = true,
    updatePrevRot = true,

    --functions
    onTick = params.onTick,
    onCollision = params.onCollision,
    onRender = params.render,
    onPhysicsStep = params.onPhysicsStep,
    onBroadphaseCollision = params.onBroadphaseCollision,
    onWorldCollision = params.onWorldCollision,


  }

  setmetatable(rigidbody, rigidbodyMT)
  table.insert(rigidbodies.allRigidbodies, rigidbody)
  return rigidbody
end

function particles.createParticle(params) -- who could've known particles are just rigidbodies
  params.type = "particle"
  return rigidbodies.createRigidbody(params)
end

-- on delete loop througs all spritetasks and remove them if in the





function particles.cloth(pos, maxX, maxY, spacing, mass, damping, rotation, texture)
  local particlesL = {}
  local n         = 0
  local startIndex = #Fidget.rigidbodies.allRigidbodies+1
  rotation = rotation or vec3(0, 0, 0)
  for x = 1, maxX do
    for y = 1, maxY do
      particlesL[n] = Fidget.particles.createParticle({ pos = pos +
      vec(x - maxX / 2, y - maxY / 2, 0) * matrices.rotation3(rotation) * spacing, modelScale = vec(0, 0, 0), mass = mass, damping = damping, worldCollision = false, bodyCollision = true })
      n = n + 1
    end
  end

  for x = 0, maxX - 2 do
    for y = 0, maxY - 1 do
      Fidget.joints.createJoint({ rigidbody1 = particlesL[(x) + maxX * (y)], rigidbody2 = particlesL[(x + 1) + maxX * (y)], distance =
      spacing})
    end
  end
  for x = 0, maxX - 1 do
    for y = 0, maxY - 2 do
      Fidget.joints.createJoint({ rigidbody1 = particlesL[(x) + maxX * (y)], rigidbody2 = particlesL[(x) + maxX * (y + 1)], distance =
      spacing })
    end
  end

  if texture then
    for x = 0, maxX - 2 do
      for y = 0, maxY - 2 do
        local spriteTaskId = #particles.spriteTasksToRender + 1
        local index1 = (x) + maxX * (y)
        local index2 = (x) + maxX * (y + 1)
        table.insert(particlesL[index1].spriteTasksRendered, spriteTaskId)
        table.insert(particlesL[index1 + 1].spriteTasksRendered, spriteTaskId)
        table.insert(particlesL[index2].spriteTasksRendered, spriteTaskId)
        table.insert(particlesL[index2 + 1].spriteTasksRendered, spriteTaskId)
        local spriteTask = copyStorage:newSprite(math.random() .. ""):setTexture(texture,texture:getDimensions().x,texture:getDimensions().y):setSize(16*spacing,16*spacing):setRegion(1,1):setUVPixels(x,y)

        particles.spriteTasksToRender[spriteTaskId] = { startIndex + index1, startIndex + index1 + 1, startIndex + index2, startIndex + index2 + 1, spriteTask }
      end
    end
  end
  return particlesL
end

function particles.rope(pos, maxX, spacing, mass, damping, rotation, modelpart)
  local particles = {}
  local n         = 0
  rotation = rotation or vec3(0, 0, 0)
  for x = 1, maxX do
    particles[n] = Fidget.particles.createParticle({ pos = pos + vec(0, x - maxX / 2, 0) * matrices.rotation3(rotation) *
    spacing, modelScale = vec(0, 0, 0), mass = mass, damping = damping })
    n = n + 1
  end

  for x = 0, maxX - 2 do
    Fidget.joints.createJoint({ rigidbody1 = particles[(x)], rigidbody2 = particles[(x + 1)], distance = spacing, model = modelpart, stretching = true })
  end

  return particles
end

return rigidbodies, particles


--changelog:

-- optimiztions
-- added a non full sat
-- saved one multiplication in pgs loop(:oof: (with 300 constraints(~example wall of boxes 5x5) saves 300 * 1 * physicsIterations(default 2) * velocityIterations(default 4) = 2400 tick instructions :skull:))

-- added spring joints
-- added new rigidbody parameters:
-- - damping
-- - rotationDamping
-- - bodyCollision
-- - updatePrevPos
-- - updatePrevRot
-- - not meant to be updated directly but accessible: spritTasksRendered
-- added particles + functions:
-- createParticle
--
-- removed broadphase collision types, aabb is always faster thus maintaining sphere is wasted work
-- added new functions to rigidbodies:
-- get/setDamping
-- get/setRotationDamping
-- getRotation matrix
-- dirToWorld/LocalSpace
-- posToWorld/LocalSpace
-- onTick = params.onTick,
-- onCollision = params.onCollision,
-- onRender = params.render,
-- onPhysicsStep = params.onPhysicsStep,
-- onBroadphaseCollision = params.onBroadphaseCollision,
-- onWorldCollision = params.onWorldCollision
-- raycast
-- raycastBoundingAABB
-- getModel
-- setUpdatePrevPos
-- setUpdatePrevRot
--
-- renamed functions for readability
--introduced spaghetti code


-- added to joints
-- onTick
-- onPhysicsStep joint + distance
-- onRender
-- modelScale
-- model
-- modelStretching

-- fixed worldCollision parameter not working
-- fixed some blocks causing buoyancy calculation to error
-- (why does 1 layer of snow snowLayer:hasCollision() = true but snowLayer:getCollisionShape = nil????)
--made cloths have good lambertian lighting cause the default way minecraft/figura does it stinks so fucking bad

-- added to physics sim:
-- preformFullSAT
-- simulationRunning
-- spriteTaskLightDirection
-- removed broadPhaseCollision from physicsSim. Sphere based broadpahse is no longer available since it was slower 99.9% of the time
-- pauseSimulation function
-- resumeSimulation function

-- fix: made it so bodies with no linear movement cant be pulled by joints(this led to NANs)

--changed tick and render events to world_tick and world_render since this aint getting used on default/high perms anyways :skull: