local jointMT = {}
local joints = {}
joints.allJoints = {}
jointMT.__index = {}

local mtIndex = jointMT.__index
local copyStorage = models:newPart("copyStorage", "WORLD")
function mtIndex.remove(self)
    if self.model then
    self.model:getParent():removeChild(self.model)
  self.model:remove()
    end
joints.allJoints[self.index] = nil
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
local function createCopy(modelpart)
  local copy = modelpart:copy("name"):setParentType("WORLD"):setVisible(true)
  copyStorage:addChild(copy)
  return copy
end

local vec3 = vectors.vec3
local vec4 = vectors.vec4
local unpack3 = vec3().unpack
function mtIndex.setPos1(self,pos)
  self.pos1 = pos
end
function mtIndex.setPos2(self,pos)
  self.pos2 = pos
end
function mtIndex.getPos1(self)
  return self.pos1
end
function mtIndex.getPos2(self)
  return self.pos2
end
function mtIndex.setDistance(self,pos)
  self.minDistance = pos
  self.maxDistance = pos
end
function mtIndex.setMinDistance(self,pos)
  self.minDistance = pos
end
function mtIndex.setMaxDistance(self,pos)
  self.maxDistance = pos
end
function mtIndex.getMinDistance(self)
  return self.minDistance
end
function mtIndex.getMaxDistance(self)
  return self.maxDistance
end
function mtIndex.getDistance(self)
  return self.minDistance
end
function mtIndex.setStretching(self,pos)
  self.stretching = pos
end
function mtIndex.getStretching(self)
  return self.stretching
end
function mtIndex.setModelScale(self,pos)
  self.modelScale = pos
end
function mtIndex.getModelScale(self)
  return self.modelScale
end
function mtIndex.setStiffness(self,pos)
  self.modelScale = pos
end
function mtIndex.getStiffness(self)
  return self.modelScale
end
function mtIndex.getModel(self)
  return self.model
end
function mtIndex.getType(self)
  return self.type
end



function joints.createJoint(params)

  if not params.rigidbody1 or not params.rigidbody2 or not params.rigidbody1.index or not params.rigidbody2.index then
    error("§4§nJoints Creation: Both rigidbodies must be valid!")
  end
  if params.rigidbody1.index == params.rigidbody2.index then
    error("§4§nJoints Creation: Both rigidbodies must be different!")
  end

  if not params.type then 
    params.type = "distance" --default type is distance
  end
  local joint = {
    --joint attributes
    pos1 = params.pos1 or vec3(), -- in local space
    pos2 = params.pos2 or vec3(),
    worldPos1 = params.pos1 or vec3(),
    prevWorldPos1 = params.pos1 or vec3(),
    pos = params.pos1 or vec3(),
    prevPos = params.pos1 or vec3(),
    prevRot = vec(0,0,0),
    type = params.type, --distance, ball, hinge
    rigidbody1 = params.rigidbody1,
    rigidbody2 = params.rigidbody2,
    minDistance = params.minDistance or params.distance or 0,
    maxDistance = params.maxDistance or params.distance or 0,
    index = #joints.allJoints + 1,
    stiffness = params.stiffness or 1,
    onTick = params.onTick,
    onPhysicsStep = params.onPhysicsStep,
    onRender = params.onRender,
    modelScale = params.modelScale or vec3(1,1,1),
    model = params.model and createCopy(params.model) or nil,
    stretching = params.stretching or false,
  }
  setmetatable(joint,jointMT)
  table.insert(joints.allJoints, joint)
  return joint
end

return joints
