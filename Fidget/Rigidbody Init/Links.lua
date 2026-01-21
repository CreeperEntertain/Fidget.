local linksMT = {}
local links = {}
links.allLinks = {}
linksMT.__index = {}

local mtIndex = linksMT.__index
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
function mtIndex.remove(self)
  for i, rigidbody in pairs(self.rigidbodies) do
    rigidbody.isInLink = false
    rigidbody.currentLink = nil
  end
  links.allLinks[self.index] = nil
  self = nil
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





local vec3 = vectors.vec3
local vec4 = vectors.vec4
local unpack3 = vec3().unpack
local nullVec = vec3()











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
function mtIndex.getBodiesInLink(self)
  return self.rigidbodies
end











function links.createLink(bodies,params)
  if not params then
    params = {}
  end
  local centerOfMass = nullVec
  local totalMass = 0
  local newInertiaTensor = matrices.mat3()
  --loop one: early errors + total mass for center of mass

  for i, body in pairs(bodies) do
    if not body or not body.index then
      error("§4§nLink Creation: Rigidbody(index: "..i.." ,value: "..body.." ) is invalid!")
    end  
    if body.isInLink then
      error("§4§nLink Creation: Rigidbody(index: "..i.." ,value: "..body.." ) is already in a link!")
    end
    totalMass = totalMass + 1/body.invMass
    body.isInLink = true

  end
  --loop 2 calc center of mass
  for i, body in pairs(bodies) do
    centerOfMass = centerOfMass + body.pos * 1/body.invMass
  end
  centerOfMass = centerOfMass/totalMass
  
  --loop 3 calc new inertia tensor
  for i, body in pairs(bodies) do
    local rotMat = Fidget.quaternions.toRotationMatrix3(body.rot)
    --Every physics iteration the local tensor is updated, but at spawn it equels the world tensor
    local bodyTensor = rotMat * body.invInertiaTensorLOCAL *
                rotMat:transposed()

    local linkRotMat = Fidget.quaternions.toRotationMatrix3(params.rot or vec4(0 ,0 ,-1 ,0))   
    bodyTensor = linkRotMat * bodyTensor *
                linkRotMat:transposed()--keep in mind this is still inv inertia tensor we need normal
    --parallel axis theorem
    -- |r|*I3 - r*(r^T)
    local x,y,z = unpack3(body.pos - centerOfMass)
    newInertiaTensor  = newInertiaTensor + (bodyTensor:inverted() + matrices.mat3(vec3(y*y+z*z,-x*y,-x*z),vec3(-x*y,x*x+z*z,-y*z),vec3(-x*z,-y*z,x*x+y*y)))
  end






if params.linearMovement == nil then
  params.linearMovement = true
end

  local newInertiaTensor = newInertiaTensor:inverted()
  local link = {

    invMass = (1 / totalMass),
    invInertiaTensorLOCAL = newInertiaTensor,
    invInertiaTensorWORLD = newInertiaTensor,
    gravity = params.gravity or vec3(0, -100, 0),
    friction = params.friction or 0.5,
    rotMat = matrices.mat3(),

    pos = params.pos or centerOfMass,
    vel = params.vel or vec3(),
    rot = params.rot or vec4(0, 0, -1, 0):normalize(),
    rotVel = params.rotVel or vec3(),
    forceAccum = vec3(),
    torqueAccum = vec3(),
    linearMovement = params.linearMovement ,
    damping = params.damping or 1,
    rotationDamping = params.rotationDamping or 1,

    index = #links.allLinks + 1,
    isSleeping = params.isSleeping,
    sleepTimer = 0,

    prevPos = params.pos or centerOfMass,
    prevRot = params.rot or vec4(0, 0, -1, 0),
    updatePrevPos = true,
    updatePrevRot = true,


    rigidbodies = bodies,
  }
  --UgHHHH this loop hurts me sooooo much. 4 for loops that loop over the same thing 0.001 meters from eachother

  for i, body in pairs(bodies) do
    body.currentLink = link
    body.deltaPos = body.pos-link.pos 
  end

  setmetatable(link,linksMT)
  table.insert(links.allLinks, link)


  return link
end

return links
