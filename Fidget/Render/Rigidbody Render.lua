local Fidget = require("Fidget.FidgetSetup")
local quaternions = require("Fidget.quaternions")
local vec3 = vectors.vec3
local vec4 = vectors.vec4
local math_lerp = math.lerp
local normalize = vec4().normalize
local augmented = matrices.mat3().augmented
local translate = matrices.mat4().translate
local nullVec = vec(0,0,0,1)
local vertexTransformVector = vec(-1,-1,1) * 16
local offsetVector = vec(0,0,0)
local upVector = vec(1,0,0):normalize()
local function directionToEulerDegree(dirVec)
    local yaw = math.atan2(dirVec.x, dirVec.z)
    local pitch = math.atan2(dirVec.y, dirVec.xz:length())
    return vec3(-math.deg(pitch)+90, math.deg(yaw), 0)
end

function events.tick()
  for i, joint in pairs(Fidget.joints.allJoints) do
    local pos1, pos2 = (joint.pos1 * joint.rigidbody1.rotMat) + joint.rigidbody1.pos,(joint.pos2 * joint.rigidbody2.rotMat) +joint.rigidbody2.pos
    joint.prevPos = joint.pos
    joint.pos = pos2 - pos1
    joint.prevWorldPos1 = joint.worldPos1
    joint.worldPos1 = pos1
  end
end

function events.world_render(delta,context)
  local upVector = Fidget.spriteTaskLightDirection or upVector
  --for performance reasons
    for i, rigidbody in pairs(Fidget.rigidbodies.allRigidbodies) do
      if not rigidbody.isInLink then
    local pos = math.lerp(rigidbody.prevPos, rigidbody.pos, delta) * 16
    local scale = matrices.mat3(vec3(rigidbody.modelScale.x),vec3(0,rigidbody.modelScale.y),vec(0,0,rigidbody.modelScale.z)) 
    --the mind tells me: remove this corny ass name but, but
    -- BUT the other half of my brain says - leave it here so people can cringe at your fucking pathetic attempt at a joke

    local THEONEANDONLYPLEASEWELCOMETHEROTATIONMATRIXPRETENDSOMECOOLASSMUSICPLAYSHERE = augmented((quaternions.toRotationMatrix3(normalize(math_lerp(rigidbody.prevRot or nullVec, rigidbody.rot or nullVec, delta)))*(scale)))
    local mat = translate(THEONEANDONLYPLEASEWELCOMETHEROTATIONMATRIXPRETENDSOMECOOLASSMUSICPLAYSHERE,pos)
    rigidbody.model:setMatrix(mat):setLight(15)
    if rigidbody.onRender then
      rigidbody.onRender(rigidbody,delta,context)
    end
    else
    local linkRotMat = quaternions.toRotationMatrix3(normalize(math_lerp(rigidbody.currentLink.prevRot or nullVec, rigidbody.currentLink.rot or nullVec, delta)))
    local pos = ((rigidbody.deltaPos *  linkRotMat) + math_lerp(rigidbody.currentLink.prevPos,rigidbody.currentLink.pos,delta) )*16
    local scale = matrices.mat3(vec3(rigidbody.modelScale.x),vec3(0,rigidbody.modelScale.y),vec(0,0,rigidbody.modelScale.z)) 
    local THEONEANDONLYPLEASEWELCOMETHEROTATIONMATRIXPRETENDSOMECOOLASSMUSICPLAYSHERE = augmented((quaternions.toRotationMatrix3(normalize(math_lerp(rigidbody.prevRot or nullVec, rigidbody.rot or nullVec, delta)))*(scale)))
    local mat = translate(THEONEANDONLYPLEASEWELCOMETHEROTATIONMATRIXPRETENDSOMECOOLASSMUSICPLAYSHERE,pos)
        rigidbody.model:setMatrix(mat):setLight(15)
    if rigidbody.onRender then
      rigidbody.onRender(rigidbody,delta,context)
    end
    end
  end
  for i, joint in pairs(Fidget.joints.allJoints) do
    local currentDeltaPos = math.lerp(joint.prevPos,joint.pos,delta)
    if joint.model then
    joint.model:setScale(joint.modelScale * (joint.stretching and vec(1,(currentDeltaPos):length(),1) or 1)):setPos(math.lerp(joint.prevWorldPos1,joint.worldPos1,delta)*16):setRot(directionToEulerDegree((currentDeltaPos):normalize()))
    end
    if joint.onRender then
      joint.onRender(joint,(joint.pos):length(),delta,context)
    end
  end
  for i, Sprite in pairs(Fidget.particles.spriteTasksToRender) do
    --   ^^^^^^ Uppercase Sprite T~T
    local index1, index1p, index2, index2p, spriteTask = Sprite[1], Sprite[2], Sprite[3], Sprite[4], Sprite[5]

    local p1 = Fidget.rigidbodies.allRigidbodies[index1]
    local p1p = Fidget.rigidbodies.allRigidbodies[index1p]
    local p2 = Fidget.rigidbodies.allRigidbodies[index2]
    local p2p = Fidget.rigidbodies.allRigidbodies[index2p]
    if p1 and p1p and p2 and p2p then
    local p1pos = p1.pos
    local p1ppos = p1p.pos
    local p2pos = p2.pos
    --minecrafts way of calculating lighting based on normals is fucked so i just do this
    local normal = vec3(0,math.map(math.lerp(0,-1,-((p2pos - p1pos):cross(p1ppos - p1pos) ):normalize():dot(upVector)),-1,1,-0.35,-1),0)
    local pos1 = math.lerp(p1.prevPos, p1.pos, delta) * vertexTransformVector 
    local pos1p = math.lerp(p1p.prevPos, p1p.pos, delta) * vertexTransformVector 
    local pos2 = math.lerp(p2.prevPos, p2.pos, delta) * vertexTransformVector 
    local pos2p = math.lerp(p2p.prevPos, p2p.pos, delta) * vertexTransformVector 
    local verts = spriteTask:getVertices() 
    verts[1]:setPos(pos1):setNormal(normal)
    verts[2]:setPos(pos1p):setNormal(normal)
    verts[4]:setPos(pos2):setNormal(normal)
    verts[3]:setPos(pos2p):setNormal(normal)
    end
  end
end
