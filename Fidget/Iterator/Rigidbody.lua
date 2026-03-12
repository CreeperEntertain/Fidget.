--This file is the iterator for rigidbodies.
--Here we itarate through rigidbodies aka integrate their pos and rotations, call collision functions and update variables for rendering.








local cache = require("Fidget.Iterator.Cache")
local quaternions = require("Fidget.quaternions")
local Fidget = require("Fidget.FidgetSetup") --i do have a global table to avoid circular dependencies but here its bot loaded yet lmao
local greedy = require("Fidget.Greedy_Meshing")
local vec3 = vectors.vec3
local vec4 = vectors.vec4
local unpack3 = vec3().unpack
local length = vec3().length
local normalized = vec3().normalized
local normalize4 = vec4().normalize
local crossed = vec3().crossed
local dot = vec3().dot
local q_u_otationMatrix3 = quaternions.toRotationMatrix3
local qultiply = quaternions.multiply
local worldGetBlockState = world.getBlockState
local hasCollision = worldGetBlockState(0, 0, 0).hasCollision
local zeroVec = vec3()
local copy = vec3().copy
local mat3 = matrices.mat3
local transposed3 = mat3().transpose
local math_floor = math.floor
local math_abs = math.abs
local oneVec = vec3() + 1
local zeropointfivevec = vec3() + 0.5
local math_sqrt = math.sqrt
local math_clamp = math.clamp
local lnext = next
local ltonumber = tonumber
local ltostring = tostring
local cacheMultiplier = 0
local nullMat = mat3() * 0
local matCopy = mat3().copy
local cacheRetrieve = cache.retrieve
local cacheAdd = cache.add
local doBroadPhaseCollision = doBroadPhaseCollision
local nullVec = vec3(0,0,0)
local rigidbodyTypeSortingTable = {
["cuboid"] = 1,
["particle"] = 2,
["sphere"] = 3,
["capsule"] = 4,
}
local doFineCollision = doFineCollision
figuraMetatables.Vector3.__pow = dot
figuraMetatables.Vector3.__mod = crossed

local constraints = {}


local globalSnappingAxis1 = vec3(1)
local globalSnappingAxis2 = vec3(0, 1)
local globalSnappingAxis3 = vec3(0, 0, 1)
--[[formerly
local globalAxisForNormalSnapping = {
  vec3(1),
  vec3(0, 1),
  vec3(0, 0, 1)
}]]
--table for water heights used for buoyancy and approximating the volume of the rigidbody underwater
--i think its not correct but it works fine soooo
local waterHeight = {
  [0] = 9 * 14 / (16 * 9),
  8 * 14 / (16 * 9),
  7 * 14 / (16 * 9),
  6 * 14 / (16 * 9),
  5 * 14 / (16 * 9),
  4 * 14 / (16 * 9),
  3 * 14 / (16 * 9),
  2 * 14 / (16 * 9),
  1 * 14 / (16 * 9)
}




--used for the lineAABB()
local v1 = vec3(1, 1, -1)
local v2 = vec3(1, -1, 1)
local v3 = vec3(-1, 1, 1)
local v4 = vec3(-1, 1, -1)
local v5 = vec3(1, -1, -1)
local v6 = vec3(-1, -1, 1)
local color = vec3(0, 1, 1)

--debug visual
local function lineAABB(pos, aabb)
  createLine(pos + aabb, pos + aabb * v1, color)
  createLine(pos + aabb, pos + aabb * v3, color)
  createLine(pos + aabb * v1, pos + aabb * v4, color)
  createLine(pos + aabb * v3, pos + aabb * v4, color)



  createLine(pos + aabb, pos + aabb * v2, color)
  createLine(pos - aabb, pos + aabb * v4, color)
  createLine(pos + aabb * v3, pos + aabb * v6, color)
  createLine(pos + aabb * v1, pos + aabb * v5, color)


  createLine(pos + aabb * v2, pos + aabb * v5, color)
  createLine(pos + aabb * v2, pos + aabb * v6, color)
  createLine(pos + aabb * v5, pos - aabb, color)
  createLine(pos + aabb * v6, pos - aabb, color)
end
--calculating vertices for le cuboid
local function recalculateVertices(rigidbody)
  local pos = rigidbody.pos
  local rtm = rigidbody.rotMat
  local vertices = {}
  local xdim, ydim, zdim = unpack3(rigidbody.halfDimensions)


  local xAxis = rtm[1] * xdim
  local yAxis = rtm[2] * ydim
  local zAxis = rtm[3] * zdim

  local a = xAxis + yAxis + zAxis
  local b = xAxis - yAxis - zAxis
  local c = xAxis - yAxis + zAxis
  local d = xAxis + yAxis - zAxis
  vertices[1] = pos - a
  vertices[2] = pos + b
  vertices[3] = pos + c
  vertices[4] = pos - d
  vertices[5] = pos - c
  vertices[6] = pos + d
  vertices[7] = pos + a
  vertices[8] = pos - b
  rigidbody.vertices = vertices
end



local v1 = vec3(1)
local v2 = vec3(0, 1)
local v3 = vec3(0, 0, 1)



--time spent reflecting on my life decisions:
--fixing the sat 1.5 hours
--fixing polygon clipping 1.5 hours
--fixing a weird bug i can diagnose 4 hours*

--time spent reflecting on my shortcomings:
--50hours at least







local function calculateBoundingAABBCuboid(verts)
  local minx, maxx = 694202137000, -694202137000
  local miny, maxy = 694202137000, -694202137000
  local minz, maxz = 694202137000, -694202137000

  for i, vert in lnext, verts do --its just generating an aabb from verts like there isnt anythin particularly complicated about that
    local vx, vy, vz = unpack3(vert)
    if vx < minx then
      minx = vx
    end
    if vx > maxx then
      maxx = vx
    end
    if vy < miny then
      miny = vy
    end
    if vy > maxy then
      maxy = vy
    end
    if vz < minz then
      minz = vz
    end
    if vz > maxz then
      maxz = vz
    end
  end

  return vec3((maxx - minx), (maxy - miny), (maxz - minz)) * 0.5
end



local function vclamp(v1, v2, v3)
  return vec3(math_clamp(v1.x, v2.x, v3.x), math_clamp(v1.y, v2.y, v3.y), math_clamp(v1.z, v2.z, v3.z))
end




--returns colliders/blocks that are touching the rigidbody
local function getWorldColliders(rigidbody)
  local worldMesh = {}
  local aabbs = {}
  local n = 1
  local difPlus, difMinus = rigidbody.pos + rigidbody.boundingAABB, rigidbody.pos - rigidbody.boundingAABB
  if rigidbody.type ~= "particle" then
    local min = (difMinus):floor()
    local max = (difPlus):floor()

    for x = min.x, max.x do
      worldMesh[x] = worldMesh[x] or {}
      local worldMeshx = worldMesh[x]
      for y = min.y, max.y do
        worldMeshx[y] = worldMeshx[y] or {}
        local worldMeshxy = worldMeshx[y]
        for z = min.z, max.z do
          local block = (worldGetBlockState(x, y, z))
          local colliders = block:getCollisionShape()
          if hasCollision(block) and colliders[1] and colliders[1][1] == zeroVec and colliders[1][2] == oneVec then
            worldMeshxy[z] = true
          else
            for i, collider in lnext, colliders do
              local dims = (collider[2] - collider[1])
              local pos = collider[1] + vec3(x, y, z)
              local max1 = difPlus
              local min1 = difMinus
              local max2 = pos + dims
              local min2 = pos

              if max1 > min2 and max2 > min1 then
                aabbs[n] = { pos = pos + dims * 0.5, halfDimensions = dims * 0.5, type = "cuboid" }
                n = n + 1
              end
            end
          end
        end
      end
    end



    local greedy = greedy(worldMesh)
    for i, cuboid in lnext, greedy do
      if cuboid.id then
        aabbs[n] = { pos = cuboid.pos + cuboid.dims, halfDimensions = cuboid.dims , type = "cuboid" }
        n = n + 1
      end
    end
  else
    local block = worldGetBlockState(rigidbody.pos)
    local colliders = block:getCollisionShape()
    if hasCollision(block) and colliders[1] and colliders[1][1] == zeroVec and colliders[1][2] == oneVec then
      aabbs[n] = { pos = (rigidbody.pos):floor() + zeropointfivevec, halfDimensions = zeropointfivevec , type = "cuboid" }
      n = n + 1
    else
      for i, collider in lnext, colliders do
        local dims = (collider[2] - collider[1])
        local pos = collider[1] + (rigidbody.pos):floor()
        local max1 = difPlus
        local min1 = difMinus
        local max2 = pos + dims
        local min2 = pos

        if max1 > min2 and max2 > min1 then
          aabbs[n] = { pos = pos + dims * 0.5, halfDimensions = dims * 0.5, type = "cuboid" }
          n = n + 1
        end
      end
    end
  end
  return aabbs
end








--creates constriants for the solver to well solve
local function createConstraints(
    rigidbody1, rigidbody2, verts, faceNormal2, i2, type)
  local n = #constraints + 1
  local frictionCoef = (rigidbody1.friction + (type and rigidbody2.friction or 0)) * (type and 0.5 or 1)

local tangent1, tangent2

    if math.abs(faceNormal2.x) > 0.5 then
      tangent1 = vec3(faceNormal2.y, -faceNormal2.x, 0)
    else
      tangent1 = vec3(0, faceNormal2.z, -faceNormal2.y)
    end
    
    tangent1 = normalized(tangent1)
    tangent2 = normalized(faceNormal2 % tangent1)
    if rigidbody1.isInLink then
      rigidbody1 = rigidbody1.currentLink
    end
    if rigidbody2.isInLink then
      rigidbody2 = rigidbody2.currentLink
    end
    local name1 = type and "contact" or "contactWorld"
    local name2 = type and "friction" or "frictionWorld"
    local cacheRetrievalForRigidbody2 = type and rigidbody2.index or rigidbody2

    for i, vert in lnext, verts do
      constraints[n] = {
        rigidbody1 = rigidbody1,
        rigidbody2 = rigidbody2,
        bodyTwo = type,
        contactPoint = vert[3],
        normal = faceNormal2,
        penetration = vert[4],
        normalIndex = i2,
        type = name1,
        baumgarte = true,
        cached = true,
        impulse = cacheRetrieve(rigidbody1.index, cacheRetrievalForRigidbody2, vert, i2) * cacheMultiplier,
        edges = vert
      }

  if frictionCoef > 0 then
      constraints[n + 1] = {
        rigidbody1 = rigidbody1,
        rigidbody2 = rigidbody2,
        bodyTwo = type,
        contactPoint = vert[3],
        normal = tangent1,
        friction = frictionCoef,
        normalConstraint = constraints[n],
        type = name2,
        impulse = 0
      }


      constraints[n + 2] = {
        rigidbody1 = rigidbody1,
        rigidbody2 = rigidbody2,
        bodyTwo = type,
        contactPoint = vert[3],
        normal = tangent2,
        friction = frictionCoef,
        normalConstraint = constraints[n],
        type = name2,
        impulse = 0
      }
      n = n + 2
      
    end
    n= n+1
  end
end


function events.world_tick()

  --for performance reasons
  local physicsSim = Fidget.physicsSim
  local physicsSimDebug = physicsSim.debug
  local showAxis = physicsSimDebug.axis
  local showContacts = physicsSimDebug.contacts
  local showBroadphaseAABB = physicsSimDebug.broadphaseAABB
  local showWorldColliders = physicsSimDebug.worldColliders
  local dt = physicsSim.dt
  local halfdt = dt * 0.5
  cacheMultiplier = physicsSim.cacheMultiplier --update cause an external func uses this
  local normalSnappingThreshold = physicsSim.normalSnappingThreshold
  local sleepTimeThreshold = physicsSim.sleepTimeThreshold
  local isSleepAllowed = physicsSim.sleeping
  local sleepVelocityThreshold = physicsSim.sleepVelocityThreshold
  local sleepRotVelocityThreshold = physicsSim.sleepRotVelocityThreshold
  local broadPhaseCollision = physicsSim.broadPhaseCollision
  local rigidbodies = Fidget.rigidbodies.allRigidbodies
  local links = Fidget.links.allLinks
  local collisionNormals = physicsSimDebug.collisionNormals
  local solver = physicsSim.solver
  local waterDensity = physicsSim.waterDensity
  local showWaterVolumes = physicsSimDebug.waterVolumes
  local waterDamping = physicsSim.waterDamping

  local physicsIterations = physicsSim.physicsIterations
  local joints = Fidget.joints.allJoints
  local showJoints = physicsSimDebug.joints
  local simulationRunning = physicsSim.simulationRunning
  local oneOverPhysicsIters = (1 / physicsIterations)
  local baumgarteOverDt = (physicsSim.baumgarteMultiplier / dt)
  local jointBaumgarteOverDt = (physicsSim.jointBaumgarteMultiplier / dt)
  


  if simulationRunning then
      local timA = client:getSystemTime()
    for i = 1, physicsIterations do
      physicsSim.step = physicsSim.step + 1
      constraints = {}
      removeAllLines()


      --le Broadphase collision with aabbs
      local potentialCollisions = doBroadPhaseCollision(rigidbodies, broadPhaseCollision)







      for i, jtbl in lnext, potentialCollisions do
        for j in lnext, jtbl do
          local rigidbody1 = rigidbodies[i]
          local rigidbody2 = rigidbodies[j]
          if rigidbody1.onBroadphaseCollision then
            rigidbody1.onBroadphaseCollision(rigidbody1, rigidbody2)
          end
          if rigidbody2.onBroadphaseCollision then
            rigidbody2.onBroadphaseCollision(rigidbody2, rigidbody1)
          end

          if not (rigidbody1.isSleeping and rigidbody2.isSleeping) then
            local rtm1 = rigidbody1.rotMat
            local rtm2 = rigidbody2.rotMat
            local axis = {
              rtm1[1], rtm1[2], rtm1[3],
              rtm2[1], rtm2[2], rtm2[3],
            }
            local mtv, verts, i2, faceNormal2, faceNormal1 = doFineCollision(rigidbody1, rigidbody2, axis) --provides me with the contact normal, penetration + collision type to determine how to genatrate contact points

            if mtv then
              --clipping the colliding face of le cube






              --for stability this snaps contact normals to the nearest world axis assuming its close enough.
              --I noticed that the normals have slight errors in them which lead to shitty collision
              if normalSnappingThreshold > 0 then
                --unpacked loop for performance

                local similarity = faceNormal2 ^ globalSnappingAxis1
                if similarity > 1 - normalSnappingThreshold then
                  faceNormal2 = globalSnappingAxis1
                elseif similarity < -1 + normalSnappingThreshold then
                  faceNormal2 = -globalSnappingAxis1
                else
                  similarity = faceNormal2 ^ globalSnappingAxis2
                  if similarity > 1 - normalSnappingThreshold then
                    faceNormal2 = globalSnappingAxis2
                  elseif similarity < -1 + normalSnappingThreshold then
                    faceNormal2 = -globalSnappingAxis2
                  else
                    similarity = faceNormal2 ^ globalSnappingAxis3
                    if similarity > 1 - normalSnappingThreshold then
                      faceNormal2 = globalSnappingAxis3
                    elseif similarity < -1 + normalSnappingThreshold then
                      faceNormal2 = -globalSnappingAxis3
                    end
                  end
                end
              end


              if rigidbody1.onCollision then
                rigidbody1.onCollision(rigidbody1, rigidbody2, faceNormal1, faceNormal2, verts)
              end
              if rigidbody2.onCollision then
                rigidbody2.onCollision(rigidbody2, rigidbody1, faceNormal1, faceNormal2, verts)
              end
              createConstraints(
                rigidbody1, rigidbody2,
                verts, faceNormal2, i2,
                true
              )

            end
          end
        end
      end










      --World Colissioj << "Colissioj" << ""Colissioj"" << """Colissioj""" << """"Colissioj"""" << """""Colissioj""""" << """"""Colissioj"""""" << """""""Colissioj""""""" << """"""""Colissioj"""""""" << """""""""Colissioj""""""""" << """"""""""Colissioj"""""""""" << """""""""""Colissioj""""""""""" << """"""""""""Colissioj""""""""""""
      for j, rigidbody in lnext, rigidbodies do
        if rigidbody.worldCollision and rigidbody.boundingAABB and rigidbody.linearMovement and not rigidbody.isSleeping then
          local aabbs = getWorldColliders(rigidbody)

          for i, aabb in lnext, aabbs do
            if showWorldColliders then
              lineAABB(aabb.pos, aabb.halfDimensions)
            end
            local rtm = rigidbody.rotMat
            local axis = {
              rtm[1], rtm[2], rtm[3],
              v1, v2, v3,
            }
            local mtv, verts,i2,faceNormal2,faceNormal1 = doFineCollision(rigidbody, aabb, axis)

            --mtv = minimum translation vector i.e. direction of minimun interpenetration
            if mtv then
              if rigidbody.onWorldCollision then
                rigidbody.onWorldCollision(rigidbody, faceNormal2, faceNormal1, verts)
              end

              createConstraints(
                rigidbody, ltostring(aabb.pos),
                verts, faceNormal2, i2
              --imagine a ,false here(saves 1 instruction per rigidbody touching the ground, improvement: with a 6x5 wall and 2 physics steps a world_tick 601990 >> 601978(0.00002%!!!!!!!) instructions!!!!!!!! crazy stuff!!! insane )
              )
            end
          end
        end
      end








      --creating joint constraints
      for i, joint in lnext, joints do
        if (joint.rigidbody1.linearMovement or joint.rigidbody2.linearMovement) then
          local pos1, pos2 = (joint.pos1 * joint.rigidbody1.rotMat) + joint.rigidbody1.pos,
              (joint.pos2 * joint.rigidbody2.rotMat) +
              joint.rigidbody2.pos -- local coords into wrold
          local penetration = -length(pos1 - pos2) 
          if penetration > -joint.minDistance then
            penetration = joint.minDistance + penetration
          elseif penetration < -joint.maxDistance then
            penetration = joint.maxDistance + penetration
          else
            penetration = 0
          end

          local rigidbody1 = joint.rigidbody1
          if rigidbody1.isInLink then
            rigidbody1 = rigidbody1.currentLink
          end
          local rigidbody2 = joint.rigidbody2
          if rigidbody2.isInLink then
            rigidbody2 = rigidbody2.currentLink
          end
          if joint.type == "distance" then
            constraints[#constraints + 1] = {
              rigidbody1 = rigidbody1,
              rigidbody2 = rigidbody2,
              bodyTwo = true,
              contactPoint = pos1,
              contactPoint2 = pos2, --in "wrold" space
              normal = -normalized(pos2 - pos1),
              type = "distance",
              impulse = 0,
              joint = true,
              baumgarte = true,
              penetration = penetration,
            }
            if joint.onPhysicsStep then
              joint.onPhysicsStep(joint, penetration)
            end
            if showJoints then
              createLine(pos1, pos2, vec3(1, 0, 0))
            end
          elseif joint.type == "spring" then
            local dir = normalized(pos2 - pos1)
            local rigidbody1, rigidbody2 = joint.rigidbody1, joint.rigidbody2
            local d1, d2 = (pos1 - rigidbody1.pos), (pos2 - rigidbody2.pos)
          local penetration = -length(pos1 - pos2) 
          if penetration > -joint.minDistance then
            penetration = joint.minDistance + penetration
          elseif penetration < -joint.maxDistance then
            penetration = joint.maxDistance + penetration
          else
            penetration = 0
          end
            local springForce = (joint.stiffness * (penetration) - ((rigidbody2.vel + (rigidbody2.rotVel% d1)) - (rigidbody1.vel + (rigidbody1.rotVel% d2)) ^ dir)) *
                dir * 0.9
            joint.rigidbody1:addForceAtPoint(pos1, -springForce)
            joint.rigidbody2:addForceAtPoint(pos1, springForce)
            if joint.onPhysicsStep then
              joint.onPhysicsStep(joint, penetration)
            end
            if showJoints then
              createLine(pos1, pos2, vec3(0, 1, 0))
            end
          end
        end
      end

















      for i, constraint in lnext, constraints do
        local rigidbody1, rigidbody2 = constraint.rigidbody1, constraint.rigidbody2
          local d1, d2 = (constraint.contactPoint - rigidbody1.pos)


          if constraint.bodyTwo then
            if not constraint.joint then
              d2 = (constraint.contactPoint - rigidbody2.pos)
            else
              d2 = (constraint.contactPoint2 - rigidbody2.pos)
            end
          end


          if constraint.bodyTwo then
            local RaCrossN, RbCrossN = (d1% (constraint.normal)), (d2% (constraint.normal))
            local r1InertiaCrossed = RaCrossN * rigidbody1.invInertiaTensorWORLD
            local r2InertiaCrossed = RbCrossN * rigidbody2.invInertiaTensorWORLD
            local inertiaTerm1 = (r1InertiaCrossed ^ (RaCrossN))
            local inertiaTerm2 = (r2InertiaCrossed ^ (RbCrossN))
            constraint.invEffMass = -1 / (rigidbody1.invMass + rigidbody2.invMass + inertiaTerm1 + inertiaTerm2)
            constraint.d1 = d1
            constraint.d2 = d2
            constraint.r1InertiaCrossed = r1InertiaCrossed
            constraint.r2InertiaCrossed = r2InertiaCrossed
            constraint.r1normalInvmass = constraint.normal * rigidbody1.invMass
            constraint.r2normalInvmass = constraint.normal * rigidbody2.invMass
 
            --baumgarte calc
          else
            
            local RaCrossN = (d1% (constraint.normal))
            local inertiaTerm1 = (RaCrossN * rigidbody1.invInertiaTensorWORLD) ^ (RaCrossN)
            constraint.invEffMass = -1 / (rigidbody1.invMass + inertiaTerm1)
            --baumgarte calc
            constraint.d1 = d1
            constraint.r1InertiaCrossed = rigidbody1.invInertiaTensorWORLD * RaCrossN
            constraint.r1normalInvmass = constraint.normal * rigidbody1.invMass
          end
          if constraint.baumgarte then
            local bias = constraint.penetration --math.max(penetration,0)
            if bias < 0 and not constraint.joint then
              bias = 0
            else
              bias = bias * baumgarteOverDt
            end
            if constraint.joint then
              bias = constraint.penetration * jointBaumgarteOverDt
            end
            constraint.bias = bias
          else
            constraint.bias = 0
          end

          if constraint.cached then
            rigidbody1.vel = rigidbody1.vel + constraint.impulse * constraint.r1normalInvmass
            rigidbody1.rotVel = rigidbody1.rotVel +
            constraint.r1InertiaCrossed * constraint.impulse
            if constraint.bodyTwo then
              rigidbody2.vel = rigidbody2.vel - constraint.impulse * constraint.r2normalInvmass
              rigidbody2.rotVel = rigidbody2.rotVel -
              constraint.r2InertiaCrossed * constraint.impulse
            end
          end
      end






      --now time for pgs

     if solver == "regularPGS" then
        for j = 1, physicsSim.velocityIterations do
          for i, constraint in lnext, constraints do
            if not constraint.joint then
              if constraint.bodyTwo then
                local rigidbody1, rigidbody2 = constraint.rigidbody1, constraint.rigidbody2
                local r1vel, r2vel = rigidbody1.vel, rigidbody2.vel
                local r1rotVel, r2rotVel = rigidbody1.rotVel, rigidbody2.rotVel



                local relativeVelocity = constraint.normal^((r1vel + (r1rotVel% constraint.d1)) - (r2vel + (r2rotVel% constraint.d2)))
                local lambda = constraint.invEffMass * (relativeVelocity - constraint.bias) --impulse calc

                local oldImpulse = constraint.impulse

                local totalImpulse = oldImpulse + (lambda)
                if constraint.friction then
                  local maxFriction = constraint.friction *
                      constraint.normalConstraint
                      .impulse -- for thos?e curious why in the most critical part of code there is a huge indexxing thing going on, well its faster(in instructions at least) trust
                  if totalImpulse > maxFriction then
                    totalImpulse = maxFriction
                  elseif totalImpulse < -maxFriction then
                    totalImpulse = -maxFriction
                  end
                else
                  if totalImpulse < 0 then
                    totalImpulse = 0
                  end
                end
                constraint.impulse = totalImpulse
                local impulse = (totalImpulse - oldImpulse)
                rigidbody1.vel = r1vel + impulse * constraint.r1normalInvmass
                rigidbody1.rotVel = r1rotVel +
                    (constraint.r1InertiaCrossed * impulse)

                rigidbody2.vel = r2vel - impulse * constraint.r2normalInvmass
                rigidbody2.rotVel = r2rotVel -
                    (constraint.r2InertiaCrossed * impulse)
              else






                local rigidbody1 = constraint.rigidbody1
                local r1vel = rigidbody1.vel
                local r1rotVel = rigidbody1.rotVel


                local relativeVelocity = constraint.normal ^ (r1vel + (r1rotVel% constraint.d1))
                local lambda = constraint.invEffMass * (relativeVelocity -
                  constraint.bias) --impulse calc


                local oldImpulse = constraint.impulse
                local totalImpulse = oldImpulse + (lambda)
                if constraint.friction then
                  local maxFriction = constraint.friction * constraint.normalConstraint.impulse
                  if totalImpulse > maxFriction then
                    totalImpulse = maxFriction
                  elseif totalImpulse < -maxFriction then
                    totalImpulse = -maxFriction
                  end
                else
                  if totalImpulse < 0 then
                    totalImpulse = 0
                  end
                end
                constraint.impulse = totalImpulse
                local impulse = (totalImpulse - oldImpulse)
                rigidbody1.vel = r1vel + constraint.r1normalInvmass * impulse
                rigidbody1.rotVel = r1rotVel +
                    (constraint.r1InertiaCrossed * impulse)
              end
            elseif constraint.type == "distance" then
              local rigidbody1, rigidbody2 = constraint.rigidbody1, constraint.rigidbody2
              local r1vel, r2vel = rigidbody1.vel, rigidbody2.vel
              local r1rotVel, r2rotVel = rigidbody1.rotVel, rigidbody2.rotVel
              local relativeVelocity = constraint.normal^
                ((r1vel + (r1rotVel% constraint.d1)) - (r2vel + (r2rotVel% constraint.d2)))
              local lambda = constraint.invEffMass * (relativeVelocity) -
                  constraint.invEffMass *
                  constraint.bias --impulse calc
              local oldImpulse = constraint.impulse
              constraint.impulse = oldImpulse + lambda

              local P = constraint.normal * lambda
              rigidbody1.vel = r1vel + P * rigidbody1.invMass
              rigidbody1.rotVel = r1rotVel + (constraint.r1InertiaCrossed * lambda)
              rigidbody2.vel = r2vel - P * rigidbody2.invMass
              rigidbody2.rotVel = r2rotVel - (constraint.r2InertiaCrossed * lambda)
            end
          end
        end

        for i, constraint in lnext, constraints do
          local constraintType = constraint.type
          if constraintType == "contact" then
            cacheAdd(constraint.rigidbody1.index, constraint.rigidbody2.index, constraint.edges, constraint.normalIndex,
              constraint.impulse)
          elseif constraintType == "contactWorld" then
            cacheAdd(constraint.rigidbody1.index, ltostring(constraint.rigidbody2), constraint.edges,
              constraint.normalIndex,
              constraint.impulse)
          end
          if showContacts then
            createLine(constraint.contactPoint, constraint.contactPoint + constraint.impulse * constraint.normal*1000,
              vec3(1))
          end
          --particles:newParticle("end_rod", constraint.contactPoint)
        end










        --split impulse but its not split impulse and its not that uber
      elseif solver == "scuffedSplitImpulse" then
        for j = 1, physicsSim.velocityIterations do
          for i, constraint in lnext, constraints do
            if not constraint.joint then
              if constraint.bodyTwo then
                local rigidbody1, rigidbody2 = constraint.rigidbody1, constraint.rigidbody2
                local r1vel, r2vel = rigidbody1.vel, rigidbody2.vel
                local r1rotVel, r2rotVel = rigidbody1.rotVel, rigidbody2.rotVel


                local relativeVelocity = constraint.normal^
                  ((r1vel + (r1rotVel% constraint.d1)) - (r2vel + (r2rotVel% constraint.d2)))
                local lambda = constraint.invEffMass * (relativeVelocity) --impulse calc

                local oldImpulse = constraint.impulse

                local totalImpulse = oldImpulse + (lambda)
                if constraint.friction then
                  local maxFriction = constraint.friction *
                      constraint.normalConstraint
                      .impulse -- for thos curious why in the most critical part of code there is a huge indexxing thing going on, well its faster(in instructions at least) trust
                  if totalImpulse > maxFriction then
                    totalImpulse = maxFriction
                  elseif totalImpulse < -maxFriction then
                    totalImpulse = -maxFriction
                  end
                else
                  if totalImpulse < 0 then
                    totalImpulse = 0
                  end
                end
                constraint.impulse = totalImpulse
                local impulse = (totalImpulse - oldImpulse)
                rigidbody1.vel = r1vel + impulse * constraint.r1normalInvmass
                rigidbody1.rotVel = r1rotVel +
                    (constraint.r1InertiaCrossed * impulse)

                rigidbody2.vel = r2vel - impulse * constraint.r2normalInvmass
                rigidbody2.rotVel = r2rotVel -
                    (constraint.r2InertiaCrossed * impulse)
              else
                local rigidbody1 = constraint.rigidbody1
                local r1vel = rigidbody1.vel
                local r1rotVel = rigidbody1.rotVel


                local relativeVelocity = constraint.normal ^ ((r1vel + (r1rotVel% constraint.d1)))
                local lambda = constraint.invEffMass * (relativeVelocity) --impulse calc


                local oldImpulse = constraint.impulse
                local totalImpulse = oldImpulse + (lambda)
                if constraint.friction then
                  local maxFriction = constraint.friction * constraint.normalConstraint.impulse
                  if totalImpulse > maxFriction then
                    totalImpulse = maxFriction
                  elseif totalImpulse < -maxFriction then
                    totalImpulse = -maxFriction
                  end
                else
                  if totalImpulse < 0 then
                    totalImpulse = 0
                  end
                end
                constraint.impulse = totalImpulse
                local impulse = (totalImpulse - oldImpulse)
                rigidbody1.vel = r1vel + constraint.r1normalInvmass * impulse
                rigidbody1.rotVel = r1rotVel +
                    (constraint.r1InertiaCrossed * impulse)
              end
            elseif constraint.type == "distance" then
              local rigidbody1, rigidbody2 = constraint.rigidbody1, constraint.rigidbody2
              local r1vel, r2vel = rigidbody1.vel, rigidbody2.vel
              local r1rotVel, r2rotVel = rigidbody1.rotVel, rigidbody2.rotVel
              local relativeVelocity = constraint.normal ^
                (r1vel + (r1rotVel% constraint.d1)) - (r2vel + (r2rotVel% constraint.d2))
              local lambda = constraint.invEffMass * (relativeVelocity) -
                  constraint.invEffMass *
                  constraint.bias --impulse calc
              local oldImpulse = constraint.impulse
              constraint.impulse = oldImpulse + lambda

              local P = constraint.normal * lambda
              rigidbody1.vel = r1vel + P * rigidbody1.invMass
              rigidbody1.rotVel = r1rotVel + (constraint.r1InertiaCrossed * lambda)
              rigidbody2.vel = r2vel - P * rigidbody2.invMass
              rigidbody2.rotVel = r2rotVel - (constraint.r2InertiaCrossed * lambda)
            end
          end
        end

        for i, constraint in lnext, constraints do
          local constraintType = constraint.type
          if constraintType == "contact" then
            local rigidbody1, rigidbody2 = constraint.rigidbody1, constraint.rigidbody2
            local impulse = -constraint.invEffMass * constraint.bias
            cacheAdd(rigidbody1.index, rigidbody2.index, constraint.edges, constraint.normalIndex, constraint.impulse)
            local P = constraint.normal * impulse
                rigidbody1.vel = rigidbody1.vel + impulse * constraint.r1normalInvmass
                rigidbody1.rotVel = rigidbody1.rotVel +
                    (constraint.r1InertiaCrossed * impulse)

                rigidbody2.vel = rigidbody2.vel - impulse * constraint.r2normalInvmass
                rigidbody2.rotVel = rigidbody2.rotVel -
                    (constraint.r2InertiaCrossed * impulse)
          elseif constraintType == "contactWorld" then
            local rigidbody1 = constraint.rigidbody1
            cacheAdd(rigidbody1.index, ltostring(constraint.rigidbody2), constraint.edges, constraint.normalIndex,
              constraint.impulse)
            local impulse = -constraint.invEffMass * constraint.bias
                rigidbody1.vel = rigidbody1.vel + constraint.r1normalInvmass * impulse
                rigidbody1.rotVel = rigidbody1.rotVel +
                    (constraint.r1InertiaCrossed * impulse)
          end
          if showContacts then
            createLine(constraint.contactPoint, constraint.contactPoint + constraint.impulse * constraint.normal * 1000,
              vec3(1))
          end
          --particles:newParticle("end_rod", constraint.contactPoint)
        end
      end













      --integration and stuff
      for j, rigidbody in lnext, rigidbodies do
        if not rigidbody.isInLink then
        if rigidbody.onPhysicsStep then
          rigidbody.onPhysicsStep(rigidbody)
        end

        if isSleepAllowed and length(rigidbody.vel) < sleepVelocityThreshold and length(rigidbody.rotVel) < sleepRotVelocityThreshold then
          rigidbody.sleepTimer = rigidbody.sleepTimer + dt
          if rigidbody.sleepTimer >= sleepTimeThreshold then
            rigidbody.isSleeping = true
          end
        else
          rigidbody.sleepTimer = 0
          rigidbody.isSleeping = false
        end

        if not rigidbody.isSleeping then
          --apply gravity
          rigidbody:addForce(1 / rigidbody.invMass * rigidbody.gravity * dt)
          --apply bouyancy
          local applyWaterDamping
          if rigidbody.worldCollision then
            local diplacedVolume = 0
            local totalDepth = 0
            if rigidbody.type ~= "particle" then
              local min, max = rigidbody.pos - rigidbody.halfDimensions, rigidbody.pos + rigidbody.halfDimensions
              local fmin, fmax = min:floor(), max:floor()
              for x = fmin.x, fmax.x do
                for y = fmin.y, fmax.y do
                  for z = fmin.z, fmax.z do
                    local waterLevel = ltonumber((worldGetBlockState(x, y, z)).properties.level)
                    local waterLevel2 = ltonumber((worldGetBlockState(x, y + 1, z)).properties.level)
                    if waterLevel and waterHeight[waterLevel] then
                      local maxBlockY = y + waterHeight[waterLevel]
                      if waterLevel2 then
                        maxBlockY = y + 1
                      end
                      local minBlock = vec3(x, y, z)
                      local maxBlock = vec3(x + 1, maxBlockY, z + 1)
                      local cmin = vclamp(copy(min), minBlock, maxBlock)
                      local cmax = vclamp(copy(max), minBlock, maxBlock)
                      local dims = (cmax - cmin)
                      local pos = cmin + dims * 0.5

                      diplacedVolume = diplacedVolume + dims.x * dims.y * dims.z
                      if showWaterVolumes then
                        lineAABB(pos, dims * 0.5)
                      end
                    end
                  end
                end
              end



              if diplacedVolume > 0 then
                applyWaterDamping = true
                local n = 0
                local pointsToApplyForce = {}
                for i, vert in lnext, rigidbody.vertices do
                  local block = (worldGetBlockState(vert)).properties

                  if block.level then
                    local depth = vert.y - math_floor(vert.y) - waterHeight[ltonumber(block.level)]
                    local d = 1
                    while (worldGetBlockState(vert + vec3(0, d))).properties.level do
                      d = d + 1
                    end
                    depth = depth - d + 1
                    if depth < 0 then
                      n = n + 1
                      totalDepth = totalDepth + depth
                      pointsToApplyForce[n] = { vert, depth }
                    end
                  end
                end
                for i, point in lnext, pointsToApplyForce do
                  rigidbody:addForceAtPoint(point[1], -waterDensity * diplacedVolume * rigidbody.gravity *
                    (point[2] / totalDepth))
                end
              end
            else

            end
          end
          --give rigidbody default engine forces

          if applyWaterDamping then
            rigidbody.vel = (rigidbody.vel + rigidbody.forceAccum * rigidbody.invMass * dt) *
                (waterDamping ^ oneOverPhysicsIters)
            if rigidbody.type ~= "particle" then
              rigidbody.rotVel = rigidbody.rotVel +
                  (rigidbody.torqueAccum * rigidbody.invInertiaTensorWORLD) * dt * (waterDamping ^ oneOverPhysicsIters)
            end
          else
            rigidbody.vel = (rigidbody.vel + rigidbody.forceAccum * rigidbody.invMass * dt) *
                (rigidbody.damping ^ oneOverPhysicsIters)
            if rigidbody.type ~= "particle" then
              rigidbody.rotVel = rigidbody.rotVel +
                  (rigidbody.torqueAccum * rigidbody.invInertiaTensorWORLD) * dt *
                  (rigidbody.damping ^ oneOverPhysicsIters)
            end
          end
          --semi implicit euler integrator


          rigidbody.forceAccum = copy(zeroVec)
          rigidbody.torqueAccum = copy(zeroVec)

          --update pos
          --pos more like
          --physician hah hahhahahahahhah I so funny soo sooooo soooooooo funny bro
          --update previous pos and rotation for rendering

          if i == 1 then
          if rigidbody.updatePrevPos then
            rigidbody.prevPos = rigidbody.pos
          end
          if rigidbody.updatePrevRot then
            rigidbody.prevRot = rigidbody.rot
          end
          end

          
          if rigidbody.linearMovement then
            rigidbody.pos = rigidbody.pos + rigidbody.vel * dt
            rigidbody.rot = normalize4(rigidbody.rot + (qultiply(rigidbody.rot, -rigidbody.rotVel._xyz * halfdt)))
          else
            rigidbody.vel = copy(zeroVec)
            rigidbody.rotVel = copy(zeroVec)
            rigidbody.invInertiaTensorLOCAL = matCopy(nullMat)
            rigidbody.invMass = 0
            normalize4(rigidbody.rot)
          end

          if rigidbody.type ~= "particle" then
            rigidbody.rotVel = rigidbody.rotVel * rigidbody.rotationDamping ^ oneOverPhysicsIters

            rigidbody.rotMat = q_u_otationMatrix3(rigidbody.rot) --get it? haha, "q_u_otation" quote -> quotation

            rigidbody.invInertiaTensorWORLD = rigidbody.rotMat * rigidbody.invInertiaTensorLOCAL *
                (rigidbody.rotMat):transposed()
          else
            rigidbody.rotMat = mat3()
          end
          if showAxis then
            createLine(rigidbody.pos, rigidbody.pos + vec3(1) * rigidbody.rotMat * 5, vec3(1))
            createLine(rigidbody.pos, rigidbody.pos + vec3(0, 1) * rigidbody.rotMat * 5, vec3(1))
            createLine(rigidbody.pos, rigidbody.pos + vec3(0, 0, 1) * rigidbody.rotMat * 5, vec3(1))
          end
        end
          if rigidbody.type == "cuboid" or rigidbody.type == "particle" then
        recalculateVertices(rigidbody)
        rigidbody.boundingAABB = calculateBoundingAABBCuboid(rigidbody.vertices)
          elseif rigidbody.type == "sphere" then
            local temp = vec3()+rigidbody.radius
            rigidbody.boundingAABB = temp
            rigidbody.halfDimensions = temp
          elseif rigidbody.type == "capsule" then
            local offset = rigidbody.length * rigidbody.rotMat[2] * 0.5
            local point1 = rigidbody.pos - offset
            local point2 = rigidbody.pos + offset
            local min,max = copy(nullVec), copy(nullVec)
            if point1.x > point2.x then
              max.x = point1.x
              min.x = point2.x
            else
              max.x = point2.x
              min.x = point1.x
            end
            if point1.y > point2.y then
              max.y = point1.y
              min.y = point2.y
            else
              max.y = point2.y
              min.y = point1.y
            end
            if point1.z > point2.z then
              max.z = point1.z
              min.z = point2.z
            else
              max.z = point2.z
              min.z = point1.z
            end
            min = min - rigidbody.radius
            max = max + rigidbody.radius
            rigidbody.boundingAABB = (max-min) * 0.5
            rigidbody.halfDimensions = vec3(0,rigidbody.length*0.5)+rigidbody.radius
          end
        if showBroadphaseAABB then
          lineAABB(rigidbody.pos, rigidbody.boundingAABB)
        end
      end
    end





















--Integration stuff for links       --integration and stuff
for k, link in lnext, links do




        if isSleepAllowed and length(link.vel) < sleepVelocityThreshold and length(link.rotVel) < sleepRotVelocityThreshold then
          link.sleepTimer = link.sleepTimer + dt
          if link.sleepTimer >= sleepTimeThreshold then
            link.isSleeping = true
          end
        else
          link.sleepTimer = 0
          link.isSleeping = false
        end

        if not link.isSleeping then




          --apply bouyancy


          --give rigidbody default engine forces
          






          local applyWaterDamping


        if i == 1 then
          if link.updatePrevPos then
            link.prevPos = link.pos
          end
          if link.updatePrevRot then
            link.prevRot = link.rot
          end
        end

          link:addForce(1 / link.invMass * link.gravity * dt)




          if applyWaterDamping then
            link.vel = (link.vel + link.forceAccum * link.invMass * dt) *
                (waterDamping ^ oneOverPhysicsIters)
            if link.type ~= "particle" then
              link.rotVel = link.rotVel +
                  (link.torqueAccum * link.invInertiaTensorWORLD) * dt * (waterDamping ^ oneOverPhysicsIters)
            end
          else
            link.vel = (link.vel + link.forceAccum * link.invMass * dt) *
                (link.damping ^ oneOverPhysicsIters)
            if link.type ~= "particle" then
              link.rotVel = link.rotVel +
                  (link.torqueAccum * link.invInertiaTensorWORLD) * dt *
                  (link.damping ^ oneOverPhysicsIters)
            end
          end
          link.forceAccum = copy(zeroVec)
          link.torqueAccum = copy(zeroVec)
          if link.linearMovement then
            link.pos = link.pos + link.vel * dt
            link.rot = normalize4(link.rot + (qultiply(link.rot, -link.rotVel._xyz * halfdt)))
          else
            link.vel = copy(zeroVec)
            link.rotVel = copy(zeroVec)
            link.invInertiaTensorLOCAL = matCopy(nullMat)
            link.invMass = 0
            normalize4(link.rot)
          end
          link.rotMat = q_u_otationMatrix3(link.rot) 

      for j, rigidbody in lnext, link.rigidbodies do

        if rigidbody.onPhysicsStep then
          rigidbody.onPhysicsStep(rigidbody)
        end


          --apply gravity


          --semi implicit euler integrator


          rigidbody.forceAccum = copy(zeroVec)
          rigidbody.torqueAccum = copy(zeroVec)

          --update pos
          --pos more like
          --physician hah hahhahahahahhah I so funny soo sooooo soooooooo funny bro
          --update previous pos and rotation for rendering






          if rigidbody.worldCollision then
            local diplacedVolume = 0
            local totalDepth = 0
            if rigidbody.type ~= "particle" then
              local min, max = rigidbody.pos - rigidbody.halfDimensions, rigidbody.pos + rigidbody.halfDimensions
              local fmin, fmax = min:floor(), max:floor()
              for x = fmin.x, fmax.x do
                for y = fmin.y, fmax.y do
                  for z = fmin.z, fmax.z do
                    local waterLevel = ltonumber((worldGetBlockState(x, y, z)).properties.level)
                    local waterLevel2 = ltonumber((worldGetBlockState(x, y + 1, z)).properties.level)
                    if waterLevel and waterHeight[waterLevel] then
                      local maxBlockY = y + waterHeight[waterLevel]
                      if waterLevel2 then
                        maxBlockY = y + 1
                      end
                      local minBlock = vec3(x, y, z)
                      local maxBlock = vec3(x + 1, maxBlockY, z + 1)
                      local cmin = vclamp(copy(min), minBlock, maxBlock)
                      local cmax = vclamp(copy(max), minBlock, maxBlock)
                      local dims = (cmax - cmin)
                      local pos = cmin + dims * 0.5

                      diplacedVolume = diplacedVolume + dims.x * dims.y * dims.z
                      if showWaterVolumes then
                        lineAABB(pos, dims * 0.5)
                      end
                    end
                  end
                end
              end



              if diplacedVolume > 0 then
                applyWaterDamping = true
                local n = 0
                local pointsToApplyForce = {}
                for i, vert in lnext, rigidbody.vertices do
                  local block = (worldGetBlockState(vert)).properties

                  if block.level then
                    local depth = vert.y - math_floor(vert.y) - waterHeight[ltonumber(block.level)]
                    local d = 1
                    while (worldGetBlockState(vert + vec3(0, d))).properties.level do
                      d = d + 1
                    end
                    depth = depth - d + 1
                    if depth < 0 then
                      n = n + 1
                      totalDepth = totalDepth + depth
                      pointsToApplyForce[n] = { vert, depth }
                    end
                  end
                end
                for i, point in lnext, pointsToApplyForce do
                  link:addForceAtPoint(point[1], -waterDensity * diplacedVolume * link.gravity/4 *
                    (point[2] / totalDepth))
                end
              end
            else

            end
          end




          if i == 1 then
          if rigidbody.updatePrevPos then
            rigidbody.prevPos = rigidbody.pos
          end
          if rigidbody.updatePrevRot then
            rigidbody.prevRot = rigidbody.rot
          end
          end

          

            rigidbody.vel = copy(zeroVec)
            rigidbody.pos = link.pos + rigidbody.deltaPos * link.rotMat
            rigidbody.rot = normalize4(rigidbody.rot + (qultiply(rigidbody.rot, -link.rotVel._xyz * halfdt)))
            rigidbody.pos = (rigidbody.deltaPos * rigidbody.currentLink.rotMat) + rigidbody.currentLink.pos 

          if rigidbody.type ~= "particle" then

            rigidbody.rotMat = q_u_otationMatrix3(rigidbody.rot) --get it? haha, "q_u_otation" quote -> quotation

            rigidbody.invInertiaTensorWORLD = rigidbody.rotMat * rigidbody.invInertiaTensorLOCAL *
                (rigidbody.rotMat):transposed()
          else
            rigidbody.rotMat = mat3()
          end
          if showAxis then
            createLine(rigidbody.pos, rigidbody.pos + vec3(1) * rigidbody.rotMat * 5, vec3(1))
            createLine(rigidbody.pos, rigidbody.pos + vec3(0, 1) * rigidbody.rotMat * 5, vec3(1))
            createLine(rigidbody.pos, rigidbody.pos + vec3(0, 0, 1) * rigidbody.rotMat * 5, vec3(1))
          end
          if rigidbody.type == "cuboid" or rigidbody.type == "particle" then
        recalculateVertices(rigidbody)
        rigidbody.boundingAABB = calculateBoundingAABBCuboid(rigidbody.vertices)
          elseif rigidbody.type == "sphere" then
            rigidbody.boundingAABB = vec3()+rigidbody.radius
          elseif rigidbody.type == "capsule" then
            local offset = rigidbody.length * rigidbody.rotMat[2] * 0.5
            local point1 = rigidbody.pos - offset
            local point2 = rigidbody.pos + offset
            local min,max = copy(nullVec), copy(nullVec)
            if point1.x > point2.x then
              max.x = point1.x
              min.x = point2.x
            else
              max.x = point2.x
              min.x = point1.x
            end
            if point1.y > point2.y then
              max.y = point1.y
              min.y = point2.y
            else
              max.y = point2.y
              min.y = point1.y
            end
            if point1.z > point2.z then
              max.z = point1.z
              min.z = point2.z
            else
              max.z = point2.z
              min.z = point1.z
            end
            min = min - rigidbody.radius
            max = max + rigidbody.radius
            rigidbody.boundingAABB = (max-min) * 0.5
          end
        if showBroadphaseAABB then
          lineAABB(rigidbody.pos, rigidbody.boundingAABB)
        end


      end







      end


  end































    end

    for i, rigidbody in lnext, rigidbodies do
      if rigidbody.onTick then
        rigidbody.onTick(rigidbody)
      end
    end
    for i, joint in pairs(joints) do
      if joint.onTick then
        joint.onTick(joint)
      end
    end

    --[[if #rigidbodies ~= 0 then
      tic = tic + 1
      inst[avg] = avatar:getCurrentInstructions()
      tims[avg] = client:getSystemTime() - timA
      if avg > 50 then
        avg = 1
      else
        avg = avg + 1
      end

      if tic == 150 then
      local sum,sum2 = 0, 0
      for i, num in pairs(inst) do
        sum2 = sum2 + tims[i]
        sum = sum + num
      end
        log(sum / #inst, sum2/#tims)
      end
    end]]
  else
    for i, rigidbody in pairs(rigidbodies) do
      rigidbody.prevPos = rigidbody.pos
      rigidbody.prevRot = rigidbody.rot
    end
  end
end
--[[
tic = 1
inst = {}
tims = {}
avg = 1
tim = 1]]





-- changes:
--[[
 - restructured main loop for creating contact points
 - removed the collisionNormals debug visual as it is useless 
 - Moved all contact point creation code to `Fine Collision` 
 - removed 2 arguments from the
 - idk when but apparently i changed the placeholder texture

]]
