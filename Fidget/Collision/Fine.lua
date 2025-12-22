local Fidget = require("Fidget.FidgetSetup")

local math_abs = math.abs
local vec3 = vectors.vec3
local mat3 = matrices.mat3
local v1 = vec3(1)
local v2 = vec3(0, 1)
local v3 = vec3(0, 0, 1)
local dot3 = vec3().dot
local lnext = next
local unpack = vec3().unpack
local normalize = vec3().normalize
local crossed3 = vec3().crossed
local transposed = mat3().transposed
local err = 0.000001
local length = vec3().length
local performFullSAT = true
local nullVec = vec3()
local identMat = mat3(vec3(1), vec3(0, 1), vec3(0, 0, 1))
--https://gamma.cs.unc.edu/users/gottschalk/main.pdf
function events.world_tick()
  performFullSAT = Fidget.physicsSim.performFullSAT
end

function doFineCollision(rigidbody1, rigidbody2, separatingAxis)
  local minIndex = -1
  local minimumTranslationVector
  local minimumTranslationDistance = 694202137000
  local noParticleFlag = true

  --1-3 >> axis1
  --4-6 >> axis2
  --7-15>> edges --> banished to the shadow realm(edges are genareted with face face colission)
  if rigidbody1.type == "particle" then
    noParticleFlag = false

  elseif rigidbody2.type == "particle" then
    noParticleFlag = false
    rigidbody1, rigidbody2 = rigidbody2, rigidbody1
  end

  if noParticleFlag then
    local delta = rigidbody2.pos - rigidbody1.pos
    if performFullSAT then
      local l = 7
      for i = 1, 3 do
        for j = 4, 6 do
          separatingAxis[l] = normalize(crossed3(separatingAxis[i], separatingAxis[j]))
          l = l + 1
        end
      end
    end

    local rdims1 = rigidbody1.halfDimensions
    local rdims2 = rigidbody2.halfDimensions
    local rdims1x, rdims1y, rdims1z = unpack(rdims1)
    local rdims2x, rdims2y, rdims2z = unpack(rdims2)

    for i, axis in lnext, separatingAxis do -- not many people know about this way to write for loops
      if length(axis) == 0 then
        goto leEdgeIsSoSmallIts___Its___UHHHH_WHATISIT__ItsLiterallyAllZeros
      end
      local s, r1, r2
      if i <= 3 then
        s = dot3(delta, axis)
        r1 = rdims1[i]
        r2 = (rdims2x * math_abs(dot3(separatingAxis[4], axis)) + rdims2y * math_abs(dot3(separatingAxis[5], axis)) + rdims2z * math_abs(dot3(separatingAxis[6], axis)))
      elseif i <= 6 then
        s = dot3(delta, axis)
        r1 = (rdims1x * math_abs(dot3(separatingAxis[1], axis)) + rdims1y * math_abs(dot3(separatingAxis[2], axis)) + rdims1z * math_abs(dot3(separatingAxis[3], axis)))
        r2 = rdims2[i - 3]
      else
        s = dot3(delta, axis)
        r1 = (rdims1x * math_abs(dot3(separatingAxis[1], axis)) + rdims1y * math_abs(dot3(separatingAxis[2], axis)) + rdims1z * math_abs(dot3(separatingAxis[3], axis)))
        r2 = (rdims2x * math_abs(dot3(separatingAxis[4], axis)) + rdims2y * math_abs(dot3(separatingAxis[5], axis)) + rdims2z * math_abs(dot3(separatingAxis[6], axis)))
      end

      local penetration = (r1 + r2) - (s < 0 and -s or s)

      if penetration <= 0 then
        return
      end

      if penetration < minimumTranslationDistance then
        minIndex = i
        minimumTranslationVector = (separatingAxis[minIndex] * (s < 0 and -1 or 1))
        minimumTranslationDistance = penetration
      end

      ::leEdgeIsSoSmallIts___Its___UHHHH_WHATISIT__ItsLiterallyAllZeros::
    end




    --Garbage ahhh shit turns out you do in fact need to check all 15 axis. fuck me
    if performFullSAT then
      local axis = separatingAxis[minIndex]
      local maxMTV, maxIndex = dot3(axis, (separatingAxis[1])), 1
      if minIndex > 6 then
        for i = 2, 6 do
          local a = dot3(axis, (separatingAxis[i]))
          local b = a
          if a < 0 then
            a = -a
          end
          if maxMTV < 0 then
            maxMTV = -maxMTV
          end
          if maxMTV < a then
            maxMTV = b
            maxIndex = i
          end
        end
        minIndex = maxIndex
        --this is bs this is not correct but hopefully it worksTM
        minimumTranslationVector = separatingAxis[minIndex] * (minimumTranslationDistance)
      end
    end
  else

    local aabb = {
      -(rigidbody2.halfDimensions),
      (rigidbody2.halfDimensions),
    }

    local localSpaceR1 = (rigidbody2.pos - rigidbody1.pos) * (rigidbody2.type and rigidbody2.rotMat:transposed() or identMat)
    
    if localSpaceR1 > aabb[1] and localSpaceR1 < aabb[2] then
      local penetration = -math.abs(localSpaceR1.x) + rigidbody2.halfDimensions.x
      minimumTranslationVector = (localSpaceR1.x < 0 and -v1 or v1)
      if -math.abs(localSpaceR1.y) + rigidbody2.halfDimensions.y < penetration then
        penetration = -math.abs(localSpaceR1.y) + rigidbody2.halfDimensions.y
        minimumTranslationVector = (localSpaceR1.y < 0 and -v2 or v2)
        minIndex = -2
      end
      if -math.abs(localSpaceR1.z) + rigidbody2.halfDimensions.z < penetration then
        penetration = -math.abs(localSpaceR1.z)+rigidbody2.halfDimensions.z
        minimumTranslationVector = (localSpaceR1.z < 0 and -v3 or v3)
        minIndex = -3
      end


      minimumTranslationVector = minimumTranslationVector *  (rigidbody2.type and rigidbody2.rotMat or identMat)  * (penetration)
    end
  end
  return minimumTranslationVector, minIndex
end
