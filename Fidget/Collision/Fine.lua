local Fidget = require("Fidget.FidgetSetup")

local math_abs = math.abs
local math_clamp = math.clamp
local vec3 = vectors.vec3
local mat3 = matrices.mat3
local v1 = vec3(1)
local v2 = vec3(0, 1)
local v3 = vec3(0, 0, 1)
local dot3 = vec3().dot
local lnext = next
local unpack = vec3().unpack
local normalize = vec3().normalize
local normalized = vec3().normalized
local crossed3 = vec3().crossed
local dot = vec3().dot
local transposed = mat3().transposed
local err = 0.000001
local copy = vec3().copy
local length = vec3().length
local lengthSquared = vec3().lengthSquared
local performFullSAT = true
local nullVec = vec3()
local identMat = mat3(vec3(1), vec3(0, 1), vec3(0, 0, 1))
local minusIdentMat = identMat * -1
local contactPointMergingThreshold = 1
local capsuleBudget = 7
local contactDeletionTable = { --deleting useless:TM: contacts(deleting in a way that aims for maximized area)
  [5] = { 1 },
  [6] = { 2, 5 },
  [7] = { 2, 4, 6 },
  [8] = { 1, 3, 5, 7 },
}

figuraMetatables.Vector3.__pow = dot
figuraMetatables.Vector3.__mod = crossed3



function events.world_tick()
  contactPointMergingThreshold = Fidget.physicsSim.contactPointMergingThreshold
  capsuleBudget = Fidget.physicsSim.capsuleCollisionIterationBudget
end
--sort=
--[[
1 - cuboid
2 - particle
3 - sphere
4 - capsule

]]
--sutherland badgerman or some shit like that polygon clipping(used for generatinh contact points)
local function clipPolygonAgainstPlane(verts, planeNormal, planeDist, index)
  local contactPoints = {}
  local count = #verts
  local n = 1
  for i = 1, count do
    local currentVertex = verts[i][3]
    local nextVertex = verts[(i % count) + 1][3]

    local currentDist = (planeNormal^currentVertex) - planeDist
    local nextDist = (planeNormal^nextVertex) - planeDist

    local currentInside = currentDist >= 0
    local nextInside = nextDist >= 0

    if currentInside and nextInside then
      contactPoints[n] = { i, [3] = nextVertex }
      n = n + 1
    elseif not nextInside and currentInside then
      local intersection = currentVertex + (nextVertex - currentVertex) * (currentDist / (currentDist - nextDist))
      contactPoints[n] = { i, index, intersection }
      n = n + 1
    elseif not currentInside and nextInside then
      local intersection = currentVertex + (nextVertex - currentVertex) * (currentDist / (currentDist - nextDist))
      contactPoints[n] = { i, index, intersection }
      contactPoints[n + 1] = { i, [3] = nextVertex }
      n = n + 2
    end
  end

  return contactPoints
end


local function closestPointOnObbToPoint(obb,point)
  local pos = obb.pos
  local obbdims = obb.halfDimensions
  local obbrotmat = obb.rotMat or identMat
  obbrotmat = obbrotmat
  local x = math_clamp((point - pos)^obbrotmat[1],-obbdims.x,obbdims.x)
  local y = math_clamp((point - pos)^obbrotmat[2],-obbdims.y,obbdims.y)
  local z = math_clamp((point - pos)^obbrotmat[3],-obbdims.z,obbdims.z) 



return  pos + x * obbrotmat[1] + y * obbrotmat[2] + z * obbrotmat[3]
end

local function closestPointOnLineToPoint(startPoint,endPoint,point)
local ab = endPoint - startPoint
local t = ((point-startPoint)^ab)/lengthSquared(ab)
if t < 0 then 
t = 0
elseif t > 1 then
t = 1
end

return startPoint + t * ab
end

local function closestPointsOnLines(p0, p1, q0, q1)
    local u = p1 - p0
    local v = q1 - q0
    local w = p0 - q0

    local a = u:dot(u)        -- always >= 0
    local b = u:dot(v)
    local c = v:dot(v)        -- always >= 0
    local d = u:dot(w)
    local e = v:dot(w)

    local D = a*c - b*b       -- always >= 0
    local sc, sN, sD = 0, 0, D
    local tc, tN, tD = 0, 0, D

    local EPS = 1e-8

    -- handle parallel case
    if D < EPS then
        sN = 0
        sD = 1
        tN = e
        tD = c
    else
        sN = (b*e - c*d)
        tN = (a*e - b*d)

        if sN < 0 then
            sN = 0
            tN = e
            tD = c
        elseif sN > sD then
            sN = sD
            tN = e + b
            tD = c
        end
    end

    if tN < 0 then
        tN = 0
        if -d < 0 then
            sN = 0
        elseif -d > a then
            sN = sD
        else
            sN = -d
            sD = a
        end
    elseif tN > tD then
        tN = tD
        if (-d + b) < 0 then
            sN = 0
        elseif (-d + b) > a then
            sN = sD
        else
            sN = (-d + b)
            sD = a
        end
    end

    sc = (math.abs(sN) < EPS) and 0 or (sN / sD)
    tc = (math.abs(tN) < EPS) and 0 or (tN / tD)

    local closestP = p0 + u * sc
    local closestQ = q0 + v * tc

    return closestP, closestQ
end











local rigidbodyTypeSortingTable = {
  ["cuboid"] = 1,
  ["particle"] = 2,
  ["sphere"] = 3,
  ["capsule"] = 4,
}
local fineCollision = {}

--https://gamma.cs.unc.edu/users/gottschalk/main.pdf
function events.world_tick()
  performFullSAT = Fidget.physicsSim.performFullSAT
end

function doFineCollision(rigidbody1, rigidbody2, separatingAxis)
  if rigidbodyTypeSortingTable[rigidbody1.type] > rigidbodyTypeSortingTable[rigidbody2.type] then
    rigidbody1, rigidbody2 = rigidbody2, rigidbody1
  end
  local collisionType = rigidbody1.type .. rigidbody2.type
  --1-3 >> axis1
  --4-6 >> axis2
  --7-15>> edges --> banished to the shadow realm(edges are genareted with face face colission)

  return fineCollision[collisionType](rigidbody1, rigidbody2, separatingAxis)
end

function cuboidcuboid(rigidbody1, rigidbody2, separatingAxis)
  local minIndex = -1
  local minimumTranslationVector
  local minimumTranslationDistance = 694202137000
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

  for i, axis in lnext, separatingAxis do   -- not many people know about this way to write for loops
    local s, r1, r2
    if i <= 3 then
      s = (delta ^ axis)
      r1 = rdims1[i]
      r2 = (rdims2x * math_abs((separatingAxis[4] ^ axis)) + rdims2y * math_abs((separatingAxis[5] ^ axis)) + rdims2z * math_abs((separatingAxis[6] ^ axis)))
    elseif i <= 6 then
      s = (delta ^ axis)
      r1 = (rdims1x * math_abs((separatingAxis[1] ^ axis)) + rdims1y * math_abs((separatingAxis[2] ^ axis)) + rdims1z * math_abs((separatingAxis[3] ^ axis)))
      r2 = rdims2[i - 3]
    else
      if length(axis) == 0 then
        goto leEdgeIsSoSmallIts___Its___UHHHH_WHATISIT__ItsLiterallyAllZeros
      end
      s = (delta ^ axis)
      r1 = (rdims1x * math_abs((separatingAxis[1] ^ axis)) + rdims1y * math_abs((separatingAxis[2] ^ axis)) + rdims1z * math_abs((separatingAxis[3] ^ axis)))
      r2 = (rdims1x * math_abs((separatingAxis[1] ^ axis)) + rdims1y * math_abs((separatingAxis[2] ^ axis)) + rdims1z * math_abs((separatingAxis[3] ^ axis)))
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
    local maxminimumTranslationVector, maxIndex = dot3(axis, (separatingAxis[1])), 1
    if minIndex > 6 then
      for i = 2, 6 do
        local a = dot3(axis, (separatingAxis[i]))
        local b = a
        if a < 0 then
          a = -a
        end
        if maxminimumTranslationVector < 0 then
          maxminimumTranslationVector = -maxminimumTranslationVector
        end
        if maxminimumTranslationVector < a then
          maxminimumTranslationVector = b
          maxIndex = i
        end
      end
      minIndex = maxIndex
      --this is bs this is not correct but hopefully it worksTM
      minimumTranslationVector = separatingAxis[minIndex] * (minimumTranslationDistance)
    end
  end



  if minimumTranslationVector then
    local i1, i2
    local faceNormal1, faceNormal2
    if minIndex > 0 then
      local whichToSearch = 1 --added to the index later to see which cuboid to search for matching faces

      if minIndex >= 4 then
        faceNormal2 = separatingAxis[minIndex]
        i2 = minIndex
      else
        whichToSearch = 4
        faceNormal1 = separatingAxis[minIndex]
        i1 = minIndex
      end




      local bestMatch = 0
      local bestNormal
      local bestIndex = -694202137000
      for i = whichToSearch, 2 + whichToSearch do
        local d = separatingAxis[i] ^ -minimumTranslationVector
        if d < 0 then
          d = -d
        end
        if d > bestMatch then
          bestMatch = d
          bestNormal = separatingAxis[i]
          bestIndex = i
        end
      end
      if whichToSearch == 4 then
        i2 = bestIndex
        faceNormal2 = -bestNormal
      else
        i1 = bestIndex
        faceNormal1 = bestNormal
      end




      local proj1 = rigidbody1.pos ^ minimumTranslationVector
      local proj2 = rigidbody2.pos ^ minimumTranslationVector


      if proj1 > proj2 then
        minimumTranslationVector = -minimumTranslationVector
      end
      if faceNormal1 ^ minimumTranslationVector < 0 then
        faceNormal1 = -faceNormal1
      end
      if faceNormal2 ^ minimumTranslationVector > 0 then
        faceNormal2 = -faceNormal2
      end
    else
      faceNormal1 = normalized(minimumTranslationVector)
      faceNormal2 = normalized(-minimumTranslationVector)
      i2 = -minIndex
    end



      --get le axis to get some vertices of the face
  --i1 and i2 are indexes of the axis of collision i think
  local reducedVerts = {}
    local halfDims1            = rigidbody1.halfDimensions
    local halfDims2            = rigidbody2.halfDimensions
    local index1, index2       = i1 % 3 + 1, (i1 - 2) % 3 + 1
    local theAxisInQuestion1R1 = separatingAxis[index1] * halfDims1[index1]
    local theAxisInQuestion2R1 = separatingAxis[index2] * halfDims1[index2]

    local index1, index2       = (i2 - 3) % 3 + 1, (i2 - 5) % 3 + 1
    local theAxisInQuestion1R2 = separatingAxis[index1 + 3] * halfDims2[index1]
    local theAxisInQuestion2R2 = separatingAxis[index2 + 3] * halfDims2[index2]

    local middleOfFace         = rigidbody1.pos + faceNormal1 * halfDims1[i1]
    local a, b                 = theAxisInQuestion1R1 + theAxisInQuestion2R1,
        theAxisInQuestion1R1 - theAxisInQuestion2R1
    local verts                = {
      { [3] = middleOfFace + a },
      { [3] = middleOfFace + b },
      { [3] = middleOfFace - a },
      { [3] = middleOfFace - b },
    }



    local lpos = rigidbody2.pos
    local n1, n2 = theAxisInQuestion1R2, theAxisInQuestion2R2


    --[[
  local tempPoints = {
    lpos + theAxisInQuestion1R2,
    lpos - theAxisInQuestion1R2,
    lpos + theAxisInQuestion2R2,
    lpos - theAxisInQuestion2R2,
  }
  local sidePlanes = {
    { n1,  dot(tempPoints[2], n1) },
    { -n1, dot(tempPoints[1], -n1) },
    { n2,  dot(tempPoints[4], n2) },
    { -n2, dot(tempPoints[3], -n2) },
  }

  for i, plane in lnext, sidePlanes do
    verts = clipPolygonAgainstPlane(verts, plane[1], plane[2])
  end]]
    --[[absolute shitfest of readibility pretty much i replace the verts with the functions
  verts = clipPolygonAgainstPlane(verts, n1, dot(lpos - theAxisInQuestion1R2, n1))
  verts = clipPolygonAgainstPlane(verts, -n1, dot(lpos + theAxisInQuestion1R2, -n1))
  verts = clipPolygonAgainstPlane(verts, n2, dot(lpos - theAxisInQuestion2R2, n2))
  verts = clipPolygonAgainstPlane(verts, -n2, dot(lpos + theAxisInQuestion2R2, -n2))
]]

    verts =
        clipPolygonAgainstPlane(
          clipPolygonAgainstPlane(
            clipPolygonAgainstPlane(
              clipPolygonAgainstPlane(
                verts, n1, ((lpos - theAxisInQuestion1R2) ^ n1))
              , -n1, ((lpos + theAxisInQuestion1R2) ^ -n1))
            , n2, ((lpos - theAxisInQuestion2R2) ^ n2))
          , -n2, ((lpos + theAxisInQuestion2R2) ^ -n2))


    local somethingidkhowtonamethisshit = (rigidbody2.pos + faceNormal2 * halfDims2[(i2 - 4) % 3 + 1]) ^ faceNormal2
    for i, vert in lnext, verts do
      local penetration = somethingidkhowtonamethisshit - (vert[3] ^ faceNormal2)
      if penetration >= 0 then
        vert[4] = penetration
      else
        verts[i] = nil
      end
    end
    local n = #verts
    if n >= 5 then
      for i, index in lnext, contactDeletionTable[n] do
        verts[index] = nil
      end
    end



    for i, vert in lnext, verts do
      local tooClose = false
      for j, vert2 in lnext, reducedVerts do
        if length(vert[3] - vert2[3]) < contactPointMergingThreshold then
          tooClose = true
          break
        end
      end
      if not tooClose then -- an actual coherant sentence
        n = n + 1
        reducedVerts[n] = vert
      end
    end

   -- n = n + 1
   -- reducedVerts[n] = { 0, 0, rigidbody1.type == "particle" and rigidbody1.pos or rigidbody2.pos, penetration }



    return minimumTranslationVector, reducedVerts, i2, faceNormal2, faceNormal1

  end


end

function cuboidparticle(rigidbody1, rigidbody2, separatingAxis)
  local aabb = {
    -(rigidbody2.halfDimensions),
    (rigidbody2.halfDimensions),
  }

  local localSpaceR1 = (rigidbody2.pos - rigidbody1.pos) *
  (rigidbody2.type and rigidbody2.rotMat:transposed() or identMat)

  if localSpaceR1 > aabb[1] and localSpaceR1 < aabb[2] then
    local penetration = -math_abs(localSpaceR1.x) + rigidbody2.halfDimensions.x
    minimumTranslationVector = (localSpaceR1.x < 0 and -v1 or v1)
    if -math_abs(localSpaceR1.y) + rigidbody2.halfDimensions.y < penetration then
      penetration = -math_abs(localSpaceR1.y) + rigidbody2.halfDimensions.y
      minimumTranslationVector = (localSpaceR1.y < 0 and -v2 or v2)
      minIndex = -2
    end
    if -math_abs(localSpaceR1.z) + rigidbody2.halfDimensions.z < penetration then
      penetration = -math_abs(localSpaceR1.z) + rigidbody2.halfDimensions.z
      minimumTranslationVector = (localSpaceR1.z < 0 and -v3 or v3)
      minIndex = -3
    end


    minimumTranslationVector = minimumTranslationVector * (rigidbody2.type and rigidbody2.rotMat or identMat) *
    (penetration)



        local i1, i2
    local faceNormal1, faceNormal2
    if minIndex > 0 then
      local whichToSearch = 1 --added to the index later to see which cuboid to search for matching faces

      if minIndex >= 4 then
        faceNormal2 = separatingAxis[minIndex]
        i2 = minIndex
      else
        whichToSearch = 4
        faceNormal1 = separatingAxis[minIndex]
        i1 = minIndex
      end




      local bestMatch = 0
      local bestNormal
      local bestIndex = -694202137000
      for i = whichToSearch, 2 + whichToSearch do
        local d = separatingAxis[i] ^ -minimumTranslationVector
        if d < 0 then
          d = -d
        end
        if d > bestMatch then
          bestMatch = d
          bestNormal = separatingAxis[i]
          bestIndex = i
        end
      end
      if whichToSearch == 4 then
        i2 = bestIndex
        faceNormal2 = -bestNormal
      else
        i1 = bestIndex
        faceNormal1 = bestNormal
      end




      local proj1 = rigidbody1.pos ^ minimumTranslationVector
      local proj2 = rigidbody2.pos ^ minimumTranslationVector


      if proj1 > proj2 then
        minimumTranslationVector = -minimumTranslationVector
      end
      if faceNormal1 ^ minimumTranslationVector < 0 then
        faceNormal1 = -faceNormal1
      end
      if faceNormal2 ^ minimumTranslationVector > 0 then
        faceNormal2 = -faceNormal2
      end
    else
      faceNormal1 = normalized(minimumTranslationVector)
      faceNormal2 = normalized(-minimumTranslationVector)
      i2 = -minIndex
    end


   local verts = {{ 0, 0, rigidbody1.pos, penetration }}
    return minimumTranslationVector, verts, i2, faceNormal2, faceNormal1
  end
end


function cuboidsphere(rigidbody1,rigidbody2)
local point = closestPointOnObbToPoint(rigidbody1,rigidbody2.pos)
if length(point - rigidbody2.pos) < rigidbody2.radius then
  local minimumTranslationVector = -(point - rigidbody2.pos) + normalized(point - rigidbody2.pos)*rigidbody2.radius
  local faceNormal2 = normalized(minimumTranslationVector)
    if rigidbody1.index and rigidbody1.index < rigidbody2.index  then
    faceNormal2 = - faceNormal2
  end
  return minimumTranslationVector, {{ 0, 0, point , length(minimumTranslationVector) }}, 1, -faceNormal2, faceNormal2
end
end

function spheresphere(rigidbody1,rigidbody2)
local delta = rigidbody1.pos - rigidbody2.pos
if length(delta) < rigidbody1.radius + rigidbody2.radius then
local point = rigidbody1.pos - normalize(delta)*rigidbody1.radius
local point2 = rigidbody2.pos + delta*rigidbody2.radius
local minimumTranslationVector = point - point2


  return minimumTranslationVector, {{ 0, 0, point , length(minimumTranslationVector) }}, 1, delta, -delta
end
end

function particlesphere(rigidbody1,rigidbody2)
local delta = rigidbody1.pos - rigidbody2.pos
if length(delta) < rigidbody1.radius + rigidbody2.radius then
local point = rigidbody1.pos 
local point2 = rigidbody2.pos + normalize(delta)*rigidbody2.radius
local minimumTranslationVector = point - point2


  return minimumTranslationVector, {{ 0, 0, point , length(minimumTranslationVector) }}, 1, delta, -delta
end
end

function cuboidcapsule(rigidbody1,rigidbody2)
  local offset = rigidbody2.length * rigidbody2.rotMat[2] * 0.5
  local point1 = rigidbody2.pos - offset
  local point2 = rigidbody2.pos + offset

  local n = capsuleBudget
  local startpoint = copy(point1)
  local cubePoint
  local verts = {}
  local mtv
  local faceNormal2 
  while n > capsuleBudget/2 do
    cubePoint = closestPointOnObbToPoint(rigidbody1,startpoint)
    startpoint = closestPointOnLineToPoint(point1,point2,cubePoint)
    n = n - 1
    if length(cubePoint - startpoint) < rigidbody2.radius then
      local mt = (cubePoint - startpoint):normalize() * rigidbody2.radius -  (cubePoint - startpoint)
      faceNormal2 = mt:normalized()
      verts[1] = {0,0, cubePoint, length(mt)}
      mtv = true
      break
    end
  end

  startpoint =copy(point2)
  while n > 0 do
    cubePoint = closestPointOnObbToPoint(rigidbody1,startpoint)
    startpoint = closestPointOnLineToPoint(point1,point2,cubePoint)
    n = n - 1
    if length(cubePoint - startpoint) < rigidbody2.radius then
      local mt = (cubePoint - startpoint):normalized() * rigidbody2.radius -  (cubePoint - startpoint)
      faceNormal2 = mt:normalized()

      verts[2] = {0,0, cubePoint, length(mt)}
      mtv = true
      break
    end
  end
  if faceNormal2 and rigidbody1.index and rigidbody1.index < rigidbody2.index  then
    faceNormal2 = - faceNormal2
  end
  
  return mtv, verts, 1,faceNormal2 and -faceNormal2 or nil,faceNormal2

  end

  function capsulecapsule(rigidbody1,rigidbody2)
  local offset = rigidbody1.length * rigidbody1.rotMat[2] * 0.5
  local point1 = rigidbody1.pos - offset
  local point2 = rigidbody1.pos + offset
  offset = rigidbody2.length * rigidbody2.rotMat[2] * 0.5
  local point3 = rigidbody2.pos - offset
  local point4 = rigidbody2.pos + offset
  point1, point2 = closestPointsOnLines(point1,point2,point3,point4)
  local delta = point1 - point2
if length(delta) < rigidbody1.radius + rigidbody2.radius then
local point = point1 - normalize(delta)*rigidbody1.radius
local point2 = point2 + delta*rigidbody2.radius
local minimumTranslationVector = point - point2


  return minimumTranslationVector, {{ 0, 0, point , length(minimumTranslationVector) }}, 1, delta, -delta
end
  end

  function spherecapsule(rigidbody1,rigidbody2)
    local point1 = rigidbody1.pos
  local offset = rigidbody2.length * rigidbody2.rotMat[2] * 0.5
  local point3 = rigidbody2.pos - offset
  local point4 = rigidbody2.pos + offset
  local point2 =  closestPointOnLineToPoint(point3,point4,point1)
    local delta = point1 - point2
if length(delta) < rigidbody1.radius + rigidbody2.radius then
local point = point1 - normalize(delta)*rigidbody1.radius
local point2 = point2 + delta*rigidbody2.radius
local minimumTranslationVector = point - point2
  return minimumTranslationVector, {{ 0, 0, point , length(minimumTranslationVector) }}, 1, -delta, delta
end
  end

  function particlecapsule(rigidbody1,rigidbody2)
    local point1 = rigidbody1.pos
  local offset = rigidbody2.length * rigidbody2.rotMat[2] * 0.5
  local point3 = rigidbody2.pos - offset
  local point4 = rigidbody2.pos + offset
  local point2 =  closestPointOnLineToPoint(point3,point4,point1)
    local delta = point1 - point2
if length(delta) <  rigidbody2.radius then
local point = point1
local point2 = point2 + normalize(delta)*rigidbody2.radius
local minimumTranslationVector = point - point2
  return minimumTranslationVector, {{ 0, 0, point , length(minimumTranslationVector) }}, 1, -delta, delta
end
  end

fineCollision.cuboidcuboid = cuboidcuboid
fineCollision.cuboidparticle = cuboidparticle
fineCollision.cuboidsphere = cuboidsphere
fineCollision.spheresphere = spheresphere
fineCollision.particlesphere = particlesphere
fineCollision.cuboidcapsule = cuboidcapsule
fineCollision.capsulecapsule = capsulecapsule
fineCollision.spherecapsule = spherecapsule
fineCollision.particlecapsule = particlecapsule
