function doBroadPhaseCollision(rigidbodies, collisionType)
  --for now just a nested loop, maybe later it will be a grid or use bvh
  -- figura is so slow i can simulate like 30 bodies at most so no bvh:(

    local potentialCollisions = {}
    for i, rigidbody1 in pairs(rigidbodies) do
      for j = i + 1, #rigidbodies do
        local rigidbody2 = rigidbodies[j]
        if rigidbody1.currentLink and rigidbody2.currentLink then
          if rigidbody1.currentLink.index == rigidbody2.currentLink.index then
            goto HiIDontCollideWithYouBro
          end
        end
        if rigidbody2.collisionBlacklist and rigidbody2.collisionBlacklist[rigidbody1.index] then
          goto HiIDontCollideWithYouBro
        end
        if rigidbody1.collisionBlacklist and rigidbody1.collisionBlacklist[rigidbody2.index] then
          goto HiIDontCollideWithYouBro
        end
        local noParticleFlag =  true
        local type1,type2 = rigidbody1.type == "particle", rigidbody2.type == "particle"
        if not (type1 and type2) then 
        if type1 then
          noParticleFlag = false
        elseif type2 then
          noParticleFlag = false
          rigidbody1, rigidbody2 = rigidbody2, rigidbody1
        end
        if (rigidbody1.bodyCollision and rigidbody2.bodyCollision) and (rigidbody1.linearMovement or rigidbody2.linearMovement) and not (rigidbody1.isSleeping and rigidbody2.isSleeping) and noParticleFlag then
          local pos1, pos2 = rigidbody1.pos, rigidbody2.pos
          local max1 = pos1 + rigidbody1.boundingAABB
          local min1 = pos1 - rigidbody1.boundingAABB
          local max2 = pos2 + rigidbody2.boundingAABB
          local min2 = pos2 - rigidbody2.boundingAABB

          if max1 > min2 and max2 > min1 then
            potentialCollisions[i] = potentialCollisions[i] or {}
            potentialCollisions[i][j] = true
          end
        elseif not noParticleFlag then
          if rigidbody1.pos < rigidbody2.pos + rigidbody2.boundingAABB and rigidbody1.pos > rigidbody2.pos - rigidbody2.boundingAABB then
          potentialCollisions[i] = potentialCollisions[i] or {}
          potentialCollisions[i][j] = true
          end
        end
        end
        ::HiIDontCollideWithYouBro::
      end
    end
    return potentialCollisions

end
