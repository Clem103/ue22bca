
-- reset mark table
function resetMarks()
   for i=1,table.getn(tMark) do
      tMark[i] = 0.0
   end
end

-- compute the mark of a way_point
function markWaypoint(d,dok)
   local dokok = dok/2.0
   local dokmx = dok
   nt = 0.0
   if d <= (dokok) then
       nt = 2.0
   elseif d <= dokmx then
       nt = 2.0*math.pow(math.cos(0.5*math.pi*(d-dokok)/(dokmx-dokok)),2.0)
   else
       nt = 0.0
   end
   --printToConsole("mark",d,dok,dokok,dokmx,nt)
   return nt
end

-- compute the mark of the challenge
function computeMark()
   if motion then
      tMark[1] = 2
      for i=1,table.getn(wpTable) do
         d = distWaypoint(wpTable[i])
         if d < wpDist[i] then 
            wpDist[i] = d
            im = wpMarkIndex[i]
            tMark[im] = markWaypoint(d,wpHalfSize)
            --sim.addStatusbarMessage ("mark "..(i+1).." is "..tMark[1+i])
         end
      end
      -- make finish waypoint appearing if mark not 0
      local iEnd = wpMarkIndex[table.getn(wpTable)] -- last waypoint
      if tMark[iEnd] > 0.0 then
	 local hfin = simGetObjectHandle("Place2Finish")
	 local posfin = sim.getObjectPosition(hfin,-1)
	 local zfin = posfin[3]
         local locWp = sim.getObjectPosition(wpTable[table.getn(wpTable)],-1)
	 if locWp[3] < 0 then
	    --locWp[3] = zfin+math.abs(locWp[3])
	    locWp[3] = zfin+0.001
	 end
         sim.setObjectPosition(wpTable[table.getn(wpTable)],-1,locWp)
      end
   end
end
