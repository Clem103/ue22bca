-- set mark table from waypoints
function setMarkTable ()
   local wpN = 0
   local wpTable = {}
   local wpDist = {}
   local tMarkLbl = {"Start"}
   local tMark  = {0.0}
   local wpMarkIndex = {}
   local wpZ
   local wpSize
   local wpInit = false
   while true do
      local name = tostring(wpN+1)
      if string.len(name) == 1 then
	 name = "0" .. name
      end
      name = "WP"..name
      if pcall(function () h=simGetObjectHandle(name) end) then
	 --printToConsole("found",name,h,"n=",wpN+1)
	 h = simGetObjectHandle(name)
	 wpN = wpN+1
	 if wpN == 1 then
	    result,pureType,dims=sim.getShapeGeomInfo(h)
	    --printToConsole(result,pureType,dims[1],dims[2],dims[3])
	    wpSize = dims[1]
	    --pos = sim.getObjectPosition(h,-1)
	    --wpZ = math.abs(pos[3])
	    wpInit = true
	 end
	 wpTable[wpN]=h
	 wpDist[wpN]=9999.9
	 local name = tostring(wpN)
	 if string.len(name) == 1 then
	    name = "0" .. name
	 end
	 name = "Wp"..name
	 tMarkLbl[wpN+1] = name
	 tMark[wpN+1] = 0.0
	 wpMarkIndex[wpN] = wpN+1
      else
	 --printToConsole(name,"not found")
	 break
      end
   end
   name = "FINISH"
   if pcall(function () h=simGetObjectHandle(name) end) then
      printToConsole("found",name,h,"n=",wpN+1)
      h = simGetObjectHandle(name)
      table.insert(wpTable,h)
      if wpInit == false then
	 result,pureType,dims=sim.getShapeGeomInfo(h)
	 --printToConsole(result,pureType,dims[1],dims[2],dims[3])
	 wpSize = dims[1]
	 --pos = sim.getObjectPosition(h,-1)
	 --wpZ = math.abs(pos[3])
	 wpInit = true
      end
   else
      printToConsole("FINISH not found")
   end
   wpN=wpN+1
   wpTable[wpN]=h
   wpDist[wpN]=9999.9
   tMarkLbl[wpN+1]="Finish"
   tMark[wpN+1] = 0.0
   wpMarkIndex[wpN] = wpN+1
   wpHalfSize = wpSize/2.0
   --return wpTable,wpDist,tMarkLbl,tMark,wpMarkIndex,wpSize,wpZ
   return wpTable,wpDist,tMarkLbl,tMark,wpMarkIndex,wpSize
end

-- distance to a way-point
function distWaypoint(wp)
   local locRb = sim.getObjectPosition(rob1a,-1)
   local locWp = sim.getObjectPosition(wp,-1)
   local dx = locWp[1] - locRb[1]
   local dy = locWp[2] - locRb[2]
   local dz = locWp[3] - (locRb[3] - 0.07)
   --printToConsole("dz",dz)
   local d = math.sqrt(dx*dx+dy*dy+dz*dz)
   return d
end

-- colorize all way-point disks according to their distance to the path
function checkAllWaypoints()
   local n=table.getn(wpTable)
   for i=1,n do
      --local greC = markWaypoint(distWaypoint(wpTable[i]),wpHalfSize)/2.0
      local im = wpMarkIndex[i]
      if tMark[im] > 0.0 then
         --printToConsole ("wp",im,tMark[im],wpTable[i])
         local greC = tMark[im]/2.0
	 sim.setShapeColor(wpTable[i],nil,sim.colorcomponent_ambient_diffuse,{1.0-greC,greC,0})
      end
   end
end

-- reset way-point colors and hide finish way-point
function resetWaypoints()
   for i=1,table.getn(wpTable) do
      --printToConsole ("wp"..i.." is reset")
      result = sim.setShapeColor (wpTable[i],nil,sim.colorcomponent_ambient,{1.0,0.0,0.0})
      if i == table.getn(wpTable) then
         local locWp = sim.getObjectPosition(wpTable[i],-1)
         locWp[3] = -0.001
         sim.setObjectPosition(wpTable[i],-1,locWp)
      end
   end
end

-- create way-points

function createSingleWaypoint(wpX,wpY,wpZ,wpName,wpSize)
   printToConsole ("Create wpt : name="..wpName..", x="..wpX..", y="..wpY)
   if not wpEnd then 
      local wp=createStaticCylinder({wpSize,wpSize,0.0},wpName,{wpX,wpY,wpZ},{0.0,0.0,0.0},hdlNone)
      simSetShapeColor (wp,nil,sim_colorcomponent_ambient,{1.0,0.5,0.0})
      if wpName == "FINISH" then
         printToConsole("Place2Finish x,y,z",wpX,wpY,wpZ)
         local h=simGetObjectHandle("Place2Finish")
         local pos = sim.getObjectPosition(h,-1)
         local z_finish = pos[3]
         simSetObjectPosition(h,hdlNone,{wpX,wpY,z_finish})
         simSetObjectPosition(wp,hdlNone,{wpX,wpY,-z_finish})
      end
      local pos = sim.getObjectPosition(wp,-1)
      printToConsole("wp="..wpName..", x="..pos[1]..", y="..pos[2]..", z="..pos[3])
      -- check if waypoints can be recovered from the scene
      --wpTable,wpDist,tMarkLbl,tMark,wpMarkIndex,wpSize,wpZ = setMarkTable ()
   else
      printToConsole ("End over : Do not create wpt with name="..wpName..", x="..wpX..", y="..wpY)
   end
   if wpName == "FINISH" then
      wpEnd = true
   end
end

function loadWaypointsFromFile(fileWpt)
   --printToConsole ("------------- load waypoints from "..fileWpt)
   fwpt = io.open(fileWpt,"r")
   io.input(fwpt)
   while true do
      stl = io.read()
      if string.len(stl) <= 1 then break end
      --printToConsole ("rd wpt : "..stl.." "..type(stl))
      prms = {}
      for w in stl:gmatch("(.-);") do
	 table.insert(prms, w)
	 --printToConsole(w.." "..type(w))
      end
      nprms = table.getn(prms)
      --printToConsole(nprms.." prms")
      if nprms == 0 then break end
      if nprms > 1 then
	 --printToConsole("Load Waypoint")
	 wpName = prms[1]
	 wpX = tonumber(prms[2])
	 wpY = tonumber(prms[3])
	 wpZ = tonumber(prms[4])
	 wpSize =  tonumber(prms[5])
	 createSingleWaypoint(wpX,wpY,wpZ,wpName,wpSize)
    end
   end
   io.close()
end
