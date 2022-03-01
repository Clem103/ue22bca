os.setlocale("C")

function makeObjectRenderable (h)
   simSetObjectSpecialProperty(h,sim.objectspecialproperty_renderable)
end

function makeObjetDetectableByUltrasonics (h)
   simSetObjectSpecialProperty(h,sim_objectspecialproperty_detectable_ultrasonic)
end

function setObjectVisibilityMask (h, msk)
   -- if msk=0 object non visible, if not 0 define bit mask visibility
   --simSetObjectInt32Parameter(h,sim.objintparam_visibility_layer,msk)
   simSetObjectInt32Parameter(h,sim.objintparam_visibility_layer,msk)
end

function makeObjectRespondable(handle)
   local localRespMask = 128
   local globalRespMask = 255
   respMask = localRespMask + 256*globalRespMask
   simSetObjectInt32Parameter(h,sim_shapeintparam_respondable_mask,respMask)
end

-- options to create a pure shape are bit-coded:
-- if bit0 is set (1), backfaces are culled
-- if bit1 is set (2), edges are visible
-- if bit2 is set (4), the shape appears smooth
-- if bit3 is set (8), the shape is respondable
-- if bit4 is set (16), the shape is static
-- if bit5 is set (32), the cylinder has open ends

function createStaticCylinder(size,name,posit,orient,hRef)
   local h,i,options,orientRad
   options = 1+2+4+16
   mass = 0.0
   h=simCreatePureShape(2,options,size,mass,nil)
   --printToConsole (size[1],size[2],size[3],posit[1],posit[2],posit[3],name)
   simSetObjectName(h,name)
   simSetObjectPosition(h,hRef,posit)
   orientRad={0.,0.,0.}
   for i=1,#orient do
      orientRad[i] = orient[i]*degRad
   end
   simSetObjectOrientation(h,hRef,orientRad)
   return h
end

function createStaticRespondableCylinder(size,name,posit,orient,hRef,localRespMask,globalRespMask)
   local h,i,options,repMask,orientRad
   options = 1+2+4+8+16
   mass = 0.0
   h=simCreatePureShape(2,options,size,mass,nil)
   --printToConsole (size[1],size[2],size[3],posit[1],posit[2],posit[3],name)
   simSetObjectName(h,name)
   simSetObjectPosition(h,hRef,posit)
   orientRad={0.,0.,0.}
   for i=1,#orient do
      orientRad[i] = orient[i]*degRad
   end
   simSetObjectOrientation(h,hRef,orientRad)
   respMask = localRespMask + 256*globalRespMask
   simSetObjectInt32Parameter(h,sim_shapeintparam_respondable_mask,respMask)
   return h
end

function createDynamicCylinder(size,name,posit,orient,hRef,localRespMask,globalRespMask,mass)
   local h,i,repMask,orientRad,options
   options = 1+2+4+8
   h=simCreatePureShape(2,options,size,mass,nil)
   --printToConsole (size[1],size[2],size[3],posit[1],posit[2],posit[3],name)
   simSetObjectName(h,name)
   simSetObjectPosition(h,hRef,posit)
   orientRad={0.,0.,0.}
   for i=1,#orient do
      orientRad[i] = orient[i]*degRad
   end
   simSetObjectOrientation(h,hRef,orientRad)
   respMask = localRespMask + 256*globalRespMask
   simSetObjectInt32Parameter(h,sim_shapeintparam_respondable_mask,respMask)
   return h
end

function createStaticCuboid(size,name,posit,orient,hRef)
   local h,i,options,orientRad
   options = 1+2+4+16
   mass = 0.0
   h=simCreatePureShape(0,options,size,mass,nil)
   --printToConsole (size[1],size[2],size[3],posit[1],posit[2],posit[3],name)
   simSetObjectName(h,name)
   simSetObjectPosition(h,hRef,posit)
   orientRad={0.,0.,0.}
   for i=1,#orient do
      orientRad[i] = orient[i]*degRad
   end
   simSetObjectOrientation(h,hRef,orientRad)
   return h
end

function createStaticRespondableCuboid(size,name,posit,orient,hRef,localRespMask,globalRespMask)
   local h,i,options,repMask,orientRad
   options = 1+2+4+8+16
   mass = 0.0
   --printToConsole ("cuboid "..name)
   h=simCreatePureShape(0,options,size,mass,nil)
   --printToConsole ("static respondable cuboid",size[1],size[2],size[3],posit[1],posit[2],posit[3],orient[1],orient[2],orient[3],name)
   simSetObjectName(h,name)
   --printToConsole ("cuboid 1")
   simSetObjectPosition(h,hRef,posit)
   --printToConsole ("cuboid 2")
   orientRad={0.,0.,0.}
   for i=1,#orient do
      orientRad[i] = orient[i]*degRad
   end
   --printToConsole ("cuboid orient",orientRad[1],orientRad[2],orientRad[3],name)
   simSetObjectOrientation(h,hRef,orientRad)
   respMask = localRespMask + 256*globalRespMask
   simSetObjectInt32Parameter(h,sim_shapeintparam_respondable_mask,respMask)
   return h
end

function createDynamicCuboid(size,name,posit,orient,hRef,localRespMask,globalRespMask,mass)
   local h,i,repMask,orientRad,options
   options = 1+2+4+8
   h=simCreatePureShape(0,options,size,mass,nil)
   --printToConsole (size[1],size[2],size[3],posit[1],posit[2],posit[3],name)
   simSetObjectName(h,name)
   simSetObjectPosition(h,hRef,posit)
   orientRad={0.,0.,0.}
   for i=1,#orient do
      orientRad[i] = orient[i]*degRad
   end
   simSetObjectOrientation(h,hRef,orientRad)
   respMask = localRespMask + 256*globalRespMask
   simSetObjectInt32Parameter(h,sim_shapeintparam_respondable_mask,respMask)
   return h
end

function defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
   cuboid_prms = {}
   table.insert( cuboid_prms,size)
   table.insert( cuboid_prms,wallname)
   table.insert( cuboid_prms,posit)
   table.insert( cuboid_prms,orient)
   table.insert( cuboid_prms,hRef)
   table.insert( cuboid_prms,localRespMask)
   table.insert( cuboid_prms,globalRespMask)
   return cuboid_prms
end

function create_floor(xc, yc, zc, length, width , thickness, wallWidth, wallHeight)
   hdlNone = -1
   size = {length+2.0*wallWidth,width+2.0*wallWidth,thickness}
   name = "Floor"
   posit = {xc,yc,zc-thickness/2.0}
   orient = {0.0,0.0,0.0}
   wf = createStaticRespondableCuboid(size,name,posit,orient,hdlNone,255,255)
   
   allExtWalls = {}

   -- wall north
   size = {length+2.0*wallWidth,wallWidth,wallHeight}
   name = "ExternalWallNorth"
   posit = {xc,yc+width/2.0+wallWidth/2.0,zc+wallHeight/2.0}
   orient = {0.0,0.0,0.0}
   wf = createStaticRespondableCuboid(size,name,posit,orient,hdlNone,1,255)
   table.insert(allExtWalls,wf)
   
   -- wall south
   size = {length+2.0*wallWidth,wallWidth,wallHeight}
   name = "ExternalWallSouth"
   posit = {xc,yc-width/2.0-wallWidth/2.0,zc+wallHeight/2.0}
   orient = {0.0,0.0,0.0}
   wf = createStaticRespondableCuboid(size,name,posit,orient,hdlNone,1,255)
   table.insert(allExtWalls,wf)
   
   -- wall east
   size = {width,wallWidth,wallHeight}
   name = "ExternalWallEast"
   posit = {xc+length/2.0+wallWidth/2.0,yc,zc+wallHeight/2.0}
   orient = {0.0,0.0,90.0}
   wf = createStaticRespondableCuboid(size,name,posit,orient,hdlNone,2,255)
   table.insert(allExtWalls,wf)
   
   -- wall west
   size = {width,wallWidth,wallHeight}
   name = "ExternalWallWest"
   posit = {xc-length/2.0-wallWidth/2.0,yc,zc+wallHeight/2.0}
   orient = {0.0,0.0,90.0}
   wf = createStaticRespondableCuboid(size,name,posit,orient,hdlNone,2,255)
   table.insert(allExtWalls,wf)

   -- group elementary shapes
   localRespMask=255
   globalRespMask=255
   wf = simGroupShapes(allExtWalls,table.getn(allExtWalls))
   simSetObjectName(wf,"ExternalWalls")
   makeObjetDetectableByUltrasonics (wf)   
end

function load_rob1a_vehicle(modelFile,robStartFilename)
   -- modelFile define in rob1a_create_scene_...lua
   local rob1a=simLoadModel(modelFile)
   local pos = simGetObjectPosition(rob1a,-1)
   printToConsole("Rob1A pos =  "..pos[1].." , "..pos[2].." , "..pos[3])

   local ftrk = io.open(robStartFilename,"r")
   io.input(ftrk)
   local stl = io.read()
   io.close()
   --printToConsole ("rd : "..stl.." "..type(stl))
   local prms = {}
   for w in stl:gmatch("(.-);") do
      table.insert(prms, w)
      --printToConsole(w.." "..type(w))
   end
   local nprms = table.getn(prms)
   --printToConsole(nprms.." prms")
   if nprms > 1 then
      local x_start = tonumber(prms[1])
      local y_start = tonumber(prms[2])
      local z_start = tonumber(prms[3])
      local theta_start =  tonumber(prms[4])
      simSetObjectPosition(rob1a,-1,{x_start,y_start,z_start})
      simSetObjectOrientation(rob1a,-1,{0.0,0.0,theta_start*degRad-math.pi/2.0})
      pos = simGetObjectPosition(rob1a,-1)
      printToConsole("Rob1A pos =  "..pos[1].." , "..pos[2].." , "..pos[3])
      -- set start at the location of the robot
      local h=simGetObjectHandle("Place2Start")
      local pos_start = sim.getObjectPosition(h,-1)
      -- z_start has already been set during the creation of the track (from file)
      z_start = pos_start[3]
      printToConsole("Update Place2Start x,y,z",x_start,y_start,z_start)
      simSetObjectPosition(h,-1,{x_start,y_start,z_start})
   end

   options = 0
   camViewRob=simFloatingViewAdd(0.825,0.825,0.25,0.25,options)
   camRob=sim.getObjectHandle("CamRob")
   options = 64
   rsu=simAdjustView(camViewRob, camRob,options,"Onboard Camera")

   options = 0
   camViewOut=simFloatingViewAdd(0.825,0.175,0.25,0.25,options)
   camOut=sim.getObjectHandle("CamOut")
   options = 64
   rsu=simAdjustView(camViewOut, camOut,options,"Camera Out")

   camView = simGetObjectHandle("DefaultCamera")
   obj = simGetObjectHandle("Floor")
   rsu=simCameraFitToView(camView,{obj},0,1.0)
end

function createCenterLineShape(x0,y0,z0,x1,y1,z1,hml,thetaRad,w2Midline,options,shadingAngle)
   -- printToConsole ("0: "..x0.." "..y0)
   -- printToConsole ("1: "..x1.." "..y1)
   local midlineVertices = {}
   local xaml = x0+w2Midline*math.cos(thetaRad)
   local yaml = y0+w2Midline*math.sin(thetaRad)
   local zaml = z0+hml
   local xdml = x0+w2Midline*math.cos(thetaRad+math.pi)
   local ydml = y0+w2Midline*math.sin(thetaRad+math.pi)
   local zdml = z0+hml
   local xbml = x1+w2Midline*math.cos(thetaRad)
   local ybml = y1+w2Midline*math.sin(thetaRad)
   local zbml = z1+hml
   local xcml = x1+w2Midline*math.cos(thetaRad+math.pi)
   local ycml = y1+w2Midline*math.sin(thetaRad+math.pi)
   local zcml = z1+hml
   -- printToConsole ("A: "..xaml.." "..yaml)
   -- printToConsole ("B: "..xbml.." "..ybml)
   -- printToConsole ("C: "..xcml.." "..ycml)
   -- printToConsole ("D: "..xdml.." "..ydml)
   table.insert(midlineVertices,xaml)
   table.insert(midlineVertices,yaml)
   table.insert(midlineVertices,zaml)
   table.insert(midlineVertices,xbml)
   table.insert(midlineVertices,ybml)
   table.insert(midlineVertices,zbml)
   table.insert(midlineVertices,xcml)
   table.insert(midlineVertices,ycml)
   table.insert(midlineVertices,zcml)
   table.insert(midlineVertices,xdml)
   table.insert(midlineVertices,ydml)
   table.insert(midlineVertices,zdml)  
   local midlineIndices = {0,1,2,0,2,3}
   hdlcl=simCreateMeshShape(options,shadingAngle,midlineVertices,midlineIndices)
   -- printToConsole ("hdlcl="..hdlcl)
   nCenterLine = nCenterLine+1
   simSetObjectName (hdlcl,"CenterLine"..num2str(3,nCenterLine))
   sim.setShapeColor (hdlcl,nil,sim.colorcomponent_ambient,{1.0,1.0,1.0})
   makeObjectRenderable (hdlcl)
   return hdlcl
end

function createTrackCenterLineLinear(name,length,theta,width,height,x0,y0,z0,z1in,cenlin)
   local hdl,thetaRad,h,l,w2,options,shadingAngle,vertices,indices
   local xa,ya,xb,yb,xc,yc,xd,yd,x1,y1,z1
   local options = 0*1+1*2  -- bit 0:backfaces culling (removal) , bit1: edges visible
   local shadingAngle = 0.0
   local thetaRad = theta * degRad
   local h = height
   local l = length
   local l2 = l/2
   local w = width
   local w2 = w/2

   local x1 = x0+l*math.cos(thetaRad+math.pi/2.0)
   local y1 = y0+l*math.sin(thetaRad+math.pi/2.0)
   local z1 = z1in

   local xc = x0+l2*math.cos(thetaRad+math.pi/2.0)
   local yc = y0+l2*math.sin(thetaRad+math.pi/2.0)
   local zc = (z0+z1in)/2

   local x2 = xc+w2*math.cos(thetaRad)
   local y2 = yc+w2*math.sin(thetaRad)
   local z2 = zc

   local x3 = xc+w2*math.cos(thetaRad+math.pi)
   local y3 = yc+w2*math.sin(thetaRad+math.pi)
   local z3 = zc
   
   printToConsole ("-----------------")
   printToConsole ("0: "..x0.." "..y0)
   printToConsole ("1: "..x1.." "..y1)
   printToConsole ("2: "..x2.." "..y2)
   printToConsole ("3: "..x3.." "..y3)
   printToConsole ("C: "..xc.." "..yc)
  
   local wMidline = 0.02
   local w2Midline = wMidline/2.0
   local hml = 0.0025 -- center line 2.5 mm above the base of the track
   
   printToConsole ("cenlin : "..cenlin)
   local centerLine = {}
   if cenlin == "SN" or cenlin == "NS" then 
      hdlcl = createCenterLineShape(x0,y0,z0,x1,y1,z1,hml,thetaRad,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
   end
   if cenlin == "SEW" or cenlin == "SWE" then 
      hdlcl = createCenterLineShape(x0,y0,z0,xc,yc,zc,hml,thetaRad,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
      hdlcl = createCenterLineShape(x2,y2,z2,x3,y3,z3,hml,thetaRad+math.pi/2.0,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
   end
   if cenlin == "NEW" or cenlin == "NWE" then 
      hdlcl = createCenterLineShape(x1,y1,z1,xc,yc,zc,hml,thetaRad,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
      hdlcl = createCenterLineShape(x2,y2,z2,x3,y3,z3,hml,thetaRad+math.pi/2.0,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
   end
   if cenlin == "WSN" or cenlin == "WNS" then 
      hdlcl = createCenterLineShape(x3,y3,z3,xc,yc,zc,hml,thetaRad+math.pi/2.0,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
      hdlcl = createCenterLineShape(x0,y0,z0,x1,y1,z1,hml,thetaRad,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
   end
   if cenlin == "ESN" or cenlin == "ENS" then 
      hdlcl = createCenterLineShape(x2,y2,z2,xc,yc,zc,hml,thetaRad+math.pi/2.0,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
      hdlcl = createCenterLineShape(x0,y0,z0,x1,y1,z1,hml,thetaRad,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
   end
   if cenlin == "SW" or cenlin == "WS" then 
      hdlcl = createCenterLineShape(x0,y0,z0,xc,yc,zc,hml,thetaRad,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
      hdlcl = createCenterLineShape(xc,yc,zc,x3,y3,z3,hml,thetaRad+math.pi/2.0,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
   end
   if cenlin == "SE" or cenlin == "ES" then 
      hdlcl = createCenterLineShape(x0,y0,z0,xc,yc,zc,hml,thetaRad,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
      hdlcl = createCenterLineShape(xc,yc,zc,x2,y2,z2,hml,thetaRad+math.pi/2.0,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
   end
   if cenlin == "NW" or cenlin == "WN" then 
      hdlcl = createCenterLineShape(x1,y1,z1,xc,yc,zc,hml,thetaRad,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
      hdlcl = createCenterLineShape(xc,yc,zc,x3,y3,z3,hml,thetaRad+math.pi/2.0,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
   end
   if cenlin == "NE" or cenlin == "EN" then 
      hdlcl = createCenterLineShape(x1,y1,z1,xc,yc,zc,hml,thetaRad,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
      hdlcl = createCenterLineShape(xc,yc,zc,x2,y2,z2,hml,thetaRad+math.pi/2.0,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
   end
    
   return centerLine
end

function createTrackCenterLineCurved(name,width,height,radius,theta0,theta1,x0,y0,z0,z1,wall)
end

function createTrackCenterLineDubins(name,width,height,x0,y0,z0,theta0,x1,y1,z1,theta1,wall)
   local hdl,thetaRad,h,l,w2,options,shadingAngle,vertices,indices
   local options = 0*1+1*2  -- bit 0:backfaces culling (removal) , bit1: edges visible
   local shadingAngle = 0.0
   local thetaRad = (0.5 * (theta0+theta1) +90.0) * degRad
   local h = height
   local w = width
   local w2 = w/2
  
   local wMidline = 0.02
   local w2Midline = wMidline/2.0
   local hml = 0.0025 -- center line 2.5 mm above the base of the track
   
   printToConsole ("cenlin : "..cenlin)
   local centerLine = {}
   -- in dubins mode , only NS is needed
   if cenlin == "SN" or cenlin == "NS" or cenlin == "X" then 
      --printToConsole("x1="..x1..", y1="..y1..", z1="..z1)
      hdlcl = createCenterLineShape(x0,y0,z0,x1,y1,z1,hml,thetaRad,w2Midline,options,shadingAngle)
      table.insert(centerLine,hdlcl)
   end
    
   return centerLine
end

function createTrackElementLinear(name,length,theta,width,height,x0,y0,z0,z1in,wall)
   local hdl,thetaRad,h,l,w2,options,shadingAngle,vertices,indices
   local xa,ya,xb,yb,xc,yc,xd,yd,x1,y1,z1
   local options = 0*1+1*2  -- bit 0:backfaces culling (removal) , bit1: edges visible
   local shadingAngle = 0.0
   local thetaRad = theta * degRad
   local h = height
   local l = length
   local w = width
   local w2 = w/2.0
   local xa = x0+w2*math.cos(thetaRad)
   local ya = y0+w2*math.sin(thetaRad)
   local xd = x0+w2*math.cos(thetaRad+math.pi)
   local yd = y0+w2*math.sin(thetaRad+math.pi)
   local xb = xa+l*math.cos(thetaRad+math.pi/2.0)
   local yb = ya+l*math.sin(thetaRad+math.pi/2.0)
   local xc = xd+l*math.cos(thetaRad+math.pi/2.0)
   local yc = yd+l*math.sin(thetaRad+math.pi/2.0)
   local x1 = (xb+xc)/2.0
   local y1 = (yb+yc)/2.0
   local z1 = z1in
   --printToConsole ("-----------------")
   --printToConsole ("A: "..xa.." "..ya)
   --printToConsole ("D: "..xd.." "..yd)
   --printToConsole ("B: "..xb.." "..yb)
   --printToConsole ("C: "..xc.." "..yc)
   
   local vertices = {}
   table.insert(vertices,xa)
   table.insert(vertices,ya)
   table.insert(vertices,z0+h)
   table.insert(vertices,xb)
   table.insert(vertices,yb)
   table.insert(vertices,z1+h)
   table.insert(vertices,xc)
   table.insert(vertices,yc)
   table.insert(vertices,z1+h)
   table.insert(vertices,xd)
   table.insert(vertices,yd)
   table.insert(vertices,z0+h)
   local indices = {0,1,2,0,2,3}
   local hdl=simCreateMeshShape(options,shadingAngle,vertices,indices)
   printToConsole("hdl="..hdl)
   local trkHdl = hdl
   simSetObjectName(trkHdl,name)
   local trkWall = {}
   local hRef = -1
   local globalRespMask = 255
   local wallWidth = 0.05
   local wallHeight = 0.20
   -- side walls (length of walls should be corrected from the slope ! not done yet!)
   if string.match(wall,"N") == "N" then
      --printToConsole ("North wall")
      local localRespMask = 1
      local wallname = name.."_N"
      local size = {l,wallWidth,wallHeight}
      local posit = {(xc+xd)/2.0,(yc+yd)/2.0,(z0+z1)/2.0}
      local orient = {0.0,-math.atan2 (z1-z0,xc-xd)*radDeg,math.atan2 (yc-yd,xc-xd) * radDeg}
      local cuboid_prms = defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
      table.insert(trkWall,cuboid_prms)
   end
   if string.match(wall,"S") == "S" then
      --printToConsole ("South wall")
      local localRespMask = 1
      local wallname = name.."_S"
      local size = {l,wallWidth,wallHeight}
      local posit = {(xa+xb)/2.0,(ya+yb)/2.0,(z0+z1)/2.0}
      local orient = {0.0,-math.atan2 (z1-z0,xb-xa)*radDeg,math.atan2 (yb-ya,xb-xa) * radDeg}
      local cuboid_prms = defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
      table.insert(trkWall,cuboid_prms)
   end
   if string.match(wall,"W") == "W" then
      --printToConsole ("West wall")
      local localRespMask = 1
      local wallname = name.."_W"
      local size = {w,wallWidth,wallHeight}
      local posit = {(xa+xd)/2.0,(ya+yd)/2.0,z0}
      local orient = {0.0,0.0,math.atan2 (yd-ya,xd-xa) * radDeg}
      local cuboid_prms = defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
      table.insert(trkWall,cuboid_prms)
   end
   if string.match(wall,"E") == "E" then
      --printToConsole ("East wall")
      local localRespMask = 1
      local wallname = name.."_E"
      local size = {w,wallWidth,wallHeight}
      local posit = {(xb+xc)/2.0,(yb+yc)/2.0,z1}
      local orient = {0.0,0.0,math.atan2 (yc-yb,xc-xb) * radDeg}
      local cuboid_prms = defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
      table.insert(trkWall,cuboid_prms)
   end
   -- corners
   if string.match(wall,"S") == "S" and string.match(wall,"W") == "W" then
      local localRespMask = 2      
      local wallname = name.."_SW"
      local size = {wallWidth,wallWidth,wallHeight}
      local posit = {xa,ya,z0}
      --local orient = {0.0,0.0,math.atan2 (yb-ya,xb-xa) * radDeg}
      local orient = {0.0,-math.atan2 (z1-z0,xb-xa)*radDeg,math.atan2 (yb-ya,xb-xa) * radDeg}
      local cuboid_prms = defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
      table.insert(trkWall,cuboid_prms)
   end
   if string.match(wall,"S") == "S" and string.match(wall,"E") == "E" then
      local localRespMask = 2      
      local wallname = name.."_SE"
      local size = {wallWidth,wallWidth,wallHeight}
      local posit = {xb,yb,z1}
      --local orient = {0.0,0.0,math.atan2 (yb-ya,xb-xa) * radDeg}
      local orient = {0.0,-math.atan2 (z1-z0,xb-xa)*radDeg,math.atan2 (yb-ya,xb-xa) * radDeg}
      local cuboid_prms = defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
      table.insert(trkWall,cuboid_prms)
   end
   if string.match(wall,"N") == "N" and string.match(wall,"E") == "E" then
      local localRespMask = 2      
      local wallname = name.."_NE"
      local size = {wallWidth,wallWidth,wallHeight}
      local posit = {xc,yc,z1}
      --local orient = {0.0,0.0,math.atan2 (yd-yc,xd-xc) * radDeg}
      local orient = {0.0,-math.atan2 (z1-z0,xc-xd)*radDeg,math.atan2 (yc-yd,xc-xd) * radDeg}
      local cuboid_prms = defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
      table.insert(trkWall,cuboid_prms)
   end
   if string.match(wall,"N") == "N" and string.match(wall,"W") == "W" then
      local localRespMask = 2      
      local wallname = name.."_NW"
      local size = {wallWidth,wallWidth,wallHeight}
      --printToConsole ("D(corner): "..xd.." "..yd)
      local posit = {xd,yd,z0}
      --local orient = {0.0,0.0,math.atan2 (yc-yd,xc-xd) * radDeg}
      local orient = {0.0,-math.atan2 (z1-z0,xc-xd)*radDeg,math.atan2 (yc-yd,xc-xd) * radDeg}
      local cuboid_prms = defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
      table.insert(trkWall,cuboid_prms)
   end
   return trkHdl,trkWall,x1,y1,z1
end

function createTrackElementCurved(name,width,height,radius,theta0,theta1,x0,y0,z0,z1,wall)
   local hdl,h,w2,options,shadingAngle,vertices,indices,xc,yc
   local options = 0*1+1*2  -- bit 0:backfaces culling (removal) , bit1: edges visible
   local shadingAngle = 0.0
   local th0r = theta0 * degRad
   local th1r = theta1 * degRad
   local r = radius
   local h = height
   local w = width
   local w2 = w/2.0
   local xr = x0 + r*math.cos(th0r)
   local yr = y0 + r*math.sin(th0r)
   local x1 = xr - r*math.cos(th1r)
   local y1 = yr - r*math.sin(th1r)
   local xa = x0+w2*math.cos(th0r)
   local ya = y0+w2*math.sin(th0r)
   local xd = x0+w2*math.cos(th0r+math.pi)
   local yd = y0+w2*math.sin(th0r+math.pi)
   local xb = x1+w2*math.cos(th1r)
   local yb = y1+w2*math.sin(th1r)
   local xc = x1+w2*math.cos(th1r+math.pi)
   local yc = y1+w2*math.sin(th1r+math.pi)
   --printToConsole ("-----------------")
   --printToConsole (th0r.." "..th1r)
   --printToConsole ("0: "..x0.." "..y0)
   --printToConsole ("R: "..xr.." "..yr)
   --printToConsole ("A: "..xa.." "..ya)
   --printToConsole ("D: "..xd.." "..yd)
   --printToConsole ("B: "..xb.." "..yb)
   --printToConsole ("C: "..xc.." "..yc)
   local vertices = {}
   table.insert(vertices,xa)
   table.insert(vertices,ya)
   table.insert(vertices,z0+h)
   table.insert(vertices,xb)
   table.insert(vertices,yb)
   table.insert(vertices,z1+h)
   table.insert(vertices,xc)
   table.insert(vertices,yc)
   table.insert(vertices,z1+h)
   table.insert(vertices,xd)
   table.insert(vertices,yd)
   table.insert(vertices,z0+h)
   local indices = {0,1,2,0,2,3}
   local hdl=simCreateMeshShape(options,shadingAngle,vertices,indices)
   simSetObjectName(hdl,name)
   local trkHdl = hdl
   simSetObjectName(trkHdl,name)
   local trkWall = {}
   local hRef = -1
   local globalRespMask = 255
   local wallWidth = 0.05
   local wallHeight = 0.20
   local lab = math.sqrt((xa-xb)*(xa-xb)+(ya-yb)*(ya-yb))
   local lcd = math.sqrt((xc-xd)*(xc-xd)+(yc-yd)*(yc-yd))
   -- side walls (length of walls should be corrected from the slope ! not done yet!)
   if string.match(wall,"N") == "N" then
      --printToConsole ("North wall")
      local localRespMask = 1
      local wallname = name.."_N"
      local size = {lcd,wallWidth,wallHeight}
      local posit = {(xc+xd)/2.0,(yc+yd)/2.0,(z0+z1)/2.0}
      local angz =  math.atan2 (y1-y0,x1-x0) * radDeg
      local orient = {0.0,0.0,angz}
      local cuboid_prms = defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
      table.insert(trkWall,cuboid_prms)
   end
   if string.match(wall,"S") == "S" then
      --printToConsole ("South wall")
      local localRespMask = 1
      local wallname = name.."_S"
      local size = {lab,wallWidth,wallHeight}
      local posit = {(xa+xb)/2.0,(ya+yb)/2.0,(z0+z1)/2.0}
      local angz =  math.atan2 (y1-y0,x1-x0) * radDeg
      local orient = {0.0,0.0,angz}
      local cuboid_prms = defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
      table.insert(trkWall,cuboid_prms)
   end
   if string.match(wall,"W") == "W" then
      --printToConsole ("West wall")
      local localRespMask = 1
      local wallname = name.."_W"
      local size = {w,wallWidth,wallHeight}
      local posit = {(xa+xd)/2.0,(ya+yd)/2.0,z0}
      local orient = {0.0,0.0,math.atan2 (yd-ya,xd-xa) * radDeg}
      local cuboid_prms = defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
      table.insert(trkWall,cuboid_prms)
   end
   if string.match(wall,"E") == "E" then
      --printToConsole ("East wall")
      local localRespMask = 1
      local wallname = name.."_E"
      local size = {w,wallWidth,wallHeight}
      local posit = {(xb+xc)/2.0,(yb+yc)/2.0,z1}
      local orient = {0.0,0.0,math.atan2 (yc-yb,xc-xb) * radDeg}
      local cuboid_prms = defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
      table.insert(trkWall,cuboid_prms)
   end

   return trkHdl,trkWall,x1,y1,z1   
end

function createTrackElementDubins(name,width,height,x0,y0,z0,theta0,x1,y1,z1,theta1,wall)
   local hdl,h,w2,options,shadingAngle,vertices,indices,xc,yc
   local options = 0*1+1*2  -- bit 0:backfaces culling (removal) , bit1: edges visible
   local shadingAngle = 0.0
   printToConsole ("Dubins : theta0="..theta0..", theta1="..theta1..", wall="..wall)
   local th0r = theta0 * degRad - math.pi/2
   local th1r = theta1 * degRad - math.pi/2
   local h = height
   local w = width
   local w2 = w/2.0
   local xa = x0+w2*math.cos(th0r)
   local ya = y0+w2*math.sin(th0r)
   local xd = x0+w2*math.cos(th0r+math.pi)
   local yd = y0+w2*math.sin(th0r+math.pi)
   local xb = x1+w2*math.cos(th1r)
   local yb = y1+w2*math.sin(th1r)
   local xc = x1+w2*math.cos(th1r+math.pi)
   local yc = y1+w2*math.sin(th1r+math.pi)
   --printToConsole ("-----------------")
   --printToConsole (th0r.." "..th1r)
   --printToConsole ("0: "..x0.." "..y0)
   --printToConsole ("1: "..x1.." "..y1)
   --printToConsole ("A: "..xa.." "..ya)
   --printToConsole ("D: "..xd.." "..yd)
   --printToConsole ("B: "..xb.." "..yb)
   --printToConsole ("C: "..xc.." "..yc)
   local vertices = {}
   table.insert(vertices,xa)
   table.insert(vertices,ya)
   table.insert(vertices,z0+h)
   table.insert(vertices,xb)
   table.insert(vertices,yb)
   table.insert(vertices,z1+h)
   table.insert(vertices,xc)
   table.insert(vertices,yc)
   table.insert(vertices,z1+h)
   table.insert(vertices,xd)
   table.insert(vertices,yd)
   table.insert(vertices,z0+h)
   local indices = {0,1,2,0,2,3}
   local hdl=simCreateMeshShape(options,shadingAngle,vertices,indices)
   simSetObjectName(hdl,name)
   local trkHdl = hdl
   simSetObjectName(trkHdl,name)
   local trkWall = {}
   local hRef = -1
   local globalRespMask = 255
   local wallWidth = 0.05
   local wallHeight = 0.20
   local lab = math.sqrt((xa-xb)*(xa-xb)+(ya-yb)*(ya-yb))
   local lcd = math.sqrt((xc-xd)*(xc-xd)+(yc-yd)*(yc-yd))
   -- side walls (length of walls should be corrected from the slope ! not done yet!)
   if string.match(wall,"N") == "N" then
      --printToConsole ("North wall")
      local localRespMask = 1
      local wallname = name.."_N"
      local size = {lcd,wallWidth,wallHeight}
      local posit = {(xc+xd)/2.0,(yc+yd)/2.0,(z0+z1)/2.0}
      local angz =  math.atan2 (y1-y0,x1-x0) * radDeg
      local orient = {0.0,0.0,angz}
      local cuboid_prms = defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
      table.insert(trkWall,cuboid_prms)
   end
   if string.match(wall,"S") == "S" then
      --printToConsole ("South wall")
      local localRespMask = 1
      local wallname = name.."_S"
      local size = {lab,wallWidth,wallHeight}
      local posit = {(xa+xb)/2.0,(ya+yb)/2.0,(z0+z1)/2.0}
      local angz =  math.atan2 (y1-y0,x1-x0) * radDeg
      local orient = {0.0,0.0,angz}
      local cuboid_prms = defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
      table.insert(trkWall,cuboid_prms)
   end
   if string.match(wall,"W") == "W" then
      printToConsole ("West wall")
      local localRespMask = 1
      local wallname = name.."_W"
      local size = {w,wallWidth,wallHeight}
      local posit = {(xa+xd)/2.0,(ya+yd)/2.0,z0}
      local orient = {0.0,0.0,math.atan2 (yd-ya,xd-xa) * radDeg}
      local cuboid_prms = defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
      table.insert(trkWall,cuboid_prms)
   end
   if string.match(wall,"E") == "E" then
      printToConsole ("East wall")
      local localRespMask = 1
      local wallname = name.."_E"
      local size = {w,wallWidth,wallHeight}
      local posit = {(xb+xc)/2.0,(yb+yc)/2.0,z1}
      local orient = {0.0,0.0,math.atan2 (yc-yb,xc-xb) * radDeg}
      local cuboid_prms = defineCuboidParams(size,wallname,posit,orient,hRef,localRespMask,globalRespMask)
      table.insert(trkWall,cuboid_prms)
   end

   return trkHdl,trkWall,x1,y1,z1   
end

function createTrackFromFile(trk_filename)
   --trk_filename = "/home/newubu/Teach/ue22sal/sal-2021/scenes/trackdef.txt"
   ftrk = io.open(trk_filename,"r")
   io.input(ftrk)
   trkElems = {}
   wallElems = {}
   cenlinElems = {}
   while true do
      stl = io.read()
      if string.len(stl) <= 1 then break end
      --printToConsole ("rd : "..stl.." "..type(stl))
      prms = {}
      for w in stl:gmatch("(.-);") do
	      table.insert(prms, w)
	    --printToConsole(w.." "..type(w))
      end
      nprms = table.getn(prms)
      --printToConsole(nprms.." prms")
      if nprms == 0 then break end
      if nprms > 1 then
         if prms[1] == "L" then
            --printToConsole("Linear Track Element")
            name = prms[2]
            length = tonumber(prms[3])
            width = tonumber(prms[4])
            height = tonumber(prms[5])
            theta =  tonumber(prms[6])-90.0
            x0 = tonumber(prms[7])
            y0 = tonumber(prms[8])
            z0 = tonumber(prms[9])
            z1in = tonumber(prms[10])
            wall = prms[11]
            cenlin = prms[12]
            htrk,trkwall,x1,y1,z1 =  createTrackElementLinear(name,length,theta,width,height,x0,y0,z0,z1in,wall)
            --printToConsole("hdl trk and wall", htrk,hwall)
            table.insert(trkElems,htrk)
            for i =1,#trkwall do
               table.insert(wallElems,trkwall[i])
            end
            --printToConsole("nelem trk and wall",#trkElems,#wallElems)
            --add center line
            centerlin = createTrackCenterLineLinear(name,length,theta,width,height,x0,y0,z0,z1in,cenlin)
            for i =1,#centerlin do
               table.insert(cenlinElems,centerlin[i])
            end
         end
         if prms[1] == "C" then
            --printToConsole("Curved Track Element")
            name = prms[2]
            width = tonumber(prms[3])
            height = tonumber(prms[4])
            radius = tonumber(prms[5])
            theta0 =  tonumber(prms[6])-90.0
            theta1 =  tonumber(prms[7])-90.0
            x0 = tonumber(prms[8])
            y0 = tonumber(prms[9])
            z0 = tonumber(prms[10])
            z1in = tonumber(prms[11])
            wall = prms[12]
            htrk,trkwall,x1,y1,z1 =  createTrackElementCurved(name,width,height,radius,theta0,theta1,x0,y0,z0,z1in,wall)
            --printToConsole("hdl trk and wall", htrk,hwall)
            table.insert(trkElems,htrk)
            for i =1,#trkwall do
               table.insert(wallElems,trkwall[i])
            end
            --printToConsole("nelem trk and wall",#trkElems,#wallElems)
         end
         if prms[1] == "D" then
            -- printToConsole("Dubins Track Element")
            -- name,w,h,x0,y0,z0,theta0,x1,y1,z1,theta1,wall,cenlin
            name = prms[2]
            width = tonumber(prms[3])
            height = tonumber(prms[4])
            x0 = tonumber(prms[5])
            y0 = tonumber(prms[6])
            z0 = tonumber(prms[7])
            theta0 =  tonumber(prms[8])
            x1 = tonumber(prms[9])
            y1 = tonumber(prms[10])
            z1 = tonumber(prms[11])            
            theta1 =  tonumber(prms[12])
            wall = prms[13]
            cenlin = prms[14]
            htrk,trkwall,x1,y1,z1 =  createTrackElementDubins(name,width,height,x0,y0,z0,theta0,x1,y1,z1,theta1,wall)
            --printToConsole("hdl trk and wall", htrk,hwall)
            table.insert(trkElems,htrk)
            for i =1,#trkwall do
               table.insert(wallElems,trkwall[i])
            end
            --printToConsole("nelem trk and wall",#trkElems,#wallElems)
            --add center line
            --printToConsole("x1="..x1..", y1="..y1..", z1="..z1)
            --print (name,width,height,x0,y0,z0,theta0,x1,y1,z1,theta1,wall)
            centerlin = createTrackCenterLineDubins(name,width,height,x0,y0,z0,theta0,x1,y1,z1,theta1,wall)
            for i =1,#centerlin do
               table.insert(cenlinElems,centerlin[i])
            end
         end
      end
   end
   io.close()
   -- merge track elements
   --printToConsole("before merge nelem trk and wall",#trkElems,#wallElems)
   if  #trkElems == 1 then
      h_trk = trkElems[1]
   elseif  #trkElems > 1 then
      for i=1,#trkElems do
	 --printToConsole("trk elem",trkElems[i])
      end
      h_trk = simGroupShapes(trkElems,table.getn(trkElems))
   end
   if #trkElems >= 1 then
      simSetObjectName(h_trk,"Track")
      sim.setShapeColor (h_trk,nil,sim.colorcomponent_ambient,{0.4,0.4,0.4})
   end

   -- merge center line elements
   if #cenlinElems == 1 then
      h_cenlin = cenlinElems[1]
   elseif #cenlinElems > 1 then
      h_cenlin = simGroupShapes(cenlinElems,table.getn(cenlinElems))
   end
   if #cenlinElems >= 1 then
      simSetObjectName(h_cenlin,"CenterLine")
   end

   -- set z for places to start and finish
   local h=simGetObjectHandle("Place2Start")
   local pos = sim.getObjectPosition(h,-1)
   pos[3] = height+0.00025
   simSetObjectPosition(h,-1,pos)
   printToConsole ("Init Place2start",pos[1],pos[1],pos[3])
   h=simGetObjectHandle("Place2Finish")
   pos = sim.getObjectPosition(h,-1)
   pos[3] = height+0.00025
   printToConsole ("Init Place2Finish",pos[1],pos[1],pos[3])
   simSetObjectPosition(h,-1,pos)

   return wallElems
end

function make_walls_respondable (cuboids)
   wallElems = {}
   -- make walls respondable
   for i=1,#cuboids do
      prms = cuboids[i]
      hdl = createStaticRespondableCuboid(prms[1],prms[2],prms[3],prms[4],prms[5],prms[6],prms[7])
      table.insert(wallElems,hdl)
   end
   -- merge wall elements
   if  #wallElems == 1 then
      h_wall = wallElems[1]
   elseif  #wallElems > 1 then
      for i=1,#wallElems do
	 --printToConsole("wall elem",wallElems[i])
      end
      h_wall = simGroupShapes(wallElems,table.getn(wallElems))
   end
   if  #wallElems >= 1 then
      simSetObjectName(h_wall,"TrackWallS")
      sim.setShapeColor (h_wall,nil,sim.colorcomponent_ambient,{0.6,0.6,0.8})
      makeObjetDetectableByUltrasonics (h_wall)
   end
end

function createRespondableTrackFromFile(trkdyn_filename)
   -- dynamic part of track is made of simple pure shapes that make the robot not falling through
   ftrk = io.open(trkdyn_filename,"r")
   io.input(ftrk)
   dynElems = {}
   localRespMask = 1
   while true do
      local stl = io.read()
      if string.len(stl) <= 1 then break end
      --printToConsole ("rd : "..stl.." "..type(stl))
      local prms = {}
      for w in stl:gmatch("(.-);") do
	 table.insert(prms, w)
	 --printToConsole("respondable prm "..w.." "..type(w))
      end
      local nprms = table.getn(prms)
      --printToConsole(nprms.." prms")
      if nprms == 0 then break end
      if nprms > 1 then
	 --printToConsole("Respondable Track Element")
	 local name = prms[1]
	 local length = tonumber(prms[2])
	 local width = tonumber(prms[3])
	 local height = tonumber(prms[4])
	 local thetax =  tonumber(prms[5])
	 local thetay =  tonumber(prms[6])
	 local thetaz =  tonumber(prms[7])
	 local xc = tonumber(prms[8])
	 local yc = tonumber(prms[9])
	 local zc = tonumber(prms[10])
	 local size = {length, width, height}
	 local orient = {thetax,thetay,thetaz}
	 local posit = {xc,yc,zc}
	 local hRef = -1
	 localRespMask = localRespMask+1
	 if localRespMask == 255 then
	    localRespMask = 1
	 end
	 globalRespMask = 255
	 h = createStaticRespondableCuboid(size,name,posit,orient,hRef,localRespMask,globalRespMask)
	 table.insert(dynElems,h)
      end
   end
   if #dynElems > 0 then
      h_trk = simGroupShapes(dynElems,table.getn(dynElems))
      simSetObjectName(h_trk,"DynamicRespondableTrack")
      sim.setShapeColor (h_trk,nil,sim.colorcomponent_ambient,{0.2,0.2,0.6})
      setObjectVisibilityMask(h_trk,0) -- make invisible
   end
 end   

function loadCoverageFromFile(coverage_filename)
   fcov = io.open(coverage_filename,"r")
   io.input(fcov)
   trkElems = {}
   while true do
      stl = io.read()
      if string.len(stl) <= 1 then break end
      --printToConsole ("rd : "..stl.." "..type(stl))
      prms = {}
      for w in stl:gmatch("(.-);") do
	 table.insert(prms, w)
	 --printToConsole(w.." "..type(w))
      end
      nprms = table.getn(prms)
      --printToConsole(nprms.." prms")
      if nprms == 0 then break end
      if nprms > 1 then
	 x_min = tonumber(prms[1])
	 y_min = tonumber(prms[2])
	 x_max = tonumber(prms[3])
	 y_max = tonumber(prms[4])
	 w_max = tonumber(prms[5])
	 x_c = tonumber(prms[6])
	 y_c = tonumber(prms[7])
	 sz_x = tonumber(prms[8])
	 sz_y = tonumber(prms[9])
      end
   end
   io.close()
   return x_c,y_c,sz_x+2*w_max,sz_y+2*w_max
end
