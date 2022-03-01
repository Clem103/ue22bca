
-- Get wheel angular position (in degrees)
function getWheelAngularPosition (handle)
    angles=sim.getObjectOrientation(handle,sim.handle_parent)
    angPos = angles[3]*180.0/math.pi
    angPos = angPos + 180.0
    --printToConsole("-----> angPos=",angPos,angPos-180.0,"handle=",handle)
    return angPos
end 

-- Compute increment of odemeters
function deltaWheelAngularPosition(curPos,lastPos,wSpeed)
    if wSpeed == 0.0 then
        deltaPos = 0.0
    else
        deltaPos = -(curPos - lastPos)  -- clock wise rotation (anti trigo)
    end
    --printToConsole ("-----> delta pos ",curPos,lastPos,wSpeed,deltaPos)
    -- recenter deltaPos in [-pi,pi]
    if deltaPos < -180.0 then
        deltaPos = deltaPos + 360.0
    end
    if deltaPos > 180.0 then
        deltaPos = deltaPos - 360.0
    end
    --printToConsole ("-----> delta pos ",curPos,lastPos,wSpeed,deltaPos)
    -- removed 2022/02/07 (bug due to old 2-complement coding of odometers)
    --if deltaPos < 0.0 then
    --    if wSpeed > 0.0 then
    --        deltaPos = deltaPos + 360.0
    --    end
    --else
    --    if wSpeed < 0.0 then
    --        deltaPos = deltaPos - 360.0
    --    end
    --end
    --if deltaPos ~= 0 then
    --print ("-----> delta pos ",curPos,lastPos,wSpeed,deltaPos)
    --end
    return deltaPos
end

function updateBatteryLevel(cmdl,cmdr)
    local cl = math.abs(cmdl) 
    local cr = math.abs(cmdr)
    if cl > 255 then cl=255 end
    if cr > 255 then cr=255 end
    local c = (cl+cr)/2.0
    cmdMemory = cmdMemory*0.5+c*0.5
    batteryLevel = batteryLevel - 0.000025*(cmdMemory/5.0) - 0.000015
    if batteryLevel < 0.7 then batteryLevel = 0.7 end
    --printToConsole("Battery lvl = "..(batteryLevel*100.0).."%, cmdMemory = "..cmdMemory..", cl="..cl..", cr="..cr..", c="..c)
    vAngMax = 10.0*batteryLevel
end

-- Take speed command and convert into V-REP motor command
--   tries to simulate left-right diffrences and starting current
function defineLeftSpeed (cmd)
    local cmd1=cmd
    if cmd1 < 0 then
        cmd1 = -cmd1
    end
    if cmd1 < cmdDeadZone then
        cmd1 = 0
    end
    if cmd1 > 255 then 
        cmd1 = 255
    end
    vrepCmd = cmd1*vAngMax/500.0
    if cmd < 0 then
        vrepCmd = -vrepCmd
    end
    return vrepCmd
end

function defineRightSpeed (cmd)
    local cmd1=cmd
    if cmd1 < 0 then
        cmd1 = -cmd1
    end
    if cmd1 < cmdDeadZone then
        cmd1 = 0
    end
    if cmd1 > 255 then 
        cmd1 = 255
    end
    vrepCmd = cmd1*vAngMax/500.0
    if cmd < 0 then
        vrepCmd = -vrepCmd
    end
    return vrepCmd
end

-- add noise to distance (bias, spike and gaussian)
function realDistance (dist,sonar)
   local rDist = dist+fidelSonar*gaussian()+justSonar
   --if math.random() < probaSpike then
   --   --rDist = rDist*1000.0
   --   rDist = math.random()*100.0
   --end
   if periodSpike ~= -1 then 
      cntSpike = cntSpike - 1
      if cntSpike <= 0 then
         cntSpike = math.floor(0.5*(periodSpike+periodSpike*math.random()))
         rDist = math.random()*10.0
         --printToConsole ("cntSpike",cntSpike,rDist)
         --rDist = 25.0
      end
   end
   rDist = roundDecimal(rDist,2)
   --printToConsole ("Sonar="..sonar..", dist="..dist..", rDist="..rDist)
   return rDist
end

function realDistanceTheo(dist,sonar)
   dist = roundDecimal(dist,2)
   --printToConsole ("dist="..dist..", sonar="..sonar)
   return dist
end


function getMeasuredHeading()
   local orient = sim.getObjectOrientation(rob1a,-1)
   local heading = 90.0 - (orient[3]*180.0/math.pi)
   heading = heading+fidelCompass*gaussian()+justCompass
   heading = roundDecimal(heading,1)
   if heading < 0.0 then heading = heading+360.0 end
   if heading > 360.0 then heading = heading-360.0 end
   if heading == 360.0 then heading = 0.0 end
   return heading
end

function getMagneticSensor()
   local headingActual = getMeasuredHeading()
   local hdrad = headingActual*math.pi/180.0
   local magnetic_sensor_x = round(16384*(magnetic_sensor_coef_x*math.cos(hdrad)+magnetic_sensor_bias_x))
   local magnetic_sensor_y = round(16384*(magnetic_sensor_coef_y*math.sin(hdrad)+magnetic_sensor_bias_y))
   return {magnetic_sensor_x, magnetic_sensor_y}
end

function lineSensorMeasurement(v)
    local c1 = lineSensorAttenuation
    local c2 = 1.0-c1
    local c = c1+c2*math.random()
    local vm = c*v
    --printToConsole ("lineSensor : v="..v..", c="..c..", vm="..vm)
    return vm
end  



