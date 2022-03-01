-- same file for stud and eval  (eval if do_eval is true)
if pcall(function () printToConsole() end) then
   printToConsole ("printToConsole exists")
   sim.addStatusbarMessage("printToConsole exists")
else
   function printToConsole(st) print(st) end
   --printToConsole = function(st) sim.addStatusbarMessage(st) end
   sim.addStatusbarMessage("printToConsole has been defined")
   printToConsole ("printToConsole defined")
end

printToConsole()
printToConsole("-------------------------------------------------")
printToConsole ("run",scene,"robscene ...")
run_headless=sim.getBoolParameter(sim.boolparam_headless)
if run_headless then
   printToConsole ("run in headless mode ...")
end
require ("rob1a_utils")
require ("rob1a_shapes")
require ("rob1a_sensors")
require ("rob1a_waypoints")
require ("rob1a_marks")
if do_eval then
   require ("rob1a_simulation_loop_eval")
else
   require ("rob1a_simulation_loop")
end

clock = os.clock
sim.setThreadSwitchTiming(10) -- We wanna manually switch for synchronization purpose (and also not to waste processing time!)

frontRightMotor = sim.getObjectHandle("JointWheelRight")
frontLeftMotor = sim.getObjectHandle("JointWheelLeft")
sonarFront = sim.getObjectHandle("SonarFrontSensor")
sonarLeft = sim.getObjectHandle("SonarLeftSensor")
sonarRight = sim.getObjectHandle("SonarRightSensor")
sonarBack = sim.getObjectHandle("SonarBackSensor")
rob1a = sim.getObjectHandle("RobotCenterMarker")
trackEnd = sim.getObjectHandle("Place2Finish")

-- Socket Port number
portNb = 30100
serverOn = false
connexionTimeout = 0.01
cntTimeout = 0
socket=require("socket")
srv = nil
clt1 = nil

--fidelSonar = 0.0001 -- 0.02 -- 0.03
--justSonar = 0.0 -- 0.1
--fidelCompass = 0.0001 -- 0.02 -- 0.03
--justCompass = 0.0 -- 0.1
--periodSpike = -1 -- no spike noise
-- these parameters are read from the setup file 

--cntSpike = periodSpike
if periodSpike ~= -1 then
   cntSpike = math.floor(0.5*(periodSpike+periodSpike*math.random()))
else
   cntSpike = 1000000000
end
printToConsole ("cntSpike init",cntSpike)

-- init sonars
distFront,distLeft,distRight,distBack = 0.0,0.0,0.0,0.0


nTicks = nTicksPerRevol -- get nticks per revol from setup
print ("nTicks = "..nTicks)
speedRight, speedLeft = 0.0,0.0
lastSpeedRight, lastSpeedLeft = 0.0,0.0
if setBatteryLevel == -1.0 then
   local mySeed = os.time()
   math.randomseed(mySeed)
   batteryLevel = 0.9+0.1*math.random()
   batteryLevel = round(batteryLevel*1000.0)/1000.0
else
   batteryLevel = setBatteryLevel
end
batteryLevelStart = batteryLevel
tLogBatteryZero = nil
tLogBattery = 1.0
--batteryLevel = 1.0
printToConsole ("starting battery level : "..(batteryLevelStart*100).."%")
batteryLevel = batteryLevelStart -- restart battery
cmdMemory = 0.0 -- cmd memory for discharging battery
vAngMax = 10.0*batteryLevel
cmdDeadZone = 0
if not run_headless then
   sim.addStatusbarMessage ("battery coef = "..(batteryLevelStart*100).."%")
end
-- get orientation of Dart's body
lastRelativeOrientation={}
lastOrientationTime={}
wheelCnt={}
wheelId={}
wheels={"WheelLeftDyn","WheelRightDyn"}
for i=1,#wheels do
    handle=sim.getObjectHandle(wheels[i])
    wheelId[#wheelId+1]=handle
    lastRelativeOrientation[#lastRelativeOrientation+1]=getWheelAngularPosition(handle)
    lastOrientationTime[#lastOrientationTime+1]=sim.getSimulationTime()
    wheelCnt[#wheelCnt+1]=0.0
end
printToConsole ("run ...",scene,"robscene ...")

tLogBotPoseZero = nil
tLogBotPose = 0.5 -- log robot pose every 500 ms
logOn = false
require ("path_log")
printToConsole ("pathLog",pathLog)
pathLogN = string.len(pathLog)
pathLogLastChar = string.sub(pathLog,pathLogN,pathLogN)
--printToConsole ("pathLog",pathLogN,string.sub(pathLog,pathLogN,pathLogN))
if pathLogLastChar ~= '/' then
   pathLog = pathLog.."/"
end
logFile = pathLog.."rob1a.log"
printToConsole ("logFile",logFile)


tDisplayMarkZero = nil
tDisplayMark = 0.95 -- display mark every seconds

--wpTable,wpDist,tMarkLbl,tMark,wpMarkIndex,wpSize,wpZ = setMarkTable ()
wpTable,wpDist,tMarkLbl,tMark,wpMarkIndex,wpSize = setMarkTable ()
wpHalfSize = wpSize/2.0
--printToConsole("wpSize,wpZ",wpSize,wpZ)
--printTable(wpTable,"wpTable")
--printTable(wpDist,"wpDist")
--printTable(tMarkLbl,"tMarkLbl")
--printTable(tMark,"tMark")
--printTable(wpMarkIndex,"wpMarkIndex")

resetWaypoints ()

tStop = nil -- starting time of a robot stop
tStopMaximum = 20.0 -- maximum stop time allowed is 20 s
motion = false

-- set robot tracking camera
options_00 = 0
camViewTrk=simFloatingViewAdd(0.825,0.5,0.25,0.25,options_00)
camTrk=sim.getObjectHandle("CameraTracker")
options_64 = 64
rsu1=simAdjustView(camViewTrk, camTrk,options_64,"Tracker Camera")
obj = simGetObjectHandle("RobotCenterMarker")
rsu2=simCameraFitToView(camViewTrk,{obj},0,0.025)
printToConsole("Camtrk",camViewTrk,camTrk,rsu1,obj,rsu2)

-- set 3 small floating views for the line sensor
lineSensorViewLeft=simFloatingViewAdd(0.05,0.02,0.04,0.04,options_00)
lineSensorViewMiddle=simFloatingViewAdd(0.10,0.02,0.04,0.04,options_00)
lineSensorViewRight=simFloatingViewAdd(0.15,0.02,0.04,0.04,options_00)
print ('line sensor : '..lineSensorViewLeft..","..lineSensorViewMiddle..','..lineSensorViewRight)
lineSensorLeft=sim.getObjectHandle("LineSensorLeft")
status=simAdjustView(lineSensorViewLeft,lineSensorLeft,options_64,"Left-Line-Sensor")
lineSensorMiddle=sim.getObjectHandle("LineSensorMiddle")
status=simAdjustView(lineSensorViewMiddle,lineSensorMiddle,options_64,"Middle-Line-Sensor")
lineSensorRight=sim.getObjectHandle("LineSensorRight")
status=simAdjustView(lineSensorViewRight,lineSensorRight,options_64,"Right-Line-Sensor")


--- set line detect sensors
floorSensorHandles={-1,-1,-1}
floorSensorHandles[1]=sim.getObjectHandle("LineSensorLeft")
floorSensorHandles[2]=sim.getObjectHandle("LineSensorMiddle")
floorSensorHandles[3]=sim.getObjectHandle("LineSensorRight")
printToConsole ("center line detection sensors : "..floorSensorHandles[1]..","..floorSensorHandles[2]..","..floorSensorHandles[3])

--- init for evaluation
if do_eval then
   printToConsole ("specific eval init ...")
   speedLeftPrevious = -1
   speedRightPrevious = -1
   file_time = io.open('/tmp/time_vrep.log','w')
   printToConsole("file_time",file_time)
   spdmsg =  gettime()..";"..speedLeftPrevious..";"..speedRightPrevious
   file_time:write(spdmsg,"\n")
   file_time:close()
end

timeChallengeStart = gettime()

printToConsole ("end init ...")
printToConsole ("start thread ...")
-- Execute the thread function:
res=false
err=" not launched delibarately "
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
   if not run_headless then
      sim.addStatusbarMessage('Lua runtime error: '..err)
   end
end
--simFloatingViewRemove(camView)
printToConsole ("end of simulation !!!")
