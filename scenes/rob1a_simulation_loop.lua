-- main simulation loop function (run every 200 ms (2022), 50 ms before)
-- 200 ms has been set (instead of 50 ms) to keep 1x simulation time on slow computers
function threadFunction()
    printToConsole("sim state", sim.getSimulationState())
    printToConsole("reset marks and waypoints")
    printToConsole("sim time", sim.getSimulationTime())
    resetMarks()
    resetWaypoints()
    t00 = nil
    t00s = nil
    local simTime0 = sim.getSimulationTime()
    local dataOut = {distFront, distLeft, distRight, distBack, distFrontLeft, distFrontRight, leftEncoder, rightEncoder,
                     0, 0, 0, 1, 1}
    local doLog = 0 -- updated on socket rx
    local serverUpdateLog = false -- true if doLog updated
    while (sim.getSimulationState() ~= sim.simulation_advancing_abouttostop) do
        -- local t0 = clock()
        local t0 = gettime()
        local t0s = sim.getSimulationTime()
        local validIteration = true
        if t00 == nil then
            t00 = t0
            t00s = t0s
        else
            -- printToConsole("t0 : ", t0 - t00, t0s - t00s, (t0 - t00) / (t0s - t00s),t0s-simTime0)
            if t0s - t00s < -0.1 then
                validIteration = false
            end
            t00 = t0
            t00s = t0s
        end

        local simTime = sim.getSimulationTime()
        -- printToConsole("simulation duration is "..(simTime-simTime0))
        if tStop == nil then
            tStop = simTime
        end
        if tDisplayMarkZero == nil then
            tDisplayMarkZero = simTime
        end
        if tLogBotPoseZero == nil then
            tLogBotPoseZero = simTime
        end
        if tLogBatteryZero == nil then
            tLogBatteryZero = simTime
        end
        local t1 = gettime()
        local t2 = t1
        local t3 = t1
        local t4 = t1
        local t5 = t1
        local t6 = t1
        local t7 = t1
        if validIteration == false then
            sleep(0.001) -- sleep 1 ms if simTime not incremented by 200 ms
            printToConsole("...")
        else
            timeStamp = simTime .. ";"
            -- update or init (reinit) motion
            if motion then
                if math.abs(speedLeft) > 0 or math.abs(speedRight) > 0 then
                    tStop = simTime  -- 
                else
                    tStalled = simTime - tStop
                    --printToConsole("stop motion for " .. tStalled)
                    if tStalled > tStopMaximum then
                        printToConsole("do stop motion at " .. simTime .. " after " .. tStalled)
                        motion = false
                    end
                end
            else
                -- start or restart
                if math.abs(speedLeft) > 0 or math.abs(speedRight) > 0 then
                    motion = true
                    cntMotion = cntNoMotion
                    tStop = simTime
                    printToConsole("restart(reset) motion at " .. t0s)
                    for i = 1, table.getn(tMark) do
                        tMark[i] = 0.0
                    end
                    resetWaypoints()
                    resetElapsedTime()
                    batteryLevel = batteryLevelStart -- restart battery
                    cmdMemory = 0.0 -- cmd memory for discharging battery    
                    vAngMax = 10.0*batteryLevel                                   
                end
            end

            -- log robot pose : x , y , theta_z
            if (simTime - tLogBotPoseZero) > tLogBotPose then
                if logOn then
                    loc = sim.getObjectPosition(rob1a, -1)
                    orient = sim.getObjectOrientation(rob1a, -1)
                    file = io.open(logFile, "a")
                    -- logMessage = timeStamp..clock()..";robot pose;"..loc[1]..";"..loc[2]..";"..orient[3]*180.0/math.pi
                    logMessage =
                        timeStamp .. gettime() .. ";robot pose;" .. loc[1] .. ";" .. loc[2] .. ";" .. orient[3] * 180.0 /
                            math.pi
                    file:write(logMessage, "\n")
                    -- logMessage = timeStamp..clock()..";marks"
                    logMessage = timeStamp .. gettime() .. ";marks"
                    mark = 0.0
                    for i = 1, table.getn(tMark) do
                        mark = mark + tMark[i]
                        mrk = roundDecimal(tMark[i], 2)
                        logMessage = logMessage .. ";" .. roundDecimal(mrk, 2)
                    end
                    logMessage = logMessage .. ";" .. roundDecimal(mark, 2)
                    file:write(logMessage, "\n")
                    file:close()
                end
                tLogBotPoseZero = simTime
            end

            t1 = gettime()

            if motion then
                checkAllWaypoints()
            end

            t2 = gettime()

            -- printToConsole ("measure sonar",(numSonar+1),'log',logOn,'logFile',logFile)
            if logOn then
                file = io.open(logFile, "a")
            end
            local result, dist, dtPoint, dtObjHandle, dtSurfNorm = sim.handleProximitySensor(sonarFront)
            distFront = 0.0
            if result == 1 then
                distFront = realDistance(dist, "front")
            end
            if logOn and result == 1 then
                logMessage = timeStamp .. gettime() .. ";sonar front;" .. distFront
                file:write(logMessage, "\n")
            end
            local result, dist, dtPoint, dtObjHandle, dtSurfNorm = sim.handleProximitySensor(sonarLeft)
            distLeft = 0.0
            if result == 1 then
                distLeft = realDistance(dist, "left")
            end
            if logOn and result == 1 then
                logMessage = timeStamp .. gettime() .. ";sonar left;" .. distLeft
                file:write(logMessage, "\n")
            end
            local result, dist, dtPoint, dtObjHandle, dtSurfNorm = sim.handleProximitySensor(sonarRight)
            distRight = 0.0
            if result == 1 then
                distRight = realDistance(dist, "right")
            end
            if logOn and result == 1 then
                logMessage = timeStamp .. gettime() .. ";sonar right;" .. distRight
                file:write(logMessage, "\n")
            end
            local result, dist, dtPoint, dtObjHandle, dtSurfNorm = sim.handleProximitySensor(sonarBack)
            distBack = 0.0
            if result == 1 then
                distBack = realDistance(dist, "back")
            end
            if logOn and result == 1 then
                logMessage = timeStamp .. gettime() .. ";sonar back;" .. distBack
                file:write(logMessage, "\n")
            end
            if logOn then
                file:close()
            end

            t3 = gettime()

            -- update encoders with relative orientation of the wheel (wrt joint axis)
            for iw = 1, #wheelId do
                if iw == 1 then
                    wheelSpeed = speedLeft
                end -- left wheel
                if iw == 2 then
                    wheelSpeed = speedRight
                end -- right wheel
                --if wheelSpeed ~= 0.0 then
                handle = wheelId[iw]
                --printToConsole ("---------------------------------------------------")
                --printToConsole ("wheel",iw,handle,lastRelativeOrientation[iw])
                relativeOrientation = getWheelAngularPosition(handle)
                currentOrientationTime = sim.getSimulationTime()
                --printToConsole ("delta orient",iw,relativeOrientation,lastRelativeOrientation[iw],lastRelativeOrientation[iw]-relativeOrientation)
                --printToConsole ("wheelcnt",iw,wheelCnt[iw])
                analogCnt = deltaWheelAngularPosition(relativeOrientation, lastRelativeOrientation[iw], wheelSpeed)
                wheelCnt[iw] = wheelCnt[iw] + (analogCnt * nTicks / 360.0)
                --printToConsole ("analogcnt ",iw,analogCnt,wheelCnt[iw])
                lastRelativeOrientation[iw] = relativeOrientation
                lastOrientationTime[iw] = currentOrientationTime
                --end
            end
            leftEncoder = math.floor(wheelCnt[1])
            rightEncoder = math.floor(wheelCnt[2])
            if leftEncoder < 0 then
                leftEncoder = leftEncoder + 1
            end
            if rightEncoder < 0 then
                rightEncoder = rightEncoder + 1
            end

            -- printToConsole ("wheel cnt ",wheelCnt[1],wheelCnt[2],wheelCnt[3],wheelCnt[4])
            -- trtrackEndansformation to 16 bits integers is now done in python 

            -- get heading with actual noise
            heading = getMeasuredHeading()
            magnetic_sensor = getMagneticSensor()

            -- get line detectors
            lineSensorReading = {0.0, 0.0, 0.0}
            for i = 1, 3, 1 do
                result, data = sim.readVisionSensor(floorSensorHandles[i])
                if (result >= 0) then
                    lineSensorReading[i] = lineSensorMeasurement(data[11]) -- data[11] is the average of intensity of the image
                end
            end
            -- printToConsole ("line detection sensors : "..lineSensorReading[1]..","..lineSensorReading[2]..","..lineSensorReading[3])     

            -- printToConsole (serverOn)
            dataOut = {distFront, distLeft, distRight, distBack, distFrontLeft, distFrontRight, leftEncoder,
                       rightEncoder, lineSensorReading[1], lineSensorReading[2], lineSensorReading[3],
                       magnetic_sensor[1], magnetic_sensor[2]}

            t4 = gettime()

            if motion then
                computeMark()
            end

            t5 = gettime()

            -- modify speed only when it has changed
            if lastSpeedRight ~= speedRight then
                sim.setJointTargetVelocity(frontRightMotor, speedRight)
                lastSpeedRight = speedRight
            end
            if lastSpeedLeft ~= speedLeft then
                sim.setJointTargetVelocity(frontLeftMotor, speedLeft)
                lastSpeedLeft = speedLeft
            end
            updateBatteryLevel (lastSpeedLeft, lastSpeedRight) -- update battery batteryLevel

            if (simTime -tLogBatteryZero) > tLogBattery then
                if logOn then
                    file = io.open(logFile, "a")
                    logMessage = timeStamp .. gettime() .. ";battery;" .. batteryLevel*100.0
                    file:write(logMessage, "\n")
                    file:close()
                end
                tLogBatteryZero = simTime
            end 

            -- if logOn then
            --   local t1 = clock()
            --   local simTime1 = sim.getSimulationTime()
            --   file = io.open(logFile,"a")
            --   logMessage = timeStamp..clock()..";loop time;"..simTime1..";"..(t1-t0)..";"..(simTime1-simTime)
            --   file:write(logMessage,"\n") 
            -- end
            if (simTime - tDisplayMarkZero) > tDisplayMark then
                motionStatus = "Off "
                if motion then
                    motionStatus = "On  "
                end
                msgMark = motionStatus .. getElapsedTime() .. " ; Mark "
                for i = 1, table.getn(tMark) do
                    mrk = roundDecimal(tMark[i], 2)
                    msgMark = msgMark .. ":" .. (tMarkLbl[i]) .. "=" .. mrk
                end
                mark = 0.0
                for i = 1, table.getn(tMark) do
                    mark = mark + tMark[i]
                end
                msgMark = msgMark .. ": total=" .. roundDecimal(mark, 2)
                if not run_headless then
                    sim.addStatusbarMessage(msgMark)
                end
                tDisplayMarkZero = simTime
            end
            -- sim.auxiliaryConsolePrint(consoleHandle,msgMark.."\n")

            obj = simGetObjectHandle("RobotCenterMarker")
            rsu = simCameraFitToView(camViewTrk, {obj}, 0, 0.025)
            -- printToConsole("Tracking Camera",rsu,camViewTrk,obj)
        end

        t6 = gettime()

        if not serverOn then
            printToConsole("not connected")
            printToConsole("sim time", sim.getSimulationTime())
            srv = assert(socket.bind('127.0.0.1', portNb))
            if (srv == nil) then
                printToConsole("bad connect")
            else
                printToConsole("get socket")
                printToConsole("sim time", sim.getSimulationTime())
                ip, port = srv:getsockname()
                printToConsole("server ok at " .. ip .. " on port " .. port)
                printToConsole("sim time", sim.getSimulationTime())
                serverOn = true
                -- srv:settimeout(connexionTimeout)
                printToConsole("connexion granted !!! ")
            end
        end
        -- printToConsole (serverOn)
        serverUpdateLog = false
        if serverOn then
            srv:settimeout(connexionTimeout)
            clt1 = srv:accept()
            if clt1 == nil then
                cntTimeout = cntTimeout + 1
                -- printToConsole ("accept timeout")
                -- serverOn = false
                -- srv:close()
            else
                clt1:settimeout(connexionTimeout)
                dataIn = readSocketData(clt1)
                if dataIn ~= nil then
                    -- printToConsole (dataIn)
                    targetCmd = sim.unpackFloatTable(dataIn)
                    -- printToConsole (targetCmd[1],targetCmd[2],targetCmd[3],targetCmd[4])
                    doLog = targetCmd[1]
                    serverUpdateLog = true
                    speedLeft = defineLeftSpeed(targetCmd[2], targetCmd[4])
                    speedRight = defineRightSpeed(targetCmd[3], targetCmd[4])
                    -- Pack the data as a string:srv:close()
                    dataPacked = sim.packFloatTable(dataOut)
                    -- Send the data:
                    writeSocketData(clt1, dataPacked)
                    clt1:send(dataIn)
                else
                    printToConsole("no data")
                end
                clt1:close()
            end
        end

        -- manage log
        if serverUpdateLog then
            if doLog == 1.0 and not logOn then
                -- open new log file and start to log data
                printToConsole("start logFile", logFile)
                file = io.open(logFile, "w")
                if file == nil then
                    logOn = false
                else
                    file:write("Hello!", "\n")
                    file:close()
                    logOn = true
                end
            end
            if doLog == 0.0 and logOn then
                -- stop data log in file
                printToConsole("stop log")
                logOn = false
            end
        end

        t7 = gettime()

        msg = "Duration is " .. roundDecimal((t7 - t0) * 1000.0, 2)
        msg = msg .. "  -  " .. roundDecimal((t1 - t0) * 1000.0, 2)
        msg = msg .. "  -  " .. roundDecimal((t2 - t1) * 1000.0, 2)
        msg = msg .. "  -  " .. roundDecimal((t3 - t2) * 1000.0, 2)
        msg = msg .. "  -  " .. roundDecimal((t4 - t3) * 1000.0, 2)
        msg = msg .. "  -  " .. roundDecimal((t5 - t4) * 1000.0, 2)
        msg = msg .. "  -  " .. roundDecimal((t6 - t5) * 1000.0, 2)
        msg = msg .. "  -  " .. roundDecimal((t7 - t6) * 1000.0, 2)
        if validIteration then
            msg = msg .. " - valid is true"
        else
            msg = msg .. " - valid is false"
        end
        -- printToConsole(msg)

        -- local t1 = gettime()
        -- local t1s = sim.getSimulationTime()      
        -- printToConsole ("t1 : ",t1-t0,t1s-t0s,(t1-t0)/(t1s-t0s))
        -- printToConsole (msg)
        -- sim.addStatusbarMessage (msg)
        -- sim.auxiliaryConsolePrint(consoleHandle,msg.."\n")

        -- Now don't waste time in this loop if the simulation time hasn't changed! This also synchronizes this thread with the main script
        -- 2022 remove switchThread as it doubles the loop duration (e.g. 400 ms if set to 200 ms)
        -- sim.switchThread() -- This thread will resume just before the main script is called again

        -- local t2 = gettime()
        -- local t2s = sim.getSimulationTime()      
        -- printToConsole ("t2 : ",t2-t0,t2s-t0s,(t2-t0)/(t2s-t0s))
        -- stats = client:getstats()
        -- printToConsole ("stats : "..stats)
        -- printToConsole ("close connexion ...")
        -- client:close()
        -- printToConsole ("connexion closed")

    end

end
