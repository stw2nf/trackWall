local m2ft = 3.28

local dist1
local dist2
local alignError
local actualDist
local error = 0
local errorOld = 0

local rover_guided_mode_num = 15
local steeringOut

local pGain = param:get('SCR_USER1') -- Proportional Gain for Controller
local offsetDist = param:get('SCR_USER2') -- Distance to hold from wall (ft)
local cruiseMax = param:get('SCR_USER3') -- Maximum throttle setting for constant speed
local limSteer = param:get('SCR_USER4') -- Steering limit
local dGain = param:get('SCR_USER5') -- Derivative Gain for Controller
local dError = 0 -- Derivative of Error
local alignThresh = 1 -- cm threshold

local speedRC = rc:find_channel_for_option(300)
local directionRC = rc:find_channel_for_option(301)
local cruiseSpeed

local start_button_number = 1 -- Start Button
local trigger_button_state = false
local start_button_new_state = button:get_button_state(start_button_number)
local start_button_old_state = start_button_new_state

local rad2deg = 180/math.pi
local updateRate = 100

local directionRC_pos = 0
local directionRC_posOld = 0

local startFlag

local pMult = 4
local rng1_offset = -5 -- offset in cm
local rng2_offset = 0 -- offset in cm
vehicle:set_mode(rover_guided_mode_num)

function averageDist(range1, range2)
  local  rangeAve = (range1+range2)/2
  return rangeAve
end

function alignVehicle()
  dist1 = ((rangefinder:distance_cm_orient(7)+rng1_offset)/100)*m2ft
  dist2 = ((rangefinder:distance_cm_orient(5)+rng2_offset)/100)*m2ft
  alignError = dist2 - dist1
  actualDist = averageDist(dist1, dist2)
  if math.abs(alignError) > (alignThresh/100)*m2ft then
    steeringOut = limitSteer(pMult*math.abs(pGain)*alignError, 0.25)
    gcs:send_text(6, "RNG1: " .. tostring(dist1).." RNG2: "..tostring(dist2))
    gcs:send_text(6, "Steer: " .. tostring(steeringOut))
    vehicle:set_steering_and_throttle(steeringOut, 0.0)
    return alignVehicle, 1
  else
    vehicle:set_steering_and_throttle(0.0, 0.0)
    gcs:send_text(6, "Vehicle is Aligned")
    param:set('SCR_USER2', actualDist)
    offsetDist = actualDist
    gcs:send_text(6, "Tracking Wall at: "..tostring(offsetDist))
    return update, 5000
  end
end

function limitSteer(input, limit)
  local output = input
  if input > limit then
    output = limit
  elseif input < -limit then
    output = -limit
  end
  return output
end

function updateParams()
  pGain = param:get('SCR_USER1')
  cruiseMax = param:get('SCR_USER3')
  limSteer = param:get('SCR_USER4')
  dGain = param:get('SCR_USER5')

  local speedRC_pos = speedRC:get_aux_switch_pos()
  if speedRC_pos == 0 then
    cruiseSpeed = cruiseMax
  elseif speedRC_pos == 1 then
    cruiseSpeed = 2*cruiseMax/3
  else
    cruiseSpeed = cruiseMax/3
  end
  directionRC_posOld = directionRC_pos
  directionRC_pos = directionRC:get_aux_switch_pos()
  --gcs:send_text(6, "Switch Postition: "..tostring(directionRC_pos))
  if directionRC_pos == 2 and directionRC_posOld ~= directionRC_pos then
    param:set('SCR_USER1', -pGain)
    param:set('SCR_USER3', -cruiseMax)
    param:set('SCR_USER5', -dGain)

    if param:get('SCR_USER3') > 0 then
      gcs:send_text(6, "Forward")
    else
      gcs:send_text(6, "Reverse")
    end
    
  end
end

function update()
  updateParams()
  gcs:send_text(6, "RNG1: " .. tostring((rangefinder:distance_cm_orient(7)+rng1_offset)).." RNG2: "..tostring((rangefinder:distance_cm_orient(5)+rng2_offset)))

  if arming:is_armed() and vehicle:get_mode() == rover_guided_mode_num and startFlag == 0 then
    startFlag = 1
    return alignVehicle, 1
  end

  if arming:is_armed() and vehicle:get_mode() == rover_guided_mode_num then
    
    
    dist1 = ((rangefinder:distance_cm_orient(7)+rng1_offset)/100)*m2ft
    dist2 = ((rangefinder:distance_cm_orient(5)+rng2_offset)/100)*m2ft

    actualDist = averageDist(dist1, dist2)

    --gcs:send_text(6, "Measured " .. tostring(measuredDist) .. " Adjusted: " .. tostring(actualDist))
    errorOld = error
    error = offsetDist - actualDist
    dError = (error - errorOld)/(updateRate/1000)

    steeringOut = limitSteer(pGain*error + dGain*dError, limSteer)

    gcs:send_text(6, "Steering Out " .. tostring(steeringOut) .. " P: " .. tostring(pGain*error).." D: "..tostring(dGain*dError))
    vehicle:set_steering_and_throttle(steeringOut, cruiseSpeed)
  else
    error = 0
    errorOld = 0
    startFlag = 0
  end
  return update, updateRate -- reschedules the loop
end

return update, 1000
