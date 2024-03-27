local m2ft = 3.28

local dist1
local dist2
local distBack
local distFwd
local alignError
local actualDist
local error = 0
local errorOld = 0

local rover_guided_mode_num = 15
local steeringOut

local PARAM_TABLE_KEY = 73

assert(param:add_table(PARAM_TABLE_KEY, "TRACK_", 7), 'could not add param table') -- Track Wall parameter table 

-- create two parameters. The param indexes (2nd argument) must
-- be between 1 and 63. All added parameters are floats, with the given
-- default value (4th argument).
assert(param:add_param(PARAM_TABLE_KEY, 1, 'P', 0.2), 'could not add TRACK_P') -- Proportional Gain for Controller
assert(param:add_param(PARAM_TABLE_KEY, 2, 'DIST', 1), 'could not add TRACK_DIST') -- Distance to hold from wall (ft)
assert(param:add_param(PARAM_TABLE_KEY, 3, 'SP', 1), 'could not add TRACK_SP') -- Maximum throttle setting for constant speed
assert(param:add_param(PARAM_TABLE_KEY, 4, 'STR',0.25), 'could not add TRACK_STR') -- Steering limit
assert(param:add_param(PARAM_TABLE_KEY, 5, 'D', 0.1), 'could not add TRACK_D') -- Derivative Gain for Controller
assert(param:add_param(PARAM_TABLE_KEY, 6, 'OBS_DIST', 3), 'could not add TRACK_OBS_DIST') -- Distance within object to stop (ft)
assert(param:add_param(PARAM_TABLE_KEY, 7, 'ROLL_SP', 365), 'could not add TRACK_ROLL_SP') -- Speed for roller (PWM)

local pGain = param:get('TRACK_P') -- Proportional Gain for Controller
local offsetDist = param:get('TRACK_DIST') -- Distance to hold from wall (ft)
local cruiseSpeed = param:get('TRACK_SP') -- Maximum throttle setting for constant speed
local limSteer = param:get('TRACK_STR') -- Steering limit
local dGain = param:get('TRACK_D') -- Derivative Gain for Controller
local avoidThresh = param:get('TRACK_OBS_DIST') -- Derivative Gain for Controller
local spinnerOffset = param:get('TRACK_ROLL_SP') -- Speed for roller (PWM)
local dError = 0 -- Derivative of Error
local alignThresh = 3 -- cm threshold

local directionRC = rc:find_channel_for_option(300)
local rad2deg = 180/math.pi
local updateRate = 100

local directionRC_pos = 0
local directionRC_posOld = 0

local startFlag

local alignCmd = -0.18
local rng1_offset = 0 -- offset in cm
local rng2_offset = 0 -- offset in cm

local spinnerChannel = rc:find_channel_for_option(301)
local spinnerTrim = 1500
local spinnerCmd = spinnerTrim

local noWallThresh = 3 -- Threshold for no wall detection to trigger stop

--vehicle:set_mode(rover_guided_mode_num)

function averageDist(range1, range2)
  local  rangeAve = (range1+range2)/2
  return rangeAve
end

function alignVehicle()
  dist1 = ((rangefinder:distance_cm_orient(7)+rng1_offset)/100)*m2ft
  dist2 = ((rangefinder:distance_cm_orient(5)+rng2_offset)/100)*m2ft
  --dist1 = offsetDist
  --dist2 = offsetDist 
  alignError = dist2 - dist1
  spinnerCmd = spinnerTrim
  spinnerChannel:set_override(spinnerCmd)
  if math.abs(alignError) > (alignThresh/100)*m2ft then
    gcs:send_text(6, "FL: " .. tostring(dist1).." RL: "..tostring(dist2))
    if alignError > 0 then
      vehicle:set_steering_and_throttle(alignCmd, 0.0)
    else
      vehicle:set_steering_and_throttle(-alignCmd, 0.0)
    end
    return alignVehicle, 1
  else
    vehicle:set_steering_and_throttle(0.0, 0.0)
    gcs:send_text(6, "Vehicle is Aligned")
    -- param:set('TRACK_DIST', actualDist)
    --offsetDist = actualDist
    offsetDist = param:get('TRACK_DIST')
    gcs:send_text(6, "Tracking Wall at: "..tostring(offsetDist))
    return update, 2000
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
  offsetDist = param:get('TRACK_DIST')
  pGain = param:get('TRACK_P')
  cruiseSpeed = param:get('TRACK_SP')
  limSteer = param:get('TRACK_STR')
  dGain = param:get('TRACK_D')
  avoidThresh = param:get('TRACK_OBS_DIST')
  spinnerOffset = param:get('TRACK_ROLL_SP')
  directionRC_posOld = directionRC_pos
  directionRC_pos = directionRC:get_aux_switch_pos()
  -- gcs:send_text(6, "Switch Postition: "..tostring(directionRC_pos))
  if directionRC_pos == 2 then
    if directionRC_posOld~=directionRC_pos then
      gcs:send_text(6, "Reverse")
    end
    if cruiseSpeed > 0 then -- Check if cruise speed is incorrect
      param:set('TRACK_SP', -cruiseSpeed)
    end
    if pGain < 0 then
      param:set('TRACK_P', -pGain)
    end
    if dGain < 0 then
      param:set('TRACK_D', -dGain)
    end
  elseif directionRC_pos == 0 then
    if directionRC_posOld~=directionRC_pos then
      gcs:send_text(6, "Forward")
    end
    if cruiseSpeed < 0 then -- Check if cruise speed is incorrect
      param:set('TRACK_SP', -cruiseSpeed)
    end
    if pGain > 0 then
      param:set('TRACK_P', -pGain)
    end
    if dGain > 0 then
      param:set('TRACK_D', -dGain)
    end
  end
  pGain = param:get('TRACK_P')
  cruiseSpeed = param:get('TRACK_SP')
  dGain = param:get('TRACK_D')
end

function update()
  updateParams()
  dist1 = ((rangefinder:distance_cm_orient(7)+rng1_offset)/100)*m2ft
  dist2 = ((rangefinder:distance_cm_orient(5)+rng2_offset)/100)*m2ft
  --dist1 = offsetDist
  --dist2 = offsetDist 

  distFwd = ((rangefinder:distance_cm_orient(0))/100)*m2ft
  distBack = ((rangefinder:distance_cm_orient(4))/100)*m2ft
  --distFwd = avoidThresh + 1
  --distBack = avoidThresh + 1
  
  if arming:is_armed() == false then
    --gcs:send_text(6, "RNG FL: " .. tostring((dist1)).." RNG BL: "..tostring((dist2)).." RNG BACK: "..tostring((distBack)).." RNG FWD: "..tostring((distFwd)))
  end
  if arming:is_armed() and vehicle:get_mode() == rover_guided_mode_num and startFlag == 0 then
    startFlag = 1
    return alignVehicle, 1
  end
  if arming:is_armed() and vehicle:get_mode() == rover_guided_mode_num then
    actualDist = averageDist(dist1, dist2)
    errorOld = error
    error = offsetDist - actualDist
    dError = (error - errorOld)/(updateRate/1000)
    steeringOut = limitSteer(pGain*error + dGain*dError, limSteer)
    if (cruiseSpeed > 0 and distFwd > avoidThresh and actualDist < noWallThresh) or (cruiseSpeed < 0 and distBack > avoidThresh and actualDist < noWallThresh) then
      gcs:send_text(6, "Actual Dist: " .. tostring((actualDist)))
      --gcs:send_text(6, "Steering Out " .. tostring(steeringOut) .. " P: " .. tostring(pGain*error).." D: "..tostring(dGain*dError))
      if cruiseSpeed > 0 then
        spinnerCmd = spinnerTrim - spinnerOffset*0.95
      else
        spinnerCmd = spinnerTrim + spinnerOffset
      end
      --gcs:send_text(6, "Spinner Channel: "..tostring(spinnerChannel).." Set: "..tostring(spinnerCmd))
      spinnerChannel:set_override(spinnerCmd)
      vehicle:set_steering_and_throttle(steeringOut, cruiseSpeed)
    else
      spinnerCmd = spinnerTrim
      --gcs:send_text(6, "Spinner Channel: "..tostring(spinnerChannel).." Set: "..tostring(spinnerCmd))
      spinnerChannel:set_override(spinnerCmd)
      vehicle:set_steering_and_throttle(0.0, 0.0)
    end
    -- gcs:send_text(6, "Str" .. tostring(steeringOut) .. " Speed: " .. tostring(cruiseSpeed))
  else
    error = 0
    errorOld = 0
    -- Constantly override user input so that trim is set to 1500 on boot
    spinnerCmd = spinnerTrim
    spinnerChannel:set_override(spinnerCmd)
    
    if startFlag == 1 then
      spinnerCmd = spinnerTrim
      --gcs:send_text(6, "Spinner Channel: "..tostring(spinnerChannel).." Set: "..tostring(spinnerCmd))
      spinnerChannel:set_override(spinnerCmd)
    end
    startFlag = 0
  end
  return update, updateRate -- reschedules the loop
end

return update, 1000
