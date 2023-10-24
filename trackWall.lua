local m2ft = 3.28
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

vehicle:set_mode(rover_guided_mode_num)

function limitSteer(input)
  local output = input
  if input > limSteer then
    output = limSteer
  elseif input < -limSteer then
    output = -limSteer
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
  start_button_new_state = button:get_button_state(start_button_number)

  -- the button has changes since the last loop
  if start_button_new_state ~= start_button_old_state then
    start_button_old_state = start_button_new_state
    if start_button_new_state  == trigger_button_state then
      actualDist = (rangefinder:distance_cm_orient(0)/100)*m2ft
      param:set('SCR_USER2', actualDist)
      offsetDist = actualDist
      gcs:send_text(6, "Tracking Wall at: "..tostring(offsetDist))
    end
  end

  if arming:is_armed() and start_button_new_state == trigger_button_state and vehicle:get_mode() == rover_guided_mode_num then
    actualDist = (rangefinder:distance_cm_orient(0)/100)*m2ft
    --gcs:send_text(6, "Measured " .. tostring(measuredDist) .. " Adjusted: " .. tostring(actualDist))
    errorOld = error
    error = offsetDist - actualDist
    dError = (error - errorOld)/(updateRate/1000)

    steeringOut = limitSteer(pGain*error + dGain*dError)

    gcs:send_text(6, "Steering Out " .. tostring(steeringOut) .. " P: " .. tostring(pGain*error).." D: "..tostring(dGain*dError))
    vehicle:set_steering_and_throttle(steeringOut, cruiseSpeed)
  else
    error = 0
    errorOld = 0
  end
  return update, updateRate -- reschedules the loop
end

return update()
