-- This is an example script for a custom Lua Script based RangeFinder driver
-- This script checks all RangeFinders connected in a particular direction. They must all report the distances within a set margin. 
-- If not, then all rangefinders are ignored till the values match again

-- User settable parameters
local rotation_downward = 25    --direction to look at (25 is down)
local update_rate_ms    = 25    -- update rate (in ms) of the driver
local update_rate_error_ms  = 5000 -- update rate incase there is a fatal error (in ms)

-- Global variables (DO NOT CHANGE)
local param_num_lua_backend = 35         -- parameter number for lua rangefinder
local rangefinder_status_good_num = 4    -- this number indicated "good" status for the rangefinder
local moving_average = 0                 -- short term moving average
local average_sampling_time_ms = 1000    -- time period to average rangefinder values
local average_start_time_ms = 0          -- time when averaging started
local average_height_m = 0               -- average hegiht which is maintained

local current_step = 0
local stopping_time = 0


local least_dist_seen = 0
local max_dist_seen = 0


function reset()
  least_dist_seen = 0
  max_dist_seen = 0
  current_step = 0
  stopping_time = 0
end

function go_down()
  if (precland:get_area() == 0) then
    precland:override_vel(0.0)
    return
  end


  local area = precland:get_area()
  least_dist_seen = math.max(least_dist_seen, area)
  gcs:send_text(0, string.format("max dist %0.2f" ,least_dist_seen))

  if (area < (180*180)) then
    precland:override_vel(-0.02)
  else
    precland:override_vel(0.0)
    current_step = 1
    stopping_time = millis()
    least_dist_seen = 0
  end
end

function go_up()
  if (not rangefinder:has_data_orient(rotation_downward)) then
    precland:override_vel(0.0)
    return
  end

  local down_reading_cm = rangefinder:distance_cm_orient(25)

  max_dist_seen = math.max(max_dist_seen, down_reading_cm)

  if (max_dist_seen < 35) then
    precland:override_vel(0.02)
  else
    precland:override_vel(0.0)
    current_step = 3
    stopping_time = millis()
    max_dist_seen = 0
  end

end


function update()

  if (precland:target_acquired()) then
    rc:run_aux_function(75,1)
  else
    reset()
    rc:run_aux_function(75,0)
  end

  if vehicle:get_mode() ~= 9 then
    reset()
    return update, update_rate_ms
  end




  precland:precland_override(true)

  local target = Vector2f()
  local target_found = precland:get_target_position_relative_cm(target)


  if (target_found) then
    if (target:length() > 25) then
      precland:override_vel(0.0)
      return update, update_rate_ms
    end
  else
    reset()
    return update, update_rate_ms -- check again in 25ms
  end

  gcs:send_text(0, string.format("current_step %0.2f" ,current_step))

  if (current_step == 0) then
    go_down()
  elseif (current_step == 1) then
    precland:override_vel(0.0)
    if ((millis() - stopping_time ) > 5000) then
      current_step = 2
    end
  elseif (current_step == 2) then
    go_up()
  elseif (current_step == 3) then
    precland:override_vel(0.0)
    if ((millis() - stopping_time ) > 5000) then
      reset()
      current_step = 0
    end

  end



  return update, update_rate_ms -- check again in 25ms
end

return update(), 5000 -- first data may be  checked 5 seconds after start-up