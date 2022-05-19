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

function update()

  local sensor_count = rangefinder:num_sensors() -- number of rangefinders connected (includes all orientations)
  local same_direction_sensor_count = 0 -- number of rangefinders in the direction we are interested in

  local distances_sum = 0 -- sum of distances from each sensor looking in the same direction (used for average)
  local current_distance_m = 0 --current distance in meter 

  local lua_driver_found = false  --true if user has configured lua rangefinder backend (RNGFNDX_TYPE = 35)
  local lua_backend    -- store lua rangefinder backend here

  for j = 0, sensor_count do
    local device = rangefinder:get_backend(j)
    if ((not lua_driver_found) and  device and (device:type() == param_num_lua_backend)) then
      -- this is a lua driver
      lua_driver_found = true
      lua_backend = device
      --use this device as the primary sensor
      lua_backend:set_primary(true)
    end
  end
  if not lua_driver_found then
    -- We can't use this script if user hasn't setup a lua rangefinder
    gcs:send_text(0, string.format("Configure Lua RangeFinder"))
    return update, update_rate_error_ms
  end

  for i = 0, sensor_count do
    local device = rangefinder:get_backend(i)
    if (device and device:type() ~= param_num_lua_backend) then
      -- this backend isn't lua script
      if (device:orientation() == rotation_downward) then
        -- found a sensor in the down direction
        current_distance_m = device:distance()

        if (device:status() ~= rangefinder_status_good_num ) then
          -- device status isn't good
          if (device:has_data()) then
            -- probably just out of range
            -- return the same dist for lua driver as we expect all devices to be healthy
            -- this will turn the status driver to be out of range
            local sent_successfully = lua_backend:handle_script_msg(current_distance_m)
            if not sent_successfully then
              -- This should never happen as we already checked for a valid configured lua backend above
              gcs:send_text(0, string.format("RangeFinder Lua Script Error"))
              return update, update_rate_error_ms
            end
            return update, update_rate_ms
          else
            -- device is not conncted, do not send anything.
            return update, update_rate_error_ms
          end
        end

        same_direction_sensor_count = same_direction_sensor_count + 1

        -- distances_sum = distances_sum + dist_m

        -- if ((min_element == 0) and (max_element == 0)) then
        --     -- first driver, init the variables
        --     min_element = dist_m
        --     max_element = dist_m
        -- end

        -- -- find min and max of each driver
        -- if (dist_m < min_element) then
        --   min_element = dist_m
        -- end
        -- if (dist_m > max_element) then
        --   max_element = dist_m
        -- end
      end
    end
  end


  if (same_direction_sensor_count == 0) then
    -- no rangefinder in the required direction, do not send anything (which will trigger a pre arm error)
    gcs:send_text(0, string.format("No Range Finder Found"))
    return update, update_rate_error_ms
  end
  
  local target = Vector2f()
  local target_found = precland:get_target_position_relative_cm(target)


  if (target_found) then
    if (target:length() < 55) then
      -- target is very close expect jumps in readings
      if (current_distance_m < 0.40) then
        -- this should never happen
        current_distance_m = current_distance_m + 0.30
      end
    end
  end
  

--   if (math.abs(max_element - min_element) > max_allowed_diff_m) then
--     --skip these readings
--     gcs:send_text(0, string.format("Sensor values do not match"))
--   else
    -- send the average of all sensors
  local sent_successfully = lua_backend:handle_script_msg(current_distance_m)
  if not sent_successfully then
    -- This should never happen as we already checked for a valid configured lua backend above
    gcs:send_text(0, string.format("RangeFinder Lua Script Error"))
    return update, update_rate_error_ms
  end


  return update, update_rate_ms -- check again in 25ms
end

return update(), 5000 -- first data may be  checked 5 seconds after start-up