--[[
Name: Terrain Switch Height Calculation based on Yaw Rate and Velocity
Authors: Rishabh Singh

Terrain Switch Height Calculation based on Yaw Rate and Velocity
Terrain height will switch to use max of all grid points if vehcile velocity > TERRAIN_OPT_VEL
Terrain height will switch to use max of all grid points if vehcile yaw rate > TERRAIN_OPT_YAW

--]]

local scripting_rc1 = assert(rc:find_channel_for_option(300),"Lua: Could not find switch")
local prev_terrain_state = 0

PARAM_TABLE_KEY = 100
PARAM_TABLE_PREFIX = "TERRAIN_OPT_"


-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end


TERRAIN_OPTIONS = bind_param('TERRAIN_OPTIONS')
local terrain_options_orignal = TERRAIN_OPTIONS:get()

-- Setup EFI Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 15), 'could not add EFI_SP param table')

--[[
  // @Param: TERRAIN_OPT_VEL
  // @DisplayName: Terrain Switch Height Calculation based on Velocity
  // @Description: Terrain height will switch to use max of all grid points if vehcile velocity > TERRAIN_OPT_VEL in m/s
  // @Range: 1 30
  // @User: Standard
--]]
local TERRAIN_OPT_VEL     = bind_add_param('VEL',     1, 0)

--[[
  // @Param: TERRAIN_OPT_YAW
  // @DisplayName: Terrain Switch Height Calculation based on Yaw Rate
  // @Description: Terrain height will switch to use max of all grid points if vehcile yaw rate > TERRAIN_OPT_YAW in deg/s
  // @RangeL 5 30
  // @User: Standard
--]]
local TERRAIN_OPT_YAW     = bind_add_param('YAW',     2, 0)


-- logic to see if we need to switch terrain height calculations
function switch_terrain_max()
   local ef_yaw_rate = ahrs:get_yaw_rate_earth()
   local max_yaw_rate = TERRAIN_OPT_YAW:get()
   if (ef_yaw_rate and (max_yaw_rate > 0)) then
      if (math.abs(ef_yaw_rate) > math.rad(max_yaw_rate)) then
         -- we want to switch
         return true
      end
   end

   local vel = ahrs:get_velocity_NED()
   local max_vel =  TERRAIN_OPT_VEL:get()
   if (vel and (max_vel > 0)) then
      if vel:xy():length() > TERRAIN_OPT_VEL:get() then
         -- we are moving fast, so we want to switch
         return true
      end
   end

   return false
end


function update()
   if not arming:is_armed() then
      -- not armed, we do not want to run this then
      TERRAIN_OPTIONS:set(terrain_options_orignal)
      return
   end

   local sw_pos = scripting_rc1:get_aux_switch_pos()
   if sw_pos == 0 then
      TERRAIN_OPTIONS:set(terrain_options_orignal)
      return
   end

   if (switch_terrain_max()) then
      -- we need to switch terrain source
      TERRAIN_OPTIONS:set(terrain_options_orignal | 2)
      if (prev_terrain_state == 0) then
         -- we just switched
         prev_terrain_state = 1
         gcs:send_text(5, "Terrain Switched to use Max Height")
         return
      end
      return
   else
      TERRAIN_OPTIONS:set(terrain_options_orignal &~ 2)
      if (prev_terrain_state == 1) then
         -- we just switched
         prev_terrain_state = 0
         gcs:send_text(5, "Terrain switched to use Avg Height")
         return
      end
      return
   end
end


-- wrapper around update(). This calls update() at 2Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
   local success, err = pcall(update)
   if not success then
      gcs:send_text(0, "Internal Error: " .. err)
      -- when we fault we run the update function again after 1s, slowing it
      -- down a bit so we don't flood the console with errors
      return protected_wrapper, 1000
   end
   return protected_wrapper, 500
 end

 -- start running update loop
 return protected_wrapper()
 