-- move a servo in a sinisoidal fashion

local SERVO_NO = 94
local last_servo = 0
local rc_channel = 8

-- bottom right - SERVO5 - SCR 94
-- bottom left - SERVO6 - SCR 95
-- top left - SERVO7 - SCR 96
-- top right - SERVO8 - SCR 97

local h_config = {1200, 1200, 1200, 1200}
local plus_config = {1800, 1800, 1800, 1800}
local pwm_array = {1500,1500,1500,1500} -- bottom_right, bottom _left, top_left, top_right

function update() -- this is the loop which periodically runs

  local rc_pwm = rc:get_pwm(rc_channel)
  gcs:send_text(0, string.format("pwm: %0.2f",rc_pwm))

  if rc_pwm < 900 or rc_pwm > 2100 then
    return update, 25
  end

  -- local bottom_right = rc_pwm
  -- local bottom_left = rc_pwm - reverse
  -- local top_left = rc_pwm
  -- local top_right = rc_pwm - reverse
  if rc_pwm < 1300 then
      rc_pwm = 1300
  end
  if rc_pwm > 1700 then
    rc_pwm = 1700
  end

  -- if rc_pwm < 1700 and rc_pwm > 1300 then
  --   rc_pwm = 1500
  -- end
  pwm_array[0] = rc_pwm
  pwm_array[1] = 3000 - rc_pwm
  pwm_array[2] = rc_pwm
  pwm_array[3] = 3000 - rc_pwm

  for i = 0,4 do
    SRV_Channels:set_output_pwm(SERVO_NO+i, pwm_array[i])
  end
  
  gcs:send_text(0, string.format("pwm: %0.2f",rc_pwm))

  return update, 25

end

return update() -- run immediately before starting to reschedule
