local target_altitude = tonumber(...) or 60 -- Command line argument

local gravity = aero.getGravity().y
local altitude_sensor = peripheral.find("altitude_sensor")

---Whether the craft should apply thrust upward right now
---@param velocity number
---@return boolean
local function shouldApplyThrust(velocity, target_altitude)
    local altitude = altitude_sensor.getHeight()

    if math.abs(altitude - target_altitude) <= 7 then
        return altitude < target_altitude
    end

    if velocity < 0 and altitude < target_altitude then
        return true
    end
    local apex_if_thrust_stopped = altitude + (velocity * velocity) / (-2 * gravity)
    return apex_if_thrust_stopped < target_altitude
end

local DT = 1 / 20

while true do
    local v = sublevel.getLinearVelocity().y
    local should_apply = shouldApplyThrust(v, target_altitude)
    redstone.setOutput("bottom", should_apply)

    sleep(DT)
end
