local target_altitude = tonumber(...) or 60 -- Command line argument
local gravity = aero.getGravity().y

---Whether the craft should apply thrust upward right now
---@param velocity number
---@return boolean
local function shouldApplyThrust(velocity, target_altitude)
    local _, altitude, _ = gps.locate()

    if altitude == nil then
        print("No GPS station available")
        return velocity < 0
    else
        if velocity < 0 and altitude < target_altitude then
            return true
        end
        local apex_if_thrust_stopped = altitude + (velocity * velocity) / (-2 * gravity)
        return apex_if_thrust_stopped < target_altitude
    end
end

local DT = 1 / 10

while true do
    local v = sublevel.getLinearVelocity().y
    local should_apply = shouldApplyThrust(v, target_altitude)
    redstone.setOutput("bottom", should_apply)

    sleep(DT)
end
