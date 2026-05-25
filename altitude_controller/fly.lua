local target_altitude = tonumber(...) or 60 -- Command line argument



--Sublevel GPS
--region

---Locate the center of mass of this sublevel in global space
---@return {x: number, y: number, z: number} | nil
local function locate_sublevel()
    if sublevel.isInPlotGrid() then
        return sublevel.getLogicalPose().position()
    else
        return nil
    end
end

--endregion


local GRAVITY = aero.getGravity().y

---Whether the craft should apply thrust upward right now
---@param vy number
---@return boolean
local function shouldApplyThrust(vy, target_altitude)
    local altitude = locate_sublevel().y

    if math.abs(altitude - target_altitude) <= 2 then
        return altitude < target_altitude
    end

    if vy < 0 and altitude < target_altitude then
        return true
    end
    local apex_if_thrust_stopped = altitude + (vy * vy) / (-2 * GRAVITY)
    return apex_if_thrust_stopped < target_altitude
end

local DT = 1 / 20

while true do
    local v = sublevel.getLinearVelocity().y
    local should_apply = shouldApplyThrust(v, target_altitude)
    redstone.setOutput("bottom", should_apply)

    sleep(DT)
end
