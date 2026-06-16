--[[
Expected setup:

A computer, front facing the same way as contraption,
with blocks on the other three sides. On top of each block is redstone dust,
and on top of the computer is a navigation table. In the navigation table
is a Create Simulated magnet (to point north).

Networked to the computer is a redstone link, where the left and right inputs
yaw to the left and right, respectively.

Compasses will NOT work because they point toward world spawn.

This will attempt to keep the craft facing due NORTH. A regular alignment allows
the positional controller an easier job.
--]]

local relay = peripheral.find("redstone_relay")

---Normalize radian angle to (-pi, pi]
---@param x number
---@return number
local function normalize_radian_angle(x)
    local x_mod_2pi = x % (2 * math.pi)
    if x_mod_2pi > math.pi then
        return x_mod_2pi - 2 * math.pi
    end
    return x_mod_2pi
end

---Normalize degree angle to (-180, 180]
---@param theta number
---@return number
local function normalize_degree_angle(theta)
    local theta_mod_360 = theta % 360
    if theta_mod_360 > 180 then
        return theta_mod_360 - 360
    end
    return theta_mod_360
end

local function minecraft_bearing_from_heading(x)
    local heading_degrees = x * 180 / math.pi
    return normalize_degree_angle(
        0                 -- Start facing south
        - 90              -- Turn left to face east
        + heading_degrees -- Add heading (which is angle clockwise from east)
    )
end

local NAV_TABLE_INPUT_SUM = 15

---Get ship yaw as an angle clockwise from east, in radians
local function get_yaw()
    local north_direction_forward
    local north_direction_right
    do
        local right = rs.getAnalogInput("left") -- Computer left/right perspective is backwards
        local left = rs.getAnalogInput("right")
        local back = rs.getAnalogInput("back")
        local forward = NAV_TABLE_INPUT_SUM - (right + left + back)

        north_direction_forward = forward - back
        north_direction_right = right - left
    end

    local angle_north_from_yaw = math.atan(
        north_direction_right, north_direction_forward
    ) -- Use forward as X, right as Y for clockwise angles


    local angle_yaw_from_north = -angle_north_from_yaw
    local angle_yaw_from_east =
        0                      -- Start east
        - math.pi / 2          -- Left turn to face north
        + angle_yaw_from_north -- Add angle to ship
    return normalize_radian_angle(angle_yaw_from_east)
end

while true do
    local yaw = get_yaw()
    print(minecraft_bearing_from_heading(yaw))

    relay.setOutput("left", false)
    relay.setOutput("right", false)

    local NORTH = -math.pi / 2

    local angle_yaw_from_north = normalize_radian_angle(yaw - NORTH)
    if angle_yaw_from_north < 0 then
        relay.setOutput("right", true)
    else
        relay.setOutput("left", true)
    end
    sleep(0.05)
end
