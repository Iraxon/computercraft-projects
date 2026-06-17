local args = {...}
local targ_x = args[1] or 0
local targ_y = args[2] or 250
local targ_z = args[3] or 0

---Calculate the desired velocity on one axis, given the available
---positive and negative accelerations, the current position, the
---target position, and optionally, a safety margin to make overshoot less likely
---
---@param positive_acceleration number m/s^2, positive
---@param negative_acceleration number m/s^2, negative
---@param position number m
---@param target number m
---@param safety_factor number | nil dimensionless, 0 < safety_factor <= 1, default 0.75
---@return number
local function get_desired_velocity(
    positive_acceleration,
    negative_acceleration,
    position,
    target,
    safety_factor
)
    --Check signs
    assert(positive_acceleration >= 0)
    assert(negative_acceleration <= 0)

    --If we got zeroes or wrong signs, command velocity 1 toward the target. We can't do
    --any more complex prediction until that information is attained.
    if positive_acceleration == 0 or negative_acceleration == 0 then
        return (target - position) / math.abs(target - position)
    end

    local relative_position = position - target

    safety_factor = safety_factor or 0.75

    -- If we are very close to the target we command a stop
    if math.abs(relative_position) < 2 then
        return 0
    end

    if relative_position < 0 then
        return math.sqrt(2 * (safety_factor * negative_acceleration) * relative_position)
    else
        return -math.sqrt(2 * (safety_factor * positive_acceleration) * relative_position)
    end
    -- Derived from Toricelli's equation.
    -- Assume an ideal deceleration, starting
    -- from some x0 with some v0 and proceeding
    -- to xf = 0 in under constant deceleration a.

    -- vf^2 = v0^2 + 2 * a * (xf - x0)
    -- 0 = v0^2 + 2 * a * (0 - x0)
    -- 0 = v0^2 - 2 * a * x0
    -- v0^2 = 2 * a * x0

    -- Of the these, one pair will be both positive,
    -- the other all negative.
    -- [position, acceleration]
    -- [displacement, velocity]
end

---Wrap a thrust subroutine to track its effect
---@param subroutine fun(): nil
---@param measure_velocity fun(): number
---@param filter_acceleration_measure nil | fun(number): boolean
local function wrap_thrust_subroutine(subroutine, measure_velocity, filter_acceleration_measure)
    filter_acceleration_measure =
        filter_acceleration_measure
        or function(measured_acceleration) return math.abs(measured_acceleration) > 1 end

    local acceleration = 0

    ---Run the subroutine and measure its effect
    ---@param time number
    ---@return number acceleration
    local function run(time)
        local v0 = measure_velocity()
        local t0 = os.clock()
        subroutine()
        sleep(time)
        local v = measure_velocity()
        local t = os.clock()
        local measured_acceleration = (v - v0) / (t - t0)
        if filter_acceleration_measure(measured_acceleration) then
            acceleration = measured_acceleration
        end
        return acceleration
    end

    return {
        run = run,
        get_acceleration = function()
            return acceleration
        end
    }
end

-- BEGIN MAIN CODE

local relays = { peripheral.find("redstone_relay") }


---Locate the center of mass of this sublevel in global space
---@return {x: number, y: number, z: number}
local function locate_sublevel()
    if sublevel.isInPlotGrid() then
        return sublevel.getLogicalPose().position
    else
        return vector.new(0, 0, 0)
    end
end

local up
local down
local function get_py() return locate_sublevel().y end
local function get_vy() return sublevel.getVelocity().y end

do
    local function raw_up()
        for _, relay in ipairs(relays) do
            relay.setOutput("bottom", true)
        end
    end
    local function raw_down()
        for _, relay in ipairs(relays) do
            relay.setOutput("bottom", false)
        end
    end

    up = wrap_thrust_subroutine(raw_up, get_vy, function(measured_acceleration) return measured_acceleration > 1 end)
    down = wrap_thrust_subroutine(raw_down, get_vy,
        function(measured_acceleration) return measured_acceleration < -11 end)
    -- Stricter filter on down because we know it's at least as strong as gravity
end

local east
local west
local function get_px() return locate_sublevel().x end
local function get_vx() return sublevel.getVelocity().x end
do
    local function raw_east()
        for _, relay in ipairs(relays) do
            relay.setOutput("right", false)
            relay.setOutput("left", true)
        end
    end
    local function raw_west()
        for _, relay in ipairs(relays) do
            relay.setOutput("left", false)
            relay.setOutput("right", true)
        end
    end

    east = wrap_thrust_subroutine(raw_east, get_vx, function(measured_acceleration) return measured_acceleration > 1 end)
    west = wrap_thrust_subroutine(raw_west, get_vx, function(measured_acceleration) return measured_acceleration < -1 end)
end

local north
local south
local function get_pz() return locate_sublevel().z end
local function get_vz() return sublevel.getVelocity().z end
do
    local function raw_north()
        for _, relay in ipairs(relays) do
            relay.setOutput("front", false)
            relay.setOutput("back", true)
        end
    end
    local function raw_south()
        for _, relay in ipairs(relays) do
            relay.setOutput("back", false)
            relay.setOutput("front", true)
        end
    end

    north = wrap_thrust_subroutine(raw_north, get_vz,
        function(measured_acceleration) return measured_acceleration < -1 end)
    south = wrap_thrust_subroutine(raw_south, get_vz,
        function(measured_acceleration) return measured_acceleration > 1 end)
end

local TIME_DELTA = 0.05

parallel.waitForAny(
    function()
        while true do
            local vy_desired = get_desired_velocity(
                up.get_acceleration(),
                down.get_acceleration(),
                get_py(),
                targ_y
            )

            if get_vy() < vy_desired then
                up.run(TIME_DELTA)
            else
                down.run(TIME_DELTA)
            end
        end
    end,
    function()
        while true do
            local vx_desired = get_desired_velocity(
                east.get_acceleration(),
                west.get_acceleration(),
                get_px(),
                targ_x
            )

            if get_vx() < vx_desired then
                east.run(TIME_DELTA)
            else
                west.run(TIME_DELTA)
            end
        end
    end,
    function()
        while true do
            local vz_desired = get_desired_velocity(
                south.get_acceleration(),
                north.get_acceleration(),
                get_pz(),
                targ_z
            )

            if get_vz() < vz_desired then
                south.run(TIME_DELTA)
            else
                north.run(TIME_DELTA)
            end
        end
    end
)

local function shutdown()
    for _, relay in ipairs(relays) do
        relay.setOutput("front", false)
        relay.setOutput("left", false)
        relay.setOutput("right", false)
    end
end

shutdown()
