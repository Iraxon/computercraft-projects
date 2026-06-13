---Calculate the desired velocity on one axis, given the available
---positive and negative accelerations, the current position, the
---target position, and optionally, a safety margin to make overshoot less likely
---
---@param positive_acceleration number m/s^2 or rad/s^2, positive
---@param negative_acceleration number m/s^2 or rad/s^2, negative
---@param position number m or rad
---@param target number m or rad
---@param is_angular boolean | nil Whether this is using angles (enables better calculation of turning direction)
---@param safety_factor number | nil dimensionless, 0 < safety_factor <= 1, default 0.75
---@return number
local function get_desired_velocity(
    positive_acceleration,
    negative_acceleration,
    position,
    target,
    is_angular,
    safety_factor
)
    --Correct if we got swapped signs
    if positive_acceleration < 0 and negative_acceleration > 0 then
        local temp
        temp = positive_acceleration
        positive_acceleration = negative_acceleration
        negative_acceleration = temp
    end

    --If that wasn't enough something is really messed up and we need to error
    assert(positive_acceleration >= 0)
    assert(negative_acceleration <= 0)

    local relative_position
    if is_angular then
        relative_position = ((position - target + math.pi) % (2 * math.pi)) - math.pi
    else
        relative_position = position - target
    end

    safety_factor = safety_factor or 0.75

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
    filter_acceleration_measure = filter_acceleration_measure or function(effect) return math.abs(effect) > 1 end

    local acceleration = 0

    ---Run the subroutine and measure its effect
    ---@param time number
    ---@return number
    local function run(time)
        local before = measure_velocity()
        local start_time = os.clock()
        subroutine()
        sleep(time)
        local after = measure_velocity()
        local end_time = os.clock()
        local measured_acceleration = (after - before) / (end_time - start_time)
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

---Find the value in a table for which key_function is a minimum
---@generic T
---@param table [T]
---@param key_function fun(T): number
---@return T, number
local function min_by_key(table, key_function)
    local len = #table
    if len == 0 then
        error("Empty table passed to min_by_key")
    end

    local min = table[1]
    local min_key = key_function(min)

    for i = 2, len, 1 do
        local current = table[i]
        local current_key = key_function(current)
        if current_key < min_key then
            min = current
            min_key = current_key
        end
    end
    return min, min_key
end

-- BEGIN MAIN CODE

local relay = peripheral.find("redstone_relay")


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
local function altitude() return locate_sublevel().y end
local function vy() return sublevel.getVelocity().y end

do
    local function raw_up() relay.setOutput("front", true) end
    local function raw_down() relay.setOutput("front", false) end

    up = wrap_thrust_subroutine(raw_up, vy, function(measured_acceleration) return measured_acceleration > 1 end)
    down = wrap_thrust_subroutine(raw_down, vy, function(measured_acceleration) return measured_acceleration < -11 end)
    -- Stricter filter on down because we know it's at least as strong as gravity
end

local left
local right
---Right is negative
---@return number
local function get_yaw()
    local heading_vector = sublevel.getVelocity()
    return -math.atan(heading_vector.z, heading_vector.x)
end

---Right is negative
---@return number
local function get_target_yaw()
    local target_vector = vector.new(0, 0, 0) - locate_sublevel()
    return -math.atan(target_vector.z, target_vector.x)
end
---Right is negative
---@return number
local function get_angular_velocity()
    return sublevel.getAngularVelocity().y
end

do
    local function raw_left()
        relay.setOutput("left", false)
        relay.setOutput("right", true)
    end
    local function raw_right()
        relay.setOutput("left", true)
        relay.setOutput("right", false)
    end

    left = wrap_thrust_subroutine(raw_left, get_angular_velocity, function(measured_acceleration)
        return measured_acceleration > 0.5
    end)
    right = wrap_thrust_subroutine(raw_right, get_angular_velocity, function(measured_acceleration)
        return measured_acceleration < -0.5
    end)
end

local TIME_DELTA = 0.05

parallel.waitForAny(
    function()
        while true do
            local v_desired = get_desired_velocity(
                up.get_acceleration(),
                down.get_acceleration(),
                altitude(),
                250
            )

            if vy() < v_desired then
                up.run(TIME_DELTA)
            else
                down.run(TIME_DELTA)
            end
        end
    end,
    function()
        while true do
            if sublevel.getVelocity():length() < 3 then
                relay.setOutput("left", false)
                relay.setOutput("right", false)
            elseif get_yaw() < get_target_yaw() then
                left.run(TIME_DELTA)
            else
                right.run(TIME_DELTA)
            end
        end
    end
)

local function shutdown()
    relay.setOutput("front", false)
    relay.setOutput("left", false)
    relay.setOutput("right", false)
end

shutdown()


--TODO: Fix yaw going the wrong way
