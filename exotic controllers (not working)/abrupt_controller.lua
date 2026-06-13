--Stochastic controller

--Initialize random
math.randomseed(os.time())


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

---Locate the center of mass of this sublevel in global space
---@return {x: number, y: number, z: number} | nil
local function locate_sublevel()
    if sublevel.isInPlotGrid() then
        return sublevel.getLogicalPose().position
    else
        return nil
    end
end

---A value that changes over time, where noise is smoothed out by averaging
---@param provider fun(): number
local function new_moving_average(provider)
    local current = provider()
    local previous1 = provider()
    local previous2 = provider()
    local previous3 = provider()

    ---Update the moving average
    local function update()
        previous3 = previous2
        previous2 = previous1
        previous1 = current
        current = provider()
    end

    ---Get the moving average
    ---@param n number | nil How many recorded values to use in the average
    ---@return number
    local function get(n)
        n = n or 2
        local sum = 0
        local values = { current, previous1, previous2, previous3 }
        for i = 1, n, 1 do
            sum = sum + values[i]
        end
        return sum / n
    end

    return {
        update = update,
        get = get
    }
end

---A value that changes over time. Keeps track of previous value.
---Supports querying current and previous values as well as finite difference.
---@param provider fun(): number
local function new_delta_value(provider)
    local previous = provider()
    local current = provider()
    local delta = 0
    local smooth_delta = new_moving_average(function() return delta end)

    ---Update the value. Returns the new value.
    ---@return number
    local function update()
        previous = current
        current = provider()
        delta = current - previous
        smooth_delta.update()
        return current
    end


    local function unsigned_delta()
        return math.abs(delta)
    end

    return {
        update = update,
        current = function() return current end,
        delta = function() return delta end,
        smooth_delta = smooth_delta.get,
        get = current
    }
end

---Keep track of a subroutine and a number describing its effect
---@param subroutine fun(): nil
---@param measure_effect fun(): number
local function _action_of(subroutine, measure_effect)
    local self = {
        subroutine = subroutine,
        effect = 0
    }

    ---Get the effect value for this action
    ---@return number
    local function get_effect()
        return self.effect
    end

    ---Run this action
    local function run()
        subroutine()
        self.effect = measure_effect()
    end

    ---Reduce the effect. Use to prevent erroneous conclusions
    ---that this actions is bad from sticking.
    local function decay_effect()
        self.effect = 0.75 * self.effect
    end

    return {
        get_effect = get_effect,
        run = run,
        decay_effect = decay_effect
    }
end

---Construct an abrupt ("bang-bang" or two-state) controller.
---Assumption: The two subroutines each impose a finite change in the
---error number, in opposite directions.
---@param error_function fun(): number
---@param subroutine1 fun(): nil
---@param subroutine2 fun(): nil
local function abrupt_controller(error_function, subroutine1, subroutine2)
    local using_first = true

    local error = new_delta_value(error_function)
    local error_delta = new_delta_value(error.delta)
    local other_subroutine_error_delta = 0

    local switch_cooldown = 0 -- In increments of TIME_DELTA

    local function act()
        if using_first then
            subroutine1()
        else
            subroutine2()
        end
    end

    local function change_subroutines()
        using_first = not using_first
        other_subroutine_error_delta = error_delta.current()
    end

    local TIME_DELTA = 0.05

    local function loop()
        act()
        sleep(TIME_DELTA)
        while true do
            local current_error = error.update()
            local current_error_delta = error_delta.update()

            print("Error", current_error, "Error delta", current_error_delta)

            if
                switch_cooldown == 0
                and
                current_error_delta * current_error > other_subroutine_error_delta * current_error
            then
                -- We switch subroutines if the error is
                -- getting worse (moving away from zero) right now
                -- at a rate worse than the other subroutine would cause
                change_subroutines()
                -- Prevent switching for a bit to give the
                -- new subroutine time to affect the error
                switch_cooldown = 2
            end

            act()

            switch_cooldown = math.max(switch_cooldown - 1, 0)

            sleep(TIME_DELTA)
        end
    end

    return {
        loop = loop
    }
end

---Construct a second order abrupt controller. Use this when your subroutines,
---instead of directly changing the error in steps, changes the veocity of the error.
---
---@param get_forward_error fun(): number Sign should match the direction FROM ship to target
---@param subroutine1 fun(): nil
---@param subroutine2 fun(): nil
local function second_order_abrupt_controller(get_forward_error, subroutine1, subroutine2)
    -- We convert the problem into managing the velocity, which a normal abrupt controller
    -- can do. All we have to do is calculate what the velocity should be given the
    -- positional error.

    local forward_positional_error = new_delta_value(get_forward_error)
    local function get_velocity_error()
        local current_pos_error = forward_positional_error.update()
        local current_velocity = -forward_positional_error.delta()
        -- Negation because it's forward error
        -- Think: Target at positive offset. We approach. Positional error is decreasing,
        -- but our velocity is positive

        -- Choose velocity proportional to positional error,
        -- so that we slow down as we approach.
        local desired_velocity = current_pos_error

        print("Real velocity", current_velocity, "with target", desired_velocity)

        return desired_velocity - current_velocity
    end

    return abrupt_controller(get_velocity_error, subroutine1, subroutine2)
end



--BEGIN MAIN CODE HERE

local relay = peripheral.find("redstone_relay")

local altitude_controller = second_order_abrupt_controller(
    function() return locate_sublevel().y - 250 end,
    function() relay.setOutput("front", true) end,
    function() relay.setOutput("front", false) end
)

---Calcualte a - b for radian angles, giving a difference within +/- pi
---@param a number
---@param b number
---@return number
local function angle_difference(a, b)
    return ((a - b + math.pi) % (2 * math.pi)) - math.pi -- Normalize the range
end

local function get_yaw_offset()
    local heading_vector = sublevel.getVelocity()
    -- Craft is not moving fast enough for us to tell where it's going
    if heading_vector.x * heading_vector.x + heading_vector.z * heading_vector.z < 15 * 15 then
        return 0
    end
    local heading_angle = math.atan(heading_vector.z, heading_vector.x)
    local target_vector = vector.new(0, 0, 0) - locate_sublevel()
    local target_angle = math.atan(target_vector.z, target_vector.x)

    return angle_difference(target_angle, heading_angle)
end

--Positive is right
-- local yaw_controller = second_order_abrupt_controller(
--     get_yaw_offset,
--     function() -- Turn right
--         relay.setOutput("left", true)
--         relay.setOutput("right", false)
--     end,
--     function() -- Turn left
--         relay.setOutput("left", false)
--         relay.setOutput("right", true)
--     end
-- )

-- DUMMY YAW CONTROLLER so we can test altitude alone
local yaw_controller = {
    loop = function()
        while true do sleep(100) end
    end
}

parallel.waitForAny(
    altitude_controller.loop,
    yaw_controller.loop
)
