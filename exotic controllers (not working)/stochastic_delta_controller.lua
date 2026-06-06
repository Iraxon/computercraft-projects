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

---A value that changes over time. Keeps track of previous value.
---Supports querying current and previous values as well as finite difference.
---@param default number
local function new_delta_value(default)
    local self = {
        previous = default,
        current = default,
    }

    ---Update the value. Returns the given value.
    ---@param value number
    ---@return number
    local function update(value)
        self.previous = self.current
        self.current = value
        return value
    end

    local function previous()
        return self.previous
    end

    local function current()
        return self.current
    end

    local function delta()
        return current() - previous()
    end

    local function unsigned_delta()
        return math.abs(delta())
    end

    return {
        update = update,
        previous = previous,
        current = current,
        delta = delta,
        unsigned_delta = unsigned_delta,
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

---Construct a stochastic controller, which attempts to zero an error
---using a set of subroutines
---
---Each subroutine should directly move the error, without overshoot.
---Consider using a second order one if that doesn't describe your
---situation.
---
---@param get_error fun(): number
---@param subroutine1 fun(): nil
---@param subroutine2 fun(): nil
local function new_stochastic_controller(get_error, subroutine1, subroutine2)
    local error = new_delta_value(0.0)
    local error_delta = new_delta_value(0.0)

    local actions = {
        _action_of(subroutine1, error_delta.current),
        _action_of(subroutine2, error_delta.current)
    }
    local selected_action = actions[1]

    local TIME_DELTA = 1

    local function loop()
        while true do
            local current_error = error.update(math.abs(get_error()))
            local current_error_delta = error_delta.update(error.delta())

            print("Error", current_error)
            print("Delta Error", current_error_delta)


            local function square(x) return x * x end

            -- Select the action that will result in an error closest to zero.
            -- Include random variation so the craft will try new things when
            -- it thinks all its options do nothing.
            -- This will not cause erroneous choices with known information
            -- unless thrust-to-mass is extremely low.
            selected_action = min_by_key(
                actions,
                function(action)
                    return square(current_error + action.get_effect()) + (0.5 * math.random() - 0.25)
                end
            )

            for _, action in ipairs(actions) do
                if action == selected_action then
                    action.run()
                else
                    action.decay_effect()
                end
            end

            sleep(TIME_DELTA)
        end
    end

    return {
        loop = loop,
        get_error = function() return error.current() end,
        get_error_delta = function() return error_delta.current() end,
        get_action = function() return selected_action end
    }
end

---Construct a second order stochastic controller. Use this when your subroutines,
---instead of directly changing the error in steps, change the veocity of the error.
---
---@param get_forward_error fun(): number Sign should match the direction FROM ship to target
---@param subroutine1 fun(): nil
---@param subroutine2 fun(): nil
local function new_second_order_stochastic_controller(get_forward_error, subroutine1, subroutine2)
    -- We convert the problem into managing the velocity, which a normal stochastic controller
    -- can do. All we have to do is calculate what the velocity should be given the
    -- positional error.

    local positional_error = new_delta_value(get_forward_error())
    local function get_velocity_error()
        positional_error.update(get_forward_error())
        local current_velocity = -positional_error.delta()
        -- Negation because it's forward error
        -- Think: Target at positive offset. We approach. Positional error is decreasing,
        -- but our velocity is positive

        -- Choose velocity proportional to positional error,
        -- so that we slow down as we approach.
        local desired_velocity = positional_error.current()

        print("Real velocity", current_velocity, "with target", desired_velocity)

        return desired_velocity - current_velocity
    end

    return new_stochastic_controller(get_velocity_error, subroutine1, subroutine2)
end



--BEGIN MAIN CODE HERE

local relay = peripheral.find("redstone_relay")

local altitude_controller = new_second_order_stochastic_controller(
    function() return 250 - locate_sublevel().y end,
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
local yaw_controller = new_second_order_stochastic_controller(
    get_yaw_offset,
    function() -- Turn right
        relay.setOutput("left", true)
        relay.setOutput("right", false)
    end,
    function() -- Turn left
        relay.setOutput("left", false)
        relay.setOutput("right", true)
    end
)

parallel.waitForAny(
    altitude_controller.loop,
    yaw_controller.loop
)
