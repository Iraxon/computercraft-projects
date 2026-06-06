--Stochastic controller

--Initialize random
math.randomseed(os.time())

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

---Keep track of a subroutine and how it affects the velocity
---@param subroutine fun(): nil
local function _action_of(subroutine)
    local self = {
        subroutine = subroutine,
        delta_v = 0
    }

    ---Set the delta_v value for this action
    ---@param delta_v number
    local function update_delta_v(delta_v)
        self.delta_v = delta_v
    end

    ---Get the delta_v value for this action
    ---@return number
    local function get_delta_v()
        return self.delta_v
    end

    ---Run this action
    local function run()
        subroutine()
    end

    return {
        update_delta_v = update_delta_v,
        get_delta_v = get_delta_v,
        run = run
    }
end

---Construct a stochastic controller, which attempts to zero an error
---without knowing exactly what the given subroutines do
---
---Adapted for controlling position by applying forces (acceleration),
---or angular equivalent
---
---@param get_forward_error fun(): number
---@param subroutine1 fun(): nil
---@param subroutine2 fun(): nil
local function new_stochastic_controller(get_forward_error, subroutine1, subroutine2)
    local error = new_delta_value(0.0)
    local absolute_error = new_delta_value(0.0)
    local velocity = new_delta_value(0.0)
    local actions = {
        _action_of(function() end), -- Action for doing nothing
        _action_of(subroutine1),
        _action_of(subroutine2)
    }
    local selected_action = actions[1]

    local TIME_DELTA = 0.05

    local function loop()
        while true do
            error.update(get_forward_error())

            -- We divide by TIME_DELTA to make the delta an approximate derivative
            -- Negation because it is forward error
            -- Consider: If target is in front, forward error is positive
            -- If we approach the target, forward error decreases but our velocity is positive
            velocity.update(-error.delta() / TIME_DELTA)

            --Change in velocity from a thrust impulse, not acceleration per time
            selected_action.update_delta_v(velocity.delta())

            print("Error", error.get())
            print("Velocity", velocity.get())
            print("Delta V", selected_action.get_delta_v())

            -- Selected action changed after this point

            if math.random() < 1 / 20 then
                -- Every so often, do a random action to keep the
                -- accelerations accurate
                selected_action = actions[math.random(#actions)]
            else
                local k = 0.5
                local desired_velocity = k * error.get()

                table.sort(actions, function(a, b)
                    -- Whichever delta v gets us closer to desired velocity
                    -- Goes on the left
                    return
                        math.abs(velocity.current() + a.get_delta_v() - desired_velocity)
                        <
                        math.abs(velocity.current() + b.get_delta_v() - desired_velocity)
                end)

                selected_action = actions[1]
            end

            selected_action.run()

            sleep(TIME_DELTA)
        end
    end

    return {
        loop = loop
    }
end



--BEGIN MAIN CODE HERE

local relay = peripheral.find("redstone_relay")

local altitude_controller = new_stochastic_controller(
    function() return 250 - locate_sublevel().y end,
    function() relay.setOutput("front", false) end,
    function() relay.setOutput("front", true) end
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
local yaw_controller = new_stochastic_controller(
    get_yaw_offset,
    function() -- Turn right
        relay.setOutput("left", false)
        relay.setOutput("right", true)
    end,
    function() -- Turn left
        relay.setOutput("left", true)
        relay.setOutput("right", false)
    end
)

parallel.waitForAny(
    altitude_controller.loop,
    yaw_controller.loop
)
