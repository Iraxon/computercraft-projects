---Locate the center of mass of this sublevel in global space
---@return {x: number, y: number, z: number} | nil
local function locate_sublevel()
    if sublevel.isInPlotGrid() then
        return sublevel.getLogicalPose().position
    else
        return nil
    end
end

---Calculates the stopping distance in one direction
---@param velocity number Current velocity of object; must be positive
---@param deceleration number Acceleration caused by braking; must be negative
---@return number stopping_distance Always positive
local function stopping_distance(velocity, deceleration)
    assert(velocity >= 0)
    assert(deceleration < 0)

    if velocity == 0 then
        return 0
    end

    ---Kinematic equation for displacement
    ---under constant deceleration
    ---s = vt + 0.5at^2
    ---@param time number
    ---@return number
    local function displacement(time)
        return velocity * time + 0.5 * deceleration * time * time
    end

    local stopping_time = -velocity / deceleration -- From -b/(2a) quadratic axis (2 * 0.5 == 1)
    return math.abs(displacement(stopping_time))
end

---A value that changes over time and supports differentiation and querying the previous value
---@param default number
local function new_differentiable(default)
    local self = {
        previous = default,
        current = default,
        derivative = 0
    }

    ---Update the value. Returns the given value.
    ---@param value number
    ---@param time_delta nil | number
    ---@return number
    local function update(value, time_delta)
        self.previous = self.current
        self.current = value
        if time_delta then
            self.derivative = (self.current - self.previous) / time_delta
        end
        return value
    end

    local function previous()
        return self.previous
    end

    local function current()
        return self.current
    end

    local function derivative()
        return self.derivative
    end

    local function unsigned_derivative()
        return math.abs(derivative())
    end

    return {
        update = update,
        previous = previous,
        current = current,
        derivative = derivative,
        unsigned_derivative = unsigned_derivative,
        get = current
    }
end

---Axis controller that only needs unsigned error
---@param acceleration_subroutine fun(): nil Apply force or torque in one direction
---@param deceleration_subroutine fun(): nil Apply force or torque in the other direction
---@param unsigned_error_getter fun(): number Unsigned distance from desired position/angle. Reutrn 0 whenever you want to disable the controller.
---@param reset_subroutine nil | fun(): nil Stop applying force (optional)
local function new_unsigned_axis_controller(
    acceleration_subroutine,
    deceleration_subroutine,
    unsigned_error_getter,
    reset_subroutine
)
    reset_subroutine = reset_subroutine or function() end

    local self = {
        error = new_differentiable(unsigned_error_getter()),
        error_velocity = new_differentiable(0),
        action = 0,
        acceleration = 1,
        deceleration = 1
    }

    local TIME_DELTA = 0.05

    local function thrust(action)
        reset_subroutine()
        if action > 0 then
            acceleration_subroutine()
        elseif action < 0 then
            deceleration_subroutine()
        end
    end

    local function loop()
        while true do
            local error = self.error.update(unsigned_error_getter(), TIME_DELTA)
            local error_velocity = self.error_velocity.update(self.error.derivative(), TIME_DELTA)
            local measured_acceleration = self.error_velocity.derivative()

            --Note on sign conventions: Negative means decreasing error (toward target), positive means increasing (away from target)
            --Because error is unsigned, overly negative velocity will overshoot
            --Braking takes the form of applying *positive* acceleration

            do
                local action = self.action
                if action > 0 then
                    self.acceleration = measured_acceleration
                elseif action < 0 then
                    self.deceleration = measured_acceleration
                end
            end

            local possible_actions = {
                {
                    action = -1,
                    acceleration = self.acceleration
                },
                {
                    action = 0,
                    acceleration = measured_acceleration
                },
                {
                    action = 1,
                    acceleration = self.deceleration
                }
            }

            --Sort possible actions by acceleration, ascending order
            table.sort(possible_actions, function(a, b) return a.acceleration <= b.acceleration end)

            local reduce_error = possible_actions[1] -- Reduces error velocity (pushes error towarard zero)
            local brake = possible_actions[3]        -- Increases error velocity (decrease rate of closing toward zero)

            -- We want a negative velocity (decreasing error) that is
            -- controllable ()
            local desired_velocity
            do
                if math.abs(error_velocity) <= 0.01 then
                    desired_velocity = -100 -- If we aren't moving we just want to decrease error
                else
                local maximum_time_to_stop = math.abs(error / error_velocity)
                desired_velocity = -brake.acceleration * maximum_time_to_stop
                -- Brake acceleration should be *positive*, so we must negate.
                -- If it is not, we don't have a brake, so this doesn't matter
                end
            end

            local chosen
            if error_velocity < desired_velocity then
                -- We are going too fast (remember sign convention)
                chosen = brake
            else
                chosen = reduce_error
            end

            do
                local action = chosen.action
                thrust(action)
                --Remember what we did so next loop we can evaluate its acceleration
                self.action = chosen.action
            end

            sleep(TIME_DELTA)
        end
    end

    return {
        loop = loop
    }
end

--[[

Expected setup for this script:

- The drone is always facing the way it is moving on the horizontal plane
- Exactly one redstone relay
    - Left output causes ship to yaw left
    - Right output causes ship to yaw right
    - Both sides off does not reduce forward velocity
    - Front output causes ship to move down/fall
- Forward-facing optical sensor
- Pitch and roll are stabilized

--]]

--BEGIN MAIN CODE HERE

local relay = peripheral.find("redstone_relay")

local altitude_controller = new_unsigned_axis_controller(
    function() relay.setOutput("front", false) end,
    function() relay.setOutput("front", true) end,
    function() return 250 - locate_sublevel().y end
)

local function get_unsigned_yaw_offset()
    local heading_vector = sublevel.getVelocity():normalize()

    if heading_vector.x * heading_vector.x + heading_vector.z * heading_vector.z < 15 * 15 then
        -- Craft is not moving fast enough for us to tell where it's going
        return 0
    end

    local target_vector = (vector.new(0, 0, 0) - locate_sublevel()):noramlize()

    return math.acos(
        heading_vector:dot(target_vector)
    )
end

--Positive is right
local yaw_controller = new_unsigned_axis_controller(
    function() -- Turn right
        relay.setOutput("left", false)
        relay.setOutput("right", true)
    end,
    function() -- Turn left
        relay.setOutput("left", true)
        relay.setOutput("right", false)
    end,
    get_unsigned_yaw_offset,
    function()
        relay.setOutput("left", false)
        relay.setOutput("right", false)
    end
)

parallel.waitForAny(
    altitude_controller.loop,
    yaw_controller.loop
)
