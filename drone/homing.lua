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
    assert(deceleration <= 0)

    if velocity == 0 then
        return 0
    end

    if deceleration == 0 then
        return math.huge
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

---Construct a controller for ships. May be used for translational movement
---or rotational movement on one axis.
---@param acceleration_subroutine fun(): nil Called to apply force in positive direction
---@param deceleration_subroutine fun(): nil Apply force in negative direction
---@param target_offset_getter fun(): number Get offset to target (relative to the ship)
---@param velocity_getter fun(): number Get velocity
---@param reset_subroutine nil | fun(): nil Stop applying force
local function new_axis_controller(
    acceleration_subroutine,
    deceleration_subroutine,
    target_offset_getter,
    velocity_getter,
    reset_subroutine
)
    local self = {
        thrust_target = new_differentiable(0),
        offset_to_target = new_differentiable(0),
        velocity = new_differentiable(0),
        acceleration = 5.0,
        deceleration = -5.0,
    }

    local TIME_DELTA = 0.05

    ---Apply force toward a given target position on the axis.
    ---@param offset_to_target number
    local function thrust(offset_to_target)
        self.thrust_target.update(offset_to_target, TIME_DELTA)
        if offset_to_target > 0 then
            acceleration_subroutine()
        elseif offset_to_target < 0 then
            deceleration_subroutine()
        end
    end

    ---Apply thrust away from a given target, only if this will reduce the speed
    ---@param velocity number Signed velocity
    ---@param offset_to_target number
    local function brake(velocity, offset_to_target)
        if velocity * offset_to_target > 0 then
            thrust(-offset_to_target)
        end
    end

    ---Starts the main loop. Does not ever return. Use coroutine management to do other stuff at the same time.
    local function loop()
        while true do
            self.offset_to_target.update(target_offset_getter(), TIME_DELTA)
            self.velocity.update(velocity_getter(), TIME_DELTA)

            local offset_to_target = self.offset_to_target.current()
            local velocity = self.velocity.current()

            --Measure acceleration of last time's thrust
            local last_thrust_target = self.thrust_target.previous()
            if last_thrust_target > 0 then
                self.acceleration = self.velocity.derivative()
            elseif last_thrust_target < 0 then
                self.deceleration = self.velocity.derivative()
            end

            --Stop previous forces from last time
            if reset_subroutine then
                reset_subroutine()
            end

            --If we are moving away from target, or not moving, we apply thrust to fix that
            if velocity * offset_to_target <= 0 then
                thrust(offset_to_target)
            else
                --If we're moving toward the target, we check if accelerating will cause an overshoot before applying thrust
                local brake_deceleration
                if offset_to_target > 0 then
                    brake_deceleration = self.deceleration
                else
                    brake_deceleration = self.acceleration
                end
                if stopping_distance(math.abs(velocity), -math.abs(brake_deceleration)) < math.abs(offset_to_target) then
                    thrust(offset_to_target)
                else
                    brake(velocity, offset_to_target)
                end
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
    - Left output controls left engine
    - Right output controls right engine
    - Front output controls vertical engine (turning it off allows falling)
- Pitch and roll are stabilized by a gyroscope
--]]

--BEGIN MAIN CODE HERE

local relay = peripheral.find("redstone_relay")

local altitude_controller = new_axis_controller(
    function() relay.setOutput("front", true) end,
    function() relay.setOutput("front", false) end,
    function() return 250 - locate_sublevel().y end,
    function() return sublevel.getVelocity().y end
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

local yaw_cache = {
    value = 0.0
}

--Positive is right
local yaw_controller = new_axis_controller(
    function() -- Turn right
        relay.setOutput("left", true)
        relay.setOutput("right", false)
    end,
    function() -- Turn left
        relay.setOutput("left", false)
        relay.setOutput("right", true)
    end,
    get_yaw_offset,
    function()
        local current = get_yaw_offset()
        local angular_velocity = angle_difference(get_yaw_offset(), yaw_cache.value)
        yaw_cache.value = current
        return angular_velocity
    end,
    function()
        relay.setOutput("left", true)
        relay.setOutput("right", true)
    end
)

parallel.waitForAny(
    altitude_controller.loop,
    yaw_controller.loop
)
