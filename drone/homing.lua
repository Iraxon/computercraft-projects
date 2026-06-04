---Locate the center of mass of this sublevel in global space
---@return {x: number, y: number, z: number} | nil
local function locate_sublevel()
    if sublevel.isInPlotGrid() then
        return sublevel.getLogicalPose().position
    else
        return nil
    end
end



---Measure velocity of craft under acceleration.
---@generic S
---@param acceleration_subroutine fun(): nil
---@param locate_subroutine fun(): S
---@param time_delta number
---@return S
local function measure_velocity(
    acceleration_subroutine,
    locate_subroutine,
    time_delta
)
    local s1 = locate_subroutine()
    acceleration_subroutine()
    sleep(time_delta)
    local s2 = locate_subroutine()
    return s2 - s1
end

---Measure acceleration of craft. Moves the craft
---@generic S
---@param acceleration_subroutine fun(): nil
---@param locate_subroutine fun(): S
---@param time_delta number
---@return S
local function measure_acceleration(
    acceleration_subroutine,
    locate_subroutine,
    time_delta
)
    local v1 = measure_velocity(acceleration_subroutine, locate_subroutine, time_delta / 2)
    local v2 = measure_velocity(acceleration_subroutine, locate_subroutine, time_delta / 2)
    local acceleration = (v2 - v1) / time_delta
    print("Measured acceleration", acceleration)
    return acceleration
end

---Calculates the stopping distance in one direction
---@param velocity number Current velocity of object; must be positive
---@param deceleration number | nil Acceleration caused by braking; must be negative
---@return number stopping_distance Always positive
local function stopping_distance(velocity, deceleration)
    assert(velocity >= 0)
    assert(deceleration < 0)

    if velocity == 0 then
        return 0
    end

    deceleration = deceleration or -11 * velocity / math.abs(velocity)
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

---Construct a controller for ships. May be used for translational movement
---or rotational movement on one axis. Call the update method on the controller as often as possible for best results
---@param acceleration_subroutine fun(): nil Called to apply force in positive direction
---@param deceleration_subroutine fun(): nil Apply force in negative direction
---@param target_offset_getter fun(): number Get offset to target (relative to the ship)
---@param velocity_getter nil | fun(): number Get velocity
---@param reset_subroutine nil | fun(): nil Stop applying force
local function new_axis_controller(
    acceleration_subroutine,
    deceleration_subroutine,
    target_offset_getter,
    velocity_getter,
    reset_subroutine
)
    local self = {
        last_thrust_target = 0.0,
        last_offset = 0.0,
        last_velocity = 0.0,
        acceleration = 5.0,
        deceleration = -5.0,
        thrust_target = 0.0
    }

    ---Apply force toward a given target position on the axis.
    ---@param offset_to_target number
    local function thrust(offset_to_target)
        self.thrust_target = offset_to_target
        if self.thrust_target > 0 then
            acceleration_subroutine()
        elseif self.thrust_target < 0 then
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

    local TIME_DELTA = 0.05

    ---Starts the main loop. Does not ever return. Use coroutine management to do other stuff at the same time.
    local function loop()
        while true do
            local offset_to_target = target_offset_getter()
            local velocity
            if velocity_getter then
                velocity = velocity_getter()
            else
                velocity = offset_to_target - self.last_offset
            end

            --Measure acceleration of last time's thrust
            if self.last_thrust_target > 0 then
                self.acceleration = (velocity - self.last_velocity) / TIME_DELTA
            elseif self.last_thrust_target < 0 then
                self.deceleration = (velocity - self.last_velocity) / TIME_DELTA
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

            self.last_offset = offset_to_target
            self.last_velocity = velocity
            --self.last_thrust_target updates itself
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
    - Both side reduces forward velocity
    - Front output causes ship to move down/fall
- Forward-facing optical sensor
- Pitch and roll are stabilized

--]]

--BEGIN MAIN CODE HERE

local relay = peripheral.find("redstone_relay")

local altitude_controller = new_axis_controller(
    function() relay.setOutput("front", false) end,
    function() relay.setOutput("front", true) end,
    function() return 250 - locate_sublevel().y end,
    function() return sublevel.getVelocity().y end
)

--Positive is right
local yaw_controller = new_axis_controller(
    function() -- Turn right
        relay.setOutput("left", false)
        relay.setOutput("right", true)
    end,
    function() -- Turn left
        relay.setOutput("left", true)
        relay.setOutput("right", false)
    end,
    function()  -- Get yaw offset
        local heading_vector = sublevel.getVelocity()
        local heading_angle = math.atan(heading_vector.z, heading_vector.x)
        local target_vector = vector.new(0, 0, 0) - locate_sublevel()
        local target_angle = math.atan(target_vector.z, target_vector.x)

        return ((target_angle - heading_angle + math.pi) % (2 * math.pi)) - math.pi -- Normalize the range
    end
)

parallel.waitForAny(
    altitude_controller.loop,
    yaw_controller.loop
)
