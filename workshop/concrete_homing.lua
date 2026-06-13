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
---@param provider fun(): number
local function new_differentiable(provider)
    local self = {
        previous = provider(),
        current = provider(),
        derivative = 0
    }

    ---Update the value. Returns the new value.
    ---@param time_delta nil | number
    ---@return number
    local function update(time_delta)
        self.previous = self.current
        self.current = provider()
        if time_delta and time_delta ~= 0 then
            self.derivative = (self.current - self.previous) / time_delta
        end
        return self.current
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

---Calcualte a - b for radian angles, giving a difference within +/- pi
---@param a number
---@param b number
---@return number
local function angle_difference(a, b)
    return ((a - b + math.pi) % (2 * math.pi)) - math.pi -- Normalize the range
end




local function up() relay.setOutput("front", true) end
local function down() relay.setOutput("front", false) end
local function altitude() return locate_sublevel().y - 250 end
local function vy() return sublevel.getVelocity().y end

if altitude * vy > 0 then
    --Reverse if error is getting worse
    if vy > 0 then
        down()
    else
        up()
    end

else
    --Brake
    if vy > 0 then
        down()
    else
        up()
    end
end



local function right()
    relay.setOutput("left", true)
    relay.setOutput("right", false)
end
local function left()
    relay.setOutput("left", false)
    relay.setOutput("right", true)
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
local function get_angular_velocity()
    --Negative to convert to right-positive yaw
    return -sublevel.getAngularVelocity().y
end

local yaw_cache = {
    value = 0.0
}
