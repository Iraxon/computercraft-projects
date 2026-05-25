--[[

Expected setup for this script:

- The drone is always facing the way it is moving on the horizontal plane
- Exactly one redstone relay
    - Left output causes ship to yaw left
    - Right output causes ship to yaw right
    - Both side reduces forward velocity
    - Bottom output causes ship to move down/fall
- Forward-facing optical sensor
- Pitch and roll are stabilized

--]]

--Vector utils
--#region

---a^2 + b^2
---@param a number
---@param b number
---@return number
local function sumSquared(a, b)
    return a * a + b * b
end

---Distance squared in two dimensions
---@param a_vector_2 [number, number]
---@param b_vector_2 [number, number]
---@return number
local function distanceSquared2D(a_vector_2, b_vector_2)
    local a = a_vector_2
    local b = b_vector_2
    local X = 1 -- Index
    local Y = 2 -- Index
    return sumSquared(b[X] - a[X], b[Y] - a[Y])
end

---Returns if key_fn(a) > key_fn(b)
---@generic T : any
---@param a T
---@param b T
---@param key_fn fun(a: T): number
---@return boolean
local function greaterThanByKey(a, b, key_fn)
    if key_fn(a) > key_fn(b) then
        return true
    end
    return false
end

---Returns positive if vector A is right (clockwise) of vector B, negative or 0
---@param a_vector_2 [number, number]
---@param b_vector_2 [number, number]
---@return number
local function cross_product_2D(a_vector_2, b_vector_2)
    --Implementation courtesy of ChatGPT

    local ax, az = a_vector_2[1], a_vector_2[2]
    local bx, bz = b_vector_2[1], b_vector_2[2]

    -- 2D cross product
    local cross = ax * bz - az * bx
    return cross
end

---Converts to 2D list vector
---@param vector {x: number, y: number, z: number}
local function to2D(vector)
    return { vector.x, vector.z }
end
--#endregion

--#endregion

--Sublevel GPS
--region
---Locate the center of mass of this sublevel in global space
---@return {x: number, y: number, z: number} | nil
local function locate_sublevel()
    if sublevel.isInPlotGrid() then
        return sublevel.getLogicalPose().position()
    else
        return nil
    end
end
--#endregion

--Altitude control
--#region

local GRAVITY = aero.getGravity().y

---Whether the craft should apply thrust upward right now
---@param vy number
---@return boolean
local function shouldApplyThrust(vy, target_altitude)
    local altitude = locate_sublevel().y

    if math.abs(altitude - target_altitude) <= 2 then
        return altitude < target_altitude
    end

    if vy < 0 and altitude < target_altitude then
        return true
    end
    local apex_if_thrust_stopped = altitude + (vy * vy) / (-2 * GRAVITY)
    return apex_if_thrust_stopped < target_altitude
end


local redstone_relay = peripheral.find("redstone_relay")

local function getTargetVelocity()
    local target = {
        x = 0,
        y = 0,
        z = 0
    }
    local p = locate_sublevel()
    if p == nil then
        error("Not on a vehicle")
    end
    return vector.new(
        target.x - p.x,
        target.y - p.y,
        target.z - p.z
    )
end

local SLEEP_TIME = 1 / 10

while true do
    -- Determine whether to turn left or right
    local v = sublevel.getVelocity()
    print("Velocity", v)
    local v_2D = to2D(v)

    local t = getTargetVelocity()
    print("Target velocity", t)

    local cross = cross_product_2D(v_2D, to2D(t))

    local TOLERANCE = 1

    if cross > TOLERANCE then
        print("Turning left")
        redstone_relay.setOutput("left", true)
        redstone_relay.setOutput("left", false)
    elseif cross < TOLERANCE then
        print("Turning right")
        redstone_relay.setOutput("right", false)
        redstone_relay.setOutput("right", true)
    else
        print("Continuing straight")
    end

    redstone.setOutput("bottom", not shouldApplyThrust(v.y, 200))

    sleep(SLEEP_TIME)
end
