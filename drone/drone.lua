--[[

Expected setup for this script:

- The drone is always facing the way it is moving on the horizontal plane
- Exactly one redstone relay
    - Top output stops engine
    - Craft turns right by default and turns left when bottom output
- Forward-facing optical sensor
- Wireless modem for GPS

--]]

--Vector utils
--#region

---a^2 + b^2
---@param a number
---@param b number
---@return number
local function sum_squared(a, b)
    return a * a + b * b
end

---Distance squared in two dimensions
---@param a_vector_2 [number, number]
---@param b_vector_2 [number, number]
---@return number
local function distance_squared_2(a_vector_2, b_vector_2)
    local a = a_vector_2
    local b = b_vector_2
    local X = 1 -- Index
    local Y = 2 -- Index
    return sum_squared(b[X] - a[X], b[Y] - a[Y])
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

---Rotate a 2D vector 90 degrees clockwise about the origin
---@param vector_2 [number, number]
---@return [number, number]
local function rotate_90_right(vector_2)
    return { -1 * vector_2[2], vector_2[1] }
end

---Returns if vector A is right (clockwise) of vector B
---@param a_vector_2 [number, number]
---@param b_vector_2 [number, number]
---@return boolean
local function isARightOfB(a_vector_2, b_vector_2)
    --Implementation courtesy of ChatGPT

    local ax, az = a_vector_2[1], a_vector_2[2]
    local bx, bz = b_vector_2[1], b_vector_2[2]

    -- 2D cross product
    local cross = ax * bz - az * bx
    return cross > 0
end

---Converts to 2D list vector
---@param vector {x: number, y: number, z: number}
local function to2D(vector)
    return { vector.x, vector.z }
end
--#endregion

local redstone_relay = peripheral.find("redstone_relay")

local function getTargetVelocity()
    return vector.new(0, 0, -1)
end

local SLEEP_TIME = 1 / 10

while true do
    -- Determine whether to turn left or right
    local v = sublevel.getVelocity()
    print("Velocity", v)
    local v_2D = to2D(v)

    local t = getTargetVelocity()
    print("Target velocity", t)

    if isARightOfB(v_2D, to2D(t)) then
        print("Turning left")
        redstone_relay.setOutput("bottom", true)
    else
        print("Turning right")
        redstone_relay.setOutput("bottom", false)
    end

    sleep(SLEEP_TIME)
end
