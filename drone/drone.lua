--[[

Expected setup for this script:

- The drone is always facing the way it is moving on the horizontal plane
- Exactly one redstone relay
    - Top output stops engine
    - Craft turns right by default and turns left when bottom output
- Forward-facing optical sensor
- Wireless modem for GPS

--]]

-- Vector utils
--#region

local function sum_squared(a, b)
    return a * a + b * b
end

local function distance_squared_2(a_vector_2, b_vector_2)
    local a = a_vector_2
    local b = b_vector_2
    local X = 1 -- Index
    local Y = 2 -- Index
    return sum_squared(b[X] - a[X], b[Y] - a[Y])
end

local function greaterThanByKey(a, b, key_fn)
    if key_fn(a) > key_fn(b) then
        return true
    end
    return false
end

local function rotate_90_right(vector_2)
    return { -vector_2[2], vector_2[1] }
end

local function isARightOfB(a_vector_2, b_vector_2)
    -- A is right of B if rotating A right moves it away from B
    -- This is not exact, but it is good enough
    return greaterThanByKey(rotate_90_right(a_vector_2), a_vector_2, function(vec_2)
        distance_squared_2(vec_2, b_vector_2)
    end)
end

--#endregion

local redstone_relay = peripheral.find("redstone_relay")

local function getTargetVelocity()
    return { 0, 0, -1 }
end

local SLEEP_TIME = 1/10

while true do
    -- Determine whether to turn left or right
    local v = sublevel.getLinearVelocity() or {0, 0, 0}
    local v_2D = { v[1], v[3] } -- x and z

    if isARightOfB(v_2D, getTargetVelocity()) then
        -- Turn left
        redstone_relay.setOutput("bottom", true)
    else
        -- Turn right
        redstone_relay.setOutput("bottom", false)
    end

    sleep(SLEEP_TIME)
end
