
local SLEEP_TIME = 1/4

local BOMB_SIDES = {
    "bottom",
}

function output_bomb_sides(redstone_value)
    for _, side in ipairs(BOMB_SIDES) do
        redstone.setOutput(side, redstone_value)
    end
end

local THRESHOLD = 0.5

function is_near_zero(vector)

    for _, v in pairs(vector) do
        if math.abs(v) > THRESHOLD then return false end
    end
    return true
end

while true do

    armed = redstone.getInput("top")
    v = sublevel.getLinearVelocity()
    stationary = is_near_zero(v)

    if
    armed
    and stationary
    then
        output_bomb_sides(true)
    else
        output_bomb_sides(false)
    end

    sleep(SLEEP_TIME)

end
