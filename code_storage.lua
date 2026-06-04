-- Things I wrote and didn't need, but thought I might need later

local function maxByKey(a, b, key_fn, default_to_b)
    if default_to_b == nil then
        default_to_b = false
    elseif key_fn(a) > key_fn(b) then
        return a
    elseif key_fn(b) > key_fn(a) then
        return b
    elseif default_to_b then
        return b
    else
        return a
    end
end

local function lessThanByKey(a, b, key_fn)
    if key_fn(a) < key_fn(b) then
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

-- Workspace for WIP things
