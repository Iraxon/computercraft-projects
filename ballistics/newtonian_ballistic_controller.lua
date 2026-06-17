local bullet_speed = ...
bullet_speed = bullet_speed or 60

local pitches = { peripheral.find("create_radar:auto_pitch_controller") }
local yaws = { peripheral.find("create_radar:auto_yaw_controller") }

local radars = { peripheral.find("create_radar:radar_bearing") }
local plane_radars = { peripheral.find("create_radar:plane_radar") }

local all_radars = {}
do
    for _, bearing in ipairs(radars) do
        table.insert(all_radars, bearing)
    end
    for _, plane_radar in ipairs(plane_radars) do
        table.insert(all_radars, plane_radar)
    end
end

local pprint = require("cc.pretty").pretty_print

---Print something and return it. Useful for debugging
---an expression.
---@generic T
---@param x T
---@param msg string | nil
---@return T
local function inline_print(x, msg)
    if msg then
        print(msg)
    end
    pprint(x)
    return x
end

pprint(yaws)
print("yaws above, pitches below")
pprint(pitches)

---Get tracks from all radars on the network
---@return table<string, Track>
local function get_all_tracks()
    local subroutines_for_each_radar = {} ---@type fun()[]
    local tracks_for_each_radar = {} --- @type Track[][]

    -- Fill up tracks_for_each_radar
    for radar_index, r in ipairs(all_radars) do
        local function handle_this_radar()
            local tracks = r.getTracks() ---@type Track[]
            tracks_for_each_radar[radar_index] = tracks
        end
        subroutines_for_each_radar[radar_index] = handle_this_radar
    end

    parallel.waitForAll(table.unpack(subroutines_for_each_radar))

    ---@type table<string, Track>
    local all_tracks = {}

    for _, tracks in ipairs(tracks_for_each_radar) do
        for _, track in ipairs(tracks) do
            local id = track.id
            local existing = all_tracks[id] ---@type Track | nil
            if existing == nil or track.scannedTime > existing.scannedTime then
                all_tracks[id] = track
            end
        end
    end

    return all_tracks
end

---Try to derive global position from radar positions.
---Only useful outside of a sublevel.
---@return Vector
local function get_position_by_radar()
    local sum = vector.new(0, 0, 0) ---@type Vector
    local num_radars = 0
    for _, radar in ipairs(all_radars) do
        local position_table = radar.getPosition()
        sum = sum + vector.new(position_table.x, position_table.y, position_table.z)
        num_radars = num_radars + 1
    end
    return sum / num_radars
end

local set_pitch
local set_yaw
do
    local function normalize_degree_angle(theta)
        local theta_mod = theta % 360
        local r_val
        if theta_mod > 180 then
            r_val = theta_mod - 360
        else
            r_val = theta_mod
        end

        return r_val
    end

    local function is_valid_angle(theta)
        return -180 < theta and theta <= 180
    end

    local function set_controllers(controllers, theta, operation_for_error_message)
        operation_for_error_message = operation_for_error_message or "unknown"
        local theta_norm = normalize_degree_angle(theta)
        if is_valid_angle(theta_norm) then
            for _, controller in ipairs(controllers) do
                controller.setAngle(theta_norm)
            end
        else
            print("Got invalid angle", theta, "normalizes to", theta_norm, "Operation:", operation_for_error_message)
        end
    end

    ---Set pitch for all cannons
    ---@param theta number
    function set_pitch(theta)
        set_controllers(pitches, theta)
    end

    ---Set yaw for all cannons
    ---@param theta number
    function set_yaw(theta)
        set_controllers(yaws, theta)
    end
end

---Vector from x y z table, to ensure access to methods
---@param table {x: number, y: number, z: number}
---@return Vector
local function vector_from_table(table)
    return vector.new(table.x, table.y, table.z)
end

---Vector position of gun station in world space
local function position()
    if sublevel.isInPlotGrid() then
        return sublevel.getLogicalPose().position
    else
        return vector_from_table(get_position_by_radar())
    end
end


local get_nearest_player_track

do
    ---Get all player tracks
    local function get_player_tracks()
        do
            ---@type [Track]
            local t = {}
            for id, track in pairs(get_all_tracks()) do
                if track.category == "PLAYER" then
                    table.insert(t, track)
                end
            end
            return t
        end
    end

    ---Track of nearest player
    function get_nearest_player_track()
        local t = get_player_tracks()
        local min_track = nil
        local min_distance = math.huge
        for _, player_track in ipairs(t) do
            local relative = vector_from_table(player_track.position) - position()
            local distance = relative:length()
            if distance < min_distance then
                min_track = player_track
                min_distance = distance
            end
        end
        if min_track then
            return min_track
        else
            return nil
        end
    end
end

---Convert from radians to degrees
---@param x number
---@return number
local function convert_pitch(x)
    return x * 180 / math.pi
end

---Convert from radians, clockwise, east-0
---to degrees, clockwise, south-0
---@param x number
---@return number
local function convert_yaw(x)
    return
        (
            x
            * 180 / math.pi -- Degrees east-0 from radians east-0
            - 90            -- South-0 from east-0
        )
        % 360               -- Normalize
end

local last_target_pos = nil
local last_target_velocity = nil
local last_time = nil


---Yields control for as short a time
---as possible.
local function yield()
    os.queueEvent("transientYieldEvent") ---@diagnostic disable-line: undefined-field
    os.pullEvent("transientYieldEvent") ---@diagnostic disable-line: undefined-field
end

while true do
    local target_track = get_nearest_player_track()

    if target_track then
        -- We start by getting kinematic info on the target

        local now = target_track.scannedTime / 20
        local dt = now - (last_time or (now - 0.05))

        if dt > 1e-9 then
            -- All numbers relative to gun; note that sublevel velocity is not taken into account
            -- because CBC ignores it. Updating is not yet possible because CBC AT does not support it.
            local target_pos = vector_from_table(target_track.position) - position()
            local target_velocity = (target_pos - (last_target_pos or target_pos)) / dt
            local target_acceleration = (target_velocity - (last_target_velocity or target_velocity)) / dt

            local impact_point
            do
                ---If we know the impact point, time can be calculated from bullet speed
                ---@param point Vector
                ---@return number
                local function time_from_impact_point(point)
                    return point:length() / bullet_speed
                end

                ---If we know the impact time, we can calculate impact point, since we
                ---know the target will be there at that time
                ---@param t number
                ---@return Vector
                local function impact_point_from_time(t)
                    return
                        target_pos
                        + target_velocity * t
                        + target_acceleration * t * t * 0.5
                end

                -- The plan is to start with an estimated time and
                -- make it better by bouncing between those two functions

                local impact_time = 0

                for _ = 1, 3, 1 do
                    impact_point = impact_point_from_time(impact_time)
                    impact_time = time_from_impact_point(impact_point)
                end

                impact_point = impact_point_from_time(impact_time)
            end

            do
                local aim_pos = impact_point

                parallel.waitForAll(
                    function() redstone.setOutput("front", true) end,
                    function()
                        set_yaw(
                            convert_yaw(math.atan(aim_pos.z, aim_pos.x))
                        )
                    end,
                    function()
                        set_pitch(
                            convert_pitch(math.atan(
                                aim_pos.y,
                                math.sqrt(
                                    aim_pos.x * aim_pos.x
                                    + aim_pos.z * aim_pos.z
                                )
                            ))
                        )
                    end

                )
            end

            last_target_pos = target_pos
            last_target_velocity = target_velocity
            last_time = now
        end
    else
        redstone.setOutput("front", false)
    end

    yield()
end
