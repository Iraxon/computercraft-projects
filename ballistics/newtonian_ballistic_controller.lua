local bullet_speed = ...
bullet_speed = bullet_speed or 130

local pitches = { peripheral.find("create_radar:auto_pitch_controller") }
local yaws = { peripheral.find("create_radar:auto_yaw_controller") }

-- Combined radar
--#region

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

---Vector from x y z table, to ensure access to methods
---@param table {x: number, y: number, z: number}
---@return Vector
local function vector_from_table(table)
    return vector.new(table.x, table.y, table.z)
end

local position
do
    local radar_position_cache = nil ---@type Vector | nil

    ---Try to derive global position from radar positions.
    ---Only useful outside of a sublevel.
    ---@return Vector
    local function get_position_by_radar()
        if radar_position_cache then return radar_position_cache end
        local sum = vector.new(0, 0, 0) ---@type Vector
        local num_radars = 0
        for _, radar in ipairs(all_radars) do
            local position_table = radar.getPosition()
            sum = sum + vector_from_table(position_table)
            num_radars = num_radars + 1
        end
        local result = sum / num_radars
        radar_position_cache = result
        return result
    end

    ---Vector position of gun station in world space
    function position()
        if sublevel.isInPlotGrid() then
            return sublevel.getLogicalPose().position
        else
            return vector_from_table(get_position_by_radar())
        end
    end
end



---Get the target track, searching all radars on the network
---and using the nearest and most recent target.
local function get_nearest_target_track()
    local system_position = position()

    local best_for_each_radar = {} ---@type (Track | nil)[]
    do
        local best_getter_subroutines = {}
        for radar_index, r in ipairs(all_radars) do
            local function get_best_of_this_radar()
                local tracks = r.getTracks() ---@type Track[]
                local best ---@type Track | nil
                local best_distance ---@type number | nil
                for _, track in ipairs(tracks) do
                    if track.category == "PLAYER" then
                        local track_distance = (vector_from_table(track.position) - system_position):length()
                        if
                            (not best)
                            or
                            track_distance < best_distance
                        then
                            best = track
                            best_distance = track_distance
                        end
                    end
                end
                best_for_each_radar[radar_index] = best
            end
            best_getter_subroutines[radar_index] = get_best_of_this_radar
        end
        parallel.waitForAll(table.unpack(best_getter_subroutines))
    end

    local track_id_merge_table = {} --- @type table<string, Track>

    for radar_index, track in ipairs(best_for_each_radar) do
        local id = track.id
        local existing = track_id_merge_table[id] ---@type Track | nil
        if not existing or track.scannedTime > existing.scannedTime then
            track_id_merge_table[id] = track
        end
    end

    do
        local nearest ---@type Track | nil
        local nearest_distance ---@type number | nil
        for id, track in pairs(track_id_merge_table) do
            local track_distance = (vector_from_table(track.position) - system_position):length()
            if
                (not nearest)
                or
                track_distance < nearest_distance
            then
                nearest = track
                nearest_distance = track_distance
            end
        end

        return nearest
    end
end

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

--#endregion

--Debugs utils
--#region
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

local stopwatch_name ---@type string | nil
local stopwatch_start_time ---@type number | nil

---Start a debug stopwatch.
---@param name string | nil Name to display for stopwatch
local function stopwatch_start(name)
    stopwatch_name = name
    stopwatch_start_time = os.clock()
end

---End the current stopwatch, if it exists.
---Print the stopwatch's time.
local function stopwatch_end()
    local name = stopwatch_name or ""
    if stopwatch_start_time then
        print(name, os.clock() - stopwatch_start_time)
    else
        print("Undefined stopwatch", name)
    end
    stopwatch_name = nil
    stopwatch_start_time = nil
end

--#endregion

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

---Find the position where an accurate shot will hit the target.
---This is the place to aim at.
---
---Position must be relative to the cannon. Velocity and acceleration
---must be absolute. The cannon's velocity is ignored because CBC,
---at the version used for this event, ignores it. The updated version
---cannot be used because CBC AT does not support it.
---
---@param target_pos Vector m
---@param target_velocity Vector m/s
---@param target_acceleration Vector m/s^2
---@param projectile_speed number m/s
---@param iterations integer
local function impact_point_from_target_kinematics(
    target_pos,
    target_velocity,
    target_acceleration,
    projectile_speed,
    iterations
)
    local impact_point

    ---The target will be at the impact point at the time
    ---of impact. Therefore, we can calculate the point
    ---from the time.
    ---@param t number
    ---@return Vector impact_point
    local function impact_point_from_time(t)
        return
            target_pos
            + target_velocity * t
            + target_acceleration * t * t * 0.5
    end

    ---If we know the impact point, impact time
    ---can be calculated from bullet speed
    ---@param point Vector
    ---@return number time
    local function time_from_impact_point(point)
        return point:length() / projectile_speed
    end

    -- The plan is to start with an estimated time and
    -- make it better by bouncing between those two functions

    local impact_time = 0

    for _ = 1, iterations, 1 do
        impact_point = impact_point_from_time(impact_time)
        impact_time = time_from_impact_point(impact_point)
    end

    impact_point = impact_point_from_time(impact_time)
    return impact_point
end

---Yields control for as short a time
---as possible.
local function yield()
    os.queueEvent("transientYieldEvent") ---@diagnostic disable-line: undefined-field
    os.pullEvent("transientYieldEvent") ---@diagnostic disable-line: undefined-field
end

local last_target_pos = nil ---@type Vector | nil Position of target at last scan
local last_target_velocity = nil ---@type Vector | nil Velocity of target at last scan
local last_target_acceleration = nil ---@type Vector | nil Acceleration of target at last scan
local last_scan_time = nil ---@type number | nil Time of last scan according to Track.scannedTime (in seconds of world time)

local last_scan_time_by_clock --- @type number | nil Time of last scan according to os.clock() (in seconds since computer start)

local function get_time_since_last_scan()
    if last_scan_time_by_clock then
        return os.clock() - last_scan_time_by_clock
    end
    return 0
end

while true do
    -- stopwatch_start("loop")
    stopwatch_start("getting_scan")
    local target_track = get_nearest_target_track()

    if target_track then
        local current_scan_time = target_track.scannedTime / 20
        local scan_dt = current_scan_time - (last_scan_time or (current_scan_time - 0.05))

        -- We start by getting kinematic info on the target. How we do this
        -- depends on whether we have a fresh scan.
        local target_pos ---@type Vector
        local target_velocity ---@type Vector
        local target_acceleration ---@type Vector

        stopwatch_end()

        if scan_dt > 1e-9 then
            stopwatch_start("new_scan_case")
            -- A new scan has ocurred.
            last_scan_time_by_clock = os.clock()

            target_pos = vector_from_table(target_track.position) - position()
            target_velocity = (target_pos - (last_target_pos or target_pos)) / scan_dt
            target_acceleration = (target_velocity - (last_target_velocity or target_velocity)) / scan_dt

            last_target_pos = target_pos
            last_target_velocity = target_velocity
            last_target_acceleration = target_acceleration
            last_scan_time = current_scan_time
            stopwatch_end()
        else
            stopwatch_start("old_scan_case")
            --We are between scans. Predict assuming constant acceleration.
            local dt = get_time_since_last_scan()

            target_pos =
                last_target_pos
                + last_target_velocity * dt
                + last_target_acceleration * dt * dt * 0.5

            target_velocity =
                last_target_velocity
                + last_target_acceleration * dt

            target_acceleration = last_target_acceleration or vector.new(0, 0, 0)
            stopwatch_end()
        end

        do
            -- stopwatch_start("ballistics")
            local aim_pos = impact_point_from_target_kinematics(
                target_pos,
                target_velocity,
                target_acceleration,
                bullet_speed,
                3
            )
            -- stopwatch_end()

            -- stopwatch_start("controller_commands")
            parallel.waitForAll(
                function() redstone.setOutput("front", true) end, -- Assemble cannon
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
            -- stopwatch_end()
        end
    else
        -- No target track. Disassemble cannon.
        redstone.setOutput("front", false)
    end

    -- stopwatch_end()
    yield()
end
