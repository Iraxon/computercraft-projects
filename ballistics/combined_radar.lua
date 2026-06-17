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

return {
    get_all_tracks = get_all_tracks,
    get_position_by_radar = get_position_by_radar
}
