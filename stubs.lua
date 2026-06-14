--- ComputerCraft Vector from standard library
---@class Vector
---@field x number
---@field y number
---@field z number
---@field length fun(): number
local Vector = {}

---Radar track from Create Radars radar.getTracks() method
---@class Track
---@field position {x: number, y: number, z: number}
---@field id string
---@field category string
---@field scannedTime number
---@field entityType string
local Track = {}
