---@meta

--- ComputerCraft Vector from standard library
---@class Vector
---@field x number
---@field y number
---@field z number
---@field add fun(self: Vector, other: Vector): Vector
---@operator add(Vector): Vector
---@field sub fun(self: Vector, other: Vector): Vector
---@operator sub(Vector): Vector
---@field mul fun(self: Vector, k: number): Vector
---@operator mul(number): Vector
---@field div fun(self: Vector, k: number): Vector
---@operator div(number): Vector
---@operator unm(): Vector
---@field dot fun(self: Vector, other: Vector): number
---@field cross fun(self: Vector, other: Vector): Vector
---@field length fun(self: Vector): number
---@field normalize fun(): Vector
---@field round fun(self: Vector, other: number|nil): Vector
---@field tostring fun(self: Vector): string
---@field equals fun(self: Vector, other: any): boolean
local Vector = {}

---Radar track from Create Radars radar.getTracks() method
---@class Track
---@field position {x: number, y: number, z: number}
---@field id string
---@field category string
---@field scannedTime number
---@field entityType string
local Track = {}
