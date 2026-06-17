local relays = { peripheral.find("redstone_relay") }

rs.setOutput("front", false)
rs.setOutput("left", false)
rs.setOutput("right", false)
rs.setOutput("back", false)
rs.setOutput("bottom", false)
rs.setOutput("top", false)

for _, relay in ipairs(relays) do
    relay.setOutput("front", false)
    relay.setOutput("left", false)
    relay.setOutput("right", false)
    relay.setOutput("back", false)
    relay.setOutput("bottom", false)
    relay.setOutput("top", false)
end
