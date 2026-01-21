FidgetMetatables = {}

--yes this is global. its used in other scripts of the lib instead of doing singular requires.
Fidget = {}
Fidget.rigidbodies, Fidget.particles  = require("Fidget.Rigidbody Init.Rigidbody")
Fidget.physicsSim = require("Fidget.Physics Simulation Vars.Physics Simulation")
Fidget.joints = require("Fidget.Rigidbody Init.Joints")
Fidget.links = require("Fidget.Rigidbody Init.Links")
Fidget.quaternions = require("Fidget.quaternions")
return Fidget