import carla
client = carla.Client('localhost', 2000)
world = client.get_world()
world.unload_map_layer(carla.MapLayer.ParkedVehicles)
world.unload_map_layer(carla.MapLayer.StreetLights)
world.unload_map_layer(carla.MapLayer.Walls)