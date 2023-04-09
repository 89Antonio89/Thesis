import io
import carla

# # Read the .osm data
# f = io.open("Porto.osm", mode="r", encoding="utf-8")
# osm_data = f.read()
# f.close()

# # Define the desired settings. In this case, default values.
# settings = carla.Osm2OdrSettings()
# # set traffic ligths
# settings.generate_traffic_lights = False
# settings.use_offsets = False

# # Set OSM road types to export to OpenDRIVE
# settings.set_osm_way_types(["motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential"])
# # Convert to .xodr
# xodr_data = carla.Osm2Odr.convert(osm_data, settings)

# # save opendrive file
# f = open("Porto.xodr", 'w',encoding="utf-8")
# f.write(xodr_data)
# f.close()

# Read the .osm data
with open("CampoAlegre1.osm", mode="r", encoding="utf-8") as osmFile:
    osm_data = osmFile.read()

# Define the desired settings
settings = carla.Osm2OdrSettings()

# Set OSM road types to export to OpenDRIVE
settings.set_osm_way_types([
    "motorway",
    "trunk",
    "primary",
    "residential"
])
settings.default_lane_width = 6.0
settings.generate_traffic_lights = True
settings.all_junctions_with_traffic_lights = False
settings.center_map = True

# Convert to .xodr
xodr_data = carla.Osm2Odr.convert(osm_data, settings)

# save opendrive file
with open("CampoAlegre.xodr", "w", encoding="utf-8") as xodrFile:
    xodrFile.write(xodr_data)
