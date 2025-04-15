from shapely.geometry import Polygon, LineString
import geopandas as gpd
import matplotlib.pyplot as plt

# Définir les géométries des bâtiments, de la route et de la rivière
building1 = Polygon([(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)])
building2 = Polygon([(2, 2), (3, 2), (3, 3), (2, 3), (2, 2)])
road = LineString([(0, 1.5), (3, 1.5)])
river = LineString([(0, 1), (3, 0.5)])

# Convertir les géométries en GeoDataFrames avec une approche explicite
buildings = gpd.GeoDataFrame(
    {'name': ['Building 1', 'Building 2'],
     'geometry': [building1, building2]},
    crs="EPSG:4326"
)

roads = gpd.GeoDataFrame(
    {'name': ['Road'],
     'geometry': [road]},
    crs="EPSG:4326"
)


rivers = gpd.GeoDataFrame(
    {'name': ['River'],
     'geometry': [river]},
    crs="EPSG:4326"
)

# Visualisation
fig, ax = plt.subplots(figsize=(8, 6))

# Tracer chaque couche sur la carte
buildings.plot(ax=ax, color='gray', edgecolor='black', label='Buildings')
roads.plot(ax=ax, color='black', linewidth=2, label='Road')
rivers.plot(ax=ax, color='blue', linewidth=2, linestyle='--', label='River')

# Ajouter une légende et des étiquettes
ax.legend()
ax.set_title("Map with Buildings, Road, and River")
ax.set_xlabel("Longitude")
ax.set_ylabel("Latitude")

# Afficher la carte
plt.show()
