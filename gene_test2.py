from shapely.geometry import Polygon, LineString
from shapely.geometry import Point
import geopandas as gpd
import matplotlib.pyplot as plt
import numpy as np

# Définir les géométries des bâtiments, de la route et de la rivière
building1 = Polygon([(20, 20), (40, 20), (40, 50), (20, 50), (20, 20)])
building2 = Polygon([(60, 60), (80, 60), (80, 90), (60, 90), (60, 60)])
#building3 = Polygon([(120, 20), (140, 20), (140, 50), (120, 50), (120, 20)])
building4 = Polygon([(160, 160), (180, 160), (180, 180), (160, 180), (160, 160)])

road1 = LineString([(0, 100), (200, 100)])
road2 = LineString([(100, 0), (100, 200)])

# Générer une rivière avec de vraies courbes
x = np.linspace(0, 200, 100)
y = 50 + 2 * np.sin(x / 30)  # Fonction sinus pour des courbes naturelles
river_points = [Point(x[i], y[i]) for i in range(len(x))]
river = LineString(river_points)

# Convertir les géométries en GeoDataFrames
buildings = gpd.GeoDataFrame(
    {'name': ['Building 1', 'Building 2', 'Building 4'],
     'geometry': [building1, building2, building4]},
    crs="EPSG:4326"
)

roads = gpd.GeoDataFrame(
    {'name': ['Road 1', 'Road 2'],
     'geometry': [road1, road2]},
    crs="EPSG:4326"
)

rivers = gpd.GeoDataFrame(
    {'name': ['River'],
     'geometry': [river]},
    crs="EPSG:4326"
)

# Visualisation
fig, ax = plt.subplots(figsize=(10, 10))

# Tracer chaque couche sur la carte
buildings.plot(ax=ax, color='gray', edgecolor='black', label='Buildings')
roads.plot(ax=ax, color='black', linewidth=2, label='Roads')
rivers.plot(ax=ax, color='blue', linewidth=2, linestyle='--', label='River')

# Ajouter une légende et des étiquettes
ax.legend()
ax.set_title("Map with Buildings, Roads, and River")
ax.set_xlim(0, 200)
ax.set_ylim(0, 200)
ax.set_xlabel("")
ax.set_ylabel("")

# Afficher la carte
plt.show()
