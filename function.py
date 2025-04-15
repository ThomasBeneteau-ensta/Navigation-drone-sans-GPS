import matplotlib.pyplot as plt
from math import atan2, degrees
import numpy as np
import osmnx as ox
import pandas as pd

def gen_vect_map(lat, lon, dist):
    """
    Génère les données vectorielles OSM pour une zone autour d'un point central.

    Args:
        lat (float): Latitude du centre.
        lon (float): Longitude du centre.
        dist (int): Distance en mètres entre le centre et les bords de la zone.

    Returns:
        gdf (GeoDataFrame): Données vectorielles OSM avec types d'objets.
        bbox (tuple): Limites (north, south, east, west) de la zone.
    """

    # Définir un point central (lat, lon) et une zone de 2*dist x 2*dist autour du point
    center_lat, center_lon = lat, lon
    distance = dist  # distance entre le point central et les côtés (donc bord_map = 2 * dist)

    north, south, east, west = ox.utils_geo.bbox_from_point((center_lat, center_lon), dist=distance)

    # Définir les tags pour récupérer plusieurs types d'éléments
    tags = {
        'building': True,  # Tous les bâtiments
        'highway': True,  # Toutes les routes et chemins
        #'waterway': True,  # Toutes les rivières et canaux
        #'natural': ['water', 'wood'],  # Éléments naturels (eau)
        #'landuse': 'forest',  # Zones de forêt
        #'amenity': ['school', 'hospital', 'restaurant'],  # Écoles, hôpitaux et restaurants
        #'leisure': 'park',  # Parcs de loisirs
        #'tourism': 'hotel',  # Hôtels
        #'railway': 'station'  # Stations ferroviaires
    }

    # Télécharger les données dans cette zone
    gdf = ox.geometries_from_bbox(north, south, east, west, tags)

    # Ajouter une colonne indiquant le type d'objet
    def get_object_type(row):
        for tag in ['building', 'highway', 'waterway']:
            if tag in row and not pd.isna(row[tag]):
                return tag
        return "unknown"

    gdf['object_type'] = gdf.apply(get_object_type, axis=1)

    return gdf, (north, south, east, west)


def plot_vect_map(gdf, bbox, title="Carte"):

    north, south, east, west = bbox

    # Créer une carte avec les données récupérées
    fig, ax = plt.subplots(figsize=(10, 10))

    # Afficher les données (différentes couleurs pour les types d'éléments)
    gdf[gdf['building'].notna()].plot(ax=ax, color='purple', alpha=0.5, edgecolor='black', label='Bâtiments')
    #gdf[gdf['waterway'].notna()].plot(ax=ax, color='cyan', label='Rivières')
    gdf[gdf['highway'].notna()].plot(ax=ax, color='grey', label='Routes')
    #gdf[gdf['natural'] == 'wood'].plot(ax=ax, color='olive', label='bois')
    #gdf[gdf['natural'] == 'water'].plot(ax=ax, color='aqua', label='eau')
    #gdf[gdf['landuse'].notna()].plot(ax=ax, color='green', label='forest')

    # Fixer les limites de la carte pour respecter 200m × 200m
    ax.set_xlim(west, east)
    ax.set_ylim(south, north)

    # Ajouter une légende et un titre
    ax.legend()
    #plt.title(f"Carte OpenStreetMap : {2*distance}m x {2*distance}m autour de ({center_lat}, {center_lon})")
    plt.title(title)
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.show()


def calculate_bbox_from_dimensions(width_m, height_m):
    """
    Calcule la bbox (north, south, east, west) à partir des dimensions en mètres,
    en supposant que l'origine est (0,0) et que l'image est centrée.

    Args:
        width_m (float): Largeur de l'image en mètres.
        height_m (float): Hauteur de l'image en mètres.

    Returns:
        tuple: Limites de la bbox (north, south, east, west).
    """
    north = height_m / 2
    south = -height_m / 2
    east = width_m / 2
    west = -width_m / 2

    return north, south, east, west



def convert_to_relative_coordinates(gdf, bbox):
    """
    Convertit les coordonnées absolues (latitude, longitude) en coordonnées relatives par rapport au centre de la bbox.

    Args:
        gdf (GeoDataFrame): Données des objets en lat/lon.
        bbox (tuple): Limites (north, south, east, west) de la bbox.

    Returns:
        GeoDataFrame: gdf avec des coordonnées relatives.
    """
    north, south, east, west = bbox
    center_lat = (north + south) / 2
    center_lon = (east + west) / 2

    # Vérifier si le CRS est en lat/lon (EPSG:4326)
    if gdf.crs is None or gdf.crs.to_epsg() != 4326:
        raise ValueError("Le GeoDataFrame doit être en EPSG:4326 (latitude/longitude) avant conversion.")

    # Transformer le gdf en projection métrique (EPSG:3857 pour des distances en mètres)
    gdf = gdf.to_crs(epsg=3857)

    # Transformer le centre de la bbox en projection métrique
    import geopandas as gpd
    from shapely.geometry import Point

    center_point = gpd.GeoDataFrame(geometry=[Point(center_lon, center_lat)], crs="EPSG:4326").to_crs(epsg=3857)
    center_x, center_y = center_point.geometry.x[0], center_point.geometry.y[0]

    # Vérifier et convertir toutes les géométries en points
    gdf["geometry"] = gdf["geometry"].centroid  # Convertir les polygones et lignes en centroïdes

    # Maintenant, accéder aux coordonnées devient possible
    gdf["rel_x"] = gdf.geometry.x - center_x
    gdf["rel_y"] = gdf.geometry.y - center_y

    return gdf





