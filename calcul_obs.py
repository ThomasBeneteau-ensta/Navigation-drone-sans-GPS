import pandas as pd
from pyproj import Geod
from math import atan2, degrees
import matplotlib.pyplot as plt

import function

def calculate_distances_and_angles(gdf, bbox):
    """
    Calcule les distances et angles relatifs des objets par rapport au centroïde global de la carte.

    Args:
        gdf (GeoDataFrame): Objets géographiques.

    Returns:
        GeoDataFrame: Objets avec distances et angles relatifs ajoutés.
        tuple: (lat, lon) du centre calculé.
        DataFrame: Tableau des labels, distances et angles pour chaque objet.
    """
    # Calculer la position centrale de la carte
    north, south, east, west = bbox  # Récupérer les limites
    center_lat = (north + south) / 2
    center_lon = (east + west) / 2

    # Initialiser un géodésique pour les calculs
    geod = Geod(ellps="WGS84")

    # Calculer la distance et l'angle
    def compute_distance_and_angle(row):
        obj_x, obj_y = row.geometry.centroid.xy
        obj_lon, obj_lat = obj_x[0], obj_y[0]

        # Distance en mètres
        _, _, distance = geod.inv(center_lon, center_lat, obj_lon, obj_lat)

        # Angle relatif
        dx = obj_lon - center_lon
        dy = obj_lat - center_lat
        angle = degrees(atan2(dy, dx)) % 360  # Angle en [0, 360]

        return pd.Series({'distance_m': distance, 'angle_deg': angle})

    # Appliquer les calculs
    gdf[['distance_m', 'angle_deg']] = gdf.apply(compute_distance_and_angle, axis=1)

    # Créer un tableau avec les labels, distances et angles
    table = gdf[['object_type', 'distance_m', 'angle_deg']].reset_index(drop=True)

    return gdf, (center_lat, center_lon), table


def plot_map_with_distances(gdf, bbox, center, title="Carte avec distances et angles"):
    """
    Affiche une carte avec les objets et leurs distances/angles relatifs.

    Args:
        gdf (GeoDataFrame): Objets géographiques avec distances et angles calculés.
        bbox (tuple): Limites (north, south, east, west) de la zone.
        center (tuple): Latitude et longitude du centre calculé.
        title (str): Titre de la carte.
    """
    center_lat, center_lon = center
    north, south, east, west = bbox

    # Créer une carte
    fig, ax = plt.subplots(figsize=(10, 10))

    # Afficher tous les objets
    gdf.plot(ax=ax, color='green', alpha=0.7, label='Objets')

    # Afficher la position centrale (drone)
    ax.plot(center_lon, center_lat, 'ro', markersize=10, label='Position supposé')

    # Annoter les distances et angles
    for _, row in gdf.iterrows():
        obj_geom = row.geometry.centroid
        ax.annotate(f"{row['distance_m']:.1f}m\n{row['angle_deg']:.1f}°",
                    xy=(obj_geom.x, obj_geom.y),
                    xytext=(obj_geom.x + 0.0001, obj_geom.y + 0.0001),
                    arrowprops=dict(facecolor='black', arrowstyle="->"),
                    fontsize=8)

    # Ajuster les limites de la carte
    ax.set_xlim(west, east)
    ax.set_ylim(south, north)

    # Ajouter une légende et un titre
    plt.legend()
    plt.title(title)
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.show()


# Exemple d'utilisation
if __name__ == "__main__":

    # Définir un point central (latitude et longitude) et une distance pour la bbox
    #center_lat, center_lon = 46.525630, -1.176239  # Exemple
    center_lat, center_lon = 46.520938, -1.174474
    distance_carte_connue = 200  # distance bbox carte connues
    distance_obs = 80  # distance bbox observation

    # Générer les données vectorielles
    gdf_connue, bbox_known_map = function.gen_vect_map(center_lat, center_lon, distance_carte_connue)
    gdf_obs, bbox_obs = function.gen_vect_map(center_lat, center_lon, distance_obs)

    function.plot_vect_map(gdf_connue, bbox_known_map, title="Carte connue")
    function.plot_vect_map(gdf_obs, bbox_obs, title="Observation")

    # Ici on donne directement la bbox, mais sur une carte générer par l'image du drone
    # il faudra la recalculer en fonction de la hauteur du drone (dimension de l'image en fonction de la caméra)

    # Calculer les distances et angles pour tous les objets
    gdf_obs_dist_angle, computed_center_obs, table_obs = calculate_distances_and_angles(gdf_obs, bbox_obs)
    gdf_connue_dist_angle, computed_center_connue, table_known = calculate_distances_and_angles(gdf_connue, bbox_known_map)

    # Afficher les résultats
    plot_map_with_distances(gdf_obs_dist_angle, bbox_obs, computed_center_obs, title="Objets sur la carte avec distances et angles")
    plot_map_with_distances(gdf_connue_dist_angle, bbox_known_map, computed_center_connue, title="Objets sur la carte avec distances et angles")

    # Afficher dans le terminal
    print("Position calculée du drone (centroïde) :", computed_center_obs)
    print("Objets avec distances et angles :")
    print(table_obs)
