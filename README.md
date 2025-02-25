## 🧭 NAVigation par Reconnaissance de l'Environnement et Cartographie (NAVREC)

<p align="left">
  <img src="https://github.com/user-attachments/assets/ac08ac7e-f5ce-44bd-9b43-7882292135c2" alt="Description de l'image">
</p>

> projet de 3ème année : Systèmes embarquées [ENSTA Bretagne](https://www.ensta-bretagne.fr/fr)


## 📌 Description
<p align="justify">
Ce projet permet de faciliter le développement d'un algorythme de géolocalisation d'un quadricopter par reconnaissance du paysage vu du ciel, et comparaison à une carte vectorielle en mémoire. 
On vient pour cela simuler un drone qui cherche à suivre une navigation préparée à l'avance, et on vient utiliser des images satellites de l'IGN (20cm/pixel) pour simuler une prise d'image par la caméra du drone, pointée sur le sol.
</p>

<p align="justify">

</p>

Il utilise :
- 🔹 **Langage principal** : `Python`
- 🔹 **Framework utilisé** : `OSMnx`
- 🔹 **Base de données** : `BD ORTHO®`


## 🎨 Fonctionnalités principales

✅ **function.py** 



## 🗺️ Estimation de position : État de l'art
Pour estimer la position d'un point mobile sur une carte en utilisant les distances et angles relatifs aux bâtiments environnants, plusieurs méthodes de localisation peuvent être envisagées. Voici quelques-unes des approches pertinentes :

<p align="justify">
  
**1. Triangulation**
<p align="justify">
</p>
La triangulation est une méthode qui utilise les angles mesurés entre le point mobile et des repères fixes (ici, les bâtiments, routes, etc) pour déterminer la position. En connaissant les positions des bâtiments sur la carte et les angles sous lesquels ils sont observés depuis le point mobile, il est possible de tracer des lignes de visée dont l'intersection indique la position estimée du point mobile.
<p align="justify">
</p>

**2. Trilatération**
<p align="justify">
</p>
La trilatération repose sur la mesure des distances entre le point mobile et au moins trois repères connus. En traçant des cercles centrés sur chaque bâtiment avec un rayon correspondant à la distance mesurée, l'intersection de ces cercles fournit la position estimée du point mobile.
<p align="justify">
</p>
  
**3. Filtrage particulaire**
<p align="justify">
</p>
Le filtrage particulaire est une méthode probabiliste qui utilise un ensemble de "particules" pour représenter des hypothèses sur la position du point mobile. Chaque particule est pondérée en fonction de la correspondance entre les mesures (distances et angles) et les positions possibles sur la carte. Cette méthode permet de gérer les incertitudes et le bruit dans les mesures, offrant ainsi une estimation plus robuste de la position.
<p align="justify">
</p>

**4. Algorithmes de correspondance avec la carte (Map Matching)**
<p align="justify">
</p>
Cette approche consiste à ajuster l'estimation de la position en comparant les observations (distances et angles par rapport aux bâtiments) avec une carte détaillée de l'environnement. En identifiant les correspondances les plus probables entre les observations et les éléments de la carte, il est possible de raffiner l'estimation de la position du point mobile.
<p align="justify">
</p>

**5. SLAM (Simultaneous Localization and Mapping)**
<p align="justify">
</p>
Le SLAM est une méthode permettant de se localiser tout en construisant simultanément une carte de son environnement inconnu. Cette technique est couramment utilisée en robotique mobile pour naviguer sans GPS ni repères préalablement identifiés.


## 🗺️ Estimation de position : Solution retenue

Un algorithme de Map Matching m'a semblé être une bonne solution dans un premier temps. En effet le fait de reconstruire une carte au fur et à mesure dans le cas du SLAM ne me parait pas nécessaire étant donné que l'on possède déjà une carte connue en mémoire. Quand à aux méthodes de triangulation et trilatération, elles ne me semblent pas réalisable dans notre cas puisqu'on aurait besoin d'identifier des points précis (bâtiments spécifique, point de repère, etc), et notre détection d'image ne nous permettra probablement pas d'identifier des bâtiments spécifiques. (On aura probablement seulement des labels "batiments", "route", "rivière", etc.)

Après réfléxion, voici un pseudo-code de l'algorithme de map matching que je souhaite réalisé : 

```python
def map_matching(observations, map_objects, position_prev, search_radius):
    best_position = None
    best_score = float('inf')

    # Générer des positions candidates autour de la position précédente
    candidate_positions = generate_positions(position_prev, search_radius)

    for position in candidate_positions:
        score = 0
        
        for obs in observations:  # Parcours des objets observés
            x_obs = position.x + obs.distance * cos(obs.angle)
            y_obs = position.y + obs.distance * sin(obs.angle)

            # Rechercher les objets correspondants dans la carte
            candidates = filter_map_objects(map_objects, obs.label)
            nearest_object = find_nearest_object(candidates, x_obs, y_obs)

            # Calculer l'erreur
            dist_error = abs(obs.distance - compute_distance(position, nearest_object))
            angle_error = abs(obs.angle - compute_angle(position, nearest_object))

            # Additionner les erreurs pondérées
            score += weight_distance * dist_error + weight_angle * angle_error
        
        # Mettre à jour la meilleure position
        if score < best_score:
            best_score = score
            best_position = position

    return best_position
```

Ce que l'on veut réaliser avec cet algorithme : 

**- Génération de positions candidates** : Autour de la position précédente ou dans une zone de recherche définie. Typiquement on peut imaginer un filtre de kalman nous permettant de calculer une ellipse autour du point précédent, représentant les différentes positions possible pour notre point mobile. Dans cette ellipse on pourra alors choisir des points (les plus probables dans un premier temps) pour les comparer à notre point actuel (mais inconnu, on a à ce moment seulement l'observation depuis notre point)

**- Projection des observations sur la carte globale** :

Pour chaque objet observé (distance dk​, angle θk​, label Lk​) autour du point mobile :

** Calcul des coordonnées relatives **: 
    À partir de l'observation locale, calculez les coordonnées de l'objet autour du point mobile supposé (xt,yt)(xt​,yt​) :
    xk′=xt+dk⋅cos⁡(θk)
    yk′​=yt​+dk​⋅sin(θk​)

    où (xk′,yk′) sont les coordonnées estimées de l'objet dans la carte globale.

    Filtrage des objets candidats :
    Parcourez la carte globale et sélectionnez les objets dont le label correspond à LkLk​ (par exemple "bâtiment") et dont la distance par rapport à (xk′,yk′)(xk′​,yk′​) est minimale
 
**- Évaluation du score** : Le score mesure l'erreur entre les observations (distance et angle) et les positions réelles des objets sur la carte.

**- Optimisation** : Sélection de la position qui minimise l'erreur globale

AJOUTER EXPLICATION sur la façon dont on calcul la position 

Partie mise en place de tout ça, 

générer une carte vectorielle (depuis Openstreetmap) 
Créer une carte vectorielle avec shapely

calcul de distance et angles aux objets et modification du geodataframe

partie estimation de position avec filtre de kalman
