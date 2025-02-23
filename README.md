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

✅ **create_satellite_map.py** – <br>
Permet de selectionner une zone géographique carrée et de créer une image satellite ainsi qu'une carte vectorielle de cette zone pour simulation. ces dernières serviront pour simuler les prise de vus par le drone en vol. 


## 🗺️ Estimation de position : État de l'art
Pour estimer la position d'un point mobile sur une carte en utilisant les distances et angles relatifs aux bâtiments environnants, plusieurs méthodes de localisation peuvent être envisagées. Voici quelques-unes des approches pertinentes :

<p align="justify">
  
**1. Triangulation**
</p>
La triangulation est une méthode classique qui utilise les angles mesurés entre le point mobile et des repères fixes (ici, les bâtiments, routes, etc) pour déterminer la position. En connaissant les positions des bâtiments sur la carte et les angles sous lesquels ils sont observés depuis le point mobile, il est possible de tracer des lignes de visée dont l'intersection indique la position estimée du point mobile.
</p>

**2. Trilatération**
</p>
La trilatération repose sur la mesure des distances entre le point mobile et au moins trois repères connus. En traçant des cercles centrés sur chaque bâtiment avec un rayon correspondant à la distance mesurée, l'intersection de ces cercles fournit la position estimée du point mobile.
</p>
  
**3. Filtrage particulaire**
</p>
Le filtrage particulaire est une méthode probabiliste qui utilise un ensemble de "particules" pour représenter des hypothèses sur la position du point mobile. Chaque particule est pondérée en fonction de la correspondance entre les mesures (distances et angles) et les positions possibles sur la carte. Cette méthode permet de gérer les incertitudes et le bruit dans les mesures, offrant ainsi une estimation plus robuste de la position.
</p>

**4. Algorithmes de correspondance avec la carte (Map Matching)**
</p>
Cette approche consiste à ajuster l'estimation de la position en comparant les observations (distances et angles par rapport aux bâtiments) avec une carte détaillée de l'environnement. En identifiant les correspondances les plus probables entre les observations et les éléments de la carte, il est possible de raffiner l'estimation de la position du point mobile.
</p>

<p align="justify">
