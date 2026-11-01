# Textures et modèles des corps célestes

État au 30 octobre 2026 (décisions P48 et P49 de `Choix_architecturaux.md`). Les cartes sont dans
`assets/textures/<nom anglais en minuscules>.jpg` et les modèles de forme dans `assets/models/<Nom>.obj`.

## Formats attendus

- **Carte** : équirectangulaire 2:1, nord en haut. La convention de longitude de chaque carte est notée dans
  `MAP_LON` (`gui/sim_view.py`) : par défaut, longitude 0 au centre et est vers la droite ; sinon, longitude est de
  0 à 360° depuis le bord gauche. Toute carte est lue en RVB et réduite à 8 192 px de large au plus.
- **Modèle** : OBJ en km, ou de taille unité ramenée au rayon moyen. Ses axes sont pris comme ceux du repère du corps
  (X le grand axe, tourné vers la planète ; Z le pôle), ce qui se règle modèle par modèle dans `SHAPE_MODELS`. Le
  placage de la carte est recalculé depuis la forme ; les coordonnées de texture du fichier ne servent pas.
- Un corps sans carte est dessiné dans une couleur unie. Un corps sans modèle est dessiné en sphère de son rayon
  moyen.

## État par corps

| Corps | Carte | Modèle | Orientation de la carte |
|---|---|---|---|
| Soleil, Terre, Lune | oui | sphère | — |
| Mercure, Vénus, Mars, Jupiter, Saturne (et anneaux), Uranus, Neptune, Pluton | oui | sphère | supposée (0 au centre) |
| Phobos | oui | oui | vérifiée (Stickney et Limtoc vus de 49° O) |
| Deimos | oui | oui | défaut, rendu cohérent, sans repère contrôlé |
| Io | oui | sphère | vérifiée (Pélé, Loki) |
| Europe | oui | sphère | vérifiée (Pwyll, hémisphère arrière) |
| Ganymède | oui | sphère | vérifiée (Galileo Regio, Osiris) |
| Callisto | oui | sphère | vérifiée (Valhalla) |
| Amalthée | oui (basse résolution) | oui (Stooke) | défaut, plausible (grande dépression sur celle du modèle) |
| Thébé | — (aucune donnée) | oui (Stooke) | — |
| Métis | — (aucune donnée) | — (aucune donnée) | — |
| Himalia | — | — | — |
| Mimas | oui | sphère | vérifiée (Herschel) |
| Encelade | oui | sphère | supposée comme les autres lunes de Saturne |
| Téthys | oui | sphère | vérifiée (Odysseus) |
| Dioné | oui | sphère | déduite (traînées de l'hémisphère arrière) |
| Rhéa | oui | sphère | déduite (traînées) ; Inktomi peu net, à confirmer |
| Titan | oui | sphère | vérifiée (Xanadu) |
| Hypérion | oui (basse résolution) | oui (Thomas) | défaut, vérifiée (grand cratère sur le bassin du modèle) ; rotation chaotique, axes par convention |
| Japet | oui | oui (relief Schenk) | carte vérifiée (Cassini Regio) ; longitude 0 du relief non vérifiée |
| Phœbé, Janus, Épiméthée, Prométhée, Pandore, Hélène, Atlas | — | — | — |
| Miranda, Ariel, Umbriel, Titania, Obéron | oui | sphère | non vérifiée (seul l'hémisphère sud a été photographié) |
| Triton | oui | sphère | non vérifiée (même raison) |
| Larissa | — (aucune donnée) | oui (Stooke) | — |
| Protée, Galatée, Despina, Thalassa, Naïade | — | — | — |
| Néréide | — | — | jamais résolue : sphère grise |
| Charon | oui | sphère | non vérifiée |
| Nix, Hydra | — | — | — |

## Manques

1. **Cartes manquantes** pour des modèles déjà fournis : Thébé, Larissa (aucune donnée disponible d'après
   l'utilisateur, 30 octobre 2026 : elles restent en couleur unie).
2. **Modèles et cartes manquants** : Himalia, Phœbé, Janus, Épiméthée, Prométhée, Pandore, Hélène, Atlas, Protée,
   Galatée, Despina, Thalassa, Naïade, Nix, Hydra. Métis : aucune donnée disponible d'après l'utilisateur.
3. **Orientations à confirmer** : Miranda, Ariel, Umbriel, Titania, Obéron, Triton, Charon (aucun repère sûr), Rhéa,
   Deimos, Amalthée, longitude 0 du relief de Japet, cartes des planètes.
4. **Crédits à compléter** dans la fenêtre About : la ligne des textures, qui ne cite encore que celles de la Terre,
   de la Lune et du Soleil (les modèles de Phobos et de Deimos : NASA/JPL-Caltech, ajouté le 30 octobre 2026).
5. **Données de mémoire, à vérifier** : données physiques et rotation des planètes (`[sim.planets]`), fiches Info des
   planètes, rayons des anneaux de Saturne (`tool.display.sim_saturn_rings_km`), couleurs des points.
6. **Éphémérides des lunes** : éléments moyens du JPL, forme et orientation des orbites justes mais phase qui dérive
   avec les années (résonance de Laplace d'Io, Europe et Ganymède : 180° en 2000, 190° en 2050). Pour des scénarios
   précis autour d'une lune, il faudra de vraies éphémérides (fichiers SPICE du JPL).
7. **Corps centraux autres que la Terre et la Lune** (P50) : J2 du Soleil, de Pluton et des lunes laissés à 0 (aucune source lue ne le donne ; Io, Europe, Ganymède, Callisto et Titan en ont un notable) ; pas de bruit ni de fouillis radar pour ces corps, qui ne font qu'occulter, et ne figurent donc pas dans le ciel d'une mission exportée.
