# Outil de préparation de mission radar : choix architecturaux

Ce document recense tous les choix d'architecture faits jusqu'ici : organisation du logiciel, modèle du radar, et pour chaque variable sa définition, sa valeur, son statut et la raison de cette valeur. Il décrit l'état du dépôt après l'étape 1.2 de la feuille de route (modèle V6.2.0, 20 octobre 2026).

> **Avertissement.** Les missions de `tests/missions/` sont des **missions de test**. Les valeurs figées de `tests/snapshots/` sont des **valeurs figées de test**. Elles servent seulement à détecter les régressions du code. Ce ne sont ni des exigences ni des performances du radar, dont les capacités réelles ne sont pas établies.

> **Source de vérité.** Les valeurs ci-dessous sont celles de `radar_system.toml` à la date du document. En cas d'écart, c'est le fichier qui fait foi : ce document explique les choix, il ne remplace pas le fichier.

---

## Sommaire

1. [But de l'outil et méthode de travail](#1-but-de-loutil-et-méthode-de-travail)
2. [Architecture logicielle](#2-architecture-logicielle)
3. [Le fichier de paramètres unique](#3-le-fichier-de-paramètres-unique)
4. [Banc de tests](#4-banc-de-tests)
5. [Paramètres système, section par section](#5-paramètres-système-section-par-section)
6. [Grandeurs dérivées (calculées, sans entrée dans le fichier)](#6-grandeurs-dérivées-calculées-sans-entrée-dans-le-fichier)
7. [Modèles et formules](#7-modèles-et-formules)
8. [Paramètres de l'outil (`[tool.*]`)](#8-paramètres-de-loutil-tool)
9. [Modèle de mission (entrées opérateur)](#9-modèle-de-mission-entrées-opérateur)
10. [Feuille de route, renvois et points ouverts](#10-feuille-de-route-renvois-et-points-ouverts)
11. [Historique des décisions](#11-historique-des-décisions)

---

## 1. But de l'outil et méthode de travail

### 1.1 But

Construire un outil dans lequel l'opérateur prépare sa mission. Cette mission est ensuite **validée contre les contraintes et les ressources du système radar dans son ensemble** : radiofréquence, ordonnancement, puissance, thermique, calcul, etc. Le simple bilan de liaison radio ne suffit pas.

### 1.2 Le radar modélisé

- Radar AESA spatial à **4 faces** réparties à 90° autour de l'axe du vaisseau, toutes inclinées de 25° vers l'avant. Ce modèle est dit « phare » : l'axe du vaisseau est vertical, la proue au zénith.
- Architecture **entièrement numérique** : un convertisseur analogique-numérique (CAN) par module d'émission-réception (TRM), retard vrai numérique par TRM, formation de faisceau en réception par calcul.
- **Bande S, de 2,7 à 2,9 GHz** : veille, c'est-à-dire la recherche des cibles.
- **Bande Ku, de 13,5 à 14,5 GHz** : confirmation des détections et poursuite active.
- Chaque face est un pavage bi-bande de **196 TRSA**. Une TRSA est une tuile de 25 × 25 TRM Ku et 5 × 5 TRM S, et les TRSA sont disposées en octogone.

### 1.3 Méthode (règles fixées par l'utilisateur)

| Règle | Contenu |
|---|---|
| Feuille de route | Phases 0 à 5, suivies étape par étape, de session en session (§10). |
| Expliquer avant de coder | Avant chaque étape : rappeler sa raison d'être, ses entrées et ses sorties, puis obtenir l'accord. |
| Demander chaque variable | Pour chaque étape, la valeur de chaque variable est demandée à l'utilisateur. Aucune n'est choisie en silence. |
| Fichier global | Toutes les variables système vivent dans un seul fichier, modifiable rapidement (§3). |
| Valeurs de test | Les missions de test et les valeurs figées ne sont jamais présentées comme des références ou des performances. |
| Commits | Un commit par étape, et seulement sur demande. |

---

## 2. Architecture logicielle

### 2.1 Modules

| Module | Rôle |
|---|---|
| `core/params.py` | Charge `radar_system.toml`, une seule fois par processus. Expose `P.get`, `P.meta`, `P.section` et l'empreinte des paramètres (`fingerprint`). La variable d'environnement `RADAR_PARAMS` permet de désigner un autre fichier. |
| `core/radar_model.py` | Modèle radar V6.2.0 : désadaptation active, marge de lobe de réseau, lobes visibles selon le dépointage, pureté de polarisation, bandes, gains, diagramme, pertes, équation radar sous forme énergétique, besoin en SNR de détection (Shnidman) et de poursuite (borne de Cramér-Rao), grille de veille dans l'espace (u, v), et `solve()`. |
| `core/antenna.py` | **Étapes 1.1 et 1.2.** Diagramme de rayonnement calculé à partir de la composition des TRSA : faisceau d'émission uniforme, pondération de réception par niveau de lobes, étendue RMS de l'ouverture, profil des lobes lointains sur tout l'espace visible (lobes de jeu compris). Résultat mis en cache dans `core/antenna_cache.json`. |
| `core/optimizer.py` | Optimiseur de la veille S. Choisit pour chaque secteur la fréquence et les formes d'onde par tranche d'angle de dépointage, et établit le bilan énergie et temps par face. |
| `core/tracking.py` | Confirmation (modes KU1+7, KU7 et S2) et poursuite active (AT) en Ku, ajoutées au bilan par face. |
| `core/cost_estimate.py` | Estimation rapide pour l'interface, à partir de `core/cost_table.json`. Elle n'est pas faite pour décider : seul le calcul exact fait foi. |
| `core/mission.py` | Modèle de mission : valeurs par défaut, validation, chargement et sauvegarde JSON et CSV. |
| `core/redundancy.py` | Détecte si un secteur couvre déjà le besoin d'un autre, dans les deux sens. |
| `core/geometry.py` | Trame de la sphère des directions pour la 3D et l'estimation rapide. Modèle 3D du vaisseau. |
| `gui/` | Interface PyQt5 : préparation, résultats, vue 3D (pyvista). |
| `tools/gen_cost_table.py` | Reconstruit `core/cost_table.json`. |
| `tests/` | Banc de tests (§4). |

### 2.2 Fichiers dérivés et leur cohérence

| Fichier | Contenu | Garantie de cohérence |
|---|---|---|
| `core/cost_table.json` | Énergie minimale par pointage selon la distance, la SER, la vitesse et l'angle, à 2,8 GHz. Il enregistre aussi l'option de lobes de réception utilisée. | Contient l'empreinte des paramètres. L'interface alerte si elle ne correspond plus, et un test vérifie qu'elle est à jour. Pour la reconstruire : `python tools/gen_cost_table.py`. |
| `core/antenna_cache.json` | Diagrammes calculés pour chaque bande. | Indexé par un hachage de toutes ses entrées : composition, options, n̄, réglages `[tool.pattern]` et version de l'algorithme. Il est recalculé automatiquement si les entrées changent, en environ 8 s. Il n'est **réécrit que pour le fichier de paramètres par défaut** : un essai sur une copie modifiée ne touche pas au cache du dépôt. Un test vérifie qu'il est à jour. |

### 2.3 Choix techniques

| Choix | Raison |
|---|---|
| Python 3.11 au minimum | Le module `tomllib`, qui lit le TOML, est dans la bibliothèque standard depuis cette version. |
| numpy seulement, sans scipy | Évite d'ajouter une dépendance. Les fonctions de Bessel J₀ et J₁ et les zéros de J₁ sont recodés : intégrale de Bessel par la règle du point milieu, puis Newton (étape 1.1, décision Q4). |
| Processus parallèles pour l'optimiseur | Le calcul est en Python pur et sollicite le processeur ; des fils d'exécution seraient bridés par le GIL. Le pool de processus est démarré une fois par session. |
| Cache des portées par processus | Les relances depuis l'interface et les secteurs qui partagent une menace réutilisent les résultats déjà calculés. |
| Symétrie de l'octogone exploitée dans `antenna.py` | Le facteur de réseau devient une somme réelle de cos·cos sur un seul quadrant, et chaque coupe se ramène à un produit matriciel. Sans cela, le calcul en Ku (122 500 éléments) serait trop lent. |

---

## 3. Le fichier de paramètres unique

| Décision | Valeur retenue |
|---|---|
| Format | TOML (étape 0.1). |
| Organisation | **Un seul fichier monolithique**, `radar_system.toml`. Les sections système viennent d'abord, puis un bloc `[tool.*]` séparé pour les réglages de l'outil. |
| Format d'une entrée système | `nom = { value, unit, status, source, date }`. Les cinq clés sont obligatoires et le chargeur refuse une entrée incomplète. |
| Statuts | **V** = verrouillé par l'utilisateur. **D** = dérivé ou proposé, à confirmer. **?** = conflit connu entre sources, à trancher. |
| Unités | `"-"` pour un nombre sans dimension, `""` pour un booléen ou un texte. |
| Réglages `[tool.*]` | Valeur seule ou table complète. Ils ne décrivent pas le radar : ce sont des grilles de recherche, des trames, des caches et des valeurs par défaut de l'interface. |
| Modification | À la main dans le fichier pour l'instant. Un onglet de l'interface est prévu à l'étape 5.5. |
| Autre fichier | Variable d'environnement `RADAR_PARAMS=<chemin>`, héritée par les processus de l'optimiseur. |
| Revue | Toutes les valeurs ont été validées une par une lors de la revue 0.3. Les ajouts de l'étape 1.1 ont été validés pendant cette étape. |

**Bilan actuel : 126 entrées système, dont 82 V, 41 D et 3 ?.** Les trois entrées « ? » sont `multibeam.l8_db`, `multibeam.p` et `band.S.e_buf_trm_j`.

---

## 4. Banc de tests

| Élément | Choix |
|---|---|
| Outil | `pytest`, déclaré dans `requirements-dev.txt`. |
| Famille 1 : valeurs figées | `tests/test_snapshots.py`. Toutes les sorties numériques de l'outil sont comparées avec une **tolérance relative de 1e-9**. |
| Famille 2 : physique et invariants | `tests/test_physics.py`. Lois qui doivent tenir quelles que soient les valeurs : R ∝ σ^¼, R ∝ P^¼, cohérence du nombre de pointages, du cache, de la borne de Cramér-Rao, des options de lobes, etc. Le self-test du modèle est conservé. |
| Missions de test | `mission_example` (en JSON et en CSV), `mission_veille500`, `mission_redundancy_demo`, `mission_synthetic_mix`. Les deux premières ont été récupérées dans l'historique git. |
| Rapport d'impact | `python tests/impact_report.py`, à chaque étape, avant de régénérer les valeurs figées. |
| Régénération | `python tests/update_snapshots.py`. Chaque régénération est consignée dans `tests/snapshots/JOURNAL.md`, avec sa cause et les formes d'onde ou modes qui ont changé. |
| État | 77 tests passent après l'étape 1.2. |

---

## 5. Paramètres système, section par section

Les colonnes **Valeur** et **Statut** reprennent le fichier. La colonne **Justification** résume la source et la raison du choix.

### 5.1 `[meta]`

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `file_version` | Version du format du fichier de paramètres. | "1.0" | D | Créé à l'étape 0.1 : les constantes ont été sorties du code sans changer de valeur. |

### 5.2 `[geometry]` : géométrie des faces

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `face_phi_deg` | Azimut des normales des 4 faces autour de l'axe Z, la proue. | [0, 90, 180, 270]° | V | 4 faces × 90°, confirmé lors de la revue 0.3. |
| `tilt_deg` | Inclinaison commune des 4 réseaux vers +Z. | 25° | V | Valeur mise à jour (20° en V6.0.12). Toutes les formules en ε et θ en dérivent, y compris le cône de montage de la vue 3D. |

Le **rayon du secteur d'une face** n'a pas d'entrée. Il est calculé comme la plus large limite de balayage des bandes, soit 75° aujourd'hui (revue 0.3). La géométrie suit ainsi toujours les limites de balayage. Les frontières entre faces sont les plans d'azimut ±45°.

### 5.3 `[aperture]` : composition de l'antenne et diagramme

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `ku_trm_pitch_m` | Pas d'un TRM Ku : sa face émissive de 10,4 × 10,4 mm. | 0,0104 m | V | Note de conception du réseau (revue 0.3). λ/2 à 14 GHz vaut 10,7 mm. |
| `ku_trm_per_trsa_side` | TRM Ku par côté de TRSA. | 25, soit 625 par TRSA | V | Note de conception. |
| `s_trm_per_trsa_side` | TRM S par côté de TRSA. | 5, soit 25 par TRSA | V | Pas de 52 mm, proche de λ/2 en S. C'est le seul diviseur de 25 proche de λ/2 : chaque TRM S recouvre exactement un bloc de 5 × 5 TRM Ku, et le code vérifie cette divisibilité. |
| `trsa_rows` | Nombre de TRSA par rangée de la grille. Les rangées sont centrées et la liste doit être un palindrome (symétrie). | [6, 8, 10, 12, 14, 16, 16, 16, 16, 16, 16, 14, 12, 10, 8, 6], soit 196 TRSA | V | Octogone approché (revue 0.3). |
| `trsa_gap_m` | Jeu mécanique libre entre TRSA voisines, qui forme une zone morte. | 0,0005 m | V | **Ramené de 2 mm à 0,5 mm à l'étape 1.2** (décision de l'utilisateur). Avec 2 mm, la rupture du pas entre TRSA créait des lobes de jeu voisins du lobe de réseau (−16,6 dB en Ku) qui ne tenaient pas les options de réception aux grands dépointages. 0,5 mm est le plus petit écart qui libère presque tout. Il **suppose des matériaux appariés en dilatation** entre TRSA et plaque de fond : environ 0,08 mm d'excursion sur 100 K, contre environ 0,6 mm pour de l'aluminium sur fibre de carbone (estimation à confirmer par un mécanicien). Le gain utilise la surface émissive seule. |
| `rx_sll_options_db` | Niveaux de crête des lobes secondaires de la **pondération de réception** offerts à l'opérateur pour chaque volume de veille. | [−25, −30, −35] dB | V | **Étape 1.1.** Le réglage est fait par volume dans la préparation de mission (clé `rx_sll_db`), avec un seul réglage pour S et Ku et −25 dB par défaut. Descendre en niveau rejette mieux les échos forts hors de l'axe (cible brillante, Lune ou Terre, brouilleur) au prix du gain en réception : −0,56 dB à −30, −1,04 dB à −35. |
| `rx_taper_nbar` | Paramètre n̄ de la loi de Taylor circulaire de réception. | 5 | V | Étape 1.1. Loi standard ; son niveau de conception est **calé numériquement** pour que le niveau **atteint** sur l'octogone réel égale l'option demandée. |
| `rx_taper_edge` | Rayon où la loi de Taylor atteint sa valeur de bord. | "inscribed", soit D_env/2 | D | Exploration de l'étape 1.1 : à niveau atteint égal, le cercle inscrit donne un meilleur rendement que le cercle circonscrit. Les éléments des coins gardent la valeur de bord. L'autre valeur possible est "circumscribed". |
| `grating_lobe_g0` | Largeur, en u, de la remontée de \|Γ\| quand le premier lobe de réseau approche de l'espace visible : \|Γ_LR\| = exp(−g/g₀). | 0,02 | V | **Étape 1.2, Q3.** Ce terme seul coûte environ 3,0 dB par passage à 2,9 GHz et 75° (4,1 dB avec le terme WAIM), 0,07 dB à 2,85 GHz et 75°, et presque rien à 2,8 GHz. Il explique ainsi la limite de 70° au-dessus de 2,85 GHz sans pénaliser le reste de la bande. |
| `tx_polarisation` | Polarisation d'émission. | "linear_ev" | D | **Étape 1.2, Q7.** Linéaire selon e_v, l'axe de la face contenu dans le plan de l'axe du vaisseau. Un seul PA par TRM. Elle ne change pas le bilan, puisque la réception bipolarisation capte les deux composantes ; elle sert de référence pour Faraday (2.5) et la polarimétrie (3.3). **À rappeler à l'étape 3.3** : la commutation e_u / e_v à chaque observation (un PA et un commutateur, environ 0,1 à 0,2 dB) reste une option. |
| `eta_ohmic` | Rendement de l'élément rayonnant. | 1,0 | V | Pas de réseau d'alimentation dans l'architecture numérique V6 ; les chemins locaux sont comptés dans TX_chain et T_sys. L'ancienne valeur 0,96 venait de l'alimentation à faibles pertes de V5.6.0. Ce paramètre pourra porter des pertes supplémentaires plus tard. Portée +2,06 %. |

Retraits faits lors de la revue 0.3 et de l'étape 1.1 :

| Ancienne variable | Sort | Raison |
|---|---|---|
| `hpbw_factor` = 0,886 | Retiré du fichier (revue 0.3), puis du code (étape 1.1). | 0,886 λ/D est la largeur de lobe d'une ouverture **rectangulaire** uniforme. Elle est remplacée par le diagramme calculé de l'octogone. |
| `scan_exponent` = 1,1 et section `[scan_loss]` | Supprimés (revue 0.3). | Ils comptaient deux fois la même perte. Ils sont remplacés à l'étape 1.2 par le modèle unique de \|Γ(θ, f)\| (`waim_match_deg`, `grating_lobe_g0`). |
| `scan_radius_deg` | Supprimé (revue 0.3). | Remplacé par le rayon calculé : la plus large limite de balayage des bandes. |
| Surface saisie de 13,25 m² | Remplacée par la surface calculée de 13,2496 m². | Dérivée de la composition des TRSA. Les chiffres de la note initiale contenaient une coquille : la surface totale valait 13,4368 m² avec 2 mm de jeu ; 187 200 mm² et non 134 368. |

### 5.4 `[search_grid]` : grille de veille

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `beam_spacing_hpbw` | Pas du réseau hexagonal des pointages d'émission, en largeurs de lobe d'émission. | 1,04 | V | Optimum dans le pire cas (V6.0.10) : il comble les trous aux sommets des hexagones (point le plus défavorable à −4,3 dB). Le même pas fixe le nombre de pointages et la perte de grille d'émission moyenne (1,81 dB). Depuis l'étape 1.1, il s'applique au **lobe d'émission réel** (1,035 λ/D). À revoir à l'étape 2.3. |
| `rx_beam_spacing_hpbw` | Pas du réseau des faisceaux de réception numériques formés dans chaque pointage, en largeurs de lobe de réception. | 0,5 | V | **Étape 1.1, option C.** Perte de grille résiduelle en réception de 1,672 × 0,5² = 0,42 dB, pour environ 3,4 faisceaux de réception par pointage à −25 dB. C'est le point où ajouter des faisceaux ne rapporte plus beaucoup : passer à 0,3 triple le calcul pour 0,27 dB. L'option A (perte nulle) supposait un calcul infini ; l'option B (un seul faisceau) coûtait 1,42 dB. Le nombre de faisceaux alimentera le bilan de calcul de l'étape 4.4, et l'optimiseur pourra régler ce pas aux étapes 3.1 et 5.2. |
| `enable_grid_loss` | Prise en compte de la perte de grille moyenne. | vrai | V | La V6.0.0 était optimiste d'environ 3,3 dB sans elle. Le choix entre moyenne et pire point sera revu à l'étape 2.3. |
| `frame_interleaved` | Attente de l'écho masquée par l'entrelacement des pointages. | vrai | D | Hypothèse d'ordonnancement, possible grâce à la réception numérique. À vérifier à l'étape 4.3. Avec « faux », la trame ajoute 2R/c par pointage. |

### 5.5 `[detection]`

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `pd` | Probabilité de détection d'une observation, pour tous les secteurs. | 0,9 | V | Pd cumulée et réglage par secteur aux étapes 2.3 et 5.1. |
| `pfa` | Probabilité de fausse alarme par case distance-Doppler. | 1e-8 | V | À 1e-6, confirmer les fausses alarmes coûtait deux fois la veille elle-même. L'ajustement de Shnidman reste valable jusqu'à 1e-9. Le nombre réel de cases sera modélisé à l'étape 2.2. |

### 5.6 `[power]` : chaîne de puissance

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `pae` | Rendement en puissance ajoutée de la chaîne d'émission, commun à S et Ku. | 0,75 | V | Remplace 65 %. Valeur constante pour l'instant ; une **PAE variable** par bande, longueur d'impulsion et facteur de forme est demandée pour l'étape 1.3. |
| `psu_eff` | Rendement de l'alimentation. | 0,95 | V | Verrouillé lors de la revue 0.3. |
| `distribution_eff` | Rendement de la distribution. | 0,995 | V | Verrouillé lors de la revue 0.3. |
| `overhead_frac` | Part de la consommation hors émission. | 0,03 | V | Valeur conservée. Elle sera remplacée par un vrai bilan (réception numérique et CAN, environ 130 kW par face, contrôle, refroidissement) aux étapes 4.1 et 4.4. |
| `face_elec_mw` | Puissance électrique allouée à une face. C'est la valeur par défaut d'une mission. | 2,0 MW | V | Verrouillé lors de la revue 0.3. |
| `ship_elec_total_mw` | Puissance totale du vaisseau disponible pour le radar. | 8,0 MW | V | **Répartissable à la demande entre les faces**, par exemple 3 MW sur F1 et 5 MW sur F2 à F4. Le logiciel radar gère cette répartition avec les autres paramètres (étapes 4.1 et 5.2). Aujourd'hui, la validation vérifie seulement qu'une face ne dépasse pas 8 MW : c'est temporaire, sur décision de l'utilisateur. |

Valeur dérivée : CHAIN_EFF = 0,95 × 0,995 × (1 − 0,03) = 0,917. La puissance RF moyenne disponible pour 2 MW électriques vaut 2 MW × 0,917 × 0,75 = 1,375 MW.

### 5.7 `[migration]` : migration en distance

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `mode` | Traitement de la migration en distance. | "COMPENSATED" | V | Traitement de type keystone ou Radon. Le coût des bancs de vitesse sera évalué aux étapes 2.2 et 4.4. |
| `residual_db` | Perte résiduelle quand la migration est compensée. | 1,0 dB | V | Valeur forfaitaire. La perte en fonction du nombre de cases traversées viendra à l'étape 2.2, le couplage distance-Doppler à l'étape 2.1. |

### 5.8 `[multibeam]` : multifaisceau en émission

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `l8_db` | Perte en émission avec 8 faisceaux, selon une loi unique. | 0,8 dB | ? | Vient de V5.6.0, sa définition est inconnue. La perte en réception vaut 0 (réception numérique). |
| `p` | Exposant de la loi L8 × (log₂B / 3)^p. | 1,1 | ? | Donne 0,24, 0,51 et 0,80 dB pour 2, 4 et 8 faisceaux. |

Décision : « tous les modes sont possibles et sont à évaluer par l'optimiseur » (étape 3.1). Trois modes sont à modéliser : (a) superposition de faisceaux dans chaque TRM, avec le recul du PA qui en découle (étape 1.3) ; (b) partition de l'ouverture, à partir du diagramme de l'étape 1.1 ; (c) multiplexage en temps ou en fréquence (étape 2.1).

### 5.9 `[coherence]` : cohérence et bruit de phase

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `enable` | Active la perte de cohérence, en intégration cohérente seulement. | vrai | D | Modèle V5.4.0, sans source connue. |
| `t0_s` | Constante de temps de cohérence. | 0,25 s | D | **Aucune source connue** : on ne sait pas si elle relève de l'oscillateur, de la plateforme ou de la cible. Un T₀ par type de cible viendra à l'étape 2.4. |
| `p` | Exposant du terme de cohérence. | 2,0 | D | Aucune source connue. |
| `eta_min` | Plancher de l'efficacité de cohérence, qui plafonne la perte à 1,55 dB. | 0,70 | D | Atteint à T = t₀(−ln η_min)^(1/p) = 149 ms. **Disposition provisoire :** les générateurs de formes d'onde écartent les intégrations cohérentes plus longues (`COH_T_MAX_S`), car au-delà le plancher créditait N·τ sans limite et provoquait des égalités exactes dans l'optimiseur. Ce plafond sera remplacé à l'étape 2.2 par des blocs cohérents suivis d'une intégration non cohérente. |

Retiré : `sigma_t`, la gigue d'horloge. Son effet est négligeable (1e-7 à 2,8 GHz).

### 5.10 `[estimation]` : précision des mesures

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `angle_crlb_margin` | Marge appliquée à la **borne de Cramér-Rao angulaire** de l'ouverture numérique complète. | 1,13 | V | **Étape 1.1, option a.** σ_u = marge × λ / (2π x_rms √(2 SNR)), où x_rms vient de la composition des TRSA (0,248 D). Cette précision **ne dépend pas de la pondération de réception** : l'estimateur ML utilise toute l'ouverture. La marge est la même que pour k_f et k_r. Elle remplace k_θ = 0,5 × HPBW(0,886), qui était légèrement sous la borne. Effet : +1,27 dB de SNR demandé pour une poursuite pilotée par l'angle. |
| `k_f` | Constante de l'estimateur Doppler. | 0,6 | V | Borne pour une sinusoïde seule, 0,551, multipliée par 1,13 (revue 0.3). L'ancienne valeur 0,5 était sous la borne. |
| `range_crlb_kr` | Constante de l'estimateur de distance. | 0,6 | V | Borne pour un spectre rectangulaire, 0,551, multipliée par 1,13 ; `sig_bw` est la bande **totale**. Ancienne valeur : 1,3, sans source. La pondération de Taylor sera déduite de la modulation réelle à l'étape 2.1. Une suggestion extérieure donnait k_r = 0,32 pour une bande RMS. Elle a été corrigée (la bonne valeur serait 0,16) et ne s'applique pas ici, puisque la bande utilisée est la bande totale. |

Retiré : `k_theta` = 0,5, remplacé par `angle_crlb_margin` à l'étape 1.1.

### 5.11 `[band.S]` : bande S (veille)

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `usable_lo_ghz` / `usable_hi_ghz` | Bande utile. | 2,7 à 2,9 GHz | V | Confirmé lors de la revue 0.3. |
| `core_inset_divisor` | La bande centrale est la bande utile amputée de 1/6 de sa largeur de chaque côté. | 6,0 | D | La loi de bord de V5.6.0 (§5.2.3) avait été établie sur l'ancienne bande Ku de 14,8 à 17,2 GHz. Elle est gardée **en fraction de bande** (option a). Une table propre à chaque bande viendra à l'étape 1.4. |
| `edge_derating_db` | Dégradation maximale aux bords de la bande utile, en cosinus surélevé. | 1,5 dB | D | Même origine ; la bande S copie la bande Ku (étape 1.4). |
| `scan_max_deg` | Limite de balayage jusqu'à `scan_break_ghz`. | 75° | V | Confirmé. |
| `scan_break_ghz` | Fréquence au-dessus de laquelle la limite de balayage baisse. | 2,85 GHz | V | Avec le pas moyen réel (étape 1.2), la marge de lobe de réseau à 2,9 GHz et 75° valait 0,35 % avec 2 mm de jeu (désadaptation 4,1 dB par passage), d'où les 70° au-dessus. Avec 0,5 mm, elle vaut 0,9 % (1,8 dB par passage, sous le seuil d'alerte). Les limites restent verrouillées (Q5) ; les relâcher au-dessus de 2,85 GHz serait une décision à prendre. |
| `scan_max_hi_deg` | Limite de balayage au-dessus de `scan_break_ghz`. | 70° | V | Idem. |
| `p_pk_trm_w` | Puissance crête par TRM, **en sortie de PA** (plan de référence 1, là où la PAE est définie). | 2000 W | V | Convention verrouillée lors de la revue 0.3. Le chemin du PA à l'élément est la perte dérivée TX_chain. |
| `eta_illum_tx` | Rendement d'illumination en émission. | 1,0 | V | Émission uniforme, PA en saturation. |
| `allowed_roles` | Rôles permis en bande S. | ["SEARCH", "TWS"] | D | Politique d'usage des bandes du logiciel radar (étapes 3.x et 4.3). Le « toucher » S2 de confirmation sort de cette règle (noté pour l'étape 3.2). |
| `pol_rx_dual` | Réception numérique bipolarisation. | vrai | V | Un CAN par TRM. |
| `waim_match_deg` | Angle de dépointage où la couche WAIM adapte le réseau (modèle de la feuille de courant). | 45° | V | **Étape 1.2, Q2.** Perte par passage : 0,13 dB dans l'axe, 0 à 45°, 0,56 dB à 70°, 1,05 dB à 75°. C'est l'équilibre entre l'axe et les grands dépointages (40° favoriserait l'axe, 50° les grands angles). |
| `pol_loss_dual_db` | Perte de polarisation avec la réception bipolarisation. | 0,5 dB | D | **Étape 1.2, Q8.** Valeur provisoire pour la recombinaison des deux voies de polarisation, jusqu'à l'étape 1.5. Les deux voies étant captées, la pureté de l'antenne ne coûte pas de SNR ; elle est calculée en sortie (`pol_purity`). |
| `pol_loss_single_db` | Perte avec une réception monopolarisation sur une cible qui dépolarise. | 3,0 dB | V | Gardée pour étudier un récepteur monopolarisation. Utilisée seulement si `pol_rx_dual` est faux. |
| `e_buf_trm_j` | Réserve d'énergie par TRM. | 5,0 J | ? | Sans source. Affaissement de 0,24 dB sur WF-LRS. Étape 1.3. |
| `buffer_max_fraction` | Fraction maximale de la réserve qu'une impulsion peut consommer. | 0,5 | D | `solve()` refuse une impulsion qui la dépasse. Étape 1.3. |
| `bi_max_mhz` | Plafond de bande instantanée. | 200 MHz | D | Toute la bande S « pour l'instant ». Retard vrai numérique par TRM, donc pas de dépointage avec la fréquence. Sans retard vrai, le plafond serait d'environ 74 MHz à 75°. |
| `duty_max` | Plafond du facteur de forme d'un TRM. | 0 (non défini) | D | Étape 1.3. |

Retirés : `eta_illum_rx` (0,88), désormais **dérivé** de la pondération du volume (§6), et `t_sys_k`, désormais **dérivé** de la chaîne de réception.

#### `[band.S.losses_db]` : pertes fixes en S (sommées)

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `DSP` | Traitement numérique. | 2,0 dB | V | Pondération et désadaptation ≈ 1,0, chevauchement de cases en distance et Doppler ≈ 0,7, précision finie ≈ 0,3 (contexte spatial lointain, 2050). |
| `CFAR` | Perte du détecteur CFAR. | 0,5 dB | V | Environnement homogène, plancher de bruit connu, pas de fouillis, grande fenêtre de référence. |
| `calibration` | Résidu d'étalonnage. | 0,5 dB | V | Confirmé. |
| `Ku_blockage` | Masquage de l'ouverture S par le réseau Ku. | 0,2 dB | V | TRSA bi-bande. |

Une perte s'ajoute : **TX_chain**, dérivée de `[band.S.tx]`, soit 0,6 dB. Le chemin de l'élément au LNA n'est plus une perte de signal : son atténuation **et** son bruit sont dans T_sys. Avant la revue 0.3, ils étaient comptés deux fois.

#### `[band.S.rx]` : chaîne de réception S (Friis, ramenée au port d'antenne)

Choix « frontal optimisé », verrouillé avec la **valeur médiane** de chaque fourchette estimée (à revoir si l'architecture s'affine). Résultat : **T_sys = 154,3 K**. Le bas des fourchettes donnerait environ 95 K. L'ancienne valeur, 346,5 K, correspondait à un module T/R classique.

| Variable | Définition | Valeur | Statut | Fourchette (médiane retenue) |
|---|---|---|---|---|
| `t_ant_k` | Température d'antenne, ciel froid (fond cosmologique et galactique). | 10 K | V | 5 à 15 K. Dépendance à la direction à l'étape 1.5. |
| `t_phys_frontend_k` | Température physique du frontal. | 320 K | V | 290 à 350 K (dissipation des TRM, étape 4.2). |
| `circulator_loss_db` | Circulateur ou commutateur T/R. **Partagé avec l'émission** : le même composant est traversé deux fois. | 0,3 dB | V | 0,2 à 0,4 dB. |
| `limiter_loss_db` | Limiteur qui protège le LNA de la fuite de 2 kW. | 0,35 dB | V | 0,2 à 0,5 dB. |
| `pre_lna_line_db` | Filtre et lignes avant le LNA. | 0,2 dB | V | 0,1 à 0,3 dB. |
| `lna_nf_db` | Facteur de bruit du LNA. | 0,75 dB | V | 0,5 à 1,0 dB. |
| `lna_gain_db` | Gain du LNA. | 27,5 dB | V | 25 à 30 dB. |
| `post_lna_nf_db` | Facteur de bruit de la chaîne analogique après le LNA. | 8,0 dB | V | 6 à 10 dB. |
| `post_lna_gain_db` | Gain de la chaîne après le LNA. | 20 dB | V | 15 à 25 dB. |
| `adc_nf_db` | Facteur de bruit du CAN, un par TRM. | 27,5 dB | V | 25 à 30 dB. |
| `digital_noise_db` | Bruit du traitement numérique (précision finie). | 0,05 dB | V | Moins de 0,1 dB. |

#### `[band.S.tx]` : chaîne d'émission S (sortie du PA vers l'élément)

TX_chain = filtre d'harmoniques + circulateur (traversée en émission) + coupleur d'étalonnage + ligne = 0,15 + 0,3 + 0,05 + 0,1 = **0,6 dB**. Ce sont les médianes retenues lors de la revue 0.3.

| Variable | Définition | Valeur | Statut |
|---|---|---|---|
| `harmonic_filter_db` | Filtre d'harmoniques après le PA. | 0,15 dB | V |
| `cal_coupler_db` | Coupleur d'étalonnage. | 0,05 dB | V |
| `line_to_element_db` | Ligne et transition vers l'élément rayonnant. | 0,1 dB | V |

### 5.12 `[band.Ku]` : bande Ku (confirmation et poursuite)

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `usable_lo_ghz` / `usable_hi_ghz` | Bande utile. | 13,5 à 14,5 GHz | V | Confirmé lors de la revue 0.3. |
| `core_inset_divisor` | Comme en S. | 6,0 | D | Fraction de bande (étape 1.4). |
| `edge_derating_db` | Comme en S. | 1,5 dB | D | Étape 1.4. |
| `scan_max_deg` | Limite de balayage jusqu'à `scan_break_ghz`. | 75° | V | Confirmé. |
| `scan_break_ghz` | Seuil de fréquence de la limite. | 14,25 GHz | V | Cinq fois le seuil S. |
| `scan_max_hi_deg` | Limite au-dessus du seuil. | 70° | V | Confirmé. |
| `p_pk_trm_w` | Puissance crête par TRM Ku, en sortie de PA. | 75 W | V | Même convention qu'en S. |
| `eta_illum_tx` | Illumination en émission. | 1,0 | V | Uniforme comme en S. L'ancienne valeur 0,88 de V5.6.0 réduisait le gain mais pas la puissance rayonnée. Le contrôle des lobes passe par la pondération de réception de chaque volume. Portée Ku +3,25 %. |
| `allowed_roles` | Rôles permis en Ku. | ["CONFIRM", "PROMOTE", "TRACK"] | D | Politique du logiciel radar (étapes 3.x et 4.3). Les rôles TWS et PROMOTE existent mais aucune forme d'onde ne les utilise. |
| `pol_rx_dual` | Réception bipolarisation. | vrai | V | Comme en S. |
| `waim_match_deg` | Angle d'adaptation de la couche WAIM. | 45° | V | Le même qu'en S (étape 1.2). |
| `pol_loss_dual_db` | Perte de polarisation. | 0,5 dB | D | Comme en S : valeur provisoire pour la recombinaison, jusqu'à l'étape 1.5. L'ancienne valeur 0 venait des 8 dB forfaitaires de V5.6.0, qui ne couvraient que le traitement. |
| `pol_loss_single_db` | Réception monopolarisation. | 3,0 dB | V | Comme en S. |
| `e_buf_trm_j` | Réserve d'énergie par TRM Ku. | 0 (non modélisée) | D | Étape 1.3. |
| `buffer_max_fraction` | Inutilisé tant que la réserve vaut 0. | 0,5 | D | Étape 1.3. |
| `bi_max_mhz` | Bande instantanée. | 200 MHz | V | Plafond matériel. Retard vrai numérique par TRM. |
| `duty_max` | Plafond du facteur de forme. | 0,25 | D | Appliqué à l'espace de recherche des observations Ku, pas encore dans le modèle (étape 1.3). |

L'ouverture Ku est **partagée** avec la bande S : mêmes TRSA bi-bandes et même octogone. C'est une conséquence directe de la composition des TRSA, validée à l'étape 1.1.

#### `[band.Ku.losses_db]`

Mêmes postes qu'en S, avec des valeurs crédibles en Ku (revue 0.3). Ils remplacent les **8 dB forfaitaires** de V5.6.0, qui ne couvraient que le traitement.

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `DSP` | Traitement numérique. | 2,0 dB | D | Même traitement qu'en S. |
| `CFAR` | Détecteur. | 0,5 dB | D | Même environnement qu'en S. |
| `calibration` | Résidu d'étalonnage. | 0,8 dB | D | Plus élevé qu'en S : à tolérances mécaniques égales, l'erreur de phase est 5 fois plus grande en Ku, et il y a 25 fois plus d'éléments. |
| `S_blockage` | Diffusion par les éléments S intercalés dans la TRSA. | 0,3 dB | D | Symétrique de `Ku_blockage` en S. |

TX_chain Ku dérivée = 0,2 + 0,45 + 0,1 + 0,2 = **0,95 dB**.

#### `[band.Ku.rx]` : médianes (statut D, estimations à confirmer)

**T_sys = 238,7 K**, contre 400 K saisis en bloc auparavant. Le bas des fourchettes donnerait environ 138 K, le haut environ 387 K.

| Variable | Valeur | Fourchette |
|---|---|---|
| `t_ant_k` | 5 K | 3 à 8 K (fond cosmologique, galactique négligeable) |
| `t_phys_frontend_k` | 320 K | 290 à 350 K |
| `circulator_loss_db` | 0,45 dB | 0,3 à 0,6 dB |
| `limiter_loss_db` | 0,45 dB | 0,3 à 0,6 dB (émission de 75 W) |
| `pre_lna_line_db` | 0,3 dB | 0,2 à 0,4 dB |
| `lna_nf_db` | 1,15 dB | 0,8 à 1,5 dB |
| `lna_gain_db` | 25 dB | 20 à 30 dB |
| `post_lna_nf_db` | 8,0 dB | 6 à 10 dB (conversion vers le bas) |
| `post_lna_gain_db` | 20 dB | 15 à 25 dB |
| `adc_nf_db` | 27,5 dB | 25 à 30 dB |
| `digital_noise_db` | 0,05 dB | moins de 0,1 dB |

#### `[band.Ku.tx]` : médianes (V)

| Variable | Valeur |
|---|---|
| `harmonic_filter_db` | 0,2 dB |
| `cal_coupler_db` | 0,1 dB |
| `line_to_element_db` | 0,2 dB |

### 5.13 `[targets.*]` : cibles de référence

Elles ne servent qu'au rapport en ligne de commande et aux tests. Une mission utilise la SER, le cas de Swerling et la vitesse propres à chaque secteur. Une bibliothèque de menaces est prévue à l'étape 5.1.

| Cible | SER | Swerling | Vitesse de rapprochement | Statuts et justification |
|---|---|---|---|---|
| ST (cible standard) | 1000 m² | 1 | 5000 m/s | V, V, V |
| KKV | 0,135 m² | 1 | 25 000 m/s | SER V (milieu de 0,12 à 0,15 m²) ; Swerling D (supposé, modèles de cible à l'étape 2.4) ; vitesse V (15 km/s de croisement maximal en orbite basse + 7 km/s de vitesse terminale du KKV + marge) |
| SHIP | 3000 m² | 1 | 5000 m/s | SER V (SER latérale) ; Swerling D et vitesse D (étape 2.4) |

### 5.14 `[confirmation]` et `[tracking]`

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `confirmation.ku_freq_ghz` | Fréquence Ku de confirmation et de poursuite. | 14,0 GHz | D | **Provisoire.** Elle est dans la bande centrale, sous le seuil de 14,25 GHz. « L'optimiseur **doit** choisir une fréquence » (étape 3.2), en temps réel, d'après le spectre non brouillé. |
| `confirmation.bias_mrad` | Biais résiduel d'alignement S/Ku par axe, exprimé en **angle absolu**. | 1,1 mrad | V | Il valait 1/20 du lobe S (1,13 mrad à 2,8 GHz) et aurait grandi avec la largeur de lobe. Il est désormais constant en (u, v). Étalonnage S/Ku à l'étape 3.2. |
| `confirmation.p_min` | Probabilité minimale de confirmer une vraie cible. | 0,98 | V | Exigence opérationnelle : 0,9 × 0,98 par passage de bout en bout. Elle deviendra un réglage de mission à l'étape 5.1 ; un réglage par volume reste une option à creuser. |
| `confirmation.fa_r4_mean` | Moyenne de (R/R_det)⁴ pour une fausse alarme. | 0,2 | V | Exactement 1/5 pour une fausse alarme répartie uniformément en distance sur [0, R_det]. |
| `tracking.angle_zone_sigmas` | Ouverture de la zone angulaire, en sigmas. | 4 | V | Convention : largeur totale à ±2σ. |

Supprimée lors de la revue 0.3 : `latency_max_frac`, la règle de latence. Elle ne limitait jamais rien (2nv/c) et ignorait les vraies causes. La latence sera **reconstruite de zéro à l'étape 4.3** : attente dans la file de l'ordonnanceur, traitement et allers-retours, comparés à une exigence de délai par secteur.

### 5.15 `[ship]` : vaisseau

| Variable | Définition | Valeur | Statut |
|---|---|---|---|
| `length_m` | Longueur de la coque. | 136,4 m | V |
| `hull_diameter_m` | Diamètre maximal de la coque. | 16,8 m | V |
| `width_m` | Envergure maximale, ailes de radiateurs comprises. | 76,5 m | V |

Ces valeurs ne servent aujourd'hui qu'à la vue 3D. Le masquage des champs de vue par le vaisseau est prévu à l'étape 2.5. Seul le plancher arrière (ε ≈ −55°) est modélisé.

---

## 6. Grandeurs dérivées (calculées, sans entrée dans le fichier)

| Grandeur | Formule ou origine | Valeur actuelle |
|---|---|---|
| Côté d'une TRSA | 25 × 10,4 mm | 260 mm |
| Pas S | 260 / 5 | 52 mm |
| Nombre de TRSA | somme des rangées | 196 |
| Segments de jeu | dans les rangées + entre rangées | 360 |
| TRM par face | 196 × 25 (S), 196 × 625 (Ku) | 4 900 en S, 122 500 en Ku |
| Puissance crête par face | n × p_pk | S 9,80 MW, Ku 9,19 MW |
| Surface émissive | 196 × 0,26² | 13,2496 m² |
| Surface totale (avec les jeux) | + 360 × 0,0005 × 0,26 | 13,2964 m² (remplissage 99,65 %) ; 13,4368 m² avec 2 mm |
| Enveloppe D_env | 16 TRSA + 15 jeux | 4,1675 m (4,190 m avec 2 mm) |
| **Lobe d'émission (uniforme)** | diagramme calculé, plus petite coupe | **1,035 λ/D_env** (axe x 1,041), soit 1,525° à 2,8 GHz et 0,305° à 14 GHz ; 1ᵉʳ lobe secondaire −15,6 dB |
| **Pondération de réception à −25 dB** | Taylor n̄ = 5, conception 29,5 dB | η = 0,873, HPBW 1,172 λ/D (1,72° à 2,8 GHz), niveau atteint −25,0 dB |
| **Pondération de réception à −30 dB** | conception 35,7 dB | η = 0,773, HPBW 1,255 λ/D |
| **Pondération de réception à −35 dB** | conception 41,7 dB | η = 0,693, HPBW 1,325 λ/D |
| x_rms (étendue RMS de l'ouverture) | moyenne de x² sur les éléments | 1,039 m = 0,248 D |
| Précision angulaire × √SNR | 1,13 × borne / HPBW_TX | 0,495 largeur de lobe d'émission, en S comme en Ku |
| Faisceaux de réception par pointage | (1,04·HPBW_TX / (0,5·HPBW_RX))² | 3,38 (−25), 2,95 (−30), 2,64 (−35) |
| Perte de grille d'émission | 1,672 × 1,04² | 1,81 dB |
| Perte de grille de réception | 1,672 × 0,5² | 0,42 dB |
| Gain en S à 2,8 GHz, dans l'axe | 4πAη/λ² | émission 41,62 dBi, réception 41,03 dBi (−25 dB) |
| Gain d'émission en Ku à 14 GHz | idem | 55,60 dBi |
| T_sys | cascade de Friis | S 154,3 K, Ku 238,7 K |
| TX_chain | somme des composants | S 0,6 dB, Ku 0,95 dB |
| Pertes fixes (hors polarisation) | somme | S 3,80 dB, Ku 4,55 dB |
| Pas moyen des éléments | pas des TRSA / TRM par côté | S 52,1 mm, Ku 10,42 mm (52,5 et 10,9 mm de part et d'autre d'un jeu) |
| Désadaptation aller-retour (2,8 GHz) | 2 × perte par passage | 0,26 dB dans l'axe, 0 à 45°, 1,12 dB à 70°, 2,11 dB à 75° ; 2,24 dB à 2,85 GHz et 75° |
| Lobe lointain le plus fort (lobes de jeu compris) | grille 2D sur l'espace visible | Ku −25,3 dB, S −30,7 dB en uniforme (−16,6 et −26,8 dB avec 2 mm) ; la pondération ne réduit pas les lobes de jeu |
| Dépointage jusqu'où l'option de réception est tenue | `rx_sll_held_up_to_deg` | S : les trois options sur tout le secteur. Ku : −25 dB partout ; −30 / −35 dB jusqu'à 75° / 62° à 14 GHz, 68,5° / 58,5° à 14,25 GHz, 64° / 55° à 14,5 GHz |
| Pureté de polarisation (Ludwig 3, plan à 45°) | (1 + cos θ) / (1 − cos θ) | 15,3 dB à 45°, 6,2 dB à 70° ; infinie dans les plans principaux |
| CHAIN_EFF | psu × distribution × (1 − overhead) | 0,917 |
| Rayon du secteur | plus large limite de balayage | 75° |
| Surface du secteur en (u, v) | disque sin 75° découpé par les plans ±45°, inclinaison 25° | 2,0006 |
| Pointages par face à 2,8 GHz | surface / cellule | 3 016 (4 167 avant l'étape 1.1, 3 050 avec 2 mm de jeu) |
| Plus longue impulsion S | 0,5 × 5 J × 0,75 / 2000 W | 937,5 µs |
| Durée cohérente maximale (provisoire) | t₀ (−ln η_min)^(1/p) | 149 ms |

Pour l'ensemble de l'octogone, le facteur en λ/D ne dépend pas de la fréquence au voisinage de l'axe. Les bandes S et Ku échantillonnent la même ouverture, et leurs résultats s'accordent à 0,1 % près. Un seul calcul par bande suffit donc (étape 1.1, décision Q7).

---

## 7. Modèles et formules

### 7.1 Équation radar (forme énergétique)

R⁴ = P_faisceau · G_TX · G_RX · λ² · σ · τ · N / ((4π)³ · k · T_sys · L · SNR_req)

- P_faisceau = P_crête de la face / B_TX.
- G = 4π · A_émissive · η · η_ohmic · cos θ / λ². La désadaptation active est comptée à part, dans L (§7.3 bis).
- **η_RX** est celui de la pondération du volume (`rx_sll_db`).
- SNR_req est le SNR sur l'**énergie totale** intégrée, d'après l'ajustement de Shnidman, pour les cas Swerling 0 à 4 et une intégration cohérente ou non cohérente. La formule d'Albersheim de V5.4.0 était mal implémentée et comptait l'intégration deux fois.
- L regroupe : pertes fixes, TX_chain, bord de bande, polarisation, affaissement de la réserve (−10 log(1 − x/2)), multifaisceau, **désadaptation active à l'émission et à la réception** (étape 1.2), grille d'émission et grille de réception (veille seulement), cohérence (intégration cohérente), migration.

### 7.2 Diagramme (étape 1.1, `core/antenna.py`)

- Éléments isotropes placés selon la composition des TRSA. Facteur de réseau : AF(u, v) = Σ w · cos(kxu) · cos(kyv) sur un quadrant, la symétrie étant vérifiée.
- Coupes de 0 à 45° (symétrie de l'octogone) par pas de 1,5°, sur un rayon de 0 à 8 λ/D par pas de 0,01.
- Largeur de lobe : 2 × rayon à −3 dB, en retenant la plus petite des coupes (pire cas).
- Niveau de lobes : maximum au-delà du premier minimum de chaque coupe.
- Pondération de réception : loi de Taylor circulaire de Hansen à n̄ = 5. Le niveau de conception est cherché par fausse position (Illinois) pour que le niveau atteint soit dans [cible − 0,05 dB, cible]. Les poids négatifs sont ramenés à 0.
- η = (Σw)² / (N Σw²).

### 7.3 bis Désadaptation active, lobes de réseau et lobes lointains (étape 1.2)

- Un modèle unique pour S et Ku, appliqué à l'émission et à la réception, distinct de la dégradation de bord de bande (étape 1.4).
- Feuille de courant adaptée à θm par une couche WAIM : |Γ_WAIM| = |cos θ − cos θm| / (cos θ + cos θm), identique dans les plans E et H.
- Terme de lobe de réseau : |Γ_LR| = exp(−g/g₀), avec g = λ/d_moyen − sin θ − 1, calculé dans le pire azimut (balayage le long d'un axe de la maille).
- Combinaison en puissance : 1 − |Γ|² = (1 − |Γ_WAIM|²)(1 − |Γ_LR|²). Perte par passage = −10 log(1 − |Γ|²), comptée deux fois dans `solve()`.
- Les limites de balayage restent verrouillées. L'optimiseur alerte si une tranche utilisée dépasse 3 dB par passage, ce qui n'arrive pas à l'intérieur des limites actuelles.
- Lobes lointains : grille 2D du facteur de réseau sur tout l'espace visible de la bande (rayon (1 + sin θ_lim) D/λ au haut de la bande), voisinages des lobes de réseau exclus. On stocke un profil cumulatif : niveau maximal des lobes à moins de r de l'axe. Un lobe à r est visible dès que r·λ/D ≤ 1 + sin θ (pire azimut).
- Les jeux rompent le pas des éléments d'une TRSA à l'autre (pas des TRSA = côté + jeu), d'où des « lobes de jeu » en k·λ/(côté + jeu) pondérés par le facteur de sous-réseau d'une TRSA. Ceux voisins du lobe de réseau sont les plus forts et aucune pondération ne les réduit : −16,6 dB en Ku avec 2 mm de jeu, d'où le passage à 0,5 mm. L'optimiseur et le bilan de poursuite signalent, par secteur, jusqu'à quel dépointage l'option de réception est tenue.
- Pureté de polarisation, en sortie seulement : élément courant selon e_v, E_θ = cos θ sin φ, E_φ = cos φ ; polarisation croisée de Ludwig 3 et inclinaison de la polarisation rayonnée.

### 7.3 Grille de veille

- Les pointages sont comptés dans l'espace (u, v) : un réseau plan balaye avec un lobe de largeur constante en (u, v), qui s'élargit en 1/cos θ en angle.
- Cellule hexagonale : (√3/2) · (1,04 · HPBW_TX · λ/D)².
- Perte de grille moyenne avec un lobe gaussien : 1,672 · s² dB (pire point : 12,04 · s²/3).
- Dans chaque pointage, les faisceaux de réception numériques forment un réseau au pas de 0,5 HPBW_RX, ce qui donne la perte résiduelle et le nombre de faisceaux.

### 7.4 Précision de poursuite

- Angle : SNR = ½ · (marge · λ / (2π · x_rms · σ_θ · cos θ))², sur l'axe le plus défavorable, c'est-à-dire le plan de dépointage.
- Doppler : SNR = ½ · (k_f / (T_obs · σ_f))², avec σ_f = 2σ_v/λ et T_obs = (N − 1)·PRI + τ.
- Distance : SNR = ½ · (k_r · c / (2 · B · σ_r))².
- Le SNR retenu est le maximum de la détection et des trois termes ; le terme dominant est indiqué.

### 7.5 Optimiseur de veille (règles [D])

- Une forme d'onde par tranche de 5° de dépointage, avec au plus 4 formes d'onde distinctes par secteur ; les tranches voisines sont fusionnées par programmation dynamique.
- La puissance est réduite tranche par tranche, juste assez pour atteindre la portée (« pas d'énergie en trop »).
- Objectif : minimiser max(fraction d'énergie, fraction de temps d'émission) sur les faces, par une somme pondérée dont on balaye le poids (21 valeurs).
- Classement : d'abord la part du secteur non balayée à cause de la fréquence, puis les pointages qui n'atteignent pas la portée (tolérance 1 %), puis l'objectif.
- Distance non ambiguë imposée par la PRI, sauf si le secteur l'autorise avec `allow_ambiguous`.
- Migration évaluée à la vitesse maximale du secteur. Les secteurs sont indépendants ; un recouvrement est signalé, pas dédoublonné.
- La période de revisite (RR) est allongée en proportion quand une face est sursouscrite.

### 7.6 Confirmation et poursuite (`core/tracking.py`)

- Chaque détection S est confirmée, qu'il s'agisse d'une vraie cible ou d'une fausse alarme. Trois modes :
  - **KU1+7** : une observation Ku, puis une grappe de 7 faisceaux en cas d'échec ;
  - **KU7** : la grappe de 7 faisceaux d'emblée ;
  - **S2** : un toucher S pointé, puis KU1+7.
- Chaque mode reçoit la plus petite marge (par pas de 0,5 dB) qui atteint p_min.
- Le mode est choisi automatiquement : celui qui minimise la charge de la face la plus chargée. L'opérateur peut aussi l'imposer.
- Le toucher S2 récupère les pertes de grille d'émission **et** de réception, puisqu'il est pointé et que les faisceaux de réception sont formés sur la cible.
- Poursuite active par tranche de distance : nombre de pistes, période, précisions. Une observation Ku par mise à jour, à la limite haute de la tranche.
- Les réseaux S et Ku partagent l'énergie de la face, mais leurs temps d'émission sont comptés séparément.

### 7.7 Redondance

- Le secteur A couvre le besoin de B si : R_AB − v_B · rr_A ≥ R_B − v_B · rr_B (à 1 % près), avec en plus une PRI non ambiguë et une résolution suffisante.
- R_AB est calculé avec la forme d'onde de A, la cible de B et **la pondération de réception de B**, puisque la réception numérique peut appliquer les deux sur la même observation.

---

## 8. Paramètres de l'outil (`[tool.*]`)

Ces réglages ne décrivent pas le radar. Ils ont été validés tels quels lors de la revue 0.3, avec des renvois vers les étapes à venir.

| Section | Variable | Valeur | Rôle et renvoi |
|---|---|---|---|
| `tool.cli` | `refresh_s`, `frequency_ghz`, `scan_angle_deg`, `target` | 120 s, 2,8 GHz, 0°, "ST" | Valeurs par défaut de `python core/radar_model.py`. |
| `tool.optimizer` | `freq_candidates_ghz` | [2,7, 2,8, 2,9] | À remplacer à l'étape 3.1 par une grille fine au pas de **5 ou 10 MHz**, restreinte au spectre utilisable. Le but est d'éviter les brouilleurs étroits de forte puissance tout en gardant plus de cases fréquence qu'un masquage par multiples de 25 MHz. |
| | `tau_s` | 20 à 800 µs, plus la limite de la réserve (937,5 µs) | La plus longue impulsion est **calculée** à partir de la réserve d'énergie. |
| | `n_pulses`, `b_tx`, `sig_bw_mhz` | {1 … 32}, {1, 2, 4, 8}, {10 … 75} MHz | Bandes au-delà de 75 MHz possibles (bi_max 200) : à traiter à l'étape 3.1, avec l'étape 2.1. |
| | `pri_min_s` | 1 ms | PRI des candidats mono-impulsion et ambigus. |
| | `theta_bin_deg`, `bin_deg`, `max_wf_per_sector`, `short_tol`, `n_alphas`, `short_penalty` | 1°, 5, 4, 0,01, 21, 1e6 | Règles de l'optimiseur, revues à l'étape 3.1. |
| | `track_margin_warn` | 0,10 | Alerte de marge pour la poursuite (étape 5.3). |
| | `sphere_res_deg`, `solve_cache_size`, `mask_cache_size`, `parallel_min_sectors`, `max_workers` | 0,2°, 262 144, 64, 6, 0 (automatique) | Performances. |
| `tool.confirmation` | `ku_tau_s`, `ku_n_pulses`, `ku_pri_s`, `ku_sig_bw_mhz` | 10 à 400 µs ; 1 à 64 ; 0,5, 1 et 2 ms ; 50 et 200 MHz | Espace des observations Ku, à élargir à l'étape 3.2. |
| | `margin_db_max`, `margin_db_step` | 12 dB, 0,5 dB | Marges essayées. Le pas de 0,5 dB, qui crée des sauts, sera revu à l'étape 3.2. |
| | `n_snr_quantiles`, `gh_nodes` | 32, 12 | Statistiques. |
| `tool.redundancy` | `cover_full`, `cover_min`, `range_tol`, `min_piece_deg`, `plan_cache_max` | 0,995, 0,01, 0,01, 1°, 256 | Règles de redondance. |
| `tool.cost_table` | `freq_ghz`, `range_km`, `rcs_m2`, `v_mps`, `theta_deg` | 2,8 GHz et grilles | Table calculée à 2,8 GHz seulement (optimiste à 2,7 et 2,9 GHz) ; à recaler à l'étape 5.5. |
| `tool.pattern` | `r_max_lam_d`, `r_step_lam_d`, `phi_step_deg`, `fit_tol_db`, `far_step_lam_d` | 8, 0,01, 1,5°, 0,05 dB, 0,125 | Étapes 1.1 et 1.2 : précision du calcul du diagramme et pas de la grille des lobes lointains. |
| `tool.alerts` | `mismatch_db_per_pass` | 3,0 dB | **Étape 1.2, Q5** : seuil d'alerte de la désadaptation par passage. |
| `tool.display` | `grid_res_deg`, `viz_freq_ghz` | 0,5°, 2,8 GHz | Trame 3D et estimation rapide. |
| `tool.mission_defaults` | voir §9 | | Valeurs d'une nouvelle mission. |

---

## 9. Modèle de mission (entrées opérateur)

### 9.1 Paramètres globaux

| Clé | Défaut | Contrôle |
|---|---|---|
| `eps_min_deg` / `eps_max_deg` | −90° / +90° (pas de limite) | Fenêtre d'élévation de la mission, envoyée à l'ordonnanceur. Le plancher physique est d'environ −55°. |
| `face_elec_mw` | 2,0 MW | Doit être strictement positive et ne pas dépasser 8 MW (validation provisoire). |

### 9.2 Secteur (volume de veille)

| Clé | Défaut | Rôle |
|---|---|---|
| `id`, `shape` | S01…, "box" | Une boîte (φ, ε) ou une calotte (centre, rayon). |
| `phi_min_deg`, `phi_max_deg`, `eps_min_deg`, `eps_max_deg` | 0°, 90°, 0°, 60° | Boîte. φ_min > φ_max est accepté et signifie un passage par 0°. |
| `phi0_deg`, `eps0_deg`, `radius_deg` | 0°, 20°, 15° | Calotte. |
| `rcs_m2`, `swerling` | 1000 m², 1 | Menace. |
| `range_km`, `rr_target_s` | 2000 km, 30 s | Portée exigée et période de revisite visée. |
| `v_close_min_mps`, `v_close_max_mps` | 1000 et 5000 m/s | La vitesse maximale dimensionne la migration, le Doppler et le temps avant impact. |
| `res_max_m` | 2 m (soit 75 MHz) | Résolution en distance demandée. |
| `allow_ambiguous` | faux | Autorise une distance ambiguë. |
| **`rx_sll_db`** | **−25 dB** | **Étape 1.1.** Niveau de lobes de la pondération de réception du volume, au choix parmi −25, −30 et −35 dB. Il s'applique à la veille S et à la confirmation et poursuite Ku du volume. Un ancien fichier sans ce champ prend −25 dB. Une liste déroulante dans l'interface indique, pour chaque option, η, la perte de gain et la largeur de lobe. |
| `confirm_mode` | "auto" | auto, KU1+7, KU7 ou S2. |
| `new_targets_per_rr` | 1 | Nouvelles vraies cibles par période de revisite. |
| `tracking` | [] | Tranches de poursuite active : r_min et r_max, nombre de cibles (5), période (2 s), zone angulaire (0,2°), σ_R (10 m), σ_V (5 m/s). |
| `max_wf_per_sector`, `short_tol`, `freq_ghz_candidates` | réglages de l'optimiseur | Surcharges possibles par secteur. |

### 9.3 Formats

- Fichiers JSON (complets) ou CSV (tableau des secteurs, avec les tranches de poursuite en JSON dans une seule colonne). Les deux restent compatibles avec l'entrée de l'optimiseur.
- La validation (« Validate mission prep ») vérifie : identifiants uniques, bornes, φ et ε, v_min ≤ v_max, valeurs strictement positives, tranches de poursuite sans chevauchement et dans la portée, `rx_sll_db` parmi les options.

---

## 10. Feuille de route, renvois et points ouverts

La feuille de route complète, avec le contenu détaillé de chaque étape, se trouve dans `Feuille_de_route.md`.

### 10.1 Feuille de route (accordée le 20 octobre 2026)

| Phase | Étapes |
|---|---|
| 0, fondations | 0.1 fichier unique ✔ ; 0.2 banc de tests ✔ ; 0.3 revue des valeurs ✔ |
| 1, antenne et RF | 1.1 géométrie et diagramme réel ✔ ; **1.2 diagramme : lobes de réseau, désadaptation active \|Γ(θ, f)\|, polarisation ✔** ; 1.3 TRM et chaîne d'émission (PAE variable, réserve, facteur de forme) ; 1.4 bande (table de bord, bande instantanée) ; 1.5 réception (T_sys selon la direction, CAN, isolation, bruit de phase de l'oscillateur, deux voies de polarisation) |
| 2, signal et détection | 2.1 formes d'onde paramétriques ; 2.2 traitement (bancs de vitesse, CFAR, modèle de cohérence) ; 2.3 critère de détection ; 2.4 modèles de cibles ; 2.5 environnement (masquage par le vaisseau, Faraday, brouillage) |
| 3, fonctions radar | 3.1 veille S (multifaisceau, grille de fréquences fine) ; 3.2 confirmation Ku (fréquence Ku choisie, connaissance du spectre) ; 3.3 poursuite (Kalman ou IMM) ; 3.4 fonctions annexes |
| 4, ressources | 4.1 puissance (bus de 8 MW, répartition entre faces) ; 4.2 thermique ; 4.3 ordonnanceur et latence ; 4.4 calcul et données |
| 5, mission et validation | 5.1 modèle de mission enrichi et bibliothèque de menaces ; 5.2 optimiseur global ; 5.3 moteur de validation ; 5.4 simulation Monte-Carlo ; 5.5 interface opérateur ; 5.6 vérification, validation et documentation |

Jalons : A = phases 0 à 2 ; B = phase 3 ; C = étapes 4.1 à 4.3 ; D = phase 5.

### 10.2 Renvois enregistrés (dispositions provisoires à reprendre)

| Sujet | État actuel | Étape |
|---|---|---|
| Polarisation | Linéaire selon e_v (D) ; 0,5 dB provisoire pour la recombinaison des voies. Commutation e_u / e_v à rappeler en 3.3. | 1.5, 2.4, 2.5, 3.2-3.3 |
| Élément rayonnant | Éléments isotropes dans le calcul du facteur de réseau ; seule la projection cos θ et la désadaptation interviennent. | à placer |
| PAE | Constante à 0,75, commune à S et Ku. | 1.3 |
| Réserve d'énergie et facteur de forme | 5 J (statut ?), 0,5, facteur de forme non défini. | 1.3 |
| Densité de puissance | Environ 104 kW/m² de RF moyenne. | 1.3, 4.2 |
| Table de bord de bande | Fraction de bande copiée de l'ancienne bande Ku. | 1.4 |
| Plafond de cohérence | Provisoire, 149 ms ; t₀ et p sans source. | 2.2 |
| Bruit de phase de l'oscillateur sur l'aller-retour | Non modélisé (53 ms à 8 000 km). | 1.5, 2.2 |
| Multifaisceau | Loi unique, statut ?. Modes à évaluer par l'optimiseur. | 3.1, 5.2 |
| Fréquence Ku | Fixée à 14,0 GHz, provisoire. | 3.2 |
| Masquage par le vaisseau | Seul le plancher arrière est modélisé. | 2.5 |
| Latence | Règle supprimée, à reconstruire. | 4.3 |
| Répartition de la puissance entre faces | Une seule valeur par mission. | 4.1, 5.2 |
| Consommation hors émission | `overhead_frac` à 3 %. | 4.1, 4.4 |
| Faisceaux de réception par pointage | Calculés (environ 3,4) mais pas encore comptés dans le bilan de calcul. | 4.4 |
| p_min par volume | Option à creuser. | 5.1 |

### 10.3 Points ouverts

- **(a)** L'optimiseur départage des égalités exactes par le bruit d'arrondi : il faut un critère secondaire (étapes 3.1 et 5.2).
- **(b)** Le nombre de faces, 4, est écrit en dur dans le code.
- **(c)** Le toucher S2 sort de la règle des rôles S ; les rôles TWS et PROMOTE ne sont pas utilisés (étape 3.2).
- **(d)** `power_scale` est arrondi à 3 décimales dans la sortie de l'optimiseur (étape 5.5).
- **(e)** La table de l'estimation rapide n'est calculée qu'à 2,8 GHz (étape 5.5).
- **(f)** **Lobes de jeu des TRSA (étape 1.2), traité en grande partie.** Le jeu est passé de 2 mm à 0,5 mm. Restent en Ku, au haut de la bande : −30 dB tenu jusqu'à 68,5° (14,25 GHz) et 64° (14,5 GHz), −35 dB jusqu'à 62° (14 GHz), 58,5° et 55° ; signalé par secteur. Si la mécanique impose un jeu plus grand, la variante est une maille d'éléments uniforme à travers les jeux (TRM Ku de 10,32 mm ou éléments décalés), qui demande de séparer le pas des éléments de la taille des TRM dans `[aperture]`. Les effets de bord de tuile (couplage mutuel au bord d'un jeu) ne sont pas modélisés.
- **(g)** `rx_taper_edge` = "inscribed" (statut D) à confirmer.

---

## 11. Historique des décisions

### Étape 0.1 : fichier de paramètres unique (commit 90a5fc4)
- Toutes les constantes ont été déplacées dans `radar_system.toml`, sans changer de valeur. Les noms des constantes de module sont conservés (modèle V6.0.13).
- La fenêtre ε de la mission est passée en argument, au lieu de réécrire des constantes du module.
- Choix : TOML, un seul fichier monolithique, un bloc `[tool.*]` séparé, des entrées en valeur, unité, statut, source et date.

### Étape 0.2 : banc de tests (commit c776688)
- Deux familles de tests, tolérance relative de 1e-9, rapport d'impact, `pytest` dans `requirements-dev.txt`.
- Missions restaurées depuis l'historique git, en tant que **missions de test**.

### Étape 0.3 : revue de toutes les valeurs (commits f205a23 et 7f6bd35)
Revue en 10 lots, plus les réglages de l'outil. Principales décisions :
- Ouverture dérivée de la composition des TRSA ; `eta_ohmic` porté à 1,0 ; `hpbw_factor` retiré du fichier.
- Dépointage limité à la projection cos θ (exposant et loi de perte supprimés) ; rayon du secteur calculé.
- Puissance verrouillée ; puissance de 8 MW répartissable entre faces (étapes 4.1 et 5.2).
- Migration verrouillée ; multifaisceau au statut « ? ».
- Cohérence : t₀ et p sans source ; gigue d'horloge supprimée ; plafond provisoire de 149 ms.
- Estimation : k_f et k_r à 0,6 (borne × 1,13).
- Bord de bande en fraction de bande.
- T_sys en S et en Ku dérivés de la chaîne de réception, avec les médianes (154,3 K et 238,7 K).
- Pertes Ku détaillées par poste, sur le modèle des postes S.
- Puissance TRM en sortie de PA, TX_chain détaillée par composant.
- Retard vrai numérique en S et en Ku ; bi_max S fixé à 200 MHz.
- `eta_illum_tx` Ku porté à 1,0 ; rôles au statut D.
- Biais S/Ku en angle absolu de 1,1 mrad ; règle de latence supprimée ; vaisseau validé.

### Étape 1.1 : diagramme réel de l'antenne (20 octobre 2026)
| Question | Décision de l'utilisateur |
|---|---|
| Q1, référence de la précision angulaire | **(a)** Borne de Cramér-Rao calculée sur la géométrie, multipliée par une marge ; marge de **1,13**. |
| Q2, lobes en réception | Options **−25, −30 et −35 dB**, réglées **par volume** dans la préparation de mission, **−25 dB par défaut**, un seul réglage par volume pour S et Ku. |
| Q2 bis | Loi de Taylor circulaire n̄ = 5 calée numériquement ; `eta_illum_rx` déduit. |
| Q3, grille de veille | **Option C** : grille d'émission sur le lobe réel et grappe de faisceaux de réception au pas de **0,5 HPBW_RX**. |
| Q4, calcul | numpy seulement, calcul au chargement et cache lié aux paramètres. |
| Q5, dépointage | Lobe circulaire en (u, v), élargi en 1/cos θ dans le plan de dépointage ; axe le plus défavorable retenu pour la poursuite. |
| Q6, ouverture Ku | Partagée, statut V. |
| Q7, fréquence de calcul | Une fois par bande ; le facteur en λ/D ne dépend pas de la fréquence. |

Constat de l'exploration : le η = 0,88 saisi auparavant ne donnait que **−24 dB** de lobes sur l'octogone ; une pondération parabolique donne η = 0,83 et non 0,88.

Effet sur les missions de test (valeurs de test seulement) :
- pointages −26,8 % ;
- portée de WF-LRS −2,58 % : 7 099 → 6 916 km sur ST à 2,8 GHz ;
- énergie de veille S environ −18 % ;
- temps Ku de −8 à −40 %, car il y a moins de fausses alarmes à confirmer.

Le détail est dans `tests/snapshots/JOURNAL.md`.

### Étape 1.2 : désadaptation active, lobes de réseau, lobes lointains, polarisation (20 octobre 2026)
| Question | Décision de l'utilisateur |
|---|---|
| Q1, modèle de désadaptation | **(a)** Feuille de courant avec WAIM adaptée à θm, plus un terme de lobe de réseau. |
| Q2, angle d'adaptation | **45° en S et en Ku.** |
| Q3, largeur du terme de lobe de réseau | **g₀ = 0,02.** |
| Q4, direction de la marge | **(a)** Pire azimut. |
| Q5, limites de balayage | **(a)** Verrouillées ; marge et perte affichées ; alerte au-delà de **3 dB par passage**. |
| Q6, application | À l'émission **et** à la réception, distincte de la dégradation de bord de bande. |
| Q7, polarisation d'émission | **Linéaire selon e_v** (statut D). Rappeler à l'étape 3.3 que la commutation e_u / e_v est une option. |
| Q8, perte avec réception bipolarisation | **(a)** 0,5 dB gardés comme valeur provisoire pour la recombinaison jusqu'à l'étape 1.5. |
| Q9, pureté de polarisation | Modélisée, en sortie seulement. |
| Q10, lobes secondaires | Calculés sur tout l'espace visible, avec alerte quand une option n'est pas tenue. |

Constats de l'étape : avec 2 mm de jeu, le pas moyen réel (52,4 mm en S) réduisait la marge de lobe de réseau à 0,35 % à 2,9 GHz et 75°, et les lobes de jeu des TRSA (−16,6 dB en Ku) empêchaient de tenir les options de réception aux grands dépointages.

**Décision consécutive :** jeu ramené à **0,5 mm**, le compromis jugé le plus acceptable (S : toutes les options sur tout le secteur ; Ku : −25 dB partout). Toutes les grandeurs dérivées ont suivi : surface totale 13,2964 m², enveloppe 4,1675 m, pointages −1,1 %, portées inchangées.

Effet sur les missions de test (valeurs de test seulement) :
- portée de WF-LRS −1,48 % dans l'axe et −6,25 % à 70° (6 916 → 6 813 km sur ST dans l'axe) ;
- énergie par face de +0,2 à +9,5 %, temps S de 0 à +19 %, temps Ku de −6 à +61 % ;
- puis, avec le jeu de 0,5 mm : pointages −1,1 %, énergie par face de 0 à −4,9 %, portées inchangées.
