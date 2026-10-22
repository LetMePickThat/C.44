# Outil de préparation de mission radar : choix architecturaux

Ce document recense tous les choix d'architecture faits jusqu'ici : organisation du logiciel, modèle du radar, et pour chaque variable sa définition, sa valeur, son statut et la raison de cette valeur. Il décrit l'état du dépôt après l'étape 2.5 de la feuille de route (modèle V6.10.0, 22 octobre 2026).

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
| Regrouper par sujet | Tout ce qui touche un même sujet est traité au même endroit (même étape, même bloc de questions) ; une valeur qui en dépend d'une autre se fixe après celle-ci (règle fixée à l'étape 1.3). |
| Valeurs technologiques | Hypothèses **crédibles mais optimistes pour 2050-2060** (règle fixée à l'étape 1.3). Quand une plage est proposée, c'est **son bout optimiste** qui est retenu ; pour un critère ou une marge, le bout optimiste est le moins strict, et on le dit (règle fixée à l'étape 1.5). |
| Simulateur de scénario | Le simulateur radar sera une partie d'un **simulateur global de scénario**, qui lui fournira la position de tous les objets (cibles, planètes, astres) **dans le repère du vaisseau**. Le modèle radar tient compte des effets d'un corps devant les faces (bruit thermique), mais ne calcule jamais où sont les objets ; le changement de repère est l'affaire du simulateur global. Le prototype de ce simulateur (`RADARv5/SIMULATOR`) n'est qu'une preuve de concept : rien n'en est repris (règle fixée à l'étape 1.5). |
| Valeurs ajustables | Les valeurs peuvent encore changer (par exemple une bande jouable plus large si les TRM progressent) : tout ce qui en dépend est **calculé** à partir du fichier, et le chargeur refuse une combinaison incohérente avec un message clair (règle fixée à l'étape 1.4). |
| Sources | Le dossier `RADARv5` est **obsolète** : il ne sert pas de source de valeurs ni de modèles (règle fixée à l'étape 1.4). |
| Commits | Un commit par étape, et seulement sur demande (« c, p » = commit et push). |
| Neutralité du dépôt | Le dépôt GitHub (public) ne mentionne **aucun assistant d'IA ni son éditeur** : ni dans les fichiers, les noms de fichiers ou de dossiers, ni dans les messages de commit (pas de ligne « Co-Authored-By » ni de signature « généré par »). L'historique a été réécrit le 22 octobre 2026 pour retirer de telles lignes. Les exclusions locales passent par `.git/info/exclude`, jamais par `.gitignore` (règle fixée le 18 octobre 2026, rappelée le 22). |
| Classes de cibles | Les classes de la bibliothèque sont des **valeurs moyennes de classe**, pas la fiche d'une arme : l'armement de l'adversaire n'est pas connu, toutes les valeurs d'une plage plausible sont possibles. Pour une menace, on retient le **milieu** de la plage (le bout optimiste vaut pour nos technologies seulement) et tout reste surchargeable par secteur (règle fixée à l'étape 2.4). |
| Langues | Échanges et `Documentation/` en français ; code, commentaires, interface, `radar_system.toml` et `tests/snapshots/JOURNAL.md` en anglais. Réponses concises. |
| Valeurs verrouillées | Une valeur de statut V ne change jamais sans l'accord explicite de l'utilisateur. |

### 1.4 Reprendre le travail dans une nouvelle session

Rien de ce qui a été décidé ne vit seulement dans une conversation :

| Où | Quoi |
|---|---|
| `Documentation/Choix_architecturaux.md` (ce document) | Toutes les variables, leurs valeurs et leurs raisons, les formules, les dispositions provisoires, les points ouverts et l'historique des décisions, question par question. |
| `Documentation/Feuille_de_route.md` | La feuille de route, l'état d'avancement et, pour chaque étape, les décisions prises et les renvois reçus d'autres étapes. |
| `radar_system.toml` | Les valeurs elles-mêmes ; le champ `source` de chaque entrée résume sa justification. |
| En-tête de `core/radar_model.py` | Le journal des versions du modèle (V6.0.0 à V6.11.0) et ce que chaque version a changé. |
| `tests/snapshots/JOURNAL.md` | Chaque régénération des valeurs figées de test et ses causes. |
| Mémoire de l'assistant (hors dépôt) | Locale à chaque machine, **non partagée** : elle n'est qu'un raccourci. Tout ce qui est nécessaire pour reprendre est dans le dépôt (ce tableau, §1.3, §1.5, §1.6) ; en cas d'écart, le dépôt fait foi. |

Reprendre sur une autre machine :
1. `git clone https://github.com/LetMePickThat/C.44.git`, ou `git pull` dans le dépôt existant (le travail se fait sur `main`). Après la réécriture de l'historique du 22 octobre 2026, un ancien clone doit se recaler : `git fetch` puis `git reset --hard origin/main`, s'il n'a pas de travail local à garder.
2. Python 3.11 ou plus (testé en 3.13), puis dans `V1/` : `python -m pip install --user -r requirements-dev.txt` (inclut `requirements.txt` et pytest). Sous Windows avec le Python du Microsoft Store, ajouter au PATH le dossier `Scripts` des paquets utilisateur (`...\LocalCache\local-packages\Python313\Scripts`) pour lancer `pytest` directement ; sinon `python -m pytest`.
3. Vérifier l'installation : `python core/radar_model.py --selftest`, puis `python -m pytest -q` depuis `V1/` (tout doit passer, voir §4).
4. Lancer l'outil : `python main.py` depuis `V1/`, ou le lanceur `Radar mission prep.bat`.
5. Lire `Feuille_de_route.md` (tableau d'avancement : la prochaine étape), §1.5 (travaux en suspens) et §1.6 (connaissances de travail), puis suivre la procédure d'une étape.

Procédure d'une étape :
1. Rappeler la raison d'être, les entrées et les sorties de l'étape.
2. Poser les questions variable par variable, regroupées par sujet, et attendre les décisions.
3. Coder : entrées dans `radar_system.toml` (valeur, unité, statut, source, date), version du modèle et journal en tête de `core/radar_model.py`, self-test et `tests/test_physics.py`.
4. Régénérer la table de l'estimation rapide : `python tools/gen_cost_table.py` (les caches du diagramme et de la structure se reconstruisent seuls ; les commiter s'ils changent).
5. Lire le rapport d'impact, `python tests/impact_report.py` (chiffres clés, formes d'onde et modes de confirmation qui changent), et l'inscrire dans `tests/snapshots/JOURNAL.md`.
6. Régénérer les valeurs figées, `python tests/update_snapshots.py`, puis lancer `python -m pytest -q`.
7. Mettre à jour ce document et `Feuille_de_route.md`, puis vérifier par une recherche que chaque passage modifié est présent (§1.6).
8. Commiter et pousser sur demande (§1.3 : message neutre, sans signature d'assistant). Le hash du commit d'une étape est reporté dans le tableau d'avancement de `Feuille_de_route.md` au commit de documentation suivant.

### 1.5 Travaux en suspens hors feuille de route (au 22 octobre 2026)

| Sujet | État | Quoi faire |
|---|---|---|
| Durée de la suite de tests | Trois runs complets sur le même portable (14900HX ; la machine habituelle est un 9800X3D fixe) : le matin avant l'étape 2.4, **sur batterie**, 108 tests en 5 min 53 ; l'après-midi après l'étape 2.4, **sur batterie**, 117 tests en environ 30 min ; le soir après l'étape 2.5, **sur secteur, mode performance**, 126 tests en 6 min 10. **Le code n'est pas en cause** : le dernier run contient tout le code des étapes 2.4 et 2.5. **La machine était ralentie l'après-midi**, à code identique : la version 2.4 commitée mettait 70 s sur `mission_example`, contre 19,6 s le matin, sur batterie dans les deux cas (relevé l'après-midi : mode « Legion Balance », 37 %, en décharge). La batterie seule n'explique donc pas l'écart (le matin, sur batterie, la suite était rapide) ; la cause précise (mode d'alimentation, niveau de charge, température, autre bridage) **n'est pas identifiée**. | Rien d'obligatoire. Pour identifier la cause, mesurer `mission_example` (référence : environ 20 s) dans chaque état de la machine (secteur ou batterie, mode d'alimentation, niveau de charge). |
| Hash des étapes | 2.4 : commit `008d583`. 2.5 : commit `baeadcf`, reporté dans `Feuille_de_route.md`. | — |
| Prochaine étape | **3.1, veille S reconstruite** : cadrage proposé (blocs A grille de fréquences, B espace des formes d'onde, C multifaisceau, D environnement par forme d'onde, E règles de l'optimiseur), pas encore validé ; l'étape 3.0 (repères et navigation) est faite. Elle reçoit le plus de renvois (multifaisceau à trois modes et loi PAE / recul, grille de fréquences fine, bandes au-delà de 75 MHz, impulsions de moins de 20 µs, critère secondaire de l'optimiseur, fouillis par forme d'onde, exigence de portée devant une surface, réponse au brouillage d'autoprotection, pas de grille par secteur) : voir `Feuille_de_route.md`. | Commencer par la procédure d'une étape (§1.4). |
| Sauvegardes locales de la réécriture d'historique | Sur le portable seulement : branche `backup-before-rewrite` et `refs/original/`, gardées à la demande de l'utilisateur. | Les supprimer quand l'utilisateur le dira ; rien à faire sur une autre machine. |
| Documentation de l'étape 2.4 | Une partie des mises à jour de ce document (en-tête, modules, bilan des entrées, lignes CFAR et cohérence) n'avait pas été appliquée au commit `008d583` ; rattrapée avec l'étape 2.5. | Vérifier, à chaque étape, que chaque remplacement de la documentation a bien été appliqué. |
| Copies locales hors dépôt | `PROJECT_CONTEXT.md`, `README.txt`, `radar_params_restore.md` à la racine, exclus par `.git/info/exclude` : antérieurs à la feuille de route, remplacés par `Documentation/`. | Ne pas les commiter ; ils peuvent manquer sur une autre machine sans conséquence. |
| Construction de l'exécutable | Différée (décision du 19 octobre 2026) : la disposition actuelle (`main.py` + `core/` + `gui/`, lanceur `Radar mission prep.bat` qui installe `requirements.txt` au premier démarrage et lance `pythonw`) reste telle quelle jusqu'à la livraison. À la livraison : PyInstaller en un dossier (environ 400 à 700 Mo avec VTK et Qt), zippé hors du dépôt, script `tools/build_exe.bat`, faux positifs d'antivirus possibles. | Ne rien proposer avant que l'utilisateur annonce la livraison. |

### 1.6 Connaissances de travail (pour reprendre à l'identique)

**Façon de travailler avec l'utilisateur.**
- Il est expert radar, francophone, et attend des réponses concises. Il valide bloc par bloc (« validé », « tout validé ») et lance le codage par « go ». « c, p » ou « cp » = commit et push.
- Pour chaque variable, une proposition chiffrée avec sa plage est présentée, et la décision lui revient. Pour nos technologies, on prend le bout optimiste de la plage. Pour les menaces et l'environnement, on prend la **médiane**, ou une valeur moyenne de classe. Une valeur qu'il ne donne pas n'est jamais choisie en silence : on la redemande.
- Il pose des questions de fond en cours d'étape (exemples : usage des classes de cibles, définition du GO-CFAR). On y répond avant de poursuivre, et la réponse est consignée dans l'historique (§11).
- Les écarts et les erreurs se signalent d'emblée dans le compte rendu (exemples : la cote du cône porte-antennes, la documentation 2.4 incomplète).

**Pièges rencontrés (à éviter).**
- Le PowerShell 5.1 de Windows écrit le TOML avec un BOM UTF-8 que `tomllib` refuse : écrire les fichiers en UTF-8 sans BOM (par Python), avec des fins de ligne LF dans le dépôt.
- Une modification de la documentation par remplacements de texte peut échouer sans bruit : vérifier par une recherche que chaque passage modifié est bien présent avant le commit.
- En insérant du code avant une fonction mise en cache, vérifier que le décorateur `@functools.lru_cache` reste sur la bonne fonction : `solve()` doit rester de l'ordre de la milliseconde.
- Le profil `cProfile` d'une mission et les temps de référence ci-dessous détectent vite une régression.

**Temps de référence** (portable 14900HX, 32 fils, sur secteur en mode performance, étape 2.5 ; le fixe 9800X3D est plus rapide ; le portable a été une fois environ 3,5 fois plus lent sur batterie, sans cause identifiée, voir §1.5) :

| Opération | Durée |
|---|---|
| `python -m core.optimizer tests/missions/mission_example.json` (séquentiel) | environ 20 s |
| `python tools/gen_cost_table.py` (32 processus) | environ 1 min 40 |
| `python tests/impact_report.py` | environ 2 min 40 |
| `python tests/update_snapshots.py` | environ 3 min |
| `python -m pytest -q` (144 tests) | environ **2 min** en parallèle (6 processus, fixe 9800X3D, après l'étape 3.0) ; 6 min 30 en séquentiel (`-n 0`) |
| reconstruction automatique de `core/structure_cache.npz` / `core/antenna_cache.json` | environ 23 s (modèle 3D) / 8 s |

**Fichiers dérivés** (§2.2) : `antenna_cache.json` et `structure_cache.npz` se reconstruisent seuls quand leurs entrées changent, et ne sont réécrits que pour le fichier de paramètres par défaut. Ils sont versionnés : les commiter avec l'étape qui les change. `cost_table.json` et les valeurs figées se régénèrent à la main (procédure, points 4 à 6).

**Comportements de l'outil bons à savoir.**
- Une mission sans clés nouvelles se comporte comme avant chaque étape (valeurs par défaut neutres : classe GENERIC, aucun objet, pas de brouilleur, pas de Faraday).
- L'interface conserve les clés qu'elle n'édite pas encore : `target_type`, `range_min_km`, le ciel, les brouilleurs.

---

## 2. Architecture logicielle

### 2.1 Modules

| Module | Rôle |
|---|---|
| `core/params.py` | Charge `radar_system.toml`, une seule fois par processus. Expose `P.get`, `P.meta`, `P.section` et l'empreinte des paramètres (`fingerprint`). La variable d'environnement `RADAR_PARAMS` permet de désigner un autre fichier. |
| `core/radar_model.py` | Modèle radar V6.11.0 (repères des volumes et navigation : étape 3.0 ; géométrie du vaisseau : modèle 3D) : GO-CFAR (étape 2.5) ; bibliothèque de cibles (classes lues dans le fichier, cible d'un secteur, T₀ par classe et par bande, fluctuation en forme Gamma, optique de l'autodirecteur, accélération, dépolarisation et diversité, CFAR OS, masquage ; étape 2.4), critère de détection (Pd moyennée sur la position de la cible, Pd cumulée de la veille, politique de fausses alarmes, grille de veille à 1,4 lobe ; étape 2.3), traitement (blocs cohérents de taille optimale, oscillateur et plateforme, banc de vitesses, chevauchement et CFAR calculés, cases indépendantes pour les fausses alarmes ; étape 2.2), formes d'onde paramétriques (modulation LFM ou NLFM, pondération et perte de compression, k_r et résolution physique, effets dans l'impulsion, zone aveugle, éclipsage des trains échelonnés ; étape 2.1), chaîne de réception (T_sys de ciel froid, facteur de bruit du CAN dérivé du convertisseur, dynamique et portée minimale non saturée, récupération du limiteur, perte de recombinaison des deux voies de polarisation ; étape 1.5), bande (loi de bord par bande, occupation du spectre, retard vrai, contrôles de cohérence au chargement ; étape 1.4), bilan électrique et PAE variable par bande (étape 1.3), désadaptation active, marge de lobe de réseau, lobes visibles selon le dépointage, pureté de polarisation, bandes, gains, diagramme, pertes, équation radar sous forme énergétique, besoin en SNR de détection (Shnidman) et de poursuite (borne de Cramér-Rao), grille de veille dans l'espace (u, v), et `solve()`. |
| `core/detection.py` | **Étape 2.3.** Lois de détection exactes (Swerling 0 à 4, toute Pfa), Pd moyennée sur une loi discrète de pertes de position, Pd cumulée sur les observations d'une cible qui se rapproche. **Étape 2.4** : fluctuation donnée aussi par sa forme Gamma `Fluct(k)` (non entière), qui reproduit exactement les cas de Swerling. |
| `core/antenna.py` | **Étapes 1.1 et 1.2.** Diagramme de rayonnement calculé à partir de la composition des TRSA : faisceau d'émission uniforme, pondération de réception par niveau de lobes, étendue RMS de l'ouverture, profil des lobes lointains sur tout l'espace visible (lobes de jeu compris). Profils **moyennés en azimut** du diagramme de réception, pour l'intégrale du bruit du ciel (étape 1.5). Résultat mis en cache dans `core/antenna_cache.json`. |
| `core/environment.py` | **Étape 2.5.** Environnement d'une face : structure du vaisseau d'après le **modèle 3D** `assets/models/LP_NISO.obj` (`[ship]`, depuis le 22 octobre 2026 ; primitives auparavant), lancer de rayons exact sur ses triangles, masquage par fraction bloquée du tube d'ouverture, bruit de la structure (lobe principal et lobes lointains) ; surfaces des corps (masque, fouillis sur l'anneau d'égale distance, saturation des CAN) ; brouilleurs (classes `[jammers.*]`, bruit par le diagramme, annulation, portée de percée) ; rotation de Faraday. Tables de structure mises en cache dans `core/structure_cache.npz`. Branché dans `core/optimizer.sector_sky` (S et Ku). |
| `core/frames.py` | **Étape 3.0, couche calculateur.** Message de navigation (attitude, orbite facultative), repères des volumes (vaisseau, inertiel, RTN, limbe, deux directions) et du ciel ; `resolve()` passe la mission dans le repère du vaisseau, une seule fois, avant tout calcul (optimiseur, redondance, estimation rapide, vues 3D). Rotations : quaternion, matrice, angles 3-2-1. |
| `core/sky.py` | **Étape 1.5.** Hausse de la température d'antenne selon la direction : fond galactique et corps du ciel de la mission (Soleil, Terre, Lune...), donnés dans le repère du vaisseau ; aveuglement et centile du bilan d'un secteur. |
| `core/isolation.py` | **Étape 1.5.** Fuites entre bandes et entre faces : matrice des verdicts, règles pour l'ordonnanceur (cadence commune par bande, harmonique S dans le canal Ku). |
| `core/optimizer.py` | Optimiseur de la veille S. Choisit pour chaque secteur la fréquence et les formes d'onde par tranche d'angle de dépointage, et établit le bilan énergie et temps par face. Depuis l'étape 1.5, il évalue le T_sys du ciel sur le secteur (`sector_sky`), retire les pointages aveuglés et met les portées à l'échelle du T_sys du bilan. |
| `core/tracking.py` | Confirmation (modes KU1+7, KU7 et S2) et poursuite active (AT) en Ku, ajoutées au bilan par face. Depuis l'étape 1.5 : T_sys Ku du ciel par secteur, alerte de saturation des tranches de poursuite, règles pour l'ordonnanceur. |
| `core/cost_estimate.py` | Estimation rapide pour l'interface, à partir de `core/cost_table.json`. Elle n'est pas faite pour décider : seul le calcul exact fait foi. |
| `core/mission.py` | Modèle de mission : valeurs par défaut, validation, chargement et sauvegarde JSON et CSV. |
| `core/redundancy.py` | Détecte si un secteur couvre déjà le besoin d'un autre, dans les deux sens. |
| `core/geometry.py` | Trame de la sphère des directions pour la 3D et l'estimation rapide. Vue 3D du vaisseau : **les triangles du modèle 3D de `[ship]`**, placés par `core/environment.py`, les mêmes que le masquage. |
| `gui/` | Interface PyQt5 : préparation, résultats, vue 3D (pyvista). |
| `tools/gen_cost_table.py` | Reconstruit `core/cost_table.json`. |
| `tests/` | Banc de tests (§4). |

### 2.2 Fichiers dérivés et leur cohérence

| Fichier | Contenu | Garantie de cohérence |
|---|---|---|
| `core/cost_table.json` | Énergie minimale par pointage selon la distance, la SER, la vitesse et l'angle, à 2,8 GHz. Il enregistre aussi l'option de lobes de réception utilisée. | Contient l'empreinte des paramètres. L'interface alerte si elle ne correspond plus, et un test vérifie qu'elle est à jour. Pour la reconstruire : `python tools/gen_cost_table.py`. |
| `core/structure_cache.npz` | **Étape 2.5.** Par face : fraction bloquée et ε·T des parties touchées sur une grille (u, v), directions de la structure vues du centre de l'antenne. | Empreinte de la géométrie (`[ship]`, inclinaison, azimuts des faces, réglages `[tool.environment]`). Recalculé automatiquement si elle change (environ 15 s), réécrit seulement pour le fichier de paramètres par défaut. Un test vérifie qu'il est à jour. |
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
| Revue | Toutes les valeurs ont été validées une par une lors de la revue 0.3. Les ajouts des étapes 1.1 à 2.3 ont été validés pendant chacune de ces étapes. |

**Bilan actuel (étape 2.5, modèle 3D du vaisseau) : 286 entrées système, dont 243 V, 41 D et 2 ?.** Les deux entrées « ? » sont `multibeam.l8_db` et `multibeam.p`.

---

## 4. Banc de tests

| Élément | Choix |
|---|---|
| Outil | `pytest` et `pytest-xdist`, déclarés dans `requirements-dev.txt`. **Exécution parallèle** (22 octobre 2026, `pytest.ini` : `-n 6 --dist loadgroup`) : les tests sont indépendants (entrées propres, `tmp_path`, fichiers de paramètres passés par `RADAR_PARAMS` à des sous-processus) ; les caches dérivés s'écrivent de façon atomique (fichier temporaire puis remplacement) ; les tests qui utilisent la fixture `runs` restent dans un même processus (`tests/conftest.py`), et cette fixture calcule les missions en parallèle, un processus par mission (résultat identique au séquentiel, vérifié par `test_parallel_run_equals_sequential`). Le chemin critique est le plus long test isolé (`mission_redundancy_demo`, environ 1 min) : au-delà de 6 processus, rien ne se gagne. Séquentiel : `python -m pytest -n 0`. |
| Famille 1 : valeurs figées | `tests/test_snapshots.py`. Toutes les sorties numériques de l'outil sont comparées avec une **tolérance relative de 1e-9**. |
| Famille 2 : physique et invariants | `tests/test_physics.py`. Lois qui doivent tenir quelles que soient les valeurs : R ∝ σ^¼, R ∝ P^¼, cohérence du nombre de pointages, du cache, de la borne de Cramér-Rao, des options de lobes, etc. Le self-test du modèle est conservé. |
| Missions de test | `mission_example` (en JSON et en CSV), `mission_veille500`, `mission_redundancy_demo`, `mission_synthetic_mix`, `mission_sky_demo` (étape 1.5 : un ciel avec le Soleil, la Lune, le limbe de la Terre et un repère galactique), `mission_target_library` (étape 2.4 : un secteur par classe de cible, plus un KKV surchargé), `mission_environment` (étape 2.5), `mission_frames` (étape 3.0 : orbite basse terrestre à 400 km, vaisseau pointé vers le nadir, un secteur par repère, ciel inertiel). Les deux premières ont été récupérées dans l'historique git. |
| Rapport d'impact | `python tests/impact_report.py`, à chaque étape, avant de régénérer les valeurs figées : chiffres clés, et pour chaque mission de test les formes d'onde de veille et les modes de confirmation qui changent. |
| Régénération | `python tests/update_snapshots.py`. Chaque régénération est consignée dans `tests/snapshots/JOURNAL.md`, avec sa cause et les formes d'onde ou modes qui ont changé. |
| État | 144 tests passent après l'étape 3.0 (environ 2 min en parallèle sur le fixe 9800X3D, 6 min 30 en séquentiel ; voir §1.5 pour les écarts de durée selon la machine). |

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
| `rx_sll_options_db` | Niveaux de crête des lobes secondaires de la **pondération de réception** offerts à l'opérateur pour chaque volume de veille. | [−25, −30, −35] dB | V | **Étape 1.1.** Le réglage est fait par volume dans la préparation de mission (clé `rx_sll_db`), avec un seul réglage pour S et Ku et −25 dB par défaut. Descendre en niveau rejette mieux les échos forts hors de l'axe (cible brillante, Lune ou Terre, brouilleur) au prix du gain en réception : par rapport à l'option −25 dB, −0,53 dB à −30 et −1,00 dB à −35. |
| `rx_taper_nbar` | Paramètre n̄ de la loi de Taylor circulaire de réception. | 5 | V | Étape 1.1. Loi standard ; son niveau de conception est **calé numériquement** pour que le niveau **atteint** sur l'octogone réel égale l'option demandée. |
| `rx_taper_edge` | Rayon où la loi de Taylor atteint sa valeur de bord. | "inscribed", soit D_env/2 | D | Exploration de l'étape 1.1 : à niveau atteint égal, le cercle inscrit donne un meilleur rendement que le cercle circonscrit. Les éléments des coins gardent la valeur de bord. L'autre valeur possible est "circumscribed". |
| `grating_lobe_g0` | Largeur, en u, de la remontée de \|Γ\| quand le premier lobe de réseau approche de l'espace visible : \|Γ_LR\| = exp(−g/g₀). | 0,02 | V | **Étape 1.2, Q3.** Avec le jeu de 2 mm, ce terme seul coûtait environ 3,0 dB par passage à 2,9 GHz et 75° (4,1 dB avec le terme WAIM), ce qui expliquait la limite de 70° au-dessus de 2,85 GHz. Avec le jeu de 0,5 mm, le pas moyen est plus petit : 0,76 dB à 2,9 GHz et 75° (1,8 dB avec le terme WAIM), 0,02 dB à 2,85 GHz et 75°, presque rien à 2,8 GHz. |
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
| `beam_spacing_hpbw` | Pas du réseau hexagonal des pointages d'émission de veille, en largeurs de lobe d'émission. | 1,4 | V | **Étape 2.3, D1.** Optimum du nouveau critère (Pd moyennée sur la cellule, diagramme réel) : 1,4 pour une Pd de 0,9 par observation, 1,6 pour 0,5, plat à 0,1 dB entre 1,3 et 1,6. Était 1,04, optimum au pire point (V6.0.10) : environ 1 dB d'énergie et 45 % de positions en moins. Le coin d'une cellule perd −7,9 dB ; la grille doit être décalée à chaque trame (`grid_dithering`). Réglage par secteur à l'étape 3.1. |
| `grid_dithering` | Décalage de la grille de veille à chaque trame. | vrai | V | **Étape 2.3, option (a).** Une cible qui arrive en ligne droite garde sa direction ; sans décalage, elle resterait au même endroit de sa cellule à chaque revisite. Exigence pour l'ordonnanceur (étape 4.3) ; la valeur « faux » (grille fixe, positions corrélées) n'est pas implémentée. |
| `rx_beam_spacing_hpbw` | Pas du réseau des faisceaux de réception numériques formés dans chaque pointage, en largeurs de lobe de réception. | 0,5 | V | **Étape 1.1, option C.** Perte de grille résiduelle en réception de 1,672 × 0,5² = 0,42 dB, pour environ 3,4 faisceaux de réception par pointage à −25 dB. C'est le point où ajouter des faisceaux ne rapporte plus beaucoup : passer à 0,3 triple le calcul pour 0,27 dB. L'option A (perte nulle) supposait un calcul infini ; l'option B (un seul faisceau) coûtait 1,42 dB. Le nombre de faisceaux alimentera le bilan de calcul de l'étape 4.4, et l'optimiseur pourra régler ce pas aux étapes 3.1 et 5.2. |
| `enable_grid_loss` | Prise en compte des cellules de grille dans la Pd moyennée. | vrai | V | Depuis l'étape 2.3, la cellule d'émission et la grille de réception entrent dans la loi des pertes de position (Pd moyennée), avec le diagramme réel. |
| `frame_interleaved` | Attente de l'écho masquée par l'entrelacement des pointages. | vrai | D | Hypothèse d'ordonnancement, possible grâce à la réception numérique. À vérifier à l'étape 4.3. Avec « faux », la trame ajoute 2R/c par pointage. |

### 5.5 `[detection]`

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `pd` | **Veille** : probabilité de détection **cumulée** atteinte au plus tard à la portée `range_km` du secteur, sur les observations d'une cible qui se rapproche à v_close_max (espacées de v·RR, phase moyennée), la Pd de chaque observation étant moyennée sur la position de la cible. **Ku** (confirmation, poursuite) : Pd d'une observation. | 0,9 | V | **Étape 2.3, A1** (était la Pd d'une observation unique). Réglage par secteur à l'étape 5.1. |
| `pfa` | Probabilité de fausse alarme par case indépendante en mode « pfa_cell », et des observations Ku. | 1e-8 | V | À 1e-6, confirmer les fausses alarmes coûtait deux fois la veille elle-même. **Étape 2.3** : en mode « fa_rate » (défaut), la Pfa de la veille est déduite du taux toléré. |
| `fa_mode` | Politique de fausses alarmes de la veille. | "fa_rate" | V | **Étape 2.3, C1.** « fa_rate » : taux toléré par secteur et par revisite (clé de secteur `fa_per_rr`, défaut 10), Pfa par case = taux / (positions × cases indépendantes) ; « pfa_cell » : Pfa fixe. Réglage de mission à l'étape 5.1, coût de confirmation dans l'optimiseur à l'étape 5.2 (décision de l'utilisateur). |
| `pfa_min`, `pfa_max` | Bornes de la Pfa déduite. | 1e-14, 1e-4 | V | **C4.** Elle est en outre arrondie au vingtième de décade (moins de 0,02 dB). |
| `cum_pd_floor` | Une observation dont la Pd reste sous cette valeur ne compte plus. | 1e-3 | V | **A4**, sans autre plafond (bout optimiste). Un plafond du nombre d'observations (cible surgissante) relève de l'étape 5.1. |

### 5.5 bis `[processing]` : grille de détection et CFAR (étape 2.2)

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `range_grid_oversampling` | Suréchantillonnage de la grille de détection en distance (interpolation numérique). | ×4 | V | **C1**, bout optimiste de ×1,25 à ×4 : chevauchement moyen 0,05 dB à −25 dB (0,54 dB à ×1,25, 0,22 dB à ×2). Coût de calcul à l'étape 4.4. |
| `doppler_zero_padding` | Bourrage de zéros de la transformée Doppler d'un bloc cohérent. | ×4 | V | **C2** : 0,07 dB en moyenne (1,12 dB sans bourrage, 0,29 dB à ×2). |
| `doppler_window` | Pondération Doppler. | "none" | V | **C3.** Pas de fouillis à rejeter dans l'espace. **Étape 2.4, D1** : le masquage par une cible forte est donné en sortie (premier lobe Doppler à −13,3 dB, le point faible de l'absence de fenêtre) ; son effet sur une scène dense relève de l'étape 5.4. |
| `cfar_type` | Type de CFAR. | "CA" | V | **D1.** Bruit thermique homogène. **Étape 2.4, D3** : option "OS" (statistique d'ordre, robuste aux cibles voisines). **Étape 2.5, G1** : option "GO" (voir ci-dessous). |
| `cfar_type` = "GO" | Option « greatest of » : la plus grande des moyennes des deux demi-fenêtres fixe le seuil, ce qui tient la Pfa à un bord de fouillis. | option | V | **Étape 2.5, G1.** Perte exacte de Hansen et Sawyers : 0,05 dB à 1 024 cellules (0,76 dB à 64). Le choix par pointage relève des étapes 4.3 et 5.2 ; un secteur qui contient un limbe est signalé. |
| `cfar_os_rank_frac` | Rang de l'OS-CFAR, en fraction des cellules de référence. | 0,75 | V | **Étape 2.4, D3.** Perte exacte pour Swerling 1 sur un échantillon : 0,06 dB à 1 024 cellules et Pfa 1e-8 (CA : 0,04 dB). |
| `cfar_ref_cells` | Cellules de référence du CFAR. | 1 024 | V | **D2**, bout optimiste : perte 0,04 dB (0,64 dB à 64, 0,16 dB à 256), calculée exactement pour Swerling 1 sur un échantillon et appliquée à toute observation (D3, légèrement prudent). |

Ces trois pertes (`straddle_range`, `straddle_doppler`, `cfar`) remplacent les entrées `straddle` (0,7 dB) et `CFAR` (0,5 dB), supprimées.

### 5.6 `[power]` : chaîne de puissance

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `trm_technology` | Technologie des TRM. | « GaN-on-diamond, microchannel cooling » | V | Description de V5.6.0 : GaN sur diamant et refroidissement par microcanaux, pour une forte densité de puissance et une bonne évacuation de la chaleur en fonctionnement soutenu à forte charge crête et moyenne. Les valeurs des TRM sont des hypothèses **crédibles mais optimistes pour 2050-2060** (choix de l'utilisateur, étape 1.3). |
| `pae_duty_ref` | Facteur de forme auquel la PAE en saturation est spécifiée. | 0,10 | D | Condition habituelle des fiches techniques en impulsions (étape 1.3, Q2 bis). Remplace la constante `pae` = 0,75 commune à S et Ku, désormais retirée. |
| `psu_eff` | Rendement de l'alimentation. | 0,95 | V | Verrouillé lors de la revue 0.3. |
| `distribution_eff` | Rendement de la distribution. | 0,995 | V | Verrouillé lors de la revue 0.3. |
| `overhead_frac` | Part de la consommation hors émission. | 0,03 | V | Valeur conservée. Elle sera remplacée par un vrai bilan (réception numérique et CAN, environ 130 kW par face, contrôle, refroidissement) aux étapes 4.1 et 4.4. |
| `face_elec_mw` | Puissance électrique allouée à une face. C'est la valeur par défaut d'une mission. | 2,0 MW | V | Verrouillé lors de la revue 0.3. |
| `ship_elec_total_mw` | Puissance totale du vaisseau disponible pour le radar. | 8,0 MW | V | **Répartissable à la demande entre les faces**, par exemple 3 MW sur F1 et 5 MW sur F2 à F4. Le logiciel radar gère cette répartition avec les autres paramètres (étapes 4.1 et 5.2). Aujourd'hui, la validation vérifie seulement qu'une face ne dépasse pas 8 MW : c'est temporaire, sur décision de l'utilisateur. |

Valeur dérivée : CHAIN_EFF = 0,95 × 0,995 × (1 − 0,03) = 0,917. **Depuis l'étape 1.3, le bilan de la face est électrique** (Q1) : pour 2 MW alloués, la face dispose de 2 MW × 0,917 = 1,834 MW pour l'émission, et chaque observation coûte son énergie RF divisée par la PAE de sa bande et de sa forme d'onde. Auparavant, la conversion se faisait une fois par face avec une PAE unique (1,375 MW RF).

### 5.7 `[migration]` : migration en distance

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `mode` | Traitement de la migration en distance. | "COMPENSATED" | V | Traitement de type keystone ou Radon. Le coût des bancs de vitesse sera évalué aux étapes 2.2 et 4.4. |
| `residual_db` | Résidu de l'algorithme de compensation (keystone ou Radon) quand la cible traverse plus de 1,5 case. | 0,1 dB | V | **Étape 2.2, B5**, bout optimiste de 0,1 à 0,3 dB. Était 1,0 dB forfaitaire ; la perte du banc de vitesses est désormais calculée. |
| `bank_max_loss_db` | Pas du banc d'hypothèses de vitesse : perte d'une cible à mi-pas. | 1,0 dB | V | **Étape 2.2, B2** (plage 0,5 à 2 dB). Loi = perte dans l'impulsion (étape 2.1) + glissement sur l'observation. La perte **moyenne** sur le pas va au bilan, le nombre d'hypothèses aux fausses alarmes et au calcul (étape 4.4). |

### 5.8 `[multibeam]` : multifaisceau en émission

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `l8_db` | Perte en émission avec 8 faisceaux, selon une loi unique. | 0,8 dB | ? | Vient de V5.6.0, sa définition est inconnue. La perte en réception vaut 0 (réception numérique). |
| `p` | Exposant de la loi L8 × (log₂B / 3)^p. | 1,1 | ? | Donne 0,24, 0,51 et 0,80 dB pour 2, 4 et 8 faisceaux. |

Décision : « tous les modes sont possibles et sont à évaluer par l'optimiseur » (étape 3.1). Trois modes sont à modéliser : (a) superposition de faisceaux dans chaque TRM, avec le recul du PA qui en découle (loi PAE / recul renvoyée de l'étape 1.3 à l'étape 3.1) ; (b) partition de l'ouverture, à partir du diagramme de l'étape 1.1 ; (c) multiplexage en temps ou en fréquence (étape 2.1).

### 5.9 `[coherence]` : cohérence et bruit de phase

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `enable` | Active la perte de cohérence des blocs cohérents (cible, oscillateur, plateforme). | vrai | D | Étape 2.2. |
| `t0_k` | Facteur de la loi du temps de cohérence de la cible, **T₀ = k · λ / (2 L ω)**, L et ω de la classe de cible. | 1,0 | V | **Étape 2.4, B1.** Remplace le t₀ provisoire de 0,25 s de l'étape 2.2. T₀ dépend de la bande (environ 5 fois plus court en Ku). |
| `p` | Exposant de la loi η = exp(−(T/T₀)^p) d'un bloc cohérent. | 2,0 | V | **Étape 2.4, B2** : spectre Doppler gaussien d'une cible en rotation. |
| `lo_ref_ghz`, `lo_offsets_hz`, `lo_ssb_dbc_hz` | Spectre de bruit de phase de l'oscillateur à 10 GHz, mis à l'échelle de la porteuse (+20 log f/10 GHz). | 1 Hz −100 ; 10 Hz −120 ; 100 Hz −140 ; 1 kHz −150 ; 10 kHz −160 ; 100 kHz −170 ; 1 MHz −175 dBc/Hz | V | **Étape 2.2, A1**, bout optimiste (oscillateur photonique). Réaliste aujourd'hui en spatial : −60 / −90 / −110 / −130 / −140 / −150 / −155. Un seul oscillateur pour l'émission et la réception (A2) : l'aller-retour filtre le bruit par 4 sin²(π f τ_rt). La perte est négligeable dans les deux cas (au plus 4e-6 dB) ; l'effet visible est le plancher Doppler, en sortie. |
| `platform_motion_rms_m` | Mouvement résiduel de la plateforme et de la structure après compensation, sur un bloc. | 10 µm | V | **A5**, bout optimiste de 10 à 100 µm : 1,2 mrad en S, 6 mrad en Ku. |

Retirés : `sigma_t`, la gigue d'horloge (négligeable, 1e-7 à 2,8 GHz), et à l'étape 2.2 `eta_min` (plancher de 0,70) avec le plafond provisoire de 149 ms (`COH_T_MAX_S`) : une forme d'onde cohérente est désormais découpée en **blocs cohérents de taille optimale**, intégrés de façon non cohérente (§7.10).

### 5.10 `[estimation]` : précision des mesures

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `angle_crlb_margin` | Marge appliquée à la **borne de Cramér-Rao angulaire** de l'ouverture numérique complète. | 1,13 | V | **Étape 1.1, option a.** σ_u = marge × λ / (2π x_rms √(2 SNR)), où x_rms vient de la composition des TRSA (0,248 D). Cette précision **ne dépend pas de la pondération de réception** : l'estimateur ML utilise toute l'ouverture. La marge est la même que pour k_f et k_r. Elle remplace k_θ = 0,5 × HPBW(0,886), qui était légèrement sous la borne. Effet : +1,27 dB de SNR demandé pour une poursuite pilotée par l'angle. |
| `k_f` | Constante de l'estimateur Doppler. | 0,6 | V | Borne pour une sinusoïde seule, 0,551, multipliée par 1,13 (revue 0.3). L'ancienne valeur 0,5 était sous la borne. |
| `range_crlb_kr` | Constante de l'estimateur de distance, **spectre rectangulaire** (depuis l'étape 2.1, le k_r d'une observation en est **dérivé** : 0,6 / rapport des largeurs de bande RMS de la pondération, soit 0,70 / 0,75 / 0,79 à −25 / −30 / −35 dB). | 0,6 | V | Borne pour un spectre rectangulaire, 0,551, multipliée par 1,13 ; `sig_bw` est la bande **totale**. Ancienne valeur : 1,3, sans source. La pondération de Taylor sera déduite de la modulation réelle à l'étape 2.1. Une suggestion extérieure donnait k_r = 0,32 pour une bande RMS. Elle a été corrigée (la bonne valeur serait 0,16) et ne s'applique pas ici, puisque la bande utilisée est la bande totale. |

Retiré : `k_theta` = 0,5, remplacé par `angle_crlb_margin` à l'étape 1.1.

### 5.11 `[band.S]` : bande S (veille)

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `usable_lo_ghz` / `usable_hi_ghz` | Bande utile. | 2,7 à 2,9 GHz | V | Confirmé lors de la revue 0.3. |
| `core_lo_ghz` / `core_hi_ghz` | Bande centrale, sans perte de bord. | 2,74 à 2,86 GHz | D | **Étape 1.4, A1-A2.** Donnée en GHz pour chaque bande. Remplace `core_inset_divisor` = 6 (bande utile amputée d'un sixième de chaque côté), une proportion copiée de l'ancienne bande Ku de 14,8 à 17,2 GHz. Valeur optimiste 2050-2060 pour une bande relative de 7 %. |
| `edge_derating_db` | Perte **totale** (aller-retour) aux bords de la bande utile ; cosinus surélevé entre la bande centrale et les bords. | 0,5 dB | D | **Étape 1.4, A2-A4.** Rendement des PA et LNA, filtres et WAIM désaccordée hors de la bande centrale. Elle est **moyennée sur le spectre occupé** [f − B/2, f + B/2]. Ancienne valeur 1,5 dB, copiée de l'ancienne bande Ku. |
| `scan_max_deg` | Limite de balayage jusqu'à `scan_break_ghz`. | 75° | V | Confirmé. |
| `scan_break_ghz` | Fréquence au-dessus de laquelle la limite de balayage baisse. | 2,85 GHz | V | Avec le pas moyen réel (étape 1.2), la marge de lobe de réseau à 2,9 GHz et 75° valait 0,35 % avec 2 mm de jeu (désadaptation 4,1 dB par passage), d'où les 70° au-dessus. Avec 0,5 mm, elle vaut 0,9 % (1,8 dB par passage, sous le seuil d'alerte). Les limites restent verrouillées (Q5) ; les relâcher au-dessus de 2,85 GHz serait une décision à prendre. |
| `scan_max_hi_deg` | Limite de balayage au-dessus de `scan_break_ghz`. | 70° | V | **Gardée à l'étape 1.4 (E1, option a).** Au haut de bande et à 75°, la désadaptation n'est plus que de 1,8 dB par passage, mais le flanc du lobe de réseau atteint l'horizon, et le diagramme de l'élément, qui l'atténuerait, n'est pas modélisé. À revoir quand il le sera. |
| `p_pk_trm_w` | Puissance crête par TRM, **en sortie de PA** (plan de référence 1, là où la PAE est définie). | 2000 W | V | Convention verrouillée lors de la revue 0.3. Le chemin du PA à l'élément est la perte dérivée TX_chain. |
| `eta_illum_tx` | Rendement d'illumination en émission. | 1,0 | V | Émission uniforme, PA en saturation. |
| `allowed_roles` | Rôles permis en bande S. | ["SEARCH", "TWS"] | D | Politique d'usage des bandes du logiciel radar (étapes 3.x et 4.3). Le « toucher » S2 de confirmation sort de cette règle (noté pour l'étape 3.2). |
| `pol_rx_dual` | Réception numérique bipolarisation. | vrai | V | Un CAN par TRM. |
| `waim_match_deg` | Angle de dépointage où la couche WAIM adapte le réseau (modèle de la feuille de courant). | 45° | V | **Étape 1.2, Q2.** Perte par passage : 0,13 dB dans l'axe, 0 à 45°, 0,56 dB à 70°, 1,05 dB à 75°. C'est l'équilibre entre l'axe et les grands dépointages (40° favoriserait l'axe, 50° les grands angles). |
| `pol_track_est_loss_db` | Perte de polarisation d'une observation de **poursuite** avec la réception bipolarisation. | 0,05 dB | V | **Étape 1.5, E2.** La polarisation de la cible est connue par les observations précédentes : recombinaison cohérente avec des poids estimés, il reste le résidu d'estimation. Les observations de **détection** (veille, confirmation) n'ont pas d'entrée : leur perte est **calculée** (§7.8). Remplace `pol_loss_dual_db` = 0,5 dB, valeur provisoire de l'étape 1.2. La bande S n'a pas de rôle de poursuite aujourd'hui ; l'entrée est gardée par symétrie. |
| `pol_loss_single_db` | Perte avec une réception monopolarisation sur une cible qui dépolarise. | 3,0 dB | V | Gardée pour étudier un récepteur monopolarisation. Utilisée seulement si `pol_rx_dual` est faux. |
| `pae_sat` | PAE du PA en saturation, au facteur de forme de référence. | 0,80 | D | **Étape 1.3, Q3.** Le GaN actuel en S de forte puissance atteint environ 55 à 65 % sur la bande, les meilleurs résultats de laboratoire environ 75 à 80 %. 0,80 est une hypothèse optimiste pour 2050-2060. |
| `pae_t_edge_s` | Temps par impulsion pendant lequel le PA consomme sans délivrer sa pleine puissance (fronts, mise en place de la polarisation et de la phase). | 0,2 µs | D | **Q4.** Aujourd'hui 0,5 à 2 µs ; 0,2 µs suppose des commutateurs intégrés et une alimentation locale. −1 % de PAE sur une impulsion de 20 µs. |
| `pae_duty_slope` | Perte relative de PAE par unité de facteur de forme au-dessus de la référence (échauffement du canal). | 0,05 | D | **Q2 bis.** Faible grâce au diamant et aux microcanaux : −2,75 % à d = 0,65. |
| `e_buf_trm_j` | Réserve d'énergie par TRM (banc de condensateurs local qui fournit l'impulsion). | 25 J | D | **Q7.** Ancienne valeur 5 J, sans source. Choisie pour permettre des impulsions de 2 ms en veille très longue portée, où une zone aveugle de 300 km est sans importance et où le temps d'observation dépend du nombre d'impulsions à la PRI longue. 5 cm³ à 5 J/cm³ ; masse des réserves en point ouvert. |
| `buffer_max_fraction` | Fraction maximale de la réserve qu'une impulsion peut consommer. | 0,2 | D | **Q7.** La puissance du PA suit l'énergie stockée : l'impulsion la plus longue finit à −0,97 dB (perte moyenne 0,46 dB). L'ancienne valeur 0,5 donnait −3 dB, trop pour un radar cohérent. Fixe τ_max. |
| `bi_max_mhz` | Plafond de bande instantanée. | 200 MHz | V | **Confirmé à l'étape 1.4 (C1)** : toute la bande S. Avec le retard vrai numérique, la limite vient des convertisseurs et filtres des TRM (débit des CAN : étape 4.4). |
| `true_time_delay` | Mode de dépointage. | "digital_per_trm" | V | **Étape 1.4, D1.** Retard vrai numérique dans chaque TRM : pas de dépointage avec la fréquence, pas de limite de remplissage de l'ouverture. L'autre valeur possible, "none" (dépointage par la phase), limiterait la bande du signal à c / (D_env sin θ), environ 75 MHz à 75°. |
| `duty_max` | Plafond du facteur de forme d'un TRM, sur les trains d'impulsions. | 0,65 | D | **Q5.** Contrôlé à la création de la forme d'onde. Avec le GaN sur diamant et les microcanaux, l'échauffement du canal n'est plus limitant ; l'optimiseur ne s'en sert que si c'est utile. L'éclipse (récepteur sourd 65 % du temps) n'est modélisée qu'à l'étape 2.1 : les choix à fort facteur de forme sont optimistes d'ici là. |

Retirés : `eta_illum_rx` (0,88), désormais **dérivé** de la pondération du volume (§6), `t_sys_k`, désormais **dérivé** de la chaîne de réception, et `pol_loss_dual_db` (0,5 dB), remplacé à l'étape 1.5 par la perte calculée du détecteur GLRT (§7.8).

#### `[band.S.losses_db]` : pertes fixes en S (sommées)

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `finite_precision` | Précision finie du traitement numérique. | 0,3 dB | V | **Étape 2.1, Q5.** Seule part de l'ancien `DSP` = 2,0 dB qui reste une entrée. La part pondération et désadaptation (≈ 1,0 dB) est désormais **calculée** par observation (perte `compression`, §5.18). |
| `calibration` | Résidu d'étalonnage. | 0,5 dB | V | Confirmé. |
| `Ku_blockage` | Masquage de l'ouverture S par le réseau Ku. | 0,2 dB | V | TRSA bi-bande. |

Une perte s'ajoute : **TX_chain**, dérivée de `[band.S.tx]`, soit 0,6 dB. Le chemin de l'élément au LNA n'est plus une perte de signal : son atténuation **et** son bruit sont dans T_sys. Avant la revue 0.3, ils étaient comptés deux fois.

#### `[band.S.rx]` : chaîne de réception S (Friis, ramenée au port d'antenne)

Choix « frontal optimisé », verrouillé avec la **valeur médiane** de chaque fourchette estimée lors de la revue 0.3 (154,3 K). Le bas des fourchettes donnerait environ 95 K. L'ancienne valeur, 346,5 K, correspondait à un module T/R classique.

**Étape 1.5** (bout optimiste de chaque plage) : `t_ant_k` devient le **ciel froid**, le ciel devant la face s'y ajoute selon la direction (§5.16) ; le facteur de bruit du CAN est **dérivé** du convertisseur (20,0 dB au lieu des 27,5 dB saisis) ; la dynamique de la chaîne est décrite. Résultat : **T_sys = 144,5 K** (ciel froid).

| Variable | Définition | Valeur | Statut | Fourchette (médiane retenue) |
|---|---|---|---|---|
| `t_ant_k` | Température d'antenne du **ciel froid** : fond cosmologique 2,73 K et minimum galactique d'environ 0,5 K à haute latitude. | 3,2 K | V | **Étape 1.5, A1.** Ancienne valeur 10 K (fond cosmologique et médiane galactique). Le plan galactique et les corps devant la face s'ajoutent par direction (§5.16). |
| `t_phys_frontend_k` | Température physique du frontal. | 320 K | V | 290 à 350 K (dissipation des TRM, étape 4.2). |
| `circulator_loss_db` | Circulateur ou commutateur T/R. **Partagé avec l'émission** : le même composant est traversé deux fois. | 0,3 dB | V | 0,2 à 0,4 dB. |
| `limiter_loss_db` | Limiteur qui protège le LNA de la fuite de 2 kW. | 0,35 dB | V | 0,2 à 0,5 dB. |
| `pre_lna_line_db` | Filtre et lignes avant le LNA. | 0,2 dB | V | 0,1 à 0,3 dB. |
| `lna_nf_db` | Facteur de bruit du LNA. | 0,75 dB | V | 0,5 à 1,0 dB. |
| `lna_gain_db` | Gain du LNA. | 27,5 dB | V | 25 à 30 dB. |
| `post_lna_nf_db` | Facteur de bruit de la chaîne analogique après le LNA. | 8,0 dB | V | 6 à 10 dB. |
| `post_lna_gain_db` | Gain de la chaîne après le LNA. | 20 dB | V | 15 à 25 dB. |
| `digital_noise_db` | Bruit du traitement numérique (précision finie). | 0,05 dB | V | Moins de 0,1 dB. |
| `adc_enob` | Nombre de bits effectifs du CAN (I/Q en bande de base, un par TRM et par voie de polarisation). | 13 bits | V | **Étape 1.5, B2.** Bout optimiste de 11 à 13 bits ; aujourd'hui environ 12,3 bits à 250 Méch/s. Remplace `adc_nf_db` = 27,5 dB, qui devient **dérivé** (§6). |
| `adc_fullscale_dbm` | Pleine échelle du CAN (sinus). | +10 dBm | V | **B2.** Bout optimiste de 0 à +10 dBm. |
| `adc_oversampling` | Rapport entre la cadence complexe et la bande instantanée. | 1,25 | V | **B1.** fs = 1,25 × 200 MHz = 250 Méch/s complexes (marge du filtre anti-repliement). |
| `clip_backoff_db` | Marge des crêtes (signal + bruit) sous la pleine échelle. | 3 dB | V | **B3.** Niveau d'écrêtage = P_FS − 3 dB. |
| `lna_p1db_in_dbm` | Compression à 1 dB du LNA, en entrée. | +10 dBm | V | **B4.** LNA GaN robuste ; sert au blocage par les fuites hors canal (§5.17). |
| `post_lna_op1db_dbm` | Compression à 1 dB de la chaîne après le LNA, en sortie. | +15 dBm | V | **B4.** Contrôle au chargement : au moins P_FS + `clip_backoff_db` (le CAN écrête le premier). |
| `limiter_recovery_s` | Récupération du limiteur après chaque impulsion. | 50 ns | V | **B5**, bout optimiste (limiteurs PIN actuels 0,1 à 1 µs). Récepteur sourd pendant τ + récupération : sortie pour l'éclipsage (étape 2.1). |
| `preselect_rej_db` | Réjection de l'autre bande par le filtre avant le LNA. | 80 dB | V | **Sujet C**, bout optimiste de 40 à 80 dB. |
| `post_rej_db` | Réjection de l'autre bande avant le CAN (conversion, anti-repliement). | 80 dB | V | **Sujet C**, bout optimiste de 40 à 80 dB. |

#### `[band.S.tx]` : chaîne d'émission S (sortie du PA vers l'élément)

TX_chain = filtre d'harmoniques + circulateur (traversée en émission) + coupleur d'étalonnage + ligne = 0,15 + 0,3 + 0,05 + 0,1 = **0,6 dB**. Ce sont les médianes retenues lors de la revue 0.3.

| Variable | Définition | Valeur | Statut |
|---|---|---|---|
| `harmonic_filter_db` | Filtre d'harmoniques après le PA. | 0,15 dB | V |
| `cal_coupler_db` | Coupleur d'étalonnage. | 0,05 dB | V |
| `line_to_element_db` | Ligne et transition vers l'élément rayonnant. | 0,1 dB | V |
| `harmonic_in_ku_dbc` | Niveau de l'harmonique du PA S qui tombe dans la bande Ku (rang **dérivé** des deux bandes : 5 aujourd'hui, 5 × 2,7-2,9 = 13,5-14,5 GHz). | −45 dBc | V |
| `harmonic_filter_rej_in_ku_db` | Réjection du filtre d'harmoniques S à cette harmonique. | 80 dB | V |

Les deux dernières entrées viennent de l'étape 1.5 (sujet C, bout optimiste : −25 à −45 dBc et 50 à 80 dB).

### 5.12 `[band.Ku]` : bande Ku (confirmation et poursuite)

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `usable_lo_ghz` / `usable_hi_ghz` | Bande utile. | 13,5 à 14,5 GHz | V | Confirmé lors de la revue 0.3. |
| `core_lo_ghz` / `core_hi_ghz` | Bande centrale. | 13,7 à 14,3 GHz | D | **Étape 1.4**, comme en S (bande relative de 7 %). |
| `edge_derating_db` | Perte totale aux bords, moyennée sur le spectre occupé. | 0,5 dB | D | **Étape 1.4**, comme en S. |
| `scan_max_deg` | Limite de balayage jusqu'à `scan_break_ghz`. | 75° | V | Confirmé. |
| `scan_break_ghz` | Seuil de fréquence de la limite. | 14,25 GHz | V | Cinq fois le seuil S. |
| `scan_max_hi_deg` | Limite au-dessus du seuil. | 70° | V | Gardée à l'étape 1.4, comme en S. |
| `p_pk_trm_w` | Puissance crête par TRM Ku, en sortie de PA. | 75 W | V | Même convention qu'en S. |
| `pae_sat` | PAE en saturation au facteur de forme de référence. | 0,65 | D | **Étape 1.3, Q3.** Le GaN actuel en Ku atteint environ 30 à 45 %, le laboratoire environ 50 à 55 % ; 0,65 est le choix de l'utilisateur pour 2050-2060. |
| `pae_t_edge_s` | Temps de front par impulsion. | 0,1 µs | D | **Q4.** Puces plus petites qu'en S. |
| `pae_duty_slope` | Perte relative de PAE par unité de facteur de forme. | 0,08 | D | **Q2 bis.** Densité de puissance plus forte qu'en S : −4,4 % à d = 0,65. |
| `eta_illum_tx` | Illumination en émission. | 1,0 | V | Uniforme comme en S. L'ancienne valeur 0,88 de V5.6.0 réduisait le gain mais pas la puissance rayonnée. Le contrôle des lobes passe par la pondération de réception de chaque volume. Portée Ku +3,25 %. |
| `allowed_roles` | Rôles permis en Ku. | ["CONFIRM", "PROMOTE", "TRACK"] | D | Politique du logiciel radar (étapes 3.x et 4.3). Les rôles TWS et PROMOTE existent mais aucune forme d'onde ne les utilise. |
| `pol_rx_dual` | Réception bipolarisation. | vrai | V | Comme en S. |
| `waim_match_deg` | Angle d'adaptation de la couche WAIM. | 45° | V | Le même qu'en S (étape 1.2). |
| `pol_track_est_loss_db` | Perte de polarisation d'une observation de poursuite. | 0,05 dB | V | **Étape 1.5**, comme en S : polarisation connue, résidu d'estimation. Les observations de confirmation ont la perte calculée du détecteur GLRT (§7.8). Remplace `pol_loss_dual_db` = 0,5 dB. |
| `pol_loss_single_db` | Réception monopolarisation. | 3,0 dB | V | Comme en S. |
| `e_buf_trm_j` | Réserve d'énergie par TRM Ku. | 1,2 J | D | **Q7.** Elle n'était pas modélisée. τ_max = 0,2 × 1,2 × 0,65 / 75 W = 2,08 ms, même raison qu'en S. |
| `buffer_max_fraction` | Fraction maximale par impulsion. | 0,2 | D | Comme en S. |
| `bi_max_mhz` | Bande instantanée. | 200 MHz | V | Plafond matériel. Retard vrai numérique par TRM. |
| `true_time_delay` | Mode de dépointage. | "digital_per_trm" | V | Comme en S (étape 1.4). |
| `duty_max` | Plafond du facteur de forme sur les trains. | 0,65 | D | Comme en S (ancienne valeur 0,25) ; borne aussi l'espace de recherche des observations Ku. |

L'ouverture Ku est **partagée** avec la bande S : mêmes TRSA bi-bandes et même octogone. C'est une conséquence directe de la composition des TRSA, validée à l'étape 1.1.

#### `[band.Ku.losses_db]`

Mêmes postes qu'en S, avec des valeurs crédibles en Ku (revue 0.3). Ils remplacent les **8 dB forfaitaires** de V5.6.0, qui ne couvraient que le traitement.

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `finite_precision` | Précision finie. | 0,3 dB | D | Comme en S (étape 2.1). |
| `calibration` | Résidu d'étalonnage. | 0,8 dB | D | Plus élevé qu'en S : à tolérances mécaniques égales, l'erreur de phase est 5 fois plus grande en Ku, et il y a 25 fois plus d'éléments. |
| `S_blockage` | Diffusion par les éléments S intercalés dans la TRSA. | 0,3 dB | D | Symétrique de `Ku_blockage` en S. |

TX_chain Ku dérivée = 0,2 + 0,45 + 0,1 + 0,2 = **0,95 dB**.

#### `[band.Ku.rx]` : médianes (statut D, estimations à confirmer)

**T_sys = 238,7 K** à la revue 0.3, contre 400 K saisis en bloc auparavant. Le bas des fourchettes donnerait environ 138 K, le haut environ 387 K. **Étape 1.5** : ciel froid et CAN dérivé, comme en S : **T_sys = 230,8 K**.

| Variable | Valeur | Fourchette |
|---|---|---|
| `t_ant_k` | **2,8 K** (V, étape 1.5 ; était 5 K) | ciel froid : fond cosmologique, galactique négligeable |
| `t_phys_frontend_k` | 320 K | 290 à 350 K |
| `circulator_loss_db` | 0,45 dB | 0,3 à 0,6 dB |
| `limiter_loss_db` | 0,45 dB | 0,3 à 0,6 dB (émission de 75 W) |
| `pre_lna_line_db` | 0,3 dB | 0,2 à 0,4 dB |
| `lna_nf_db` | 1,15 dB | 0,8 à 1,5 dB |
| `lna_gain_db` | 25 dB | 20 à 30 dB |
| `post_lna_nf_db` | 8,0 dB | 6 à 10 dB (conversion vers le bas) |
| `post_lna_gain_db` | 20 dB | 15 à 25 dB |
| `digital_noise_db` | 0,05 dB | moins de 0,1 dB |
| `adc_enob`, `adc_fullscale_dbm`, `adc_oversampling`, `clip_backoff_db` | 13 bits, +10 dBm, 1,25, 3 dB (V, étape 1.5) | comme en S ; `adc_nf_db` (27,5 dB) devient dérivé : 20,0 dB |
| `lna_p1db_in_dbm`, `post_lna_op1db_dbm` | +5 dBm, +15 dBm (V) | LNA Ku plus petit qu'en S |
| `limiter_recovery_s` | 25 ns (V) | bout optimiste, composants plus petits qu'en S |
| `preselect_rej_db`, `post_rej_db` | 80 dB, 80 dB (V) | réjection de la bande S |

#### `[band.Ku.tx]` : médianes (V)

| Variable | Valeur |
|---|---|
| `harmonic_filter_db` | 0,2 dB |
| `cal_coupler_db` | 0,1 dB |
| `line_to_element_db` | 0,2 dB |

### 5.13 `[targets.*]` : bibliothèque de cibles (étape 2.4)

Une table par **classe** de cible. Ce sont des **valeurs moyennes de classe**, pas la fiche d'une arme précise : l'armement de l'adversaire n'est pas connu, et toutes les valeurs d'une plage plausible sont possibles (décision de l'utilisateur). Pour une menace, la valeur retenue est le milieu de la plage ; le bout optimiste reste réservé à nos technologies. Un secteur désigne une classe (clé `target_type`) et peut surcharger n'importe quelle valeur (§9.2). **Une classe s'ajoute ou se modifie dans le fichier seul**, sans toucher au code ; le chargeur refuse une classe incomplète avec un message clair.

Schéma d'une classe :
- `[targets.<NOM>]` : `v_close_mps`, `length_m` (L), `rotation_dps` (ω), `accel_max_mps2` (accélération sur la ligne de visée) ; facultatives : `optics_diameter_m`, `optics_f_ratio`, `optics_retro_eff` (absentes = pas d'optique) ;
- `[targets.<NOM>.band.S]` et `[targets.<NOM>.band.Ku]` : `rcs_m2` (SER **copolaire**, selon e_v, **frontale** : l'aspect d'une cible qui se rapproche), `fluctuation` ("SW0" stable, "SW1/2" nombreux réflecteurs, "SW3/4" un réflecteur dominant), `depol_db` (SER croisée / copolaire).

Rôles : **ST** est la cible conventionnelle de **référence** (banc WF-LRS, self-test, rapport en ligne de commande), pas une classe de menace. **GENERIC** complète un secteur qui ne désigne aucune classe (`tool.targets.default_type`).

| Classe | v (km/s) | L (m) | ω (°/s) | a_max | SER frontale S / Ku | Fluctuation | Dépol. | T₀ S / Ku |
|---|---|---|---|---|---|---|---|---|
| ST (référence) | 5 | 10 | 0,1 | 0 | 1 000 / 1 000 m² | SW1/2 | −10 dB | 3,1 / 0,6 s |
| GENERIC | 10 (toujours surchargée) | 3 | 1 | 1 g | 1 / 1 m² (toujours surchargée) | SW1/2 | −10 dB | 1,0 / 0,20 s |
| KKV | 25 | 1,5 | 1 | 10 g | corps 0,135 + optique, soit **1,27 / 13,8 m²** | SW3/4 | −20 dB | 2,0 / 0,41 s |
| SHIP | 5 | 100 | 0,1 | 1 g | **300 / 300 m²** (frontale ; 3 000 m² en latéral) | SW1/2 | −6 dB | 0,31 / 0,06 s |
| DEBRIS | 15 | 0,5 | 10 (culbute) | 0 | 0,01 / 0,01 m² | SW1/2 | −3 dB | 0,61 / 0,12 s |
| DECOY | 25 | 1,5 | 0,5 | 0 | 0,135 / 0,135 m² | SW0 (ballon) | −20 dB | 4,1 / 0,82 s |

Toutes ces valeurs sont V (étape 2.4). Justifications :
- **KKV** : la face avant est un **autodirecteur IR à miroir de grand diamètre** (concept proche du KKV de l'intercepteur GBI), métallique donc conducteur parfait aux fréquences radar, et pointé vers sa cible. Composante optique (moyennes de classe) : D = 0,3 m (milieu de 0,2 à 0,4), f/D = 1 (0,8 à 1,5), fraction rétroréfléchie par l'ensemble focal η_r = 0,1 (moyenne logarithmique de 0,01 à 1), sans contre-mesure RF. Une contre-mesure, ou un leurre qui imite l'optique, se décrit par une surcharge de secteur. Le réflecteur dominant donne SW3/4 dans les deux bandes ; sa réflexion spéculaire garde la polarisation (−20 dB). En S, le miroir ne mesure que 2 à 4 λ (zone de résonance) : c'est un ordre de grandeur.
- **SHIP** : SER frontale de 300 m² (décision de l'utilisateur).
- **DECOY** : imite le corps du KKV sans son retour optique ; stable et peu dépolarisant, ce qui servira à la discrimination (étape 3.3).
- **ST** : mêmes SER et famille dans les deux bandes, pour la continuité du banc de référence.

### 5.14 `[confirmation]` et `[tracking]`

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `confirmation.ku_freq_ghz` | Fréquence Ku de confirmation et de poursuite. | 14,0 GHz | D | **Provisoire.** Elle est dans la bande centrale, sous le seuil de 14,25 GHz. « L'optimiseur **doit** choisir une fréquence » (étape 3.2), en temps réel, d'après le spectre non brouillé. |
| `confirmation.bias_mrad` | Biais résiduel d'alignement S/Ku par axe, exprimé en **angle absolu**. | 1,1 mrad | V | Il valait 1/20 du lobe S (1,13 mrad à 2,8 GHz) et aurait grandi avec la largeur de lobe. Il est désormais constant en (u, v). Étalonnage S/Ku à l'étape 3.2. |
| `confirmation.p_min` | Probabilité minimale de confirmer une vraie cible. | 0,98 | V | Exigence opérationnelle : 0,9 × 0,98 par passage de bout en bout. Elle deviendra un réglage de mission à l'étape 5.1 ; un réglage par volume reste une option à creuser. |
| `confirmation.fa_r4_mean` | Moyenne de (R/R_det)⁴ pour une fausse alarme. | 0,2 | V | Exactement 1/5 pour une fausse alarme répartie uniformément en distance sur [0, R_det]. |
| `tracking.angle_zone_sigmas` | Ouverture de la zone angulaire, en sigmas. | 4 | V | Convention : largeur totale à ±2σ. |

Supprimée lors de la revue 0.3 : `latency_max_frac`, la règle de latence. Elle ne limitait jamais rien (2nv/c) et ignorait les vraies causes. La latence sera **reconstruite de zéro à l'étape 4.3** : attente dans la file de l'ordonnanceur, traitement et allers-retours, comparés à une exigence de délai par secteur.

### 5.15 `[ship]` : structure du vaisseau (étape 2.5 ; modèle 3D depuis le 22 octobre 2026)

**Référence : le modèle 3D basse résolution `assets/models/LP_NISO.obj`** (Wavefront OBJ exporté de Blender, en mètres, 62 098 triangles), fourni par l'utilisateur le 22 octobre 2026. Il sert à **tous** les calculs qui dépendent de la géométrie du vaisseau : masquage, bruit de la structure, position des éléments, vue 3D. Il remplace les primitives tirées du dessin à l'étape 2.5. Le code place le modèle dans le repère du vaisseau : Z est l'axe (+Z vers la proue), **z = 0 est la station des ouvertures**, r est la distance à l'axe. Dans le modèle, la proue est vers −Z : le repère du vaisseau est le modèle tourné de 180° autour de X. Pour un nouveau modèle, il suffit de remplacer le fichier et de mettre à jour les entrées ci-dessous.

Le chargeur contrôle, et refuse avec un message clair :
- la longueur, l'envergure et le diamètre de la coque du modèle, contre les cotes verrouillées (à 1 % près) ;
- pour chaque face, que le centre de l'ouverture est sur la face avant d'un panneau radar du modèle, et que la normale de ce panneau est la normale de la face à `geometry.tilt_deg` (à 0,5° près). Un changement d'inclinaison ou de position des antennes qui ne correspond plus au modèle est donc refusé.

| Entrée | Contenu | Valeur | Statut |
|---|---|---|---|
| `model_path` | Fichier du modèle 3D, relatif au dossier de l'application | `assets/models/LP_NISO.obj` | V |
| `model_nose_axis` | Sens de la proue dans le modèle | "−Z" | V |
| `model_array_station_z_m` | z du modèle des centres des faces avant des quatre panneaux radar (le z = 0 du repère du vaisseau) | −59,298 m | V |
| `length_m` | Longueur hors tout, contrôlée contre le modèle | **149,46 m** (136,4 m avant) | V |
| `width_m` | Envergure hors tout (bouts des radiateurs, ±37,34 m), contrôlée contre le modèle | **74,68 m** (76,5 m avant) | V |
| `hull_diameter_m` | Plus grand diamètre du corps de la coque (la pièce du modèle la plus longue selon l'axe, r 8,612 m vers l'arrière), contrôlé contre le modèle | **17,22 m** (16,8 m avant) | V |
| `array_radius_m` | Distance à l'axe du centre des ouvertures (face avant des panneaux) | **5,248 m** (4,926 m avant) | V |
| `block_mask_db` | Seuil de masquage : un pointage est retiré du balayage quand la structure coûte plus que cela par passage | 3 dB (f ≈ 0,29) | V |
| `default_material` | Matériau des triangles hors des régions | "hull" | V |
| `[ship.material_regions]` | Le fichier OBJ ne porte pas de matériau : un triangle dont le centre de gravité est dans une région (bornes facultatives `abs_x_min_m`, `abs_x_max_m`, `abs_y_min_m`, `abs_y_max_m`, `z_min_m`, `z_max_m`, repère du vaisseau) prend son matériau ; la première région qui convient l'emporte | `radiator` : \|x\| ≥ 9,6 m, \|y\| ≤ 0,3 m, z de −116,5 à −66,0 m (ailes de radiateurs au-delà des rails) | V (confirmé par l'utilisateur le 22 octobre 2026) |
| `[ship.materials.hull]` | Émissivité RF [S, Ku] et température physique | 0,01 / 0,01 ; 290 K | V (B1, B2 : bout optimiste) |
| `[ship.materials.radiator]` | Idem | 0,05 / 0,05 ; **750 K** (médiane d'un vaisseau nucléaire, utilisateur) | V |

Éléments du modèle, dans le repère du vaisseau :
- **Panneaux radar** ×4 (0, 90, 180, 270°) : octogones de 5,17 m aux sommets (4,91 m aux plats), 0,75 m d'épaisseur, inclinés de **25,00°** vers la proue (confirmé par le concepteur de la géométrie). L'ouverture modélisée (enveloppe de 4,17 m tirée de la composition des TRSA) est centrée sur leur face avant.
- **Coque** : pointe de la tourelle de nez à z +15,4 m ; autour des antennes, cône de pente 11,35° (r 4,3 à 6,1 m), sur lequel les panneaux à 25° sont posés en coin ; cylindre principal r 8,42 m de z −54,3 à −121,6 m environ ; plus grand rayon 8,61 m ; cône moteur jusqu'à z −134,0 m.
- **Ailes de radiateurs** ×2 (±X, plan XZ), 4 étages de z −66,6 à −115,7 m, bouts à 30,40, **37,34**, 29,27 et 21,21 m.
- **Tourelles latérales** ×4 (0, 90, 180, 270°) vers z −61,9 m, dômes jusqu'à r 16,66 m ; mâts à 45° ×4 vers z −96,6 à −105,9 m, jusqu'à r 20,4 m.

**Lancer de rayons.** Exact sur les triangles : un rayon issu d'un point touche un triangle quand sa direction est dans le cône que le triangle sous-tend depuis ce point (les trois produits mixtes ont le signe du triangle). Seuls les triangles devant le plan de l'ouverture d'une face sont testés (environ 1 100). Tables reconstruites en environ 23 s.

**Constat (inchangé avec le modèle).** Avec l'inclinaison de 25° vers l'avant, la structure ne bloque que des directions à plus de 71,7° de la normale d'une face, toutes attribuées à une autre face qui les voit avec un dépointage plus faible : **aucun pointage n'est masqué**, et **les ailes de radiateurs ne sont jamais vues** par les faces. Le bruit de la structure se réduit à celui de la coque (ε·T = 2,9 K, par les lobes lointains : négligeable). Le calcul suit le modèle : un autre modèle peut changer ce constat.

### 5.16 `[sky]` et `[sky.tb_k]` : bruit du ciel (étape 1.5)

T_ant(direction) = ciel froid (`t_ant_k`, dans T_sys) + hausse galactique + somme des corps devant la face. Le ciel est une **entrée de la mission** (clé globale `sky`, §9.1), dans le repère du vaisseau ; le simulateur global de scénario la remplira plus tard. Le modèle (`core/sky.py`) est décrit au §7.8.

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `blind_rise_db` | Hausse de T_sys au-delà de laquelle un pointage est **aveuglé** : il n'est pas balayé, et il est signalé avec le corps responsable. | 10 dB | V | **A5.** Sépare le Soleil dans le lobe (≈ 17 dB en S comme en Ku) de la Terre (≈ 2,5 à 4 dB en S) et de la Lune (au plus ≈ 2,7 dB en Ku). |
| `noise_element_pattern` | Diagramme d'élément de l'intégrale du bruit. | "cos_front" | D | **A4.** cos θ (cohérent avec la loi de gain), rien derrière la face (plan de masse). Premier pas sur le point ouvert « élément isotrope ». |
| `gal_ref_ghz` | Fréquence des trois températures galactiques. | 2,8 GHz | V | Mise à l'échelle par (f / f_réf)^−indice. |
| `gal_t_high_k` | Fond galactique à haute latitude. | 0,5 K | V | **A2.** Synchrotron du relevé à 408 MHz × (f / 0,408)^−2,7. Déjà compris dans `t_ant_k` : la hausse est T_gal − cette valeur. |
| `gal_t_plane_k` | Plan galactique loin du centre. | 2 K | V | **A2.** |
| `gal_t_centre_k` | Vers le centre galactique, dilué dans un lobe de 1,5°. | 8 K | V | **A2.** C'est aussi le **pire cas** appliqué partout quand la mission ne donne pas de repère galactique (signalé). |
| `gal_sigma_b_deg` | Épaisseur du plan. | 3° | V | T_gal = T_haut + (T_plan − T_haut) e^−(b/σb)² + (T_centre − T_plan) e^−(b/σb)² e^−(l/σl)². |
| `gal_sigma_l_deg` | Étendue du renflement central en longitude. | 30° | V | |
| `gal_spectral_index` | Indice spectral du fond galactique. | 2,5 | V | Entre le libre-libre (2,1) et le synchrotron (2,7) : Ku = S × 0,018. |

Table des températures de brillance `[sky.tb_k]`, en [S, Ku] (A3, statut V). Un corps de la mission peut la surcharger (clé `tb_k` : un nombre, ou {"S": …, "Ku": …}, par exemple pour une éruption solaire scénarisée). Un type absent de la table exige cette surcharge.

| Type | S (2,8 GHz) | Ku (14 GHz) | Justification |
|---|---|---|---|
| `sun` | 1,0e5 K | 1,2e4 K | Disque calme en milieu de cycle (F10.7 ≈ 170 sfu) |
| `sun_quiet` | 4,5e4 K | 1,0e4 K | Minimum solaire (F10.7 ≈ 70) |
| `sun_active` | 1,5e5 K | 1,5e4 K | Maximum solaire (F10.7 ≈ 250) ; les éruptions sont une surcharge du scénario |
| `earth` | 290 K | 290 K | Pire cas (émissivité 1) |
| `earth_ocean` | 110 K | 150 K | Faible émissivité de l'océan en S, atmosphère et nuages en Ku |
| `earth_land` | 270 K | 275 K | Terre émergée |
| `moon` | 230 K | 230 K | Moyenne du disque, environ ±30 K avec la phase |

### 5.16 bis `[sky.surface]` : surfaces diffusantes (étape 2.5)

Par type de corps : rayon et rétrodiffusion à « γ constant », σ₀ = γ cos θᵢ, γ étant l'albédo radar du corps (σ / πR²). Valeurs médianes (environnement, pas notre technologie).

| Type | Rayon | γ S / Ku | Statut |
|---|---|---|---|
| `earth`, `earth_ocean`, `earth_land` | 6 371 km | −12 / −10 dB (une moyenne des mers et des terres) | V |
| `moon` | 1 737,4 km | −11,5 / −11,5 dB (albédo mesuré ≈ 0,07) | V |

Clés nouvelles d'un corps du ciel de la mission : `closing_mps` et `closing_spread_mps` (vitesse de rapprochement de la surface et son étalement, donnés par le scénario) ; sans elles, le fouillis n'est jamais rejeté en Doppler (cas prudent).

### 5.16 ter `[jamming]` et `[jammers.*]` : brouillage (étape 2.5)

| Variable | Valeur | Statut | Justification |
|---|---|---|---|
| `jamming.null_depth_db` | 40 dB | V | Annulation adaptative des brouilleurs dans les lobes secondaires, bout optimiste de 20 à 40 dB (D4) ; pas d'annulation dans le lobe principal. |
| `jamming.max_nulled` | 8 par face | V | Les plus forts d'abord (D4) ; coût de calcul à l'étape 4.4. |
| `jamming.null_min_offset_hpbw` | 1 lobe de réception | D | Au-delà, un brouilleur est dans les lobes secondaires. |

Classes de brouilleurs (**valeurs moyennes de classe**, barrage sur toute la bande utile sauf `cover_lo_ghz` / `cover_hi_ghz`, extensibles dans le fichier seul ; D5, médianes 2050-2060) :

| Classe | Nature | Hypothèse | Densité de PIRE S / Ku |
|---|---|---|---|
| SOJ_SHIP | à distance | 30 kW sur 200 MHz, 30 dBi | 50 / 50 dBW/MHz |
| SOJ_ESCORT | à distance | 3 kW, 30 dBi | 40 / 40 dBW/MHz |
| SPJ_SHIP | autoprotection | 1 kW, 20 dBi | 27 / 27 dBW/MHz |
| SPJ_POD | autoprotection (nacelle du KKV) | 30 W, 15 dBi | 10 / 10 dBW/MHz |
| SPJ_KKV | autoprotection (sur le KKV) | 100 W, 10 dBi | 5 / 5 dBW/MHz |

Un brouilleur à distance est déclaré dans le ciel de la mission (`jammers` : `id`, `class`, `phi_deg`, `eps_deg`, `range_km`, surcharge facultative `erp_dbw_mhz`) ; un brouilleur d'autoprotection est porté par la cible d'un secteur (`jammer_type`, surcharge `jammer_erp_dbw_mhz`).

### 5.17 `[isolation]` : isolation entre bandes et entre faces (étape 1.5)

Valeurs au **bout optimiste** de chaque plage (décision de l'utilisateur). Le modèle (`core/isolation.py`) est décrit au §7.8.

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `interface_mode` | Partage d'une bande par les faces. | "sync" | V | **C, Q1-Q2.** Cadence commune d'émission et de réception de toutes les faces, par bande : aucune face ne reçoit une bande pendant qu'une autre l'émet. Il faudrait environ 143 dB d'isolation pour s'en passer, hors de portée physique. L'annulation active ("cancel") est **à revoir avec l'ordonnanceur** (étape 4.3) ; elle n'est pas implémentée et le chargeur la refuse. |
| `coupling_adjacent_db` | De la puissance d'émission totale d'une face à **un** élément d'une face voisine. | 90 dB | V | Bout optimiste de 60 à 90 dB (normales à 80°, champ proche, lobes lointains, élément rasant). Ordre de grandeur : une simulation électromagnétique serait nécessaire (point ouvert). |
| `coupling_opposite_db` | Idem vers la face opposée. | 120 dB | V | Bout optimiste de 90 à 120 dB (normales à 130°, coque entre les deux). |
| `intra_trsa_s_to_ku_db` | Même TRSA : un élément S vers les éléments Ku voisins (fondamentale et harmonique). | −30 dB | V | Bout optimiste de −15 à −30 dB. |
| `intra_trsa_ku_to_s_db` | Même TRSA : les 25 TRM Ku d'une maille S vers l'élément S. | −30 dB | V | Bout optimiste de −15 à −30 dB. |
| `desens_margin_db` | Une fuite **dans** le canal reçu doit rester à cette distance sous le bruit. | 6 dB | V | Bout optimiste, c'est-à-dire le moins strict, de 6 à 15 dB : hausse du bruit d'environ 1 dB à la limite (0,4 dB avec 10 dB). |
| `lna_block_backoff_db` | Marge sous la compression du LNA pour une fuite hors canal. | 6 dB | V | Bout optimiste de 6 à 15 dB. |

Les autres entrées de l'isolation sont dans les chaînes : `preselect_rej_db` et `post_rej_db` (§5.11, §5.12), `harmonic_in_ku_dbc` et `harmonic_filter_rej_in_ku_db` (`[band.S.tx]`).

### 5.18 `[waveform]` : formes d'onde (étape 2.1)

Valeurs au bout optimiste de chaque plage. Le modèle est décrit au §7.9.

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `modulations` | Familles de modulation modélisées. | ["lfm", "nlfm"] | V | **A1.** La LFM tolère le Doppler ; la NLFM forme le spectre par sa loi de fréquence (lobes bas sans perte de pondération). Les codes de phase sont écartés : ils tolèrent mal le Doppler à 25 km/s (f_d ≈ 470 kHz en S, 2,3 MHz en Ku). |
| `nlfm_doppler_max_frac` | La NLFM est choisie tant que f_d,max / B ne dépasse pas cette valeur ; au-delà, LFM pondérée. | 0,02 | V | **A2-A3**, bout optimiste de 1 à 5 %. À 25 km/s, la NLFM n'est possible qu'au-dessus d'environ 23 MHz en S et 117 MHz en Ku. Le choix est automatique, par observation. |
| `nlfm_mismatch_loss_db` | Perte de désadaptation de la NLFM. | 0,1 dB | V | **A6**, bout optimiste de 0,1 à 0,3 dB. Même élargissement et même bande RMS que la LFM pondérée de même niveau de lobes. |
| `range_psl` | Niveau des lobes en distance. | "volume" | V | **A4.** Le même que l'option de lobes en angle du volume (`rx_sll_db`) : une seule politique de lobes par volume. Un nombre négatif imposerait un niveau fixe. |
| `range_taylor_nbar` | Pondération de Taylor de la LFM, à la réception seule. | 5 | V | **A5.** Même loi que l'ouverture ; les PA restent saturés. Perte 0,41 / 0,68 / 0,93 dB et élargissement ×1,18 / ×1,27 / ×1,34 à −25 / −30 / −35 dB (calculés). |
| `pa_am_pm_deg_per_db` | Conversion AM/PM du PA, résidu après correction. | 2°/dB | V | **B5-B6**, bout optimiste de 2 à 10°/dB. Dérive de phase pendant que la réserve se vide (au plus 0,97 dB), quasi linéaire : donnée en sortie, sans perte. |
| `stagger_n_pri` | Nombre de PRI d'un train ambigu en distance, échelonné. | 3 | V | **C5.** Lève l'ambiguïté et déplace les zones aveugles ; minimum courant pour une décision M sur N. |
| `stagger_spread` | Écart des PRI autour de la PRI nominale. | ±10 % | V | **C5**, plage 5 à 20 %. L'observation est comptée sur la PRI nominale (moyenne). |

Réglage de mission associé : `tool.mission_defaults.range_min_frac` = 0,1 (portée minimale d'intérêt par défaut d'un secteur, en fraction de sa portée, C1).

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
| **Lobe d'émission (uniforme)** | diagramme calculé, plus petite coupe | **1,036 λ/D_env** (axe x 1,041), soit 1,525° à 2,8 GHz et 0,305° à 14 GHz ; 1ᵉʳ lobe secondaire −15,6 dB |
| **Pondération de réception à −25 dB** | Taylor n̄ = 5, conception 29,5 dB | η = 0,873, HPBW 1,173 λ/D (1,73° à 2,8 GHz), niveau atteint −25,0 dB |
| **Pondération de réception à −30 dB** | conception 35,7 dB | η = 0,773, HPBW 1,255 λ/D |
| **Pondération de réception à −35 dB** | conception 41,7 dB | η = 0,693, HPBW 1,325 λ/D |
| x_rms (étendue RMS de l'ouverture) | moyenne de x² sur les éléments | 1,033 m = 0,248 D |
| Précision angulaire × √SNR | 1,13 × borne / HPBW_TX | 0,495 largeur de lobe d'émission, en S comme en Ku |
| Faisceaux de réception par pointage | (1,04·HPBW_TX / (0,5·HPBW_RX))² | 3,38 (−25), 2,95 (−30), 2,64 (−35) |
| Perte de grille d'émission | 1,672 × 1,04² | 1,81 dB |
| Perte de grille de réception | 1,672 × 0,5² | 0,42 dB |
| Gain en S à 2,8 GHz, dans l'axe | 4πAη/λ² | émission 41,62 dBi, réception 41,03 dBi (−25 dB) |
| Gain d'émission en Ku à 14 GHz | idem | 55,60 dBi |
| T_sys (ciel froid) | cascade de Friis | S 144,5 K, Ku 230,8 K (154,3 et 238,7 K avant l'étape 1.5) ; le ciel devant la face s'y ajoute par direction |
| Facteur de bruit du CAN | P_FS − (6,02·ENOB + 1,76) − 10 log(k T₀ fs) | 20,0 dB en S et en Ku (13 bits, +10 dBm, 250 Méch/s). Un bit de plus vaut −6 dB, 6 dB de pleine échelle en plus valent +6 dB : ici ils se compensent |
| Gain jusqu'au CAN | −L_pré + G_LNA + G_post | S 46,65 dB, Ku 43,8 dB |
| Bruit au CAN | k T_sys fs + gain − P_FS | S −56,4 dBFS, Ku −57,2 dBFS (24 dB au-dessus du bruit de quantification) |
| Niveau d'écrêtage à l'élément | P_FS − 3 dB − gain | S −39,6 dBm, soit 53,4 dB au-dessus du bruit ; Ku −36,8 dBm, 54,2 dB |
| Portée minimale non saturée (axe, pleine puissance, sans perte) | écho par élément = écrêtage | S : ST 12,2 km, KKV 1,3 km, SHIP 16,1 km ; Ku : ST 10,2 km, KKV 1,1 km, SHIP 13,5 km |
| Perte de polarisation (GLRT) | SNR_req(n + 1) − SNR_req(n), Shnidman | 0,54 dB pour un échantillon cohérent (valeur exacte 0,42 dB), 0,06 dB pour 24 impulsions non cohérentes ; 0,05 dB en poursuite |
| Soleil dans l'axe (−25 dB) | §7.8 | S 6 670 K (+16,7 dB), Ku 10 000 K (+16,5 dB) : aveuglé |
| Lune dans l'axe | §7.8 | S 15 K (+0,4 dB), Ku 188 K (+2,6 dB) |
| Normalisation de l'intégrale du bruit | corps couvrant tout le demi-espace avant | 1,00 à 1,04 × T_b selon le faisceau |
| Fond galactique au pire cas | T_centre − T_haut | +7,5 K en S (T_sys du bilan 152,1 K, +0,22 dB), +0,13 K en Ku |
| Pondération en distance (Taylor n̄ = 5) | `range_weighting` | −25 / −30 / −35 dB : perte 0,41 / 0,68 / 0,93 dB, élargissement ×1,18 / ×1,27 / ×1,34, bande RMS ×0,855 / ×0,801 / ×0,759 |
| k_r d'une observation | 0,6 / rapport de bande RMS | 0,70 / 0,75 / 0,79 (−25 / −30 / −35 dB) |
| Résolution à 75 MHz | élargissement × c / 2B | 2,37 / 2,53 / 2,68 m ; 2 m demandent 89 / 95 / 100 MHz |
| Bande minimale de la NLFM à 25 km/s | f_d,max / 2 % | 23 MHz en S, 117 MHz en Ku |
| Cases traversées dans l'impulsion | 2 v τ B / c | 25 pour 2 ms, 75 MHz, 25 km/s ; 0,5 pour WF-LRS (200 µs, 5 km/s) |
| Écart de vitesse pour 1 dB | n_c = 1,016 | 13 filtres de vitesse pour 2 ms, 75 MHz, 0 à 25 km/s ; 1 pour WF-LRS |
| Couplage distance-Doppler (LFM) | c f_d τ / 2B | 1,9 km pour 2 ms, 75 MHz, 25 km/s en S (corrigé par la mesure du Doppler) |
| Zone aveugle | c (τ + récupération) / 2 | 3,0 km (20 µs), 15 km (100 µs), 150 km (1 ms), 300 km (2 ms) |
| Éclipsage moyen d'un train échelonné | −10 log(1 − (τ + réc.) / PRI) | 0,46 dB à d = 0,1 ; 7 dB à d = 0,8 |
| TX_chain | somme des composants | S 0,6 dB, Ku 0,95 dB |
| Pertes fixes (hors polarisation) | somme | S 3,80 dB, Ku 4,55 dB |
| Pas moyen des éléments | pas des TRSA / TRM par côté | S 52,1 mm, Ku 10,42 mm (52,5 et 10,9 mm de part et d'autre d'un jeu) |
| Désadaptation aller-retour (2,8 GHz) | 2 × perte par passage | 0,26 dB dans l'axe, 0 à 45°, 1,12 dB à 70°, 2,11 dB à 75° ; 2,15 dB à 2,85 GHz et 75° |
| Lobe lointain le plus fort (lobes de jeu compris) | grille 2D sur l'espace visible | Ku −25,3 dB, S −30,7 dB en uniforme (−16,6 et −26,8 dB avec 2 mm) ; la pondération ne réduit pas les lobes de jeu |
| Dépointage jusqu'où l'option de réception est tenue | `rx_sll_held_up_to_deg` | S : les trois options sur tout le secteur. Ku : −25 dB partout ; −30 / −35 dB jusqu'à 75° / 62° à 14 GHz, 68,5° / 58,5° à 14,25 GHz, 64° / 55° à 14,5 GHz |
| Pureté de polarisation (Ludwig 3, plan à 45°) | (1 + cos θ) / (1 − cos θ) | 15,3 dB à 45°, 6,2 dB à 70° ; infinie dans les plans principaux |
| CHAIN_EFF | psu × distribution × (1 − overhead) | 0,917 |
| Rayon du secteur | plus large limite de balayage | 75° |
| Surface du secteur en (u, v) | disque sin 75° découpé par les plans ±45°, inclinaison 25° | 2,0006 |
| Pointages par face à 2,8 GHz | surface / cellule | 3 016 (4 167 avant l'étape 1.1, 3 050 avec 2 mm de jeu) |
| Plus longue impulsion (τ_max) | fraction × E_réserve × PAE_sat / P_crête − t_front | S 2,0 ms, Ku 2,08 ms (937,5 µs en S avant l'étape 1.3) |
| PAE à d = 0,65, impulsion longue | η_sat (1 − a × 0,55) | S 0,778, Ku 0,621 |
| Puissance électrique disponible pour l'émission, 2 MW | 2 MW × CHAIN_EFF | 1,834 MW par face |
| Dissipation d'un TRM à d = 0,65 | P_crête × d × (1/PAE − 1) | S ≈ 370 W, Ku ≈ 30 W pendant l'observation (exigence pour les microcanaux, étape 4.2) |
| Blocs cohérents | M minimisant SNR_req(M) + perte de cohérence d'un bloc | ex. 32 impulsions à PRI 60 ms : 16 blocs de 2 impulsions (61 ms) ; plus de plafond (149 ms jusqu'à l'étape 2.1) |
| Banc de vitesses (pas à 1 dB) | §7.10 | WF-LRS (24 impulsions, 621 ms) sur 0 à 4 km/s : 637 hypothèses, perte moyenne 0,35 dB ; trains des missions de test : 50 à 535 hypothèses ; impulsion seule : 1 à 2 |
| Chevauchement moyen | grille ×4 | distance 0,05 dB (−25 dB), Doppler 0,07 dB |
| Perte CA-CFAR | 1 024 cellules | 0,04 dB |
| Pertes de WF-LRS sur ST, axe | somme | 5,34 dB (7,00 dB à l'étape 2.1) ; portée 8 325 km (valeur de test) |

Pour l'ensemble de l'octogone, le facteur en λ/D ne dépend pas de la fréquence au voisinage de l'axe. Les bandes S et Ku échantillonnent la même ouverture, et leurs résultats s'accordent à 0,1 % près. Un seul calcul par bande suffit donc (étape 1.1, décision Q7).

---

## 7. Modèles et formules

### 7.1 Équation radar (forme énergétique)

R⁴ = P_faisceau · G_TX · G_RX · λ² · σ · τ · N / ((4π)³ · k · T_sys · L · SNR_req)

- P_faisceau = P_crête de la face / B_TX.
- G = 4π · A_émissive · η · η_ohmic · cos θ / λ². La désadaptation active est comptée à part, dans L (§7.3 bis).
- **η_RX** est celui de la pondération du volume (`rx_sll_db`).
- SNR_req est le SNR sur l'**énergie totale** intégrée, d'après l'ajustement de Shnidman, pour les cas Swerling 0 à 4 et une intégration cohérente ou non cohérente. La formule d'Albersheim de V5.4.0 était mal implémentée et comptait l'intégration deux fois.
- Bande (étape 1.4) : le spectre du signal [f − B/2, f + B/2] doit tenir dans la bande utile, sinon la forme d'onde n'est pas jouable à cette fréquence ; la perte de bord est l'intégrale exacte du cosinus surélevé sur ce spectre, divisée par B ; sans retard vrai, B ≤ c / (D_env sin θ). Au chargement, le modèle vérifie que la bande centrale et la fréquence de rupture sont dans la bande utile.
- Le coût d'une observation est électrique (étape 1.3) : E = P_crête · N · τ / B_TX / PAE(bande, τ, τ/PRI), comparé à P_élec × CHAIN_EFF × période de revisite. La chaleur vaut E_élec − E_RF.
- L regroupe : pertes fixes, TX_chain, bord de bande, **compression** et **éclipsage des trains échelonnés** (étape 2.1), polarisation, affaissement de la réserve (−10 log(1 − x/2)), multifaisceau, **désadaptation active à l'émission et à la réception** (étape 1.2), grille d'émission et grille de réception (veille seulement), cohérence (intégration cohérente), migration.

### 7.1 bis TRM et chaîne d'émission (étape 1.3)

- PAE d'une impulsion : η = η_sat · τ / (τ + t_front) · min(1, 1 − a (d − d_ref)), avec d = τ/PRI d'un train (d_ref pour une impulsion seule).
- Énergie électrique d'une impulsion par TRM : P_crête · τ / η, fronts compris. Elle est prélevée sur la réserve : x = E / E_réserve, perte moyenne −10 log(1 − x/2), refus si x > 0,2.
- τ_max = 0,2 · E_réserve · η_sat / P_crête − t_front.
- Facteur de forme des trains plafonné à 0,65.
- Sorties : PAE, puissance électrique moyenne de l'observation et chaleur par TRM dans `solve()` ; chaleur par face (kW et kW/m²) dans l'optimiseur et le bilan de poursuite.
- Disposition provisoire : la PAE reste constante quand la puissance est réduite (« pas d'énergie en trop », recul des observations Ku) ; c'est optimiste jusqu'à la loi PAE / recul de l'étape 3.1.

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

### 7.8 Réception (étape 1.5)

**Bruit du ciel** (`core/sky.py`). La hausse de la température d'antenne quand les faisceaux de réception d'une face pointent dans une direction vaut ΔT = (1/4π) ∫ T_b G dΩ. Avec G = G₀ · w · g(ρ) (diagramme d'élément cos θ) et dΩ = du dv / w, le facteur w s'élimine :

ΔT = T_b · (A_émissive · η_RX · η_ohmic / D_env²) · ∫ g(ρ) dρ²,

l'intégrale portant sur le corps vu dans le plan (u, v) de la face, moitié avant seulement, avec ρ la distance à l'axe du faisceau en λ/D_env. g est le **diagramme de réception moyenné en azimut** (`core/antenna.py`) : fenêtre proche au pas de 0,01 λ/D, moyennes par anneau au-delà. Par Parseval, (Aη/D²) ∫ g = 1 sur tout le plan : un corps qui couvre tout le demi-espace avant ajoute T_b (vérifié à 1,00 à 1,04 près). Le lobe principal est donc **exact**, et non un lobe gaussien.
- Petits corps (rayon ≤ 2 λ/D, le Soleil, la Lune) : échantillons fins sur le disque, tout le diagramme.
- Grands corps (la Terre) : grille polaire autour de l'axe du faisceau dans la fenêtre proche (points à l'intérieur du corps), anneaux d'échantillons sur le disque au-delà.
- Fond galactique : T_gal(l, b) − T_haut à la direction du faisceau, × (f / 2,8 GHz)^−2,5 ; pire cas constant sans repère galactique.
- T_sys(direction) = T_sys froid + ΔT × 10^(numérique/10).

**Bilan d'un secteur** (`optimizer.sector_sky`). T_sys est évalué tous les 0,5 HPBW_TX sur les pointages balayés, pondérés par leur densité (w dΩ). Un pointage dont la hausse dépasse `blind_rise_db` est **aveuglé** : retiré des pointages et signalé avec le corps qui domine. Le **T_sys du bilan** est le centile (1 − `short_tol`) des autres pointages, soit le pire point à 1 % près ; la règle du « pire point hors aveuglés » seule aurait imposé à tout secteur voyant le Soleil la hausse de la frange, près de 10 dB (décision de l'utilisateur). Toutes les portées du secteur sont multipliées par (T_froid / T_bilan)^(1/4) ; les pointages au-dessus du bilan sont signalés quand ils le dépassent de plus de `tool.alerts.sky_above_budget_db`. Même règle en Ku pour la confirmation et la poursuite du secteur (`tracking.sector_budget`).

**CAN et dynamique.** NF_CAN = P_FS − (6,02 ENOB + 1,76) − 10 log(k T₀ fs), fs = 1,25 × B_inst (I/Q). Niveaux ramenés au port d'antenne : écrêtage = P_FS − 3 dB − gain, bruit = k T_sys fs. Portée minimale non saturée : l'écho crête par élément (face entière en un faisceau, pleine puissance, aucune perte, A_él = pas² cos θ) atteint l'écrêtage ; R ∝ σ^(1/4). La numérisation se fait à l'élément, **avant** la formation de faisceau : une source forte dans n'importe quelle direction avant atteint tous les CAN. Gain fixe (un atténuateur commutable est une option de l'étape 3.3) ; alerte quand une tranche de poursuite descend sous la portée non saturée Ku. Récepteur sourd pendant τ + récupération du limiteur (`rx_deaf_us`, sortie pour l'étape 2.1).

**Polarisation.** Deux voies par TRM. Observation de détection : détecteur GLRT sur la polarisation inconnue de l'écho, estimée sur toute l'observation (une inconnue complexe de plus) : perte = SNR_req(n + 1) − SNR_req(n), n = 1 pour une observation cohérente, n impulsions sinon. Observation de poursuite : polarisation connue, `pol_track_est_loss_db`. Récepteur monopolarisation : `pol_loss_single_db`. La rotation de Faraday ne coûte rien en détection avec deux voies. Le gain de diversité de polarisation face au cas de Swerling reste à l'étape 2.4 : aucun gain n'est compté d'ici là.

**Isolation** (`core/isolation.py`). Pour chaque paire (bande émise, bande reçue, faces : même face, voisines, opposées), fuite vers un élément de réception :
- même face, autre bande : un TRM S + `intra_trsa_s_to_ku_db`, ou les 25 TRM Ku d'une maille S + `intra_trsa_ku_to_s_db` ;
- autre face : puissance crête totale de la face − couplage.
Trois contrôles : blocage du LNA (fuite après présélection si hors bande ≤ P1dB − 6 dB), écrêtage du CAN (fuite après présélection et réjection ≤ écrêtage), désensibilisation (fuite dans le canal ≤ bruit − 6 dB). L'harmonique S dans la bande Ku (rang dérivé des bandes) est dans la bande Ku : niveau = fuite + `harmonic_in_ku_dbc` − `harmonic_filter_rej_in_ku_db`, même couplage supposé à l'harmonique.

| Émission → réception | Même face | Faces voisines | Faces opposées |
|---|---|---|---|
| S → S | éclipsage (étape 2.1) | **interdite** : +9,9 dBm, LNA bloqué (−5 dB) | **interdite** : −20 dBm, CAN écrêté (−20 dB) |
| Ku → Ku | éclipsage | **interdite** (LNA −9 dB) | **interdite** (CAN −16 dB) |
| S → Ku | **règle de fréquence** : harmonique à −92 dBm, 5 dB au-dessus du seuil de désensibilisation | autorisée (+18 dB) | autorisée |
| Ku → S | autorisée (LNA +52 dB) | autorisée | autorisée |

Règles pour l'ordonnanceur (`scheduler_rules`, étape 4.3) : cadence commune d'émission et de réception de toutes les faces, par bande ; pas de réception Ku sur la face pendant une émission S dont l'harmonique [5(f − B/2), 5(f + B/2)] recouvre le canal Ku (ainsi un signal S de 75 MHz à 2,8 GHz occupe 13,81-14,19 GHz et couvre le canal Ku de 14,0 GHz : signalé par secteur). Le modèle ne les impose pas encore.

Estimation grossière du coût de la synchronisation par rapport au modèle « pur » (faces indépendantes), donnée à l'étape 1.5 :
- émissions alignées : perte d'énergie nulle ; coût du partage de τ et de la PRI au même instant, 0 à 1 dB selon la diversité des secteurs ;
- émissions non alignées avec masquage : perte −10 log(1 − Σ_autres d_j), 0,1 à 0,9 dB sur les missions de test, 2,6 dB à pleine puissance sur les 4 faces ;
- ordre de grandeur réaliste 0,5 à 1,5 dB en forte charge, négligeable en faible charge. Aucune perte n'est mise dans le bilan avant l'ordonnanceur (étape 4.3).

Coûts de l'annulation active, si elle est reprise à l'étape 4.3 : perte d'insertion avant le LNA (0,3 à 1 dB, versée dans T_sys en permanence), puissance injectée égale à la fuite (environ 1 W par élément à 70 dB de couplage, environ 1 mW à 100 dB), calcul (à faire dans l'espace des faisceaux), temps d'adaptation à chaque changement de pointage (quelques %), masse et fiabilité ; une solution mixte (synchronisation entre faces voisines, annulation entre faces opposées) avait été proposée.

### 7.9 Formes d'onde (étape 2.1)

**Modulation et compression.** Pour chaque observation : f_d,max = 2 v / λ (vitesse de rapprochement de la cible ou du secteur). Si f_d,max / B ≤ 2 % : NLFM, perte 0,1 dB. Sinon : LFM avec pondération de Taylor n̄ = 5 à la réception, au niveau de lobes du volume ; perte = perte de pondération − 20 log(1 − f_d / B) (perte Doppler sur la crête de la fonction d'ambiguïté). La perte `compression` remplace la part « pondération » de l'ancien forfait `DSP`. k_r = 0,6 / (bande RMS pondérée / bande RMS rectangulaire). Résolution physique = élargissement × c / 2B ; elle sert au choix de la bande du signal (`res_max_m`) et la résolution atteinte est signalée si elle n'est pas tenue. Le biais de couplage distance-Doppler c·f_d·τ / 2B est corrigé par la mesure du Doppler (sortie ; le résidu ira à la poursuite, étape 3.3).

**Effets dans l'impulsion.** Une cible traverse n_c = 2 v τ B / c cases pendant l'impulsion : l'écho est comprimé dans le temps, ce qui donne au filtre adapté à une autre vitesse une erreur de phase quadratique. Perte pour un écart de vitesse Δv : L = −20 log |moyenne sur x ∈ [−½, ½] de exp(j 2π n_c x²)|, n_c calculé avec Δv. Elle vaut 1 dB à n_c = 1,016, d'où le nombre de filtres de vitesse pour 1 dB sur l'étendue des vitesses du secteur (sortie pour le banc de l'étape 2.2 et le calcul de l'étape 4.4). Aucune perte n'est ajoutée au bilan avant l'étape 2.2 : un banc de vitesses est supposé, et le forfait de migration de 1 dB est gardé. Dérive AM/PM et décroissance d'amplitude : sorties seulement (négligeables).

**Chronologie de l'observation.** Zone aveugle c (τ + récupération du limiteur) / 2 ≤ `range_min_km` du secteur (par défaut 10 % de la portée) : sinon la forme d'onde n'est pas jouable pour lui. Train non ambigu : PRI ≥ 2 R / c + τ + récupération, ce qui supprime l'éclipsage par le train lui-même entre `range_min_km` et R. Train ambigu (secteur `allow_ambiguous`, ou observation Ku dont la PRI est sous 2 r / c + τ + récupération) : échelonné sur 3 PRI à ±10 %, perte moyenne −10 log(1 − (τ + réc.) / PRI). Ambiguïté Doppler f_d,max × PRI : sortie, sans perte (pas de fouillis). L'éclipsage par les autres observations et par les autres faces synchronisées relève de l'étape 4.3.

### 7.10 Traitement (étape 2.2)

**Blocs cohérents.** Une forme d'onde COHERENT de N impulsions est découpée en M blocs (M diviseur de N) : M minimise SNR_req(M blocs intégrés de façon non cohérente, Shnidman avec la fluctuation de l'observation depuis l'étape 2.4) + perte de cohérence d'un bloc de durée (N/M − 1)·PRI + τ. NONCOHERENT, ou cas déclaré de Swerling 2 ou 4 : M = N. Perte d'un bloc = cible exp(−(T/T₀)²) (T₀ de la classe et de la bande depuis l'étape 2.4) + 4,34·σ², σ² = bruit de phase de l'oscillateur filtré par l'aller-retour (4 sin²(π f τ_rt), de 1/T à B/2, à la distance d'écoute c·PRI/2) + (4π δ / λ)² de la plateforme. Le plancher Doppler (bruit résiduel dans une case de 1/T, à 100 Hz, 1 kHz et 10 kHz) est donné en sortie. La perte de polarisation (GLRT) prend n_nc = M.

**Banc de vitesses** (mode COMPENSATED). Perte d'un filtre décalé de Δv : loi dans l'impulsion de l'étape 2.1 (n_c = 2 Δv τ B / c) + glissement de m = Δv·T_obs / (c/2B) cases sur l'observation, −20 log(moyenne de sinc(m x)) si l'observation est un seul bloc cohérent, −10 log(moyenne de sinc²(m x)) sinon. Pas : perte de `bank_max_loss_db` (1 dB) à mi-pas ; hypothèses = étendue / pas, sur [v_close_min, v_close_max] du secteur en veille et en confirmation, une seule en poursuite. Bilan : perte moyenne sur le pas + `residual_db` (0,1 dB) si la cible traverse plus de 1,5 case. Mode UNCOMPENSATED : 10 log(min(cases, N)), inchangé.

**Chevauchement et CFAR.** Distance : perte moyenne d'une cible uniforme entre deux échantillons de la grille ×4, avec la réponse comprimée de la pondération de l'observation. Doppler (blocs de plus d'une impulsion) : sinc² sans fenêtre, bourrage ×4. CA-CFAR : seuil α = N (Pfa^(−1/N) − 1), Pd = (1 + α / (N (1 + S)))^(−N), comparé au SNR à bruit connu ln Pfa / ln Pd − 1.

**Fausses alarmes.** Cases indépendantes par position : (R_max − `range_min_km`) / (c/2B) × cases Doppler d'un bloc × hypothèses de vitesse × cellules angulaires (faisceaux de réception par position × 0,5² ≈ 0,85). Les grilles suréchantillonnées ne sont pas comptées (échantillons corrélés). Fausses alarmes par période de revisite = positions × cases × Pfa (1e-8) : elles alimentent la confirmation Ku.

**Constat de l'étape.** Les trains d'impulsions demandent des bancs de 50 à 535 hypothèses de vitesse sur les missions de test : leurs fausses alarmes, donc les observations Ku de confirmation, croissent d'autant. L'optimiseur de veille ne voit pas ce coût quand il choisit ses formes d'onde (il ne minimise que l'énergie et le temps de la veille). Voir §10.2.

### 7.11 Critère de détection (étape 2.3)

**Lois exactes** (`core/detection.py`). Détecteur quadratique sur n échantillons intégrés de façon non cohérente (les blocs cohérents), bruit de puissance 1 par échantillon : sous le bruit seul, la statistique suit une loi Gamma(n, 1), d'où Pfa = P(Poisson(T) ≤ n − 1). Cible stable de SNR total λ : Pd = Σ_k Pois(k ; λ) · P(Poisson(T) ≤ n + k − 1). Cibles fluctuantes : mélange sur la puissance de la cible, loi Gamma de moyenne λ, de forme 1 (Swerling 1), 2 (Swerling 3), 2n (Swerling 4), calculé par convolution sur la grille en dB ; Swerling 2 en forme fermée. Valable pour toute Pfa ; vérifié contre la forme fermée de Swerling 1 et contre Shnidman (écart d'au plus 0,3 dB, l'erreur de l'ajustement).

**Pd moyennée.** Les écarts aléatoires de la cible (cellule hexagonale de la grille d'émission et grille de réception avec les diagrammes réels moyennés en azimut, chevauchement en distance et en Doppler, écart au filtre de vitesse le plus proche) forment une loi discrète de pertes (16 quantiles par écart, sommés comme indépendants). La Pd d'une observation est la moyenne des Pd sur cette loi. Le surcoût en SNR qui en résulte est la perte « positions » du bilan (4,6 dB pour WF-LRS sur une observation, dont 3,4 dB de cellule d'émission en moyenne).

**Pd cumulée de la veille.** Une cible qui se rapproche à v_close_max est vue à R(1 + (k + φ) q), q = v·RR / R, avec une phase φ uniforme ; le SNR d'une observation suit (R / R_k)⁴. P_cum(R) = moyenne sur φ de 1 − Π_k (1 − Pd_k), les observations de Pd inférieure à `cum_pd_floor` étant ignorées. Le SNR requis à R est celui qui donne P_cum = `detection.pd`. Écart au critère d'une observation à R (Swerling 1, n = 1) : +6,0 dB pour q = 1, +0,9 dB pour q = 0,5, −6,8 dB pour q = 0,1, −7,7 dB pour q = 0,075, −11,2 dB pour q = 0,02. La puissance est réduite à « pas d'énergie en trop » avec ce critère (exact à `range_km`). La redondance applique le même critère, avec la cible et la portée de B et la revisite de A.

**Fausses alarmes.** Mode « fa_rate » : Pfa par case = `fa_per_rr` / (positions du secteur × cases indépendantes de la forme d'onde), bornée et arrondie au vingtième de décade ; le nombre de fausses alarmes à confirmer est alors le taux toléré. Les observations Ku gardent `detection.pfa`.

### 7.12 Modèles de cibles (étape 2.4)

**Cible d'un secteur** (`target_for_sector`). Avec `target_type` : les valeurs de la classe, puis celles que donne le secteur (`rcs_m2`, `depol_db` et `fluctuation` s'appliquent aux deux bandes). Sans classe : la SER et le Swerling du secteur (SW1 par défaut, le comportement d'avant), et les autres grandeurs de GENERIC. La vitesse est toujours `v_close_max_mps` du secteur.

**SER frontale.** Corps + optique de l'autodirecteur : σ_opt = max(π (2f)², η_r · 4π A² / λ²), avec f = (f/D)·D et A = π D² / 4. Le premier terme est la réflexion directe sur le primaire concave (optique géométrique, indépendante de λ) ; le second est l'effet « œil de chat » d'un ensemble focal réfléchissant, en 1/λ², donc surtout en Ku. Le lobe du retour, de largeur λ/D (20° en S, 4° en Ku pour 0,3 m), contient la direction du radar pour un KKV qui le vise.

**Temps de cohérence.** T₀ = `t0_k` · λ / (2 L ω) ; infini pour ω = 0. Il remplace le t₀ global provisoire dans la perte des blocs cohérents.

**Fluctuation d'une observation** (forme Gamma k de la puissance de la cible sur l'observation, `detection.Fluct`). Cas déclaré : k = 1 (SW1), 2 (SW3), n (SW2), 2n (SW4), infini (SW0). Sinon : famille × min(n, 1 + T_obs / T₀) échantillons indépendants, avec n = blocs cohérents et T_obs = durée de l'observation (SW1/2 : 1 par échantillon ; SW3/4 : 2 ; SW0 : stable). k est ensuite multiplié par la **diversité de polarisation** (1 + ρ)² / (1 + ρ²), ρ = SER croisée / copolaire, puisque les deux voies de réception fluctuent indépendamment. k est arrondi au centième de décade. Les lois exactes et l'ajustement de Shnidman sont écrits en k. Les tables de confirmation de `core/tracking.py` gardent le cas de Swerling le plus proche (famille ou cas déclaré).

**Dépolarisation.** Le récepteur à deux voies recueille aussi l'énergie croisée de la cible : perte « target_depolarisation » = −10 log(1 + ρ), c'est-à-dire un gain. Avec un récepteur à une seule polarisation : 0, et pas de diversité.

**Accélération** (sur la ligne de visée, a_max de la classe). Perte d'un filtre adapté à une autre accélération : phase quadratique sur un bloc cohérent 2π (a T_b² / λ) x² (même loi que l'écart de vitesse de l'étape 2.1) + courbure en distance laissée par la compensation linéaire, a T² / 8 sur l'observation (loi de glissement de l'étape 2.2). Si a_max coûte au plus `bank_max_loss_db` : pas de banc, et la perte moyenne sur a uniforme dans [0, a_max] va au bilan (« acceleration »). Sinon : banc d'hypothèses d'accélération sur ±a_max, de pas tel que la perte à mi-pas vaille 1 dB, et perte moyenne sur le demi-pas au bilan. Ces hypothèses **multiplient celles de vitesse** pour les fausses alarmes et pour la charge de calcul (étape 4.4). Une observation de poursuite connaît l'accélération par la piste (optimiste jusqu'à l'étape 3.3).

**Masquage** (sortie, la scène n'étant pas connue). Enveloppe de la réponse d'une cible forte, en dB sous son pic, à 1, 2 et 3 cellules ou plus : en distance avec la pondération de l'observation, en Doppler avec la transformée sans fenêtre d'un bloc (−13,3 dB au-delà d'une cellule). Une cible plus faible que sa voisine de plus de ce niveau, à cette distance ou plus loin, peut être masquée. Le plancher de bruit de phase est en sortie depuis l'étape 2.2.

**OS-CFAR** (option `processing.cfar_type` = "OS"). Seuil = T × la k-ième plus petite des N cellules, k = 0,75 N : Pfa(T) = Π_{i<k} (N − i) / (N − i + T), Pd = Pfa(T / (1 + S)), comparé au SNR à bruit connu.

**Alerte** : une revisite inférieure à `tool.alerts.rr_t0_ratio` (3) × T₀ est signalée, car le critère cumulé suppose des observations indépendantes.

### 7.13 Environnement (étape 2.5, `core/environment.py`)

Tous ces termes sont évalués par direction de pointage dans `sector_sky` (bandes S et Ku), sur la même grille que le bruit du ciel. Les bruits s'ajoutent à la hausse de température d'antenne ; la perte de blocage et le fouillis entrent comme **facteurs sur le bruit** : T_eff = T_sys(direction) × L_blocage × facteur de fouillis. Le budget du secteur reste le centile (1 − `short_tol`) de T_eff, la portée suit sa racine quatrième, et un pointage dont T_eff monte de plus de 10 dB est aveuglé, avec sa cause (corps, brouilleur, blocage, fouillis).

**Masquage par la structure.** Les obstacles sont dans le champ proche (limite de Fraunhofer d'environ 325 m en S) : le faisceau est un tube de la largeur de l'ouverture. Des rayons partent de 37 points de l'ouverture (anneaux hexagonaux, `tool.environment.aperture_ray_rings` = 3) dans la direction du pointage ; f = fraction de rayons qui touchent la structure (lancer de rayons exact sur les primitives). Perte par passage −20 log(1 − f), aller-retour le double ; au-delà de `ship.block_mask_db` par passage, le pointage est masqué (retiré du balayage et signalé). Table par face sur une grille (u, v) au pas de 0,01.

**Bruit de la structure.** Lobe principal : f × (ε·T des parties touchées) × part du diagramme de réception dans la zone proche. Lobes lointains : les directions de la structure vues du centre de l'antenne (20 000 directions sur la sphère), pondérées par ε·T, intégrées sur le profil lointain moyenné en azimut, comme un corps du ciel ; tabulé par face sur une grille (u, v) au pas de 0,05. Les réflexions sur la structure (Soleil, Terre) ne sont pas comptées (étape 5.4).

**Surfaces.** Distance au centre D = R / sin(rayon angulaire). Une direction qui tombe dans le disque rencontre la surface à d = D cos δ − √(R² − D² sin² δ) : rien n'existe au-delà (signalé quand d est inférieure à la portée exigée). **Fouillis** dans la case de la cible, à la portée exigée r : l'anneau d'égale distance (cos α = (r² + D² − R²) / 2rD, incidence cos θᵢ = (D² − r² − R²) / 2rR), de surface 2π R r Δr / D par case Δr = c / 2B (B = la plus étroite des bandes admissibles du secteur ; 50 MHz en Ku), échantillonné sur 720 points, vu par le diagramme aller-retour (profils d'émission et de réception, facteur d'élément cos) : C/S = σ_c / σ_cible. Le fouillis étant traité comme un bruit au seuil, S ≥ SNR_req (N + C) donne le facteur 1 / (1 − x), x = (C/S) · SNR_req (SNR d'une observation au cas de Swerling de la cible) ; à x ≥ 1, aucune puissance ne suffit : pointage **limité par le fouillis**, aveuglé. Le fouillis est rejeté si le scénario donne la bande Doppler de la surface et qu'elle ne recouvre pas l'étendue de vitesses du secteur (traitement cohérent supposé). Dans le lobe principal, une cible devant la surface n'a pas de fouillis dans sa case, sauf pour un train ambigu en distance : ces trains sont retirés de l'espace de recherche d'un secteur dont un pointage voit une surface. **CAN** : écho de la surface dans le lobe principal (empreinte du faisceau × σ₀) par élément, avant la formation de faisceau, comparé au niveau d'écrêtage (alerte). Un limbe dans le secteur est signalé (GO-CFAR conseillé).

**Brouilleurs à distance.** Température équivalente T_j = (densité de PIRE / 4πR²) / k × A_eff(direction), A_eff = surface × η de réception × cos (élément) × diagramme de réception (zone proche ou profil lointain). Un brouilleur à plus d'un lobe de réception de l'axe est dans les lobes secondaires : les 8 plus forts par pointage sont annulés de 40 dB. Ordres de grandeur avec SOJ_SHIP à 10 000 km : J/N ≈ +57 dB dans le lobe principal (aveuglé), environ +11 dB par les lobes lointains sans annulation.

**Brouilleur d'autoprotection.** Portée de percée R_bt = √(P G_t σ τ N / (4π · densité de PIRE · SNR_req · L)), avec la forme d'onde principale du secteur et avec la plus énergétique de l'espace de recherche (pleine puissance, axe) ; signalée quand elle est inférieure à la portée exigée.

**Objets non menaçants.** `objects_per_rr` objets réels par revisite, confirmés comme des cibles : leur observation Ku à leur propre portée de détection coûte autant que celle de la menace à r_det (R⁴/σ identique).

**Rotation de Faraday.** Ω = 2 × 2,36·10⁴ · B∥ · TEC / f² (aller-retour, unités SI), à partir du contenu électronique et du champ le long de la ligne de visée donnés par le scénario (`tec_tecu`, `b_parallel_nt`) ; en sortie seulement : le GLRT à deux voies récupère l'énergie.

### 7.14 Repères des volumes et navigation (étape 3.0, `core/frames.py`)

**Architecture.** L'outil est la **couche calculateur** (couche 3 de l'architecture de la feuille de route, étape 3.0) en préparation de mission : il connaît sa navigation (attitude, orbite propre, corps central) et ses seules détections, jamais la vérité des cibles. Il fait suivre les volumes au vaisseau à partir de la navigation : c'est du pilotage. En préparation de mission, la navigation est un instantané saisi par l'opérateur ; plus tard, le message de navigation du simulateur global.

**Passage au repère du vaisseau.** `frames.resolve(mission)` le fait une seule fois, avant tout calcul. Pour chaque secteur hors repère `ship`, la matrice M = A_repère←ICRF · R(q) (vaisseau → ICRF → repère du secteur) est attachée au secteur (`frame_m`), et la trame des directions (0,2°) est tournée par M avant le test de la boîte ou de la calotte : c'est exact, à la trame près. Le ciel inertiel (corps, repère galactique, brouilleurs à distance) est tourné par Rᵀ. Tout le calcul en aval (optimiseur, Ku, ciel, structure, vues 3D) reste dans le repère du vaisseau. Une mission sans clé de repère, ou une matrice identité, est traitée exactement comme avant.

**Attitude.** Quaternion de Hamilton `q_ship_to_icrf` = [w, x, y, z], scalaire en premier : v_ICRF = R(q) v_vaisseau. Saisie dans l'interface en lacet, tangage et roulis (séquence 3-2-1 : R_réf←vaisseau = R_z(lacet) R_y(tangage) R_x(roulis)), par rapport à l'ICRF ou au RTN.

| Repère | Axes (dans l'ICRF) | Remarque |
|---|---|---|
| `ship` | ceux du vaisseau | volumes liés au vaisseau (protection) |
| `inertial` | ICRF : X vers le point vernal, Z vers le pôle céleste | φ = ascension droite, ε = déclinaison |
| `rtn` | X = r / \|r\| ; Z = r × v / \|r × v\| ; Y = Z × X | demande l'orbite |
| `limb` | Z = r / \|r\| (zénith local) ; X = projection de v sur l'horizontale locale ; Y = Z × X | boîte seule ; ε compté à partir du limbe, situé à arccos((R + h_limbe) / \|r\|) sous l'horizontale locale (19,8° à 400 km d'altitude au-dessus de la Terre, limbe à 0 km) ; la bande suit l'altitude ; demande l'orbite |
| `two_dir` | Z = direction principale ; X dans le plan des deux directions, du côté de la seconde ; Y = Z × X | directions en ascension droite et déclinaison ; au moins 1° de l'alignement |

Dans tous les repères, φ est compté autour de Z, de X vers Y, et ε à partir du plan XY, vers +Z.

**Contrôles et alertes.** Navigation : un seul message (étape 5.3 pour l'enveloppe), norme du quaternion à 1e-3 près (normalisé), date ISO 8601, orbite complète (corps de `[sky.surface]`, r hors du corps, plan d'orbite défini). Secteur : repère connu, orbite donnée pour `rtn` et `limb`, vaisseau au-dessus de l'altitude du limbe, directions de `two_dir` données et non alignées. Alerte quand le corps central du ciel s'écarte de plus de 0,5° de la navigation (direction ou rayon apparent).

---

## 8. Paramètres de l'outil (`[tool.*]`)

Ces réglages ne décrivent pas le radar. Ils ont été validés tels quels lors de la revue 0.3, avec des renvois vers les étapes à venir.

| Section | Variable | Valeur | Rôle et renvoi |
|---|---|---|---|
| `tool.cli` | `refresh_s`, `frequency_ghz`, `scan_angle_deg`, `target` | 120 s, 2,8 GHz, 0°, "ST" | Valeurs par défaut de `python core/radar_model.py`. Le rapport donne aussi, pour chaque classe, la SER frontale, la fluctuation, T₀, la dépolarisation et l'accélération (étape 2.4). |
| `tool.optimizer` | `freq_candidates_ghz` | [2,7, 2,8, 2,9] | À remplacer à l'étape 3.1 par une grille fine au pas de **5 ou 10 MHz**, restreinte au spectre utilisable. Le but est d'éviter les brouilleurs étroits de forte puissance tout en gardant plus de cases fréquence qu'un masquage par multiples de 25 MHz. |
| | `tau_s` | 20 à 800 µs, plus la limite de la réserve (2,0 ms depuis l'étape 1.3) | La plus longue impulsion est **calculée** à partir de la réserve d'énergie. |
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
| `tool.detection` | `offset_quantiles` | 16 | **Étape 2.3, B4** : quantiles de chaque écart de position. |
| | `masking_offsets_cells` | [1, 2, 3] | **Étape 2.4, D1-D2** : écarts (en cellules) des niveaux de masquage donnés en sortie. |
| `tool.environment` | `aperture_ray_rings`, `block_grid_step_uv`, `struct_noise_dirs`, `clutter_ring_points` | 3, 0,01, 20 000, 720 | **Étape 2.5** : précision du lancer de rayons, de la table de blocage, de l'intégrale du bruit de la structure et de l'anneau de fouillis. |
| `tool.targets` | `default_type` | "GENERIC" | **Étape 2.4, A3** : classe qui complète un secteur sans `target_type` (sa SER et son Swerling sont gardés). |
| `tool.frames` | `q_norm_tol`, `two_dir_min_sep_deg`, `sky_nav_tol_deg`, `limb_alt_km` | 1e-3, 1°, 0,5°, 0 km | **Étape 3.0, B3, R5, R8, R4** : tolérance de la norme du quaternion (normalisé en deçà, refusé au-delà), écart minimal des deux directions, écart toléré entre le ciel et la navigation (alerte), altitude du limbe par défaut. Hors de l'empreinte des fichiers dérivés. |
| `tool.alerts` | `mismatch_db_per_pass` | 3,0 dB | **Étape 1.2, Q5** : seuil d'alerte de la désadaptation par passage. |
| | `rr_t0_ratio` | 3 | **Étape 2.4, B4** : alerte quand la revisite d'un secteur est inférieure à 3 T₀ (observations corrélées). |
| | `sky_above_budget_db` | 1,0 dB | **Étape 1.5** : les pointages au-dessus du T_sys du bilan ne sont signalés que si le pire le dépasse de plus que cela (sinon le centile signalerait toujours 1 % des pointages). |
| `tool.sky` | `eval_step_hpbw`, `near_r_step_lam_d`, `near_phi_step_deg`, `body_ring_lam_d`, `small_body_lam_d`, `body_rings_max`, `chunk` | 0,5, 0,05, 10°, 2,0, 2,0, 16, 2048 | **Étape 1.5** : pas d'évaluation du T_sys sur un secteur (A5) et précision de l'intégrale du bruit. |
| `tool.display` | `grid_res_deg`, `viz_freq_ghz` | 0,5°, 2,8 GHz | Trame 3D et estimation rapide. |
| `tool.mission_defaults` | voir §9 | | Valeurs d'une nouvelle mission. Depuis l'étape 2.1 : `res_max_m` est une résolution **physique**, et `range_min_frac` = 0,1 donne la portée minimale d'intérêt d'un secteur qui n'en fixe pas. |

---

## 9. Modèle de mission (entrées opérateur)

### 9.1 Paramètres globaux

| Clé | Défaut | Contrôle |
|---|---|---|
| `eps_min_deg` / `eps_max_deg` | −90° / +90° (pas de limite) | Fenêtre d'élévation de la mission, envoyée à l'ordonnanceur. Le plancher physique est d'environ −55°. |
| `face_elec_mw` | 2,0 MW | Doit être strictement positive et ne pas dépasser 8 MW (validation provisoire). |
| **`nav`** | absent (attitude identité, sans orbite) | **Étape 3.0.** Liste de messages de navigation, **un seul** jusqu'à l'étape 5.3 : `label`, `epoch` (ISO 8601, UTC, facultative), `q_ship_to_icrf` [w, x, y, z], orbite facultative (`central_body` de `[sky.surface]`, `r_km` et `v_kms` depuis le centre du corps, axes ICRF), `attitude_ref` (`icrf` ou `rtn`, affichage des angles dans l'interface seulement). §7.14. |
| **`sky`** | absent | **Étape 3.0 :** clé `frame` = `ship` (par défaut) ou `inertial` (ciel en axes ICRF, tourné avec l'attitude). **Étape 2.5 :** aussi `jammers` (brouilleurs à distance) et, par corps, `closing_mps` / `closing_spread_mps`. **Étape 1.5.** Ciel dans le **repère du vaisseau**, rempli plus tard par le simulateur global de scénario : `bodies` (id, type de `[sky.tb_k]`, `phi_deg`, `eps_deg`, `angular_radius_deg`, `tb_k` facultatif) et `galactic` facultatif (directions du pôle et du centre galactiques, à 90° l'une de l'autre). Sans ciel : ciel froid et fond galactique au pire cas. L'interface ne l'édite pas ; elle le conserve. |

### 9.2 Secteur (volume de veille)

| Clé | Défaut | Rôle |
|---|---|---|
| `id`, `shape` | S01…, "box" | Une boîte (φ, ε) ou une calotte (centre, rayon). |
| **`frame`** | absent = `ship` ; **`inertial`** pour un nouveau secteur de l'interface | **Étape 3.0.** Repère du volume : `ship`, `inertial`, `rtn`, `limb` (avec `limb_alt_km`, 0 km par défaut), `two_dir` (avec `frame_p_ra_deg`, `frame_p_dec_deg`, `frame_s_ra_deg`, `frame_s_dec_deg`). La boîte ou la calotte est décrite dans ce repère (§7.14). |
| `phi_min_deg`, `phi_max_deg`, `eps_min_deg`, `eps_max_deg` | 0°, 90°, 0°, 60° | Boîte. φ_min > φ_max est accepté et signifie un passage par 0°. |
| `phi0_deg`, `eps0_deg`, `radius_deg` | 0°, 20°, 15° | Calotte. |
| **`target_type`** | absent | **Étape 2.4.** Classe de la bibliothèque (§5.13). Absente : la SER et le Swerling du secteur, avec les autres valeurs de GENERIC. L'interface ne propose pas encore le choix de la classe (étape 5.1) : elle conserve la clé, montre les valeurs de la classe, et une SER ou un Swerling modifié devient une surcharge. |
| `rcs_m2`, `swerling` | 1000 m², 1 | Menace : SER copolaire frontale (les deux bandes) et cas de Swerling déclaré. Avec une classe, ce sont des surcharges facultatives ; sans `swerling`, la fluctuation se déduit de la classe et de T₀. |
| `fluctuation`, `depol_db`, `length_m`, `rotation_dps`, `accel_max_mps2`, `optics_diameter_m`, `optics_f_ratio`, `optics_retro_eff` | ceux de la classe | **Étape 2.4.** Surcharges facultatives des valeurs de la classe (par exemple un KKV à contre-mesure, ou un leurre qui imite l'optique). |
| `range_km`, `rr_target_s` | 2000 km, 30 s | Portée exigée et période de revisite visée. Depuis l'étape 2.3, la portée est atteinte en Pd **cumulée** (`detection.pd`) sur les observations espacées de v_close_max × revisite. |
| `v_close_min_mps`, `v_close_max_mps` | 1000 et 5000 m/s | La vitesse maximale dimensionne la migration, le Doppler et le temps avant impact. |
| `res_max_m` | 2 m | Résolution en distance demandée, **physique** depuis l'étape 2.1 (élargissement de la pondération compris : 2 m demandent 89 MHz à −25 dB, au-delà des 75 MHz de l'espace de recherche ; le secteur est signalé jusqu'à l'étape 3.1). |
| **`fa_per_rr`** | 10 | **Étape 2.3.** Fausses alarmes tolérées par revisite en mode « fa_rate » (réglage de mission à l'étape 5.1). |
| **`range_min_km`** | 10 % de `range_km` | **Étape 2.1.** Portée minimale d'intérêt : la zone aveugle des formes d'onde du secteur (veille et observations Ku) doit rester en dessous. À terme, les affichages ne montreront que le volume utile, sans la zone aveugle (étape 5.5). L'interface ne l'édite pas encore ; elle la conserve. |
| `allow_ambiguous` | faux | Autorise une distance ambiguë. Depuis l'étape 2.5, les trains ambigus sont retirés quand une surface est dans le lobe principal d'un pointage du secteur (fouillis replié). |
| **`jammer_type`**, `jammer_erp_dbw_mhz` | absent | **Étape 2.5.** Brouilleur d'autoprotection porté par la cible (classe SPJ de `[jammers.*]`, surcharge facultative de sa densité de PIRE) : portée de percée. |
| **`objects_per_rr`** | 0 | **Étape 2.5.** Objets réels non menaçants (débris, satellites) vus par revisite, confirmés comme des cibles. |
| **`tec_tecu`**, **`b_parallel_nt`** | absents | **Étape 2.5.** Contenu électronique (TECU) et champ magnétique (nT) le long de la ligne de visée, donnés par le scénario : rotation de Faraday en sortie. |
| **`rx_sll_db`** | **−25 dB** | **Étape 1.1.** Niveau de lobes de la pondération de réception du volume, au choix parmi −25, −30 et −35 dB. Il s'applique à la veille S et à la confirmation et poursuite Ku du volume. Un ancien fichier sans ce champ prend −25 dB. Une liste déroulante dans l'interface indique, pour chaque option, η, la perte de gain et la largeur de lobe. |
| `confirm_mode` | "auto" | auto, KU1+7, KU7 ou S2. |
| `new_targets_per_rr` | 1 | Nouvelles vraies cibles par période de revisite. |
| `tracking` | [] | Tranches de poursuite active : r_min et r_max, nombre de cibles (5), période (2 s), zone angulaire (0,2°), σ_R (10 m), σ_V (5 m/s). |
| `max_wf_per_sector`, `short_tol`, `freq_ghz_candidates` | réglages de l'optimiseur | Surcharges possibles par secteur. |

### 9.3 Formats

- Fichiers JSON (complets) ou CSV (tableau des secteurs, avec les tranches de poursuite en JSON dans une seule colonne). Les deux restent compatibles avec l'entrée de l'optimiseur.
- La validation (« Validate mission prep ») vérifie : identifiants uniques, bornes, φ et ε, v_min ≤ v_max, valeurs strictement positives, la classe de cible (connue) et ses surcharges (étape 2.4), tranches de poursuite sans chevauchement et dans la portée, `rx_sll_db` parmi les options, le ciel (champs, types connus ou température donnée, repère galactique cohérent), et depuis l'étape 3.0 la navigation et le repère de chaque secteur (§7.14). Le CSV porte `frame`, `limb_alt_km` et les quatre angles de `two_dir` ; la navigation et le ciel n'ont pas de colonne (JSON seulement).

---

## 10. Feuille de route, renvois et points ouverts

La feuille de route complète, avec le contenu détaillé de chaque étape, se trouve dans `Feuille_de_route.md`.

### 10.1 Feuille de route (accordée le 20 octobre 2026)

| Phase | Étapes |
|---|---|
| 0, fondations | 0.1 fichier unique ✔ ; 0.2 banc de tests ✔ ; 0.3 revue des valeurs ✔ |
| 1, antenne et RF | 1.1 géométrie et diagramme réel ✔ ; **1.2 diagramme : lobes de réseau, désadaptation active \|Γ(θ, f)\|, polarisation ✔** ; **1.3 TRM et chaîne d'émission ✔** ; **1.4 bande ✔** ; **1.5 réception ✔** (T_sys selon la direction, CAN, isolation, deux voies de polarisation ; le bruit de phase de l'oscillateur est passé en 2.2) |
| 2, signal et détection | **2.1 formes d'onde paramétriques ✔** ; **2.2 traitement ✔** ; **2.3 critère de détection ✔** ; **2.4 modèles de cibles ✔** ; **2.5 environnement ✔** ; 2.5 environnement (masquage par le vaisseau, Faraday, brouillage) |
| 3, fonctions radar | 3.1 veille S (multifaisceau, grille de fréquences fine) ; 3.2 confirmation Ku (fréquence Ku choisie, connaissance du spectre) ; 3.3 poursuite (Kalman ou IMM) ; 3.4 fonctions annexes |
| 4, ressources | 4.1 puissance (bus de 8 MW, répartition entre faces) ; 4.2 thermique ; 4.3 ordonnanceur et latence ; 4.4 calcul et données |
| 5, mission et validation | 5.1 modèle de mission enrichi et bibliothèque de menaces ; 5.2 optimiseur global ; 5.3 moteur de validation ; 5.4 simulation Monte-Carlo ; 5.5 interface opérateur ; 5.6 vérification, validation et documentation |

Jalons : A = phases 0 à 2 ; B = phase 3 ; C = étapes 4.1 à 4.3 ; D = phase 5.

### 10.2 Renvois enregistrés (dispositions provisoires à reprendre)

| Sujet | État actuel | Étape |
|---|---|---|
| Polarisation | Linéaire selon e_v (D). Recombinaison des deux voies par le GLRT depuis l'étape 1.5 (perte calculée) ; **depuis l'étape 2.4**, énergie croisée recueillie (1 + ρ) et gain de diversité sur la fluctuation. Commutation e_u / e_v à rappeler en 3.3 ; usage polarimétrique (discrimination des leurres) en 3.3. | 2.5 (Faraday), 3.2-3.3 |
| Élément rayonnant | Éléments isotropes dans le calcul du facteur de réseau ; seule la projection cos θ et la désadaptation interviennent. | à placer |
| Zone aveugle et éclipse | Depuis l'étape 2.1 : zone aveugle ≤ `range_min_km`, pas d'auto-éclipsage des trains non ambigus, perte moyenne des trains échelonnés. Reste l'éclipsage par les autres observations et les autres faces. | 4.3 |
| Coût des fausses alarmes dans le choix des formes d'onde de veille | Depuis l'étape 2.2, les bancs de vitesses des trains multiplient les fausses alarmes (×50 à ×535) ; l'optimiseur de veille ne compte pas le coût de confirmation qui en découle. | 2.3 (politique de fausses alarmes), 5.1 (réglage de mission), 5.2 (coût de confirmation dans l'optimiseur) : décision de l'utilisateur |
| Accélération des cibles | **Modélisée à l'étape 2.4** (phase quadratique, courbure, banc d'hypothèses). Une observation de poursuite la suppose connue par la piste (optimiste). | 3.3 |
| Capacité du formateur de faisceaux et charge de détection | Faisceaux par position calculés (≈ 3,4) ; nombre de faisceaux simultanés (positions en attente d'écho, multifaisceau, deux voies, Ku) non modélisé ; hypothèses de vitesse et grilles ×4 multiplient la charge. Capacité à fixer comme ressource. | 4.3, 4.4 |
| Bandes au-delà de 75 MHz, impulsions plus courtes que 20 µs | Modèle valide jusqu'à 200 MHz ; secteurs « résolution non tenue » signalés ; un secteur de 20 km (B2_AUTOPROT) n'a plus de forme d'onde (20 µs laissent 3 km aveugles). | 3.1 |
| Multiplexage en temps ou en fréquence du multifaisceau | Passé de l'étape 2.1 à l'étape 3.1, avec le reste du multifaisceau. | 3.1 |
| Résidu du couplage distance-Doppler | Biais corrigé par la mesure du Doppler ; résidu non compté. | 3.3 |
| Affichage du volume utile, édition de `range_min_km` | À terme, les vues ne montrent que le volume utile (sans la zone aveugle) ; l'interface n'édite pas `range_min_km`. | 5.5 |
| PAE en fonction du recul du PA | PAE constante sous réduction de puissance ; loi nécessaire au multifaisceau par superposition. | 3.1 |
| Espaces de recherche | Rien entre 0,8 et 2 ms en S, rien au-delà de 400 µs en Ku. | 3.1, 3.2 |
| Alimentation des TRM | Recharge de la réserve pendant un train, capacité d'alimentation d'un TRM, et **puissance instantanée de la face** pendant une observation (4 900 TRM S en même temps : environ 8,2 MW à d = 0,65, contre 2 MW alloués) ; seule la moyenne sur la période de revisite est vérifiée. | 4.1 |
| Thermique | Chaleur calculée (≈ 370 W par TRM S en pointe) ; capacité des microcanaux et constantes de temps non modélisées. Densité de puissance d'environ 104 kW/m² de RF moyenne. | 4.2 |
| Limite de 70° au-dessus de la fréquence de rupture | Gardée à l'étape 1.4 tant que le diagramme de l'élément n'est pas modélisé (flanc du lobe de réseau à l'horizon). | avec le diagramme de l'élément (à placer) |
| Fréquences candidates de la veille S | 2,7 / 2,8 / 2,9 GHz : depuis l'étape 1.4, aucun signal ne tient aux bords de bande (2,7 et 2,9 GHz), seule 2,8 GHz est jouable. | 3.1 (grille fine dans la bande) |
| Temps de cohérence de la cible | **Par classe et par bande depuis l'étape 2.4** (T₀ = λ / 2Lω). | — |
| Masquage par une cible forte (lobes Doppler et en distance, bruit de phase) | Niveaux en sortie depuis l'étape 2.4 ; OS-CFAR en option. Effet sur une scène dense non compté. | 5.4 |
| Table d'aspect de la SER | Clé réservée (SER frontale seule dans le bilan). | 3.3 |
| Tables de confirmation | Cas de Swerling le plus proche (famille ou cas déclaré) : sans décorrélation partielle ni diversité de polarisation, que `solve()` compte pour les observations Ku. | 3.2 |
| Choix de la classe dans l'interface | L'interface conserve `target_type`, montre les valeurs de la classe ; une SER ou un Swerling modifié devient une surcharge. | 5.1 |
| Estimation rapide | Table de coût calculée sur GENERIC en Swerling 1 ; un secteur à classe y entre par sa SER frontale en S. | 5.5 |
| Sensibilité du verdict aux valeurs de la menace | Les classes sont des moyennes : le moteur de validation dira de combien la SER (et les autres valeurs) peut baisser avant que le secteur ne tienne plus. | 5.3 |
| Synchronisation des faces par bande | Règle signalée (`scheduler_rules`), non imposée ; aucune perte dans le bilan. Annulation active à revoir. | 4.3 |
| Harmonique 5 de la S dans le canal Ku | Règle de fréquence signalée par secteur. | 3.2, 4.3 |
| T_sys du ciel d'un secteur | Centile (1 − `short_tol`) des pointages non aveuglés, gardé à l'étape 2.3 (effet spatial, pas aléatoire). | — |
| Plafond du nombre d'observations cumulées (cible surgissante), Pd et taux de fausses alarmes par secteur | Pas de plafond ; une seule Pd et un taux par défaut pour tous les secteurs. | 5.1 |
| Décalage de la grille de veille à chaque trame | Supposé (`grid_dithering`), exigence signalée. | 4.3 |
| Estimation rapide avec le nouveau critère | La table de coût reste au critère d'une observation à `detection.pfa`, avec la grille de 1,4. | 5.5 |
| Bruit de la structure du vaisseau | Modélisé à l'étape 2.5 (coque seule visible). | — |
| Fouillis terrestre et saturation des CAN qu'il cause | Modélisés à l'étape 2.5 (fouillis par l'anneau d'égale distance, alerte CAN). | — |
| Récupération du limiteur | Comptée depuis l'étape 2.1 (sortie `rx_deaf_us`) : zone aveugle c (τ + récupération) / 2, PRI des trains non ambigus ≥ 2R/c + τ + récupération, éclipsage moyen des trains échelonnés. Reste l'éclipsage par les autres observations et les autres faces (ligne « Zone aveugle et éclipse »). | ✔ 2.1 ; reste 4.3 |
| Atténuateur commutable (poursuite rapprochée) | Gain fixe, alerte de saturation des tranches de poursuite. | 3.3 |
| CAN : nombre, débit, consommation | 2 CAN par TRM (deux voies). | 4.4 |
| Estimation rapide au ciel froid | Sans le bruit du ciel. | 5.5 |
| Multifaisceau | Loi unique, statut ?. Modes à évaluer par l'optimiseur. | 3.1, 5.2 |
| Fréquence Ku | Fixée à 14,0 GHz, provisoire. | 3.2 |
| Masquage par le vaisseau | Modélisé à l'étape 2.5 (tube d'ouverture ; modèle 3D `LP_NISO.obj` depuis le 22 octobre 2026) : aucun pointage masqué avec la géométrie actuelle. Diffraction en champ proche et couplage avec la structure non modélisés. | — |
| Réflexions sur la structure | Soleil, Terre et autres faces réfléchis par la coque ou les radiateurs : non comptés. | 5.4 |
| Fouillis d'une forme d'onde | Évalué à la portée exigée avec la bande la plus étroite admissible, sans le choix de la forme d'onde par pointage ; rejet Doppler en tout ou rien. | 3.1 |
| Portée exigée derrière une surface | Signalée, mais l'exigence n'est pas réduite à la distance de la surface sur ces pointages. | 3.1 |
| Brouilleurs et confirmation Ku | La portée de percée n'est calculée que pour la veille S ; les brouilleurs à distance entrent dans le T_sys Ku. | 3.2 |
| Brouillage trompeur (répéteurs, faux échos) | Non modélisé (décision Q4). | 3.3 |
| Choix du CFAR par pointage | GO conseillé près d'un limbe ; CA partout. | 4.3, 5.2 |
| Coût de l'annulation adaptative | 8 brouilleurs par face au plus. | 4.4 |
| Latence | Règle supprimée, à reconstruire. | 4.3 |
| Répartition de la puissance entre faces | Une seule valeur par mission. | 4.1, 5.2 |
| Consommation hors émission | `overhead_frac` à 3 %. | 4.1, 4.4 |
| Faisceaux de réception par pointage | Calculés (environ 3,4) mais pas encore comptés dans le bilan de calcul. | 4.4 |
| p_min par volume | Option à creuser. | 5.1 |

### 10.3 Points ouverts

- **(a)** L'optimiseur départage des égalités exactes par le bruit d'arrondi : il faut un critère secondaire (étapes 3.1 et 5.2). La cause principale, le plancher de cohérence, a disparu à l'étape 2.2 ; il reste des égalités entre une impulsion seule étiquetée COHERENT ou NONCOHERENT (identiques).
- **(b)** Le nombre de faces, 4, est écrit en dur dans le code.
- **(c)** Le toucher S2 sort de la règle des rôles S ; les rôles TWS et PROMOTE ne sont pas utilisés (étape 3.2).
- **(d)** `power_scale` est arrondi à 3 décimales dans la sortie de l'optimiseur (étape 5.5).
- **(e)** La table de l'estimation rapide n'est calculée qu'à 2,8 GHz (étape 5.5).
- **(f)** **Lobes de jeu des TRSA (étape 1.2), traité en grande partie.** Le jeu est passé de 2 mm à 0,5 mm. Restent en Ku, au haut de la bande : −30 dB tenu jusqu'à 68,5° (14,25 GHz) et 64° (14,5 GHz), −35 dB jusqu'à 62° (14 GHz), 58,5° et 55° ; signalé par secteur. Si la mécanique impose un jeu plus grand, la variante est une maille d'éléments uniforme à travers les jeux (TRM Ku de 10,32 mm ou éléments décalés), qui demande de séparer le pas des éléments de la taille des TRM dans `[aperture]`. Les effets de bord de tuile (couplage mutuel au bord d'un jeu) ne sont pas modélisés.
- **(g)** `rx_taper_edge` = "inscribed" (statut D) à confirmer.
- **(h)** **Masse des réserves d'énergie** (étape 1.3) : 122 kJ par face en S et 147 kJ en Ku, soit environ 270 kg par face à 1 J/g (hypothèse 2050), environ 1,1 t pour les 4 faces ; aucun bilan de masse n'est prévu dans la feuille de route.
- **(i)** Le jeu de 0,5 mm suppose des matériaux appariés en dilatation (étape 4.2).
- **(j)** `mission_detection_envelope.csv` (racine de `V1`), artefact périmé d'avant la feuille de route (modèle V6.0.12), lu par aucun code et sans outil de génération : **supprimé** sur décision de l'utilisateur (20 octobre 2026). Il reste dans l'historique git.
- **(k)** Le couplage mutuel au bord des tuiles et le diagramme réel de l'élément rayonnant ne sont pas modélisés (éléments isotropes dans le facteur de réseau ; l'intégrale du bruit du ciel prend un élément en cos θ, rien vers l'arrière, depuis l'étape 1.5).
- **(l)** Les couplages de l'isolation (étape 1.5 : 90 et 120 dB entre faces, −30 dB dans une TRSA) sont des ordres de grandeur pris au bout optimiste ; les faces sont dans le champ proche l'une de l'autre (limite de Fraunhofer d'environ 325 m en S) : une simulation électromagnétique serait nécessaire.
- **(m)** Les erreurs aléatoires d'amplitude et de phase des TRM ne sont pas modélisées (environ 0,2 % de la puissance hors du lobe pour 0,2 dB et 2° rms : négligeable devant les 2 à 5 % de la pondération pour le bruit du ciel).
- **(n)** Correction à l'étape 1.5 : lors des questions, le facteur de bruit du CAN avait été annoncé à environ 8 dB avec 13 bits et +10 dBm. C'est faux : il reste à 20 dB, car 6 dB de pleine échelle en plus coûtent ce que le 13ᵉ bit rapporte. Les valeurs de T_sys annoncées (144,5 et 230,8 K) étaient justes ; la marge avant écrêtage gagne bien 6 dB.

---

## 11. Historique des décisions

### Étape 0.1 : fichier de paramètres unique (commit cc14375)
- Toutes les constantes ont été déplacées dans `radar_system.toml`, sans changer de valeur. Les noms des constantes de module sont conservés (modèle V6.0.13).
- La fenêtre ε de la mission est passée en argument, au lieu de réécrire des constantes du module.
- Choix : TOML, un seul fichier monolithique, un bloc `[tool.*]` séparé, des entrées en valeur, unité, statut, source et date.

### Étape 0.2 : banc de tests (commit 1fb4bc7)
- Deux familles de tests, tolérance relative de 1e-9, rapport d'impact, `pytest` dans `requirements-dev.txt`.
- Missions restaurées depuis l'historique git, en tant que **missions de test**.

### Étape 0.3 : revue de toutes les valeurs (commits c5c2400 et e2ebb90)
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

### Étape 1.3 : TRM et chaîne d'émission (20 octobre 2026)
| Question | Décision de l'utilisateur |
|---|---|
| Technologie | **GaN sur diamant, refroidissement par microcanaux** (description de V5.6.0) ; valeurs crédibles mais optimistes pour 2050-2060. |
| Q1, bilan | **Électrique** : chaque observation coûte E_RF / PAE ; la face dispose de P_élec × CHAIN_EFF. |
| Q2, loi de PAE | **(b)** η_sat × τ/(τ + t_front) × [1 − a (d − d_ref)], d = τ/PRI, d_ref = 0,10, plafonnée à η_sat. |
| Q2 bis, baisse avec le facteur de forme | a = **0,05 en S, 0,08 en Ku**. |
| Q3, PAE en saturation | **0,80 en S, 0,65 en Ku.** |
| Q4, temps de front | **0,2 µs en S, 0,1 µs en Ku.** |
| Q5, facteur de forme maximal | **0,65** dans les deux bandes, « quitte à ne pas l'utiliser ». |
| Q6, impulsion maximale | **(a)** La réserve seule : 2,0 ms en S, 2,08 ms en Ku. Zone aveugle renvoyée à 2.1. |
| Q7, réserve | **Fraction 0,2 ; 25 J en S, 1,2 J en Ku** (proposition de l'utilisateur, pour des impulsions très longues en veille très longue portée). |
| Q8, recharge pendant un train et alimentation des TRM | **Option A** : tout le sujet passe à l'étape 4.1, avec le bus, le stockage et la répartition des 8 MW. |
| Q9, dissipation | Calculée en sortie (par TRM, par face, par m²). |
| Q10, PAE en fonction du recul | Renvoyée à l'étape 3.1. |

Règle de méthode fixée pendant cette étape : tout ce qui touche un même sujet est traité au même endroit, et une valeur dépendante se fixe après celles dont elle dépend.

Effet sur les missions de test (valeurs de test seulement) :
- portée de WF-LRS +1,12 % (6 813 → 6 890 km sur ST), par la baisse de l'affaissement de la réserve (0,24 → 0,04 dB) ;
- énergie électrique par face de −5 à −26 %, temps S de −20 à +66 %, temps Ku de −48 à +80 % ;
- l'impulsion de 2 ms est retenue par plusieurs secteurs à longue portée.

### Étape 1.4 : la bande (20 octobre 2026)
| Question | Décision de l'utilisateur |
|---|---|
| A1, loi de bord | **(a)** Cosinus surélevé, bande centrale en GHz pour chaque bande. (L'ancienne table `rf_layer_s` de RADARv5, une ancienne bande S de 3,0 à 3,75 GHz, est écartée : RADARv5 est obsolète.) |
| A2, valeurs | **S : 2,74 à 2,86 GHz, 0,5 dB ; Ku : 13,7 à 14,3 GHz, 0,5 dB.** |
| A3, sens de la perte | **(a)** Perte totale aller-retour. |
| A4, fréquence d'évaluation | **Moyennée sur le spectre occupé.** |
| B1, occupation du spectre | **(a)** Règle stricte : le signal doit tenir dans la bande utile. |
| C1, bande instantanée S | **200 MHz confirmés (V).** |
| D1, retard vrai | **Entrée explicite**, numérique par TRM (V). |
| E1, limite de 70° | **(a)** Gardée. |
| Principe | Les valeurs restent ajustables (par exemple une bande plus large si les TRM progressent) : le code doit l'encaisser. Tout découle du fichier, le chargeur vérifie la cohérence, et un test élargit la bande S à 2,6-3,0 GHz par le seul fichier. |

Effet sur les missions de test (valeurs de test seulement) :
- l'optimiseur garde 2,8 GHz (bande centrale, sans perte de bord) : énergie par face de 0 à −1,5 %, temps S de −2,0 à +0,5 %, temps Ku de 0 à −7 % ;
- le secteur A3_HIBAND de `mission_redundancy_demo`, qui impose 2,9 GHz (le bord haut), n'a plus de forme d'onde jouable : il est conservé tel quel et exerce ce cas.

### Étape 1.5 : la réception (21 octobre 2026)

Questions de cadrage :

| Question | Décision de l'utilisateur |
|---|---|
| Q1, bruit de phase de l'oscillateur | **Tout passer en 2.2**, avec la cohérence. |
| Q2, position des corps | Le simulateur radar sera une partie d'un **simulateur global de scénario**, qui lui donnera la position de tous les objets ; le radar travaille dans le **repère du vaisseau** et ne calcule pas où sont les objets. Rien n'est repris du prototype (`RADARv5/SIMULATOR/10-BLEEDING EDGE`, preuve de concept en retard d'un an sur la RF). |
| Q2a, températures de brillance | (c) Table par type et par bande dans le fichier, surcharge possible par le scénario. |
| Q2b, fond galactique | (a) Repère galactique donné par le scénario (pôle et centre), profil en latitude et longitude ; pire cas constant sans repère. |
| Q3, T_sys d'un secteur | Pas de cône d'exclusion fixe : pointages **aveuglés** au-delà d'un seuil de hausse ; bilan au **centile (1 − `short_tol`)** des autres pointages (la règle du pire point seul aurait imposé la frange du Soleil, près de 10 dB, à tout le secteur). |
| Q4, harmonique S dans la bande Ku | Matrice d'isolation complète, avec l'harmonique. |
| Valeurs | « Prendre les valeurs optimistes à chaque fois » : bout optimiste de chaque plage, aussi pour le sujet B déjà validé. |

Variables (toutes validées) :

| Sujet | Décision |
|---|---|
| A1 ciel froid | S 3,2 K, Ku 2,8 K. |
| A2 fond galactique | 0,5 / 2 / 8 K à 2,8 GHz, σ_b 3°, σ_l 30°, indice 2,5. |
| A3 températures de brillance | Table du §5.16. |
| A4 diagramme pour le bruit | Élément cos θ, rien vers l'arrière ; erreurs aléatoires non modélisées. Le lobe principal est le diagramme réel moyenné en azimut (plus exact que le lobe gaussien proposé, signalé à l'utilisateur). |
| A5 aveuglement, évaluation | 10 dB ; pas de 0,5 HPBW_TX ; pire point (centile) ; même règle en Ku. |
| A6 structure du vaisseau | Bruit thermique de la structure traité à l'étape 2.5 avec le masquage. |
| B1 numérisation | I/Q en bande de base, fs = 1,25 × bande instantanée. |
| B2 CAN | 13 bits, +10 dBm ; facteur de bruit dérivé (20 dB, la valeur de 8 dB annoncée pendant les questions était une erreur, voir §10.3 n). |
| B3 marge d'écrêtage | 3 dB. |
| B4 compression | LNA +10 / +5 dBm (S / Ku), chaîne après le LNA +15 dBm. |
| B5 limiteur | 50 / 25 ns. |
| B6 gain | Fixe ; atténuateur à évaluer en 3.3 ; portée minimale non saturée par secteur, alerte des tranches de poursuite. |
| C isolation | Couplages 90 / 120 dB entre faces, −30 dB dans une TRSA, harmonique −45 dBc, filtre 80 dB, réjections 80 / 80 dB, marges 6 / 6 dB. |
| C mode d'interface | **Synchronisation** pour l'instant ; l'annulation active est à revoir à l'étape 4.3. Avant de décider, l'utilisateur a demandé comment font les radars multifaces (SPY-1 : un émetteur commun, une face à la fois ; SPY-6 et Sea Fire 500 : rien de public), les coûts de l'annulation active et une estimation grossière du coût de la synchronisation (§7.8). |
| E1 recombinaison des voies | GLRT sur la polarisation inconnue, perte calculée. |
| E2 poursuite | 0,05 dB. |
| E3-E4 | `pol_loss_dual_db` supprimé, `pol_loss_single_db` gardé ; déséquilibre des voies pour 3.3 ; Faraday sans coût en détection ; diversité en 2.4 ; nombre de CAN pour 4.4. |

Effet sur les missions de test (valeurs de test seulement) :
- T_sys froid S 154,3 → 144,5 K, Ku 238,7 → 230,8 K ; perte de polarisation de 0,5 dB à 0,06 dB sur WF-LRS : portées de WF-LRS +4,28 % (ST 6 890 → 7 185 km) ;
- missions sans repère galactique : fond galactique au pire cas (+7,5 K en S, T_sys du bilan 152,1 K) ; portées +0,1 à +2,4 %, énergie par face −0,6 à −1,1 %, temps S 0 à −11 %, temps Ku 0 à −9 % ; une forme d'onde de CONE_D1-D4 passe de cohérente à non cohérente (la perte GLRT est plus faible en non cohérent) ;
- nouvelle mission de test `mission_sky_demo.json` (Soleil, Lune, limbe de la Terre, repère galactique) : environ 1 % des pointages du secteur SUN aveuglés et T_sys du bilan +4,7 dB (la frange du Soleil dépasse 1 % d'un petit secteur), EARTH +2,5 dB, MOON +0,3 dB ;
- toutes les missions de test : règles pour l'ordonnanceur (synchronisation par bande ; le signal S de 75 MHz à 2,8 GHz met son harmonique sur le canal Ku de 14,0 GHz).

### Étape 2.1 : formes d'onde paramétriques (21 octobre 2026)

Questions de cadrage (toutes acceptées) :

| Question | Décision |
|---|---|
| Q1, multiplexage du multifaisceau | Passé à l'étape 3.1, avec tout le multifaisceau. |
| Q2, vitesse et migration | Étape 2.1 : perte d'un filtre adapté à une autre vitesse ; étape 2.2 : banc de vitesses et migration entre impulsions. |
| Q3, zone aveugle | Nouvelle clé de secteur `range_min_km`. L'utilisateur demande en plus qu'**à terme les vues affichent seulement le volume utile**, sans la zone aveugle (étape 5.5). |
| Q4, éclipsage | Celui du train de l'observation seule ici ; la chronologie complète à l'étape 4.3. |
| Q5, forfait `DSP` | Décomposé : pondération calculée, chevauchement 0,7 dB gardé jusqu'à 2.2, précision finie 0,3 dB. |
| Q6, bandes > 75 MHz | Modèle valide jusqu'à 200 MHz ; espace de recherche ouvert à l'étape 3.1. |

Variables (toutes validées, bout optimiste) :

| Sujet | Décision |
|---|---|
| A1-A3 | LFM et NLFM ; NLFM si f_d,max / B ≤ 2 %, sinon LFM ; codes de phase écartés. |
| A4-A6 | Lobes en distance = option du volume ; Taylor n̄ = 5 à la réception pour la LFM ; NLFM 0,1 dB. |
| A7 | Résolution physique (élargissement compris). |
| A8-A10 | k_r dérivé ; perte Doppler LFM dérivée ; biais de couplage corrigé par la mesure du Doppler, donné en sortie. |
| B1-B6 | Loi exacte de perte en vitesse (phase quadratique) ; aucune perte au bilan avant l'étape 2.2 ; AM/PM 2°/dB, résidu après correction, en sortie. |
| C1-C8 | `range_min_km` (défaut 10 % de la portée) ; zone aveugle stricte ; PRI ≥ 2R/c + τ + récupération ; trains ambigus échelonnés sur 3 PRI à ±10 % avec la perte d'éclipsage moyenne ; ambiguïté Doppler en sortie ; éclipsage de la chronologie à l'étape 4.3. |
| D1-D3 | Modèle valide jusqu'à 200 MHz ; espace de recherche inchangé ; résolution non tenue signalée. |

**Correction signalée à l'utilisateur.** Lors des questions, l'élargissement de la pondération avait été annoncé à ×1,13 / ×1,20 / ×1,27 (calcul trop grossier) et, par suite, 2 m de résolution à « 85 MHz » et 2,27 m à 75 MHz. Les valeurs justes, calculées par `range_weighting`, sont ×1,18 / ×1,27 / ×1,34, 89 MHz et 2,37 m. Les pertes (0,41 / 0,68 / 0,93 dB) et les k_r étaient justes.

Effet sur les missions de test (valeurs de test seulement) :
- toutes les observations S à 75 MHz sont des NLFM : 0,9 dB de pertes en moins ; portées de WF-LRS +5,3 % (ST 7 185 → 7 567 km) ;
- énergie par face −22 à +6 %, temps S −7 à −50 %, temps Ku −42 à +53 % (éclipsage des trains Ku ambigus contre pertes plus faibles) ;
- les secteurs courts perdent leurs impulsions longues ; B2_AUTOPROT (20 km) n'a plus de forme d'onde : l'impulsion la plus courte (20 µs) laisse 3 km aveugles, au-delà de sa portée minimale par défaut de 2 km (conservé : à reprendre avec l'espace de recherche de l'étape 3.1) ;
- tous les secteurs qui demandent 2 m (le défaut) sont signalés « résolution non tenue » (2,37 m à 75 MHz).

#### Écart cumulé avec le modèle d'avant la feuille de route (demande de l'utilisateur, étape 2.1)

Comparaison du modèle V6.0.12 (commit ac4d56e, avant l'étape 0.1) et du modèle V6.6.0, avec les mêmes formes d'onde et les mêmes missions de test (`mission_example`, `mission_veille500`, `mission_redundancy_demo`, `mission_synthetic_mix`). **Valeurs de test seulement : ce ne sont ni des exigences ni des performances du radar.**

| Grandeur | V6.0.12 | V6.6.0 | Écart |
|---|---|---|---|
| T_sys S / Ku | 346,5 / 400 K | 144,5 / 230,8 K (ciel froid) | −3,8 / −2,4 dB |
| Pointages par face à 2,8 GHz | 4 167 | 3 016 | −28 % (lobe réel, plus large) |
| Largeur du lobe d'émission à 2,8 GHz | 1,30° | 1,53° | +18 % |
| Pertes de WF-LRS, axe / 45° / 70° | 7,86 / 7,91 / 8,46 dB | 7,00 / 6,74 / 7,86 dB | −0,9 / −1,2 / −0,6 dB |
| Portée de WF-LRS sur ST, axe / 45° / 70° | 5 682 / 4 682 / 3 043 km | 7 567 / 6 459 / 4 211 km | **+33 / +38 / +38 %** |
| Portée de WF-LRS sur KKV, axe | 613 km | 816 km | +33 % |
| Poursuite Ku synthétique sur KKV, axe / 50° | 4 576 / 2 860 km | 6 240 / 4 067 km | **+36 / +42 %** |
| Énergie par face pour tenir les mêmes besoins (missions de test) | — | — | **−75 à −85 %** |
| Temps d'émission S par face | — | — | −25 à −85 % |
| Temps d'émission Ku par face | — | — | −90 à −99 % |
| Période de revisite atteignable seul (mêmes besoins) | — | — | ÷ 4 à ÷ 6,5 |
| Secteurs sans forme d'onde jouable | 0 | 2 sur 21 | A3_HIBAND (2,9 GHz, bord de bande, étape 1.4) et B2_AUTOPROT (zone aveugle, étape 2.1) |

À ressources égales, une énergie divisée par 5 vaut une portée multipliée par environ 1,5 (R ∝ E^¼). Le gain vient surtout de T_sys (revue 0.3 : frontal optimisé au lieu d'un module T/R classique, puis ciel froid et CAN dérivé à l'étape 1.5 : environ +3,8 dB en S), des pertes (pré-LNA compté une fois, rendement ohmique 1,0, polarisation par GLRT, compression NLFM), du lobe réel (moins de pointages) et du bilan électrique avec des impulsions de 2 ms (étape 1.3). Les corrections pessimistes (désadaptation active jusqu'à 2,2 dB aller-retour à 75°, bords de bande, zone aveugle, éclipsage des trains ambigus, fond galactique au pire cas, contrôles stricts) en reprennent une partie. Les étapes à venir en ajouteront d'autres (banc de vitesses et cases réelles en 2.2, critère de détection en 2.3, cibles en 2.4, ordonnanceur et synchronisation en 4.3).

### Étape 2.2 : le traitement (21 octobre 2026)

Questions de cadrage (toutes acceptées) : Q1 temps de cohérence de la cible à l'étape 2.4 (valeur provisoire ici) ; Q2 blocs cohérents puis intégration non cohérente ; Q3 banc d'hypothèses de vitesse ; Q4 Pfa par case fixe, toutes les cases comptées ; Q5 pertes moyennes (l'étape 2.3 tranche) ; Q6 CA-CFAR calculé ; Q7 bruit de phase : perte et lobes Doppler.

Variables (toutes validées, bout optimiste) :

| Sujet | Décision |
|---|---|
| A cohérence et oscillateur | Spectre photonique à 10 GHz, un oscillateur commun, perte 4,34 σ² calculée (négligeable), plancher Doppler en sortie, plateforme 10 µm, cible provisoire t₀ 0,25 s / p 2, plancher supprimé, blocs de taille optimale, SW2/SW4 impulsion par impulsion. Le calcul préalable a montré que l'oscillateur ne limite pas la cohérence (au plus 4e-6 dB même avec un oscillateur spatial réaliste). |
| B banc de vitesses | Loi dérivée (dans l'impulsion + glissement), pas à 1 dB, perte moyenne au bilan, étendue du secteur (une hypothèse en poursuite), résidu 0,1 dB, accélération à l'étape 2.4, mode UNCOMPENSATED gardé. |
| C chevauchement | Grilles ×4 en distance et en Doppler, pas de fenêtre Doppler, pertes calculées, entrée `straddle` supprimée. |
| D CFAR | CA-CFAR, 1 024 cellules, loi exacte Swerling 1, entrées `CFAR` supprimées. |
| E fausses alarmes | Cases de résolution indépendantes : distance depuis `range_min_km`, Doppler d'un bloc, hypothèses de vitesse, cellules angulaires ≈ 0,85 ; Pfa 1e-8 inchangée. |

Question annexe de l'utilisateur : le nombre de faisceaux numériques de réception est-il figé ? Non : le nombre par position est **calculé** (1,04 HPBW_TX / 0,5 HPBW_RX)², soit 3,38 / 2,95 / 2,64 selon l'option ; le nombre de faisceaux **simultanés** dépend de la chronologie (étape 4.3) ; la capacité du formateur de faisceaux est une ressource à fixer à l'étape 4.4 (renvoi inscrit).

Effet sur les missions de test (valeurs de test seulement) :
- pertes de WF-LRS 7,00 → 5,34 dB (chevauchement, CFAR et migration calculés) : portées +10,0 % (ST 7 567 → 8 325 km) ;
- énergie de veille par face −20 à −36 %, temps S −14 à −50 % ; la plupart des secteurs passent à des impulsions seules ;
- **mais** temps Ku de −53 % à ×300 : les trains demandent des bancs de 50 à 535 hypothèses de vitesse, qui multiplient les fausses alarmes à confirmer, et l'optimiseur de veille ne voit pas ce coût (§7.10 ; l'utilisateur traitera le sujet aux étapes 2.3, 5.1 et 5.2) ; plusieurs modes de confirmation passent de KU1+7 à KU7.

### Étape 2.3 : le critère de détection (21 octobre 2026)

Questions de cadrage (toutes acceptées) : Q1 Pd cumulée sur les observations successives ; Q2 Pd moyennée sur la position aléatoire de la cible ; Q3 T_sys du ciel gardé au centile ; Q4 politique de fausses alarmes à deux modes ; Q5 lois exactes ; Q6 pas de grille recalculé ; Q7 redondance au même critère.

Variables (toutes validées) : A1 `detection.pd` 0,9 en Pd cumulée à `range_km` ; A2 v_close_max ; A3 phase moyennée ; A4 pas de plafond (Pd > 0,001) ; A5 observations indépendantes ; A6 veille seulement ; A7 redondance ; B1-B4 écarts de grille, de chevauchement et de filtre de vitesse, lois exactes, une perte « positions », 16 quantiles ; C1-C5 mode « fa_rate », 10 fausses alarmes par secteur et par revisite, Pfa 1e-8 en mode « pfa_cell » et pour le Ku, bornes 1e-14 à 1e-4 ; D1 pas de 1,4 ; D2 grille de réception 0,5 ; D3 grille en distribution ; D4 réglage par secteur à l'étape 3.1.

Question de l'utilisateur : conséquences opérationnelles du passage de 1,04 à 1,4 ? Gains : 45 % de positions et environ 1 dB d'énergie en moins. Changements : couverture moins uniforme d'une observation à l'autre (−7,9 dB au coin), faisceaux de réception par position ×1,8 (charge instantanée, étape 4.4). Condition : une cible radiale reste au même endroit de sa cellule ; décision **(a)**, grille décalée à chaque trame (exigence pour l'ordonnanceur), plutôt que (b) une grille fixe avec un pas plus serré.

Mise au point pendant le codage : un premier calcul des lois était trop lent (11 s par courbe) et la quadrature de Swerling 1 imprécise (−0,1 dB) ; remplacés par une série vectorisée et une convolution sur la grille en dB (exact à 0,01 dB, 20 à 40 ms par courbe), puis par une inversion directe de la Pd cumulée (13 ms par appel du modèle).

Effet sur les missions de test (valeurs de test seulement) :
- deux régimes selon q = v·RR / R : les secteurs à longue portée (8 000 km, 120 s, 5 km/s : q = 0,075, des dizaines d'observations) demandent beaucoup moins d'énergie : **−69 à −96 %** par face ; les secteurs anti-KKV (500 km, 20 s, 25 km/s : **q = 1**, la cible parcourt toute la portée demandée entre deux observations) en demandent beaucoup plus : **+280 à +305 %** sur les faces qu'ils dominent. L'ancien critère, une observation exactement à R, était optimiste pour eux (la redondance leur donnait déjà une portée garantie nulle, R − v·RR = 0) ;
- temps Ku −80 à −99 % sur la plupart des faces (10 fausses alarmes par revisite au lieu de centaines) ; plusieurs modes de confirmation changent ;
- le banc de référence WF-LRS (une observation, sans paramètre de secteur) perd 10 % de portée : perte « positions » de 4,6 dB avec la grille de 1,4 à Pd 0,9 sur une observation.

### Étape 2.4 : les modèles de cibles (22 octobre 2026)

Questions de cadrage (validées) : Q1 clé de secteur `target_type`, valeurs explicites en surcharge ; Q2 SER frontale par bande (table d'aspect réservée à la poursuite) ; Q3 cas de Swerling calculé à partir de la famille et de T₀ ; Q4 T₀ = λ / (2 L ω) ; Q5 perte d'accélération sous une cellule, banc au-delà ; Q6 niveaux de masquage en sortie et OS-CFAR en option, effet sur une scène dense à l'étape 5.4 ; Q7 dépolarisation et diversité ; Q8 classes ST, KKV, SHIP, plus débris et leurres (utilisateur), et le code doit accepter de nouveaux types et des changements de valeurs dans le fichier seul.

Variables (toutes validées) : A1 schéma d'une classe ; A2 `target_type` et surcharges ; A3 classe par défaut, puis **GENERIC** dédiée (utilisateur : ST reste la référence) ; A4 SER copolaire frontale ; A5 table d'aspect réservée ; B1 T₀ = k λ / 2Lω, k = 1 ; B2 p = 2 ; B3 fluctuation continue ; B4 alerte sous 3 T₀ ; C1-C2 accélération et banc (1 dB à mi-pas) ; D1-D3 masquage, OS-CFAR rang 3/4 ; E1-E2 énergie croisée et diversité ; F valeurs des classes.

Décisions de l'utilisateur en cours d'étape :
- SHIP : SER **frontale** 300 m².
- KKV : la face avant est un autodirecteur IR à **miroir de grand diamètre** (concept du KKV du GBI). Le modèle ajoute une composante optique générique (réflexion directe et œil de chat), utilisable par toute classe.
- **Les classes sont des valeurs moyennes, pas absolues** : on ne peut pas présupposer de l'armement de l'adversaire, toutes les valeurs sont possibles. D'où le milieu de chaque plage (D 0,3 m, f/D 1, η_r 0,1 en moyenne logarithmique), une contre-mesure ou un leurre qui imite l'optique décrits par surcharge, et le renvoi vers l'étape 5.3 (sensibilité du verdict aux valeurs de la menace).
- Usage des classes rappelé : avant l'étape, elles ne servaient qu'au rapport en ligne de commande et aux tests ; désormais elles entrent dans le bilan des missions (classe nommée, ou GENERIC pour compléter un secteur sans classe).

Mise au point pendant le codage : le surcoût de calcul a été ramené à environ 10 % (loi stable mise en cache, table de la phase quadratique pour l'accélération, cible du secteur mise en cache).

Effet sur les missions de test (valeurs de test seulement) : voir `tests/snapshots/JOURNAL.md`. Nouvelle mission de test `mission_target_library.json` (secteurs KKV, KKV avec η_r = 1 en surcharge, SHIP, DEBRIS, DECOY).

### Étape 2.5 : l'environnement (22 octobre 2026)

Questions de cadrage (validées) : Q1 structure en primitives, masquage par le tube d'ouverture ; Q2 bruit de la structure comme corps du ciel ; Q3 fouillis de surface au bilan et alerte CAN ; Q4 brouilleurs de bruit (à distance et d'autoprotection), faux échos en 3.3 ; Q5 objets non menaçants par une densité donnée par la mission ; Q6 Faraday en sortie sans perte ; Q7 point « bruit de fond » clos ; Q8 GO-CFAR en option.

Variables (toutes validées) : A1-A3 géométrie du dessin (coque, télescopes, ailettes, tourelles, mâts, rails, ailes en 4 étages), A4 tube et seuil de 3 dB par passage, pas de repliement ; **le code doit permettre de modifier la géométrie** (fichier seul) ; vue 3D construite sur les mêmes primitives ; B1-B5 matériaux (coque 0,01 / 290 K, radiateurs 0,05 / **750 K**, médiane d'un vaisseau nucléaire), réflexions non comptées ; C1-C6 surfaces (masque, fouillis par les lobes secondaires et trains ambigus, γ Terre −12 / −10 dB et Lune −11,5 dB, rejet Doppler si le scénario le permet, alerte CAN) ; D1-D6 brouilleurs (bibliothèque de classes en moyennes, annulation 40 dB × 8, portée de percée, valeurs médianes proposées et validées) ; E objets (`objects_per_rr`) ; F Faraday (`tec_tecu`, `b_parallel_nt`) ; G GO-CFAR et alerte de limbe.

Explication demandée par l'utilisateur : le GO-CFAR (moitiés de fenêtre, plus grande moyenne, tenue de la Pfa aux bords de fouillis, perte 0,05 dB à 1 024 cellules, masquage accru par les cibles voisines).

Constats : aucun pointage masqué par la structure, radiateurs jamais vus (§5.15) ; sur la mission de test d'environnement, fouillis limitant près de l'anneau terrestre, brouilleurs à distance aveuglant quelques pointages malgré l'annulation, et portée de percée de SPJ_KKV de 11 km avec la forme d'onde choisie (114 km au mieux), contre 500 km exigés.

Mesure de performance pendant le codage : le surcoût de l'étape est négligeable (70,6 s contre 70,4 s sur `mission_example`) ; la même mission prenait 19,6 s le matin sur le même portable, également sur batterie : la machine était environ 3,5 fois plus lente l'après-midi (relevé : mode « Legion Balance », 37 %), pour une cause non identifiée ; c'est elle, et non le code, qui a allongé la suite de tests de l'étape 2.4 (§1.5).

Mise au point : en ajoutant le GO-CFAR, le cache de `masking_levels_db` (étape 2.4) s'était détaché de sa fonction, ce qui rendait chaque `solve()` 100 fois plus lent (une table de coût de plusieurs heures) ; corrigé avant la régénération des valeurs figées. Sur secteur, la table de coût se refait en 1 min 36 et le rapport d'impact en 2 min 39.

Effet sur les missions de test (valeurs de test seulement) : seul `mission_sky_demo` / EARTH change vraiment (Terre vue d'orbite basse : fouillis de l'anneau d'égale distance, T_sys du bilan 255 → 998 K, 19,8 % des pointages limités par le fouillis, énergie des faces 3 et 4 +160 à +170 %) ; voir `tests/snapshots/JOURNAL.md`.

Nouvelle mission de test `mission_environment.json` (Terre, Lune, deux brouilleurs à distance, un KKV avec SPJ_KKV, objets et Faraday).

### Géométrie du vaisseau : le modèle 3D (22 octobre 2026, modèle V6.10.1, commit 9ea4985)

Après la reprise sur une autre machine, un audit du code des étapes 2.4 et 2.5 a relevé que le cône porte-antennes des primitives (25,6°) ne correspondait pas exactement à l'inclinaison de 25°, sans contrôle au chargement. Décisions de l'utilisateur :
- les antennes sont bien inclinées de **25°** (confirmé par le concepteur de la géométrie du vaisseau) ;
- le modèle 3D basse résolution `assets/models/LP_NISO.obj` est désormais la **référence de tous les calculs** de masquage et de position des éléments ;
- cotes du modèle contrôlées par l'utilisateur et validées (« go ») : longueur 149,46 m, envergure 74,68 m, diamètre de coque 17,22 m, antennes à 5,248 m de l'axe (face avant des panneaux).

Mise en œuvre : `[ship.hull]` et `[ship.parts]` supprimés ; lancer de rayons exact sur les triangles du modèle ; matériaux par région (région des radiateurs confirmée par l'utilisateur : V) ; contrôles au chargement des cotes et des quatre ouvertures (sur la face avant d'un panneau, normale à `geometry.tilt_deg`), ce qui règle le point de l'audit. Le test du changement d'inclinaison vérifie désormais que le modèle radar suit l'inclinaison et que l'outil refuse une inclinaison qui ne correspond plus au modèle 3D.

Effet (valeurs de test seulement) : constats inchangés (aucun pointage masqué, radiateurs jamais vus) ; cellules bloquées par face 735 → 621 ; chiffres clés des missions de test à moins de 0,01 % ; aucune forme d'onde ni mode de confirmation ne change (voir `tests/snapshots/JOURNAL.md`).

### Étape 3.0 : repères des volumes et navigation (22 octobre 2026, modèle V6.11.0, commit 22114fd)

Constat de l'utilisateur : les volumes étaient définis dans le repère du vaisseau ; si le vaisseau tourne, la région veillée tourne avec lui, ce qui n'a pas de sens opérationnel. Étape ajoutée avant la 3.1.

| Question | Décision |
|---|---|
| Q1 repère des volumes | Choisi par mission parmi inertiel, orbital local et deux directions ; chaque secteur peut rester dans le repère du vaisseau. |
| Q2 attitude | Une seule par mission maintenant, puis une enveloppe (5.3), puis le profil dans le temps du simulateur global : l'architecture le prévoit dès 3.0 (liste, une seule fonction de rotation, calcul en aval dans le repère du vaisseau). |
| Q3 qui fait la rotation | Le radar (le calculateur), à partir de l'attitude fournie ; le simulateur reste la source de l'attitude et des positions. |
| Q4 ciel | Dans le même repère de mission que les secteurs. |
| Q5 régions non centrées sur le vaisseau | Renvoyées au simulateur global (parallaxe). |
| Q6 place | Nouvelle étape 3.0, avant la 3.1. |
| D1 à D6 architecture | Quatre couches, vérité et connaissance séparées (Feuille_de_route.md, étape 3.0) ; le convertisseur et la simulation RF côté simulation ; le calculateur reçoit un message de navigation ; repères vaisseau, inertiel, RTN, limbe, deux directions (le volume lié à une piste en 3.3) ; périmètre réduit au calculateur ; ciel prévu par le calculateur, vérité par la simulation. |

Question de fond de l'utilisateur : la chaîne simulation globale → repère du vaisseau → simulation radar → détection. Réponse : oui, avec le calculateur qui ne lit que la navigation ; dire si un objet donné est détecté relève d'un mode objet par objet (ordonnanceur 4.3, simulation à événements discrets 5.4), sans entrée séparée dans la feuille de route (décision de l'utilisateur) ; le simulateur devra fournir aussi la vitesse relative, la classe et, à terme, l'attitude de la cible (SER selon l'aspect).

Variables (toutes les propositions acceptées) : N1 liste d'un seul message (`label`, `epoch`, `q_ship_to_icrf`) ; N2 quaternion de Hamilton, scalaire en premier, vaisseau → ICRF ; N3 tolérance de norme 1e-3 ; N4 orbite facultative (`central_body`, `r_km`, `v_kms`), obligatoire pour RTN et limbe ; N5 saisie en lacet, tangage, roulis 3-2-1 par rapport à l'ICRF ou au RTN, quaternion modifiable ; N6 compatibilité (sans `nav` ni `frame` : comme avant). R1 à R5 repères (RTN plutôt que le LVLH du CCSDS ; limbe : ε compté à partir du limbe, `limb_alt_km` **0 km par défaut**, boîte seule ; deux directions : au moins 1°) ; R6 nouveau secteur de l'interface en `inertial` ; R7 fenêtre de balayage globale dans le repère du vaisseau ; R8 ciel en `ship` ou `inertial`, alerte à 0,5° contre la navigation. T1 à T6 tests et mission de test.

Mise en œuvre : `core/frames.py` (couche calculateur) ; `Sphere._mask` teste la boîte ou la calotte dans le repère du secteur ; `optimizer.run`, la redondance (la découpe automatique ne combine que des boîtes du même repère), l'estimation rapide et les vues 3D travaillent sur la mission passée dans le repère du vaisseau ; interface : groupe « Navigation », repère du secteur (avec l'altitude du limbe et les deux directions), vue 3D « ship » ou « ICRF » (caméra dans l'ICRF et axes ICRF) ; colonnes CSV ; `tool.frames` hors de l'empreinte des fichiers dérivés.

Mise au point : le test de rotation de 90° a révélé une matrice de secteur appliquée dans le mauvais sens (vaisseau ← ICRF au lieu de vaisseau → ICRF ; le ciel était juste) ; corrigé. La mission de test, d'attitude symétrique (rotation de 180°), ne le montrait pas.

Effet (valeurs de test seulement) : aucune valeur figée existante ne change (les missions sont dans le repère du vaisseau) ; nouvelle mission `mission_frames.json` et son instantané. Le test de rotation montre un écart de moins de 1 % sur le bilan du ciel d'un secteur qui contient la Lune, dû à l'échantillonnage du bruit du ciel par blocs, qui n'est pas invariant par rotation (et de 4 % avec le Soleil, source ponctuelle chaude) : effet de la trame, pas de la physique. C'est une limite du modèle (bilan de préparation de mission et future enveloppe d'attitudes), pas de la simulation objet par objet si elle calcule le T_sys à la direction exacte : correction par raffinement local autour des corps chauds en 3.1 (option 1, validée par l'utilisateur).
