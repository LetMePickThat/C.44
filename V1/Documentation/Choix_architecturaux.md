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
- **Bande S, de 2,60 à 2,93 GHz** (2,7 à 2,9 GHz jusqu'à l'étape 3.1) : veille, c'est-à-dire la recherche des cibles.
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
| Simulateur de scénario et architecture | Le simulateur radar sera une partie d'un **simulateur global de scénario** (règle de l'étape 1.5). **Depuis l'étape 3.0, architecture à quatre couches** séparant la **vérité** (connue de la seule simulation) de la **connaissance** (ce que le vaisseau sait) : simulation globale (orbites, objets, propagation newtonienne), convertisseur (vérité vers repère vaisseau pour la simulation RF ; message de navigation pour le calculateur), **calculateur radar** (ce que fait l'outil : il ne connaît que sa navigation et ses détections, fait suivre les volumes au vaisseau, ordonnance, choisit les formes d'onde), simulation RF (joue les ordres du calculateur face à la vérité). Les volumes et le ciel sont donnés dans un **repère de mission** ; le calculateur applique la rotation par l'attitude. Le prototype `RADARv5/SIMULATOR` n'est qu'une preuve de concept : rien n'en est repris. Détail : `Feuille_de_route.md`, étape 3.0, et §7.14. |
| Valeurs ajustables | Les valeurs peuvent encore changer (par exemple une bande jouable plus large si les TRM progressent) : tout ce qui en dépend est **calculé** à partir du fichier, et le chargeur refuse une combinaison incohérente avec un message clair (règle fixée à l'étape 1.4). |
| Sources | Le dossier `RADARv5` est **obsolète** : il ne sert pas de source de valeurs ni de modèles (règle fixée à l'étape 1.4). |
| Commits | Sur demande seulement (« c, p » = commit et push). Un commit par étape ; **depuis l'étape 3.1, un commit par passe** quand l'utilisateur le demande. **Depuis le 23 octobre 2026 (P5)** : suite complète, valeurs figées, rapport d'impact et table de coût **au plus une fois par jour, voire tous les deux jours, sur demande de l'utilisateur** (protocole de test) ; à la passe intermédiaire, self-test et tests unitaires nouveaux ou touchés, et le commit le dit (« suite complète non relancée »), l'état intermédiaire étant aussi noté dans `JOURNAL.md`. |
| Travail par passes | Les étapes lourdes se codent en **passes avec une pause** après chacune (demande de l'utilisateur, à cause de son budget de jetons) : l'état et la passe suivante sont écrits ici (§1.5) avant chaque pause. |
| Temps de calcul | **Règle changée le 25 octobre 2026 (décision de l'utilisateur)** : la règle de précision absolue (étape 3.1 : « la précision du modèle n'est jamais réduite pour gagner du temps ») est **supprimée**, les tests demandant trop de temps et de ressources machine. Des approximations sont admises si elles **ne dénaturent pas le modèle** : chacune est proposée avec son effet mesuré ou estimé sur les résultats et son gain de temps, puis validée par l'utilisateur. Hors Pmiss, le simulateur global de scénario demandera probablement des approches encore plus simplifiées pour le temps réel ou quasi réel (note du 25 octobre 2026 ; renvoi en 4.3, 4.4, 5.4). |
| Protocole de test | **Décidé le 23 octobre 2026.** Trois niveaux, choisis et lancés par `python tests/run_tests.py` (qui donne la raison et la ligne à mettre dans le message de commit) : **N1** self-test et tests de moins de 10 s (dernière durée connue) ; **N2** N1, plus les tests que pytest-testmon trouve touchés par le code Python modifié, plus ceux qui dépendent des modules lisant une entrée `[tool.*]` modifiée, **jamais les tests des valeurs figées** (`tests/test_snapshots.py`, option b de l'utilisateur : ils exécutent presque tout le code, testmon les choisirait à chaque changement ; une régression non voulue est donc vue au N3 suivant, au plus une passe plus tard) ; **N3** la suite entière, puis table de coût, rapport d'impact, valeurs figées et journal. **Fréquence (décision de l'utilisateur, 23 octobre 2026, au soir)** : la batterie complète (N3) tourne **au plus une fois par jour, voire tous les deux jours**, quand l'utilisateur la demande ; plus de 30 min de tests est inacceptable et le travail sur le modèle passe avant. Entre deux N3, rien ne tourne sans son accord : on propose au plus un N1 ou quelques tests ciblés, avec leur durée. Une valeur du modèle modifiée ou la fin d'une étape se signalent, et le N3 attend le prochain créneau choisi par l'utilisateur. **Avancement mesuré** : `tests/.progress.txt` (tests faits et à faire, part du calcul total faite, pondérée par les dernières durées, temps restant estimé, tests en cours), durées gardées dans `tests/.test_durations.json` ; les tests les plus longs démarrent en premier. Fichiers locaux, hors dépôt (`.git/info/exclude`). GPU non retenu (double précision lente sur la carte, calculs faits de petits tableaux) ; Numba retenu pour le bruit des corps. **Depuis le 25 octobre 2026**, `python tests/run_tests.py N3` fait toute la passe avec une **barre de progression globale** (pondérée par les durées du dernier passage, `tests/.n3_durations.json`, lisible aussi dans `tests/.n3_progress.txt`) : self-test, table de coût, chaque cas de test calculé **une seule fois** dans son propre processus (gardé avec l'empreinte du code, T1), rapport d'impact (`tests/.impact_last.md`), suite complète ; le regel (`update_snapshots.py`, instantané depuis les cas calculés) reste une décision prise après lecture du rapport. |
| Paramètres de mission | Les besoins de mission (portées, revisites, cibles des volumes) se fixent dans la préparation de mission par l'opérateur (Pmiss), pas en construisant l'outil ; les missions de test ne sont pas des besoins (exemple : B2_AUTOPROT à 20 km). |
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
| En-tête de `core/radar_model.py` | Le journal des versions du modèle (V6.0.0 à V6.27.0) et ce que chaque version a changé. |
| `tests/snapshots/JOURNAL.md` | Chaque régénération des valeurs figées de test et ses causes. |
| `Documentation/design/radar_design.tex` et `.pdf` | Document de conception **en anglais** (choix de l'utilisateur, exception à la règle des langues), pour un lecteur tiers : architecture du radar, modélisation RF complète, choix. À tenir à jour aux étapes qui changent le modèle ; compilation : `tectonic radar_design.tex` (moteur LaTeX autonome, à télécharger sur la machine) ou pdflatex. Le document en français `radar_design_fr.tex` / `.pdf` suit les mêmes règles. Sur le fixe (26 octobre 2026) : MiKTeX installé pour l'utilisateur (`%LOCALAPPDATA%/Programs/MiKTeX/miktex/bin/x64/pdflatex.exe`, paquets installés à la demande), 3 passes pdflatex avec `-output-directory` hors du dépôt, puis copie des deux PDF (aucun fichier auxiliaire dans le dépôt). |
| Mémoire de l'assistant (hors dépôt) | Locale à chaque machine, **non partagée** : elle n'est qu'un raccourci. Tout ce qui est nécessaire pour reprendre est dans le dépôt (ce tableau, §1.3, §1.5, §1.6) ; en cas d'écart, le dépôt fait foi. |

Reprendre sur une autre machine :
1. `git clone https://github.com/LetMePickThat/C.44.git`, ou `git pull` dans le dépôt existant (le travail se fait sur `main`). Après la réécriture de l'historique du 22 octobre 2026, un ancien clone doit se recaler : `git fetch` puis `git reset --hard origin/main`, s'il n'a pas de travail local à garder.
2. Python 3.11 ou plus (testé en 3.13), puis dans `V1/` : `python -m pip install --user -r requirements-dev.txt` (inclut `requirements.txt`, pytest et **pytest-xdist**, indispensable : `pytest.ini` lance la suite en parallèle avec `-n 6`). Sous Windows avec le Python du Microsoft Store, ajouter au PATH le dossier `Scripts` des paquets utilisateur (`...\LocalCache\local-packages\Python313\Scripts`) pour lancer `pytest` directement ; sinon `python -m pytest`. Git n'a pas d'identité configurée sur ces machines : commiter avec `git -c user.name="LMPT" -c user.email="72825026+LetMePickThat@users.noreply.github.com" commit ...` et pousser avec `git push origin main`.
3. Vérifier l'installation : `python core/radar_model.py --selftest`, puis `python -m pytest -q` depuis `V1/` (tout doit passer, voir §4).
4. Lancer l'outil : `python main.py` depuis `V1/`, ou le lanceur `Radar mission prep.bat`.
5. Lire `Feuille_de_route.md` (tableau d'avancement : la prochaine étape), §1.5 (travaux en suspens) et §1.6 (connaissances de travail), puis suivre la procédure d'une étape.

Procédure d'une étape :
1. Rappeler la raison d'être, les entrées et les sorties de l'étape.
2. Poser les questions variable par variable, regroupées par sujet, et attendre les décisions.
3. Coder : entrées dans `radar_system.toml` (valeur, unité, statut, source, date), version du modèle et journal en tête de `core/radar_model.py`, self-test et `tests/test_physics.py`.
4. Régénérer la table de l'estimation rapide : `python tools/gen_cost_table.py` (les caches du diagramme et de la structure se reconstruisent seuls ; les commiter s'ils changent).
5. Lire le rapport d'impact, `python tests/impact_report.py` (chiffres clés, formes d'onde et modes de confirmation qui changent), et l'inscrire dans `tests/snapshots/JOURNAL.md`.
6. Régénérer les valeurs figées, `python tests/update_snapshots.py`, puis lancer les tests du niveau voulu par `python tests/run_tests.py` (protocole de test, §1.3 ; `python -m pytest -q` lance toujours la suite entière).
7. Mettre à jour ce document et `Feuille_de_route.md`, puis vérifier par une recherche que chaque passage modifié est présent (§1.6).
8. Commiter et pousser sur demande (§1.3 : message neutre, sans signature d'assistant). Le hash du commit d'une étape est reporté dans le tableau d'avancement de `Feuille_de_route.md` au commit de documentation suivant.

### 1.5 Travaux en suspens hors feuille de route (mis à jour le 27 octobre 2026)

| Sujet | État | Quoi faire |
|---|---|---|
| Durée de la suite de tests | Résolu. L'écart observé sur le portable (jusqu'à 3,5 fois plus lent sur batterie, cause précise non identifiée) venait de la machine, pas du code. Depuis le 22 octobre 2026, la suite tourne en parallèle (pytest-xdist) ; durées de référence au §1.6. | Pour comparer deux machines, mesurer `mission_example` en séquentiel (§1.6), sur secteur et en mode performance. |
| Hash des commits | 2.4 `008d583` ; 2.5 `baeadcf` ; géométrie 3D V6.10.1 `9ea4985` ; 3.0 `22114fd` ; tests en parallèle `146cae9` ; 3.1 P1 `90555d8` ; bande S élargie `b4be91e` ; 3.1 P2 `cd97216` ; temps de calcul X1-X5 `f6a526c` ; 3.1 P3 `b0dbc09` ; 3.1 P4 `f8fa433` ; 3.1 P5 intermédiaire `d19829e` ; puis 5.2 P6 `78ba4e1`, 5.2 P7 `db1af20` (regel `bb2e170`), 5.3 P1 `37bcd0c`, 5.3 P2 `8565545`, 5.3 P3 `ed15e4a` (la suite au tableau d'avancement de `Feuille_de_route.md`). | — |
| Prochaine étape | **Étape 5.3 (moteur de validation) fermée le 27 octobre 2026, modèle V6.27.0** : P1 verdict, P2 conseil chiffré, P3 redondances et recouvrements ; P4 sensibilité à la menace retirée (utilisateur : le verdict porte sur les valeurs données, développement futur potentiel) ; regel fait (N3 34,3 min sur le bureau, 241 tests passés, 66 valeurs ajoutées). **Étape 5.4 (Monte-Carlo) : cadrage validé le 27 octobre 2026** (§11, « Étape 5.4 »). | Coder la **passe M1** (moteur à événements `core/montecarlo.py` et cibles test, Pd cumulée) ; N2 seulement à la fin de M1, sur accord de l'utilisateur. |
| Sauvegardes locales de la réécriture d'historique | Sur le portable seulement : branche `backup-before-rewrite` et `refs/original/`, gardées à la demande de l'utilisateur ; un `git stash` de fichiers de cache locaux (27 octobre 2026). | Les supprimer quand l'utilisateur le dira ; rien à faire sur une autre machine. |
| Fichiers réécrits par le portable | `core/structure_cache.npz` (et parfois `core/cost_table.json`) sont réécrits par les calculs du portable ; un regel y réécrit aussi `tests/snapshots/model.json` avec un bruit de dernier chiffre. | Sur le portable : ne pas les commiter, restaurer `model.json` si le cas « model » n'a pas changé. Sur le fixe, la règle du §1.4 point 4 s'applique (commiter les caches s'ils changent). |
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
- Le profil `cProfile` d'une mission et les temps de référence ci-dessous détectent vite une régression. Pour un calcul qui dure, `py-spy dump --pid <pid>` montre où il en est sans l'arrêter, et `py-spy record --subprocesses` profile aussi les processus de fréquences.
- Ne pas restaurer `core/structure_cache.npz` depuis git en cours de passe : quand le fichier de paramètres a changé, chaque processus le reconstruit alors à chaque calcul (26 % du temps mesuré le 23 octobre 2026). Il se régénère à la passe complète.
- Un cache `lru_cache` trop petit se vide en permanence quand l'espace de recherche grandit (8 pas de grille à P5 : 672 000 recalculs pour 104 000 lois distinctes) : vérifier `cache_info()` après chaque élargissement.

**Temps de référence au 25 octobre 2026** (V6.13.0, au plus 8 processus) : `mission_example` 40 s (4 processus ; 318 s en V6.12.0), `mission_sky_demo` 70 s (501 s), N3 complet 18,7 min (cas de 0,2 à 3,6 min, suite pytest 2,7 min). Les temps ci-dessous sont ceux du 23 octobre.

**Temps de référence** (fixe 9800X3D, 16 fils, V6.11.7, 23 octobre 2026, après A1-A5 et Numba ; 8 pas de grille par secteur depuis P5) :

| Opération | Durée |
|---|---|
| `mission_example` (séquentiel, `RADAR_FREQ_PARALLEL=0`) | 188 s (1 727 s avant A1-A5) ; 146 s avec les fréquences en parallèle |
| `mission_sky_demo` (séquentiel) | 478 s avec Numba, 1 242 s sans |
| `python tools/gen_cost_table.py` | 3 min |
| calcul de tous les cas de test (rapport d'impact et gel en un seul calcul, fréquences en parallèle) | 49 min, dont `mission_frames` 14 min et `mission_redundancy_demo` 10 min |
| N1 (`python tests/run_tests.py N1`) | 1,1 min (143 tests) |
| tests de physique complets (164) | 4 min 42 en parallèle (6 processus) |
| N3 complet | plus d'une heure : les tests des valeurs figées recalculent chaque mission en séquentiel (`mission_frames` seul plus de 45 min) ; à réduire avant le prochain N3 |
| reconstruction automatique de `core/structure_cache.npz` / `core/antenna_cache.json` | environ 23 s / 8 s |

| Opération | Durée |
|---|---|
| `python -m core.optimizer tests/missions/mission_example.json` (séquentiel) | environ 47 s ; environ 42 s avec les fréquences en parallèle (X3, par défaut ; `RADAR_FREQ_PARALLEL=0` le coupe). **Après P3 (V6.11.5), mesuré sur le portable** : 67 s en séquentiel (55 s avant P3 sur la même machine, mesures enchaînées), 71 s en parallèle (71 s avant P3) : X3 n'apporte plus de gain sur cette mission (caches froids dans chaque processus, moins de travail par fréquence depuis le raccourci exact de la superposition) |
| `python tools/gen_cost_table.py` | environ 1 min (fixe) à 1 min 40 (portable) ; 2 min 17 sur le portable après P3 |
| `python tests/impact_report.py` | environ 3 min avant P2 ; plus long depuis P2 (espace de recherche plus grand) ; environ 20 min sur le portable après P3 (mesure faite avec un second calcul en concurrence), 19 min après P4 |
| `python tests/update_snapshots.py` | environ 6 min depuis P2 ; 18 min 43 sur le portable après P3, 22 min 33 après P4 |
| `python -m pytest -q` (162 tests depuis P3) | environ **5 min** en parallèle (6 processus) depuis P2 et X1-X5 ; 6 min 30 à 11 min 50 sur le portable après P3 (deux passages) ; `-n 0` = séquentiel, beaucoup plus long |
| `python tests/run_tests.py N3` (passe complète) | 23 min sur le fixe à la 5.2 (P6) ; **34,6 min après la 5.2 P7** (table de saturation sur les missions avec poursuite, dont la suite de tests 12 min) ; `tests/update_snapshots.py` ensuite instantané (cache des cas) |
| reconstruction automatique de `core/structure_cache.npz` / `core/antenna_cache.json` | environ 23 s (modèle 3D) / 8 s |

**Fichiers dérivés** (§2.2) : `antenna_cache.json` et `structure_cache.npz` se reconstruisent seuls quand leurs entrées changent, et ne sont réécrits que pour le fichier de paramètres par défaut. Ils sont versionnés : les commiter avec l'étape qui les change. `cost_table.json` et les valeurs figées se régénèrent à la main (procédure, points 4 à 6).

**Comportements de l'outil bons à savoir.**
- Une mission sans clés nouvelles se comporte comme avant chaque étape (valeurs par défaut neutres : classe GENERIC, aucun objet, pas de brouilleur, pas de Faraday).
- L'interface conserve les clés qu'elle n'édite pas encore : `target_type`, `range_min_km`, le ciel, les brouilleurs.

**Hypothèses de conception données par l'utilisateur (à garder en tête).**
- Volume d'autodéfense : ultime protection contre de petits objets (micrométéorites, KKV, microdébris) de 0,01 à 0,1 m² ; portée minimale d'intérêt la plus courte **2 km** ; la saturation des CAN est acceptable si la portée non saturée reste de l'ordre de quelques km (elle est de 0,5 à 1,3 km sur cette gamme). Aucun volume réel n'a une portée limitée à 20 km (étape 3.1, bloc B).
- Dalle : la composition est entièrement dérivée par le code dans la famille actuelle ; les contrôles d'une composition révisée (tenue sur le panneau du modèle 3D, test de bout en bout) se feront lors d'une révision réelle (point ouvert de `Feuille_de_route.md`).
- Le multifaisceau Ku est à étudier en 3.2 (latence), pas comme levier de bilan ; la capacité du formateur de faisceaux et le calcul relèvent de 4.4.

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
| `core/redundancy.py` | Détecte si un secteur couvre déjà le besoin d'un autre, dans les deux sens. Avant le calcul (onglet de préparation, plans de référence) et, depuis la 5.3 P3, à la fin du calcul avec les plans retenus (`in_plan`, clé `redundancy`). |
| `core/validation.py` | **Étape 5.3.** Verdict de la mission (clé `verdict`) : une table des contraintes (secteur, confirmation, poursuite, face, vaisseau, vol), chacune en charge = besoin / limite et en état OK, ALERTE ou NOK ; ressource limitante, niveaux de priorité en défaut, état de saturation. Aucune physique : il lit les sorties des autres modules. |
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

**Bilan actuel (étape 3.1, passe P4) : 290 entrées système, dont 247 V, 43 D et aucune « ? ».** P4 ajoute `processing.doppler_window_options_db` (V). Les deux dernières entrées « ? », `multibeam.l8_db` et `multibeam.p`, sont remplacées par les modes du multifaisceau (§5.8) . Le bilan n'avait pas été tenu à jour depuis l'étape 2.5 : avant P3, le fichier comptait 241 V et 43 D.

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
| `doppler_window` | Pondération Doppler. | "none" | V | **C3.** Pas de fouillis à rejeter dans l'espace. **Étape 2.4, D1** : le masquage par une cible forte est donné en sortie (premier lobe Doppler à −13,3 dB, le point faible de l'absence de fenêtre) ; son effet sur une scène dense relève de la phase 6 (était 5.4, 27 octobre 2026). |
| `doppler_window_options_db` | Pondérations Doppler de Taylor que l'optimiseur peut donner à un train cohérent de veille S. | −30 / −40 dB | V | **Étape 3.1, P4, Q5** : proposées seulement là où une surface dont le scénario donne la vitesse de rapprochement peut ramener du fouillis dans le secteur ; perte de pondération (loi de Taylor 1D, nbar 5) et filtre élargi calculés. La valeur par défaut reste `doppler_window` = "none". |
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
| `overhead_frac` | — | **supprimé (étape 4.1)** | — | Remplacé par la consommation permanente ci-dessous (≈ 106,9 kW par face) et les pompes. |
| `adc_w` | Un CAN (250 Méch/s, 13 bits), deux par TRM, toujours allumés. | 0,1 W | V | Étape 4.1, B1 (bout optimiste ; aujourd'hui 0,3 à 1 W) : S 1,0 kW, Ku 24,5 kW par face. |
| `dbf_kw` | **Plafond** de la puissance de formation de faisceaux et de traitement numérique par face (étape 4.4, C4 ; était la consommation permanente de l'étape 4.1, B3). | 50 kW | V | Il fixe la capacité de calcul (`core/compute.py`) et reste le chiffre permanent des bilans de puissance et de chaleur qui dimensionnent le plan (prudent) ; la consommation calculée en 4.4 est rapportée à côté. |
| `control_kw` | Contrôle et servitudes par face. | 2 kW | V | Étape 4.1, B4. |
| `pump_frac` | Pompes du refroidissement, part de la chaleur dissipée moyenne. | 0,02 | V | Étape 4.1, B5 ; à reprendre en 4.2. |
| `face_feeder_max_mw` | Puissance moyenne maximale que l'alimentation d'une face peut passer. | 8 MW | V | Étape 4.1, A : une face peut prendre tout le budget du vaisseau si les autres sont au repos. |
| `face_store_mj`, `face_store_max_mw`, `store_dod` | Stockage d'énergie par face (S + Ku), puissance de décharge, profondeur utilisable. | 5 MJ, 15 MW, 0,8 | V | Étape 4.1, D1-D3 (supercondensateurs ≈ 30 Wh/kg en 2050, ≈ 46 kg). |
| `face_elec_mw` | **Allocation de référence** par face = total du vaisseau / 4. | 2,0 MW | V | **Étape 4.1** : l'optimiseur de veille arbitre énergie et temps avec elle ; les faces puisent en commun dans les 8 MW. La clé de mission `face_elec_mw` devient un **plafond facultatif** par face (nombre, liste de 4, 0 = aucun). |
| `ship_elec_total_mw` | Puissance totale du vaisseau disponible pour le radar. | 8,0 MW | V | **Répartissable à la demande entre les faces**, par exemple 3 MW sur F1 et 5 MW sur F2 à F4. Le logiciel radar gère cette répartition avec les autres paramètres (étapes 4.1 et 5.2). Aujourd'hui, la validation vérifie seulement qu'une face ne dépasse pas 8 MW : c'est temporaire, sur décision de l'utilisateur. |

**Depuis l'étape 4.1** : CHAIN_EFF = 0,95 × 0,995 = 0,945 ; consommation permanente 106,9 kW par face ; pour la référence de 2 MW, la face dispose de 2 × 0,945 − 0,107 = 1,784 MW pour l'émission. Par bande, `trm_supply_w` (S 1 700 W, Ku 80 W) et `rx.rx_chain_w` (S 0,5 W, Ku 0,1 W). Ce qui suit décrit l'état antérieur. Valeur dérivée jusqu'à l'étape 4.1 : CHAIN_EFF = 0,95 × 0,995 × (1 − 0,03) = 0,917. **Depuis l'étape 1.3, le bilan de la face est électrique** (Q1) : pour 2 MW alloués, la face dispose de 2 MW × 0,917 = 1,834 MW pour l'émission, et chaque observation coûte son énergie RF divisée par la PAE de sa bande et de sa forme d'onde. Auparavant, la conversion se faisait une fois par face avec une PAE unique (1,375 MW RF).

### 5.6 bis `[thermal]` : thermique (étape 4.2)

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `coolant_cp_kj_kgk` | Capacité calorifique du fluide (dérivé du submer SmartCoolant, diélectrique). | 2,5 kJ/(kg·K) | V | 2,26 en 2020, +10 % en 2050-2060 (donnée et choix de l'utilisateur). |
| `coolant_h_w_m2k` | Coefficient d'échange dans les microcanaux. | 3 000 W/(m²·K) | V | ≈ 2 050 en 2020 en circulation forcée, +50 %. |
| `coolant_density_kg_l` | Densité du fluide. | 0,8 | V | 0,796 en 2020. |
| `microchannel_area_factor` | Surface mouillée des microcanaux sur l'empreinte. | 10 | V | — |
| `plate_fluid_dt_max_k` | Écart plaque–fluide maximal. | 40 K | V | Plaque ≤ 60 °C pour un fluide à 20 °C. |
| `coolant_rise_k`, `coolant_inlet_c` | Échauffement du fluide sur une face, température d'entrée. | 10 K, 20 °C | V | — |
| `fluid_volume_l`, `structure_heat_capacity_kj_k`, `plate_rise_max_k` | Inertie d'une face (fluide + structure) et échauffement admissible en rafale. | 50 L, 90 kJ/K, 10 K | V | Tampon ≈ 1,9 MJ. |
| `tj_max_c` | Température maximale de jonction GaN. | 225 °C | V | Bout optimiste de 200-225 °C. |
| `ship_heat_rejection_mw` | Rejet du vaisseau réservé au radar, commun aux faces. | 6 MW | **D** | Provisoire : donnée du vaisseau à confirmer. |
| `band.<b>.rth_fast_k_w`, `tau_fast_s`, `rth_slow_k_w`, `tau_slow_s` | Réseau de Foster jonction–plaque d'un amplificateur de TRM. | S 0,02 K/W (10 µs) + 0,03 K/W (1 ms) ; Ku 0,2 K/W (5 µs) + 0,3 K/W (0,5 ms) | V | GaN sur diamant. |

Valeurs dérivées : flux admissible 120 W/cm², capacité de refroidissement ≈ 15,9 MW par face, résistance de convection par TRM 0,012 K/W (S) et 0,31 K/W (Ku). Réglage d'outil `tool.thermal.junction_check` = false (B6 : aucun effet mesuré, `duty_max` gardé).

### 5.7 `[migration]` : migration en distance

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `mode` | Traitement de la migration en distance. | "COMPENSATED" | V | Traitement de type keystone ou Radon. Le coût des bancs de vitesse sera évalué aux étapes 2.2 et 4.4. |
| `residual_db` | Résidu de l'algorithme de compensation (keystone ou Radon) quand la cible traverse plus de 1,5 case. | 0,1 dB | V | **Étape 2.2, B5**, bout optimiste de 0,1 à 0,3 dB. Était 1,0 dB forfaitaire ; la perte du banc de vitesses est désormais calculée. |
| `bank_max_loss_db` | Pas du banc d'hypothèses de vitesse : perte d'une cible à mi-pas. | 1,0 dB | V | **Étape 2.2, B2** (plage 0,5 à 2 dB). Loi = perte dans l'impulsion (étape 2.1) + glissement sur l'observation. La perte **moyenne** sur le pas va au bilan, le nombre d'hypothèses aux fausses alarmes et au calcul (étape 4.4). |

### 5.8 `[multibeam]` : multifaisceau en émission

Étape 3.1, bloc C (passe P3, modèle V6.11.5). Émission S seulement : la réception numérique forme déjà ses faisceaux (perte 0, coût en calcul à l'étape 4.4) ; le multifaisceau Ku est inscrit en 3.2. Chaque forme d'onde porte un **mode d'émission** (`tx_mode`), un nombre de faisceaux B (`b_tx`) et, pour la partition en deux, une coupe (`tx_cut`).

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `superposition_beams`, `superposition_loss_db` | Superposition en phase seule : chaque TRM émet la somme des B faisceaux à enveloppe constante (PA saturés, PAE inchangée), P / B par faisceau, gain d'émission complet. | 2 / 4 / 8 faisceaux : 0,4 / 0,6 / 0,8 dB | V | Bloc C. Remplace la loi `l8_db` × (log₂B / 3)^p de V5.6.0 (définition inconnue, statut « ? »), supprimée avec `p`. |
| `partition_beams` | Partition de l'ouverture : 2 moitiés (coupe horizontale `h` ou verticale `v`) ou 4 quadrants ; P / B et G × N_sub / N par faisceau ; faisceau de la sous-ouverture calculé (`core/antenna.py`, diagramme 2D). | 2, 4 | V | Bloc C ; Q2 (b) : la grille des positions suit le faisceau élargi ; Q3 : les deux coupes sont évaluées. |
| `tdm_beams` | Multiplexage en temps : B impulsions pleine puissance mises bout à bout dans la PRI, sans perte. | 2, 4, 8 | V | Bloc C ; Q1 : modélisé mais hors de l'espace de l'optimiseur. Zone aveugle c·B·τ/2, réserve du TRM, rapport cyclique et PAE sur B·τ (hypothèses validées). |
| `waveform.pulse_tb_min` | Impulsion la plus courte de la réduction « pas d'énergie en trop » : τ ≥ cette valeur / B. | 1 | V | Q5 (a) : produit temps × bande 1, impulsion non modulée, limite physique. Au plancher, l'énergie en trop est gardée. |
| `tool.optimizer.tx_modes` | Modes cherchés par l'optimiseur. | single, superposition, partition | — | Le multiplexage en temps en est exclu (Q1). |

Superposition linéaire et multiplexage en fréquence écartés (bloc C).

**Faisceau des sous-ouvertures** (octogone actuel, S) : moitié 2 450 TRM, largeurs 1,041 × 1,928 λ/D ; quadrant 1 225 TRM, 1,928 × 1,928 λ/D (pleine ouverture 1,041 × 1,041). La cellule de la grille de veille d'une sous-ouverture est celle de la face **étirée par les rapports des largeurs** selon u et v : ×1,85 pour une moitié, ×3,43 pour un quadrant (moins que 2 et 4 : une moitié d'octogone n'est pas deux fois moins large en largeur effective). Ses pertes de grille viennent du diagramme 2D réel sur cette cellule, le premier axe du réseau hexagonal le long de l'axe étroit du faisceau (3,30 dB en moyenne pour une moitié, 3,33 dB pour un quadrant, contre 3,37 dB pour la face). **Les deux moitiés sont le même faisceau tourné de 90°** (octogone symétrique en x et y) : même coût, une seule est cherchée tant que c'est vrai (contrôle exact dans le code, `partition_cuts`).

**Coût d'une position** de la grille de la face (`radar_model.search_position_costs`) : une observation émet `trains_per_dwell` trains à la puissance crête de la face (B pour le multiplexage, 1 sinon) et couvre `cells_per_dwell` positions (1 ; B pour la superposition et le multiplexage ; B × le rapport des cellules pour la partition). Énergie électrique = P_pk · n · τ · trains / positions par observation / PAE ; temps d'émission = n · τ · trains / positions par observation ; durée de trame = positions × temps par position / positions par observation.

**« Pas d'énergie en trop » par raccourcissement de l'impulsion** (bloc C, Q4, Q5) : là où une forme d'onde dépasse la portée demandée, l'impulsion est ramenée à s·τ avec s = (R_voulue / R)⁴, bornée par `pulse_tb_min` / B ; les PA restent saturés. L'énergie **et le temps d'émission** diminuent ; la PAE est celle de l'impulsion raccourcie (exacte : temps de front et rapport cyclique) ; les pertes de l'impulsion entière sont gardées (prudent : l'affaissement de la réserve et la zone aveugle ne peuvent que diminuer). Avant V6.11.5, la puissance était réduite (énergie seulement, PAE constante). Sortie : `pulse_scale` (était `power_scale`). Conséquence : sur une position à portée facile, le coût d'une impulsion ne descend plus sous P_pk × (1/B + t_front) / PAE_sat, et la partition, qui répartit ce coût sur 3,7 ou 13,7 positions, peut devenir la moins chère.

**Constat** (bloc C) : dans le décompte du temps d'émission avec entrelacement, la superposition de B faisceaux sur une impulsion τ équivaut presque à un faisceau sur τ/B, perte en plus ; la partition à peu près à un faisceau sur τ/B² avec une cellule un peu trop petite ; le multiplexage en temps à un faisceau. L'intérêt réel du multifaisceau est en 4.3 (chronologie, latence).

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
| `usable_lo_ghz` / `usable_hi_ghz` | Bande utile. | **2,60 à 2,93 GHz** (2,7 à 2,9 jusqu'à l'étape 3.1) | D | **Étape 3.1, W1-W2 (utilisateur).** Vers le bas, aucune limite de lobe de réseau (désadaptation inchangée jusqu'à 2,55 GHz) : limite du TRM et de l'élément. Vers le haut, le pas moyen de 52,1 mm fixe le mur : désadaptation à 70° de 0,97 dB par passage à 2,93 GHz (la valeur déjà acceptée à 75° à 2,7 GHz), 2,35 dB à 2,95 GHz, lobe de réseau visible à 3,0 GHz. |
| `core_lo_ghz` / `core_hi_ghz` | Bande centrale, sans perte de bord. | **2,66 à 2,88 GHz** (2,74 à 2,86 jusqu'à l'étape 3.1, W3 : les deux tiers de la bande utile) | D | **Étape 1.4, A1-A2.** Donnée en GHz pour chaque bande. Remplace `core_inset_divisor` = 6 (bande utile amputée d'un sixième de chaque côté), une proportion copiée de l'ancienne bande Ku de 14,8 à 17,2 GHz. Valeur optimiste 2050-2060 pour une bande relative de 7 %. |
| `edge_derating_db` | Perte **totale** (aller-retour) aux bords de la bande utile ; cosinus surélevé entre la bande centrale et les bords. | **1,0 dB** (0,5 dB jusqu'à l'étape 3.1, W4 : bande relative de 12 % au lieu de 7 ; porte aussi la baisse de PAE hors de la bande centrale, `pae_sat` 0,80 étant gardée, W7) | D | **Étape 1.4, A2-A4.** Rendement des PA et LNA, filtres et WAIM désaccordée hors de la bande centrale. Elle est **moyennée sur le spectre occupé** [f − B/2, f + B/2]. Ancienne valeur 1,5 dB, copiée de l'ancienne bande Ku. |
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
| `calibration` | Perte de gain des erreurs résiduelles du réseau, émission + réception. | **dérivée**, ≈ 0,006 dB | V | **Étape 3.4** : remplace le résidu forfaitaire de 0,5 dB (décision de l'utilisateur) ; Ruze sur les erreurs de `[band.S.errors]` en fin d'intervalle d'étalonnage. |
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
| `calibration` | Perte de gain des erreurs résiduelles du réseau, émission + réception. | **dérivée**, ≈ 0,006 dB | V | **Étape 3.4** : remplace le résidu forfaitaire de 0,8 dB (D) ; mêmes erreurs qu'en S (`[band.Ku.errors]`). |
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

**Étape 3.3 (V6.14.0)** : nouvelles clés de classe. `maneuver_tau_s` (constante de temps de Singer, A5, V) : ST 0, GENERIC 20 s, KKV 5 s, SHIP 60 s, DEBRIS 0, DECOY 5 s (sans effet : `accel_max_mps2` = 0) ; 0 = pas de manœuvre. Facultatives (F6) : `aspect_offset_deg` (aspect d'une cible qui se rapproche, hors de son nez ; 0 par défaut) et la table `aspect_deg` / `aspect_rcs_db` (SER en dB par rapport à la SER frontale, interpolée, plate au-delà) ; aucune classe n'en a aujourd'hui. Clés de secteur correspondantes : `maneuver_tau_s`, `aspect_offset_deg`.

### 5.14 `[confirmation]` et `[tracking]`

| Variable | Définition | Valeur | Statut | Justification |
|---|---|---|---|---|
| `confirmation.ku_freq_ghz` | Fréquence Ku de confirmation et de poursuite. | 14,0 GHz | D | **Provisoire.** Elle est dans la bande centrale, sous le seuil de 14,25 GHz. « L'optimiseur **doit** choisir une fréquence » (étape 3.2), en temps réel, d'après le spectre non brouillé. |
| `confirmation.bias_mrad` | Biais résiduel d'alignement S/Ku par axe, exprimé en **angle absolu**. | 1,1 mrad | V | Il valait 1/20 du lobe S (1,13 mrad à 2,8 GHz) et aurait grandi avec la largeur de lobe. Il est désormais constant en (u, v). Étalonnage S/Ku à l'étape 3.2. |
| `confirmation.p_min` | Probabilité minimale de confirmer une vraie cible. | 0,98 | V | Exigence opérationnelle : 0,9 × 0,98 par passage de bout en bout. Elle deviendra un réglage de mission à l'étape 5.1 ; un réglage par volume reste une option à creuser. |
| `confirmation.fa_r4_mean` | Moyenne de (R/R_det)⁴ pour une fausse alarme. | 0,2 | V | Exactement 1/5 pour une fausse alarme répartie uniformément en distance sur [0, R_det]. |
| `tracking.angle_zone_sigmas` | Ouverture de la zone angulaire, en sigmas. | 4 | V | Convention : largeur totale à ±2σ. |
| `tracking.p_max`, `tracking.p0` | Probabilités de Singer de l'accélération maximale et nulle. | 0, 0 | V | Étape 3.3, A4 : σ_m² = a_max²/3 (1 + 4 p_max − p₀), médiane, pas le pire cas. |
| `tracking.process_noise_floor_mps2` | Accélération aléatoire minimale de toute cible. | 0,01 m/s² | V | A6 : dynamique relative mal modélisée ; constante de temps `tool.tracking.floor_tau_s` (60 s) pour une classe sans manœuvre. |
| `tracking.pd_update` | Pd d'une observation de mise à jour. | 0,9 | V | B5 ; réglage de mission plus tard. |
| `tracking.n_miss_lost` | Manques de suite qui perdent la piste. | 3 | V | B7, chaîne de Markov exacte. |
| `tracking.vel_bank_sigmas`, `tracking.accel_bank_sigmas` | Bancs de vitesse et d'accélération d'une observation de poursuite, en sigmas prédits. | 3, 3 | V | B3, E1 (l'accélération était supposée connue). Le banc d'accélération ne dépasse pas l'a_max de la classe. |
| `tracking.init_vel_sigma_frac` | σ initial de la vitesse transverse, en fraction de v_close. | 1/3 | V | C1, borne prudente. |
| `tracking.init_conv_frac` | Fin de l'initialisation : précisions tenues à cette fraction près. | 0,1 | V | C3 ; critère pris sur les **précisions de l'opérateur** (voir §11, étape 3.3, codage). |
| `tracking.angle_floor_mrad`, `range_floor_m`, `vr_floor_mps` | Planchers de mesure, en quadrature. | 0,05 mrad, 0,05 m, 0,01 m/s | V | E4-E6. Valeurs médianes ; les bouts optimistes (0,02 mrad, 0,01 m, 0,001 m/s) ont été proposés, l'utilisateur a gardé ces valeurs. |
| `tracking.p_discrimination` | Probabilité de discrimination voulue (KKV / leurre). | 0,95 | V | F2 ; réglage de mission plus tard. |
| `tracking.polar_switch_loss_db` | Perte d'émission de l'option de commutation e_u / e_v. | 0,15 dB | V | F5 (estimation 0,1-0,2 dB de l'étape 1.2). |
| `tracking.faraday_uncert_frac` | Incertitude de la rotation de Faraday prédite, en fraction de la rotation. | 0,2 | **D** | F4 ; valeur proposée, **à confirmer**. |
| `tracking.deception_verify_rate` | Observations de vérification par mise à jour face à un brouilleur trompeur. | 1 | **D** | F7 ; valeur proposée, **à confirmer**. |
| `tracking.track_frame_k` | Rayon d'un secteur en repère `track`, en sigmas de la désignation. | 3 | **D** | F8 ; valeur proposée, **à confirmer**. |
| `band.<b>.rx.attenuator_max_db`, `attenuator_step_db` | Atténuateur commutable après le LNA. | 30 dB, pas de 10 dB | V | E3 : portée non saturée ÷ 10^(A/40). |

Réglages d'outil `[tool.tracking]` (étape 3.3) : `n_ranges` 8 (D1), `floor_tau_s` 60 s, `rr_alert_iter` 14 (bissection de la période tenable), `disc_n_max` 200 (observations comptées pour la discrimination), `separability_n_looks` 10 (F3), `gate_cells` 100 et `false_track_flag_per_s` 1e-3 (C5).

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
| Faisceaux de réception par pointage | (1,4·HPBW_TX / (0,5·HPBW_RX))² | 6,12 (−25), 5,34 (−30), 4,79 (−35) au pas de grille de 1,4 (3,38 / 2,95 / 2,64 au pas de 1,04 d'avant l'étape 2.3 ; corrigé le 23 octobre 2026) |
| Perte de grille d'émission | moyenne sur la cellule de 1,4 HPBW, diagramme réel (étape 2.3) | 3,37 dB (loi gaussienne 1,672 × 1,4² = 3,28 dB ; 1,81 dB au pas de 1,04 d'avant l'étape 2.3) |
| Perte de grille de réception | moyenne sur le réseau de 0,5 HPBW_RX, diagramme réel | 0,40 dB à −25 dB (loi gaussienne 1,672 × 0,5² = 0,42 dB) |
| Gain en S à 2,8 GHz, dans l'axe | 4πAη/λ² | émission 41,62 dBi, réception 41,03 dBi (−25 dB) |
| Gain d'émission en Ku à 14 GHz | idem | 55,60 dBi |
| T_sys (ciel froid) | cascade de Friis | S 144,5 K, Ku 230,8 K (154,3 et 238,7 K avant l'étape 1.5) ; le ciel devant la face s'y ajoute par direction |
| Facteur de bruit du CAN | P_FS − (6,02·ENOB + 1,76) − 10 log(k T₀ fs) | 20,0 dB en S et en Ku (13 bits, +10 dBm, 250 Méch/s). Un bit de plus vaut −6 dB, 6 dB de pleine échelle en plus valent +6 dB : ici ils se compensent |
| Gain jusqu'au CAN | −L_pré + G_LNA + G_post | S 46,65 dB, Ku 43,8 dB |
| Bruit au CAN | k T_sys fs + gain − P_FS | S −56,4 dBFS, Ku −57,2 dBFS (24 dB au-dessus du bruit de quantification) |
| Niveau d'écrêtage à l'élément | P_FS − 3 dB − gain | S −39,6 dBm, soit 53,4 dB au-dessus du bruit ; Ku −36,8 dBm, 54,2 dB |
| Portée minimale non saturée (axe, pleine puissance, sans perte) | écho par élément = écrêtage | S : ST 12,2 km, KKV 2,3 km, SHIP 9,1 km ; Ku : ST 10,2 km, KKV 3,5 km, SHIP 7,6 km (SER des classes de l'étape 2.4 ; corrigé le 23 octobre 2026, les valeurs précédentes dataient des SER d'avant la bibliothèque de cibles) |
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
| Pointages par face à 2,8 GHz | surface / cellule | 1 664 au pas de 1,4 (3 016 au pas de 1,04 d'avant l'étape 2.3, 4 167 avant l'étape 1.1 ; corrigé le 23 octobre 2026) |
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

**Bruit de la structure.** Lobe principal : f × (ε·T des parties touchées) × part du diagramme de réception dans la zone proche. Lobes lointains : les directions de la structure vues du centre de l'antenne (20 000 directions sur la sphère), pondérées par ε·T, intégrées sur le profil lointain moyenné en azimut, comme un corps du ciel ; tabulé par face sur une grille (u, v) au pas de 0,05. Les réflexions sur la structure (Soleil, Terre) ne sont pas comptées (sorties du modèle le 27 octobre 2026 : développement futur potentiel, liste de `Feuille_de_route.md`).

**Surfaces.** Distance au centre D = R / sin(rayon angulaire). Une direction qui tombe dans le disque rencontre la surface à d = D cos δ − √(R² − D² sin² δ) : rien n'existe au-delà (signalé quand d est inférieure à la portée exigée). **Fouillis** dans la case de la cible, à la portée exigée r : l'anneau d'égale distance (cos α = (r² + D² − R²) / 2rD, incidence cos θᵢ = (D² − r² − R²) / 2rR), de surface 2π R r Δr / D par case Δr = c / 2B (B = la plus étroite des bandes admissibles du secteur ; 50 MHz en Ku), échantillonné sur 720 points, vu par le diagramme aller-retour (profils d'émission et de réception, facteur d'élément cos) : C/S = σ_c / σ_cible. Le fouillis étant traité comme un bruit au seuil, S ≥ SNR_req (N + C) donne le facteur 1 / (1 − x), x = (C/S) · SNR_req (SNR d'une observation au cas de Swerling de la cible) ; à x ≥ 1, aucune puissance ne suffit : pointage **limité par le fouillis**, aveuglé. Le fouillis est rejeté si le scénario donne la bande Doppler de la surface et qu'elle ne recouvre pas l'étendue de vitesses du secteur (traitement cohérent supposé). Dans le lobe principal, une cible devant la surface n'a pas de fouillis dans sa case, sauf pour un train ambigu en distance : ces trains sont retirés de l'espace de recherche d'un secteur dont un pointage voit une surface. **CAN** : écho de la surface dans le lobe principal (empreinte du faisceau × σ₀) par élément, avant la formation de faisceau, comparé au niveau d'écrêtage (alerte). Un limbe dans le secteur est signalé (GO-CFAR conseillé).

**Brouilleurs à distance.** Température équivalente T_j = (densité de PIRE / 4πR²) / k × A_eff(direction), A_eff = surface × η de réception × cos (élément) × diagramme de réception (zone proche ou profil lointain). Un brouilleur à plus d'un lobe de réception de l'axe est dans les lobes secondaires : les 8 plus forts par pointage sont annulés de 40 dB. Ordres de grandeur avec SOJ_SHIP à 10 000 km : J/N ≈ +57 dB dans le lobe principal (aveuglé), environ +11 dB par les lobes lointains sans annulation.

**Brouilleur d'autoprotection.** Portée de percée R_bt = √(P G_t σ τ N / (4π · densité de PIRE · SNR_req · L)), avec la forme d'onde principale du secteur et avec la plus énergétique de l'espace de recherche (pleine puissance, axe) ; signalée quand elle est inférieure à la portée exigée.

**Environnement par forme d'onde (étape 3.1, passe P4, modèle V6.11.6).** En S, le fouillis de surface sort du budget de T_sys du secteur et est évalué **par candidate et par tranche de 1°** :
- *Rapport fouillis sur cible* y = σ_c / σ_cible, calculé par pointage pour chaque faisceau d'émission de l'espace de recherche (face complète, ou diagramme 2D de la sous-ouverture d'une partition, Q3) à la portée exigée du pointage, pour une case de référence c / (2 × 1 MHz) ; il est linéaire en c / 2B (exact). Par candidate, il est divisé par le nombre de blocs cohérents M (intégration non cohérente) et multiplié par le **résidu Doppler** ρ(v).
- *Résidu Doppler* (Q4 à Q8) : spectre gaussien de la surface (moyenne `closing_mps`, écart type `closing_spread_mps` = 1 σ), replié sur la PRF, filtré par |Σ w_k e^(j2πkx)|² / (Σw)² d'un bloc cohérent de n impulsions (pondération uniforme, ou Taylor −30 / −40 dB), plus le plancher blanc de la phase résiduelle (oscillateur et plateforme) à travers la bande de bruit du filtre ; ρ = 1 sans vitesse de rapprochement dans le scénario ou sans bloc cohérent. Évalué à 11 vitesses de cible réparties sur l'étendue [v_min, v_max] du secteur.
- *Effet sur la portée* : le fouillis est traité comme un bruit à la détection, S / (N + C) avec C / N = a · S / N, a = y ρ(v) / M. Dans chaque tranche, y est pris au centile (1 − `short_tol`) des pointages (Q2). L'augmentation G du SNR qui rend à l'observation son Pd **moyenné sur les vitesses** (le Pd par observation de la loi sans fouillis, critère cumulé compris) est résolue par bisection (R1 b1) ; la portée devient R_n · G^(−1/4). G infini : candidate injouable dans la tranche.
- *Pointages aveuglés* (R3) : un pointage où aucune candidate ne peut battre le fouillis (même à puissance infinie) est retiré des positions ; avec plusieurs surfaces, critère additif Σ_s y_s / y*_s ≥ 1 (exact pour une surface). La Pfa des candidates est calculée sur les positions avant ce retrait (prudent).
- *Portée ramenée à la surface* (Q9, Q10, R2 c2) : sur un pointage qui rencontre une surface à d, l'exigence devient d − garde, garde = max(`surface_guard_km`, 10 km par défaut ; résolution en distance) ; arrondie **vers le haut** sur une grille de 5 % de la portée exigée (classes d'exigence) ; le critère cumulé est recalculé à la portée de chaque classe ; d − garde ≤ 0 : pas de volume, pointage non balayé. Le plan affecte toujours une forme d'onde par tranche ; l'énergie et les positions courtes sont comptées par classe.
- *Brouilleur d'autoprotection dans la portée* (Q11 à Q13) : S = SNR_req (N + J), J ∝ 1 / R², résolu exactement dans `solve()` : 1 / R² = (1 / R_bt² + √(1 / R_bt⁴ + 4 / R_n⁴)) / 2 (R_n bruit seul, R_bt brouilleur seul) ; barrage à densité de PIRE fixe sur la bande de sa classe, dans le lobe principal, non annulé. Avec le fouillis : R_n · G^(−1/4) et R_bt · G^(−1/2).
- La Ku garde le modèle de l'étape 2.5 (facteur de bruit, bande Ku la plus étroite, rejet en tout ou rien) jusqu'à l'étape 3.2.

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

**Contrôles et alertes.** Navigation : un seul message (un seul en Pmiss : pas d'enveloppe d'attitudes, décision du 27 octobre 2026, phase 6), norme du quaternion à 1e-3 près (normalisé), date ISO 8601, orbite complète (corps de `[sky.surface]`, r hors du corps, plan d'orbite défini). Secteur : repère connu, orbite donnée pour `rtn` et `limb`, vaisseau au-dessus de l'altitude du limbe, directions de `two_dir` données et non alignées. Alerte quand le corps central du ciel s'écarte de plus de 0,5° de la navigation (direction ou rayon apparent).

### 7.15 Pilotage de la charge du radar (synthèse au 26 octobre 2026)

Ce paragraphe rassemble les règles qui décident **qui reçoit quelle ressource, quand, et ce qui cède quand il n'y en a pas assez**. Elles ont été fixées étape par étape (4.1, 4.2, 4.3, 4.3 bis, 4.4, 4.5, 5.2) ; le détail et l'historique des décisions sont au §11. Sauf mention, tout se passe en **préparation de mission (Pmiss)** : l'outil dimensionne un plan et vérifie qu'il tient ; l'exécution dynamique en vol relève de la phase 6 (dernier point).

**1. Les ressources et leur mesure.** Chaque face a ses propres ressources, sauf l'énergie et le rejet de chaleur, communs au vaisseau.

| Ressource | Portée | Capacité | Mesure de la charge |
|---|---|---|---|
| Énergie électrique | **vaisseau** (8 MW répartissables entre faces) et face (alimentation 8 MW, plafond facultatif de l'opérateur `face_elec_mw`) | `ship_elec_total_mw`, `face_feeder_max_mw` | puissance moyenne tirée du bus (émission à travers la chaîne + permanent + pompes) / capacité |
| Stockage de face | face | 5 MJ, 80 % utilisables, décharge 15 MW | déficit par observation et sur la rafale la plus défavorable |
| Temps d'émission S | face | 1 (part du temps) | somme des parts de temps des tâches S |
| Temps d'émission Ku | face (réseau distinct) | 1 | idem en Ku |
| Chaleur | face (plaque froide, ≈ 15,9 MW) et vaisseau (rejet 6 MW, D) | dérivées du fluide | chaleur moyenne / capacité |
| Calcul | face | ≈ 4·10¹⁸ op/s | opérations par seconde / capacité |
| Faisceaux de réception simultanés | face et bande | 512 en S, 32 en Ku | pic de faisceaux d'une visée / capacité (ne dépend pas de la revisite) |
| Mémoire, débit mémoire | face | 4 To, 50 To/s | tampons cohérents au pic ; trafic (une lecture par visée) |

La **ressource dimensionnante** d'une face est celle dont la charge est la plus haute ; pour l'énergie, c'est le plus grand de (besoin du vaisseau / 8 MW) et (besoin de la face / son plafond).

**2. Les tâches et leurs priorités** (étape 4.3, priorités de l'utilisateur ; elles servent à l'ordonnancement, à la répartition de l'énergie et au délestage) :

| Niveau | Tâche | Ne cède jamais ? |
|---|---|---|
| P0 | étalonnage (toutes les 300 s, bloc de 10 ms en S, 50 ms en Ku, retard ≤ 10 % de l'intervalle), consommation permanente | oui |
| P1 | veille des volumes cochés « autoprotection » (`self_protection`) | oui |
| P2 | confirmation et initialisation de piste (ne jamais perdre une détection), toucher S du mode S2 | oui |
| P3 | mises à jour de piste, par classe de cible pour l'énergie : KKV > SHIP > DECOY > DEBRIS > autres (tolérance ±10 % de la période, au-delà un manque) | non |
| P3' | percée Ku dans l'axe d'un brouilleur, seulement si la mission l'autorise (`burn_through_allowed`) | non |
| P4_n | veille des autres volumes, sous-priorité `search_priority` choisie par l'opérateur (P4_1 d'abord) | non |
| P5 | **réemploi de la marge** : la part de veille due aux revisites raccourcies sous la RR demandée (point 7) ; à partir de la 5.2 P7, selon `realloc_mode`, la part de veille (`PRIO_SEARCH`) ou la part de poursuite due à une cadence accélérée (`PRIO_TRACK`) (point 10) | non, rendue en premier |

Une tâche P1 d'une face passe avant une tâche P3 d'une autre face : les niveaux valent **toutes faces confondues** pour l'énergie, qui est commune.

**3. Chaîne de calcul d'une mission** (`optimizer.run`) :
1. **Plan de veille par secteur** (§7.5) : fréquence, pas de grille, forme d'onde par tranche de dépointage, échelle d'impulsion juste suffisante pour la portée ; objectif min(max(part d'énergie, part de temps S)) sur les faces, l'énergie étant rapportée à une référence de 8 MW / 4 = 2 MW par face pour arbitrer énergie et temps. Les secteurs sont planifiés indépendamment ; une direction est servie par la face la plus proche (zones de responsabilité, recouvrements non utilisés : point ouvert).
2. **Assemblage des budgets** : totaux par face ; confirmation et poursuite Ku (mode de confirmation choisi pour baisser la face la plus chargée ; formes d'onde Ku) ; calcul et données (4.4) ; **répartition des 8 MW** (point 4) ; re-pointage (4.5) ; **ordonnanceur** (point 5).
3. **Boucle globale** (5.2, P3, au plus 3 passes) : les couplages sont réinjectés dans les plans (éclipse des créneaux communs au-delà de 0,5 dB ; coût Ku des fausses alarmes en politique `pfa_cell` ; formes d'onde demandant plus de faisceaux que le formateur exclues) ; chaque secteur peut prendre une de ses alternatives (autre fréquence ou pas de grille, même couverture et même portée) si elle baisse **la face la plus chargée de la mission**, puis la somme des charges ; seuls les secteurs touchés sont replanifiés ; arrêt quand rien ne change.
4. **Réemploi de la marge** (5.2, P4 et P7, selon `realloc_mode` : veille en `PRIO_SEARCH`, cadence de poursuite en `PRIO_TRACK`, rien en `NONE` : points 7 et 10), puis **table de saturation** (5.2 P7, point 10).
5. Verdict : par ressource et par face, sur les RR **demandées**.

**4. Répartition de l'énergie entre les faces** (`core/allocation.py`, 5.2 P1). Le besoin de chaque face est décomposé par niveau (P0 avec la consommation réelle du calcul, P1, P2, P3 par classe, P3', P4_n, P5).
- **Énergie suffisante** (somme des besoins ≤ 8 MW, chaque face sous son plafond) : chaque face reçoit une part des 8 MW **au prorata de son besoin** ; l'excédent d'une face plafonnée va aux autres.
- **Manque** : les niveaux sont servis **dans l'ordre, toutes faces confondues** ; le dernier niveau servi l'est en partie : ses volumes de veille reçoivent une revisite tenable (RR / part servie), ses pistes un nombre tenable ; si P0, P1 ou P2 n'est pas servi en entier, la mission est déclarée **non tenable**. L'énergie restante est répartie au prorata.
- **Verdict** (5.2, P6) : la charge d'énergie d'une face est **besoin / allocation** ; égale pour toutes les faces quand l'énergie suffit (besoin total / 8 MW), au-dessus de 1 pour les faces dont des niveaux ne sont pas servis. L'alimentation de la face (8 MW) et le plafond opérateur restent un contrôle à part.

**5. Ordonnancement dans le temps** (`core/scheduler.py`, 4.3). Par face et par bande (S et Ku ont des chronologies d'émission distinctes) :
- **Tâches** : une visée de veille par pointage et par trame (émission + écoute 2R/c), de période la revisite ; confirmations et initialisations à arrivées de Poisson aux taux de la face ; mises à jour périodiques ; étalonnages.
- **(a) Bilan statistique** : file à priorités **non préemptive** (Cobham) ; attente moyenne de la classe k, W_k = W₀ / ((1 − σ_{k−1})(1 − σ_k)), W₀ = Σ ρ_i S_i / 2, σ_k la charge des classes jusqu'à k ; quantile à 95 % par une queue exponentielle au-delà de la probabilité d'occupation.
- **(b) Chronologie synthétique** déterministe sur la plus longue revisite de la face (au plus 60 s) : **gloutonne, par priorité puis échéance, sans préemption**, arrivées tirées une fois (graine fixe). Elle applique les règles physiques : **émission contiguë maximale** (stockage de face, ≈ 0,29 s à pleine puissance, et plaque froide), au-delà une **pause de recharge** ; étalonnage dans son retard admis.
- **Règles entre faces et bandes** : les faces émettent une bande dans des **créneaux communs**, dont la part est fixée par la face la plus chargée ; toutes les faces sont sourdes pendant ces créneaux, d'où une perte d'**éclipse** des échos de veille −10 log₁₀(1 − ρ_créneaux), signalée au-delà de 0,5 dB et réinjectée dans le plan par la boucle globale. Sur une face où l'harmonique S tombe dans le canal Ku, les observations Ku évitent les créneaux S.
- **Latence de confirmation** : attente au quantile 95 % + service + allers-retours + traitement (4.4), contre `latency_max_s` du secteur, sinon 10 % du temps avant impact.

**6. Ordre des pointages dans un volume** (`core/scanorder.py`, 4.3 bis) : la grille hexagonale du plan, en cosinus directeurs (u, v) de la face, est servie dans un **ordre de Morton à bits renversés** (van der Corput à deux dimensions) : les 4^k premiers pointages tombent un dans chacun des 4^k sous-carrés du volume, à toutes les échelles. La couverture est donc uniforme dès le début de la trame, et **une trame étirée ou coupée par le délestage se dégrade uniformément**, sans trou. La grille est décalée à chaque trame (cycle de 3 décalages qui remplissent la maille). Chaque pointage dure le temps d'émission de la forme d'onde de sa tranche ; en multifaisceau, les pointages suivants de l'ordre sont servis ensemble. Changer de pointage ne coûte rien (retard vrai numérique) : aucune proximité n'est recherchée.

**7. Marge : réemploi** (`core/margin.py`, 5.2 P4, règle de l'utilisateur du 26 octobre 2026). La RR demandée d'un volume est un **minimum**. Si les ressources ont de la marge et que `realloc_mode` = `PRIO_SEARCH` (5.2 P7), le radar raccourcit les revisites jusqu'à ce que la ressource dimensionnante atteigne **X = `load_ceiling`** (une valeur par mission, 0,80 par défaut) :
- un **même facteur g** pour tous les volumes (P1 compris), monté ensemble ; les volumes d'une face qui atteint X s'arrêtent, les autres continuent ; un volume à cheval sur plusieurs faces est borné par la plus chargée ; un volume s'arrête aussi à sa **RR minimale** `rr_min_s`, par défaut **1 s** (`tool.global_opt.rr_min_search_s`, plancher ajouté par l'utilisateur le 26 octobre 2026 en 5.2 P7 ; sans plancher, le réemploi descendait à 0,09 s) ;
- le plan est gardé (formes d'onde, grille, impulsions) : seule la revisite s'accélère (RR réelle = RR / g) ; les fausses alarmes restent au même nombre par seconde (la Pfa par case baisse comme 1 / g), donc la charge de confirmation ne bouge pas ; le gain est publié en **portée de détection cumulée à la RR réelle** ;
- la part de veille ajoutée forme le niveau **P5**, le plus bas : c'est la première ressource rendue sur une surcharge ;
- le verdict reste porté sur la RR demandée.

Règle étendue le 26 octobre 2026 (point 10, codée en 5.2 P7, V6.24.0) : `auto_alloc_max` est remplacé par deux réglages, `realloc_mode` (qui reçoit la marge et qui cède en saturation) et `load_ceiling` (plafond toujours actif) ; une mission qui contient encore `auto_alloc_max` est refusée avec un message. Une ressource très peu linéaire en g (les faisceaux simultanés, comptés en entiers) met en échec la correction par la sécante : les facteurs sont alors ramenés ensemble par **bissection** (`reuse_bisections` = 8 assemblages au plus) pour que le plafond tienne.

**8. Surcharge : ce qui cède, et dans quel ordre.**
- Plan de veille seul : une face sursouscrite en temps étire la RR de ses volumes en proportion (`rr_est_mission_s`, alerte).
- Énergie : la répartition (point 4) sert les niveaux dans l'ordre et donne la revisite ou le nombre de pistes tenable.
- Temps (ordonnanceur) : **délestage** de la veille P4 de la sous-priorité la plus basse vers la plus haute (la revisite s'étire), puis des mises à jour P3 (la période s'étire, alerte) ; **P0, P1 et P2 ne sont jamais délestés** : sinon la mission est déclarée non faisable.
- Grâce à l'ordre des pointages (point 6), une veille étirée garde une couverture uniforme.
- Faisceaux : une forme d'onde qui en demande plus que le formateur est exclue du plan.

**9. En vol** (ce qui existe et ce qui reste à faire).
- **Re-pointage** (4.5) : chaque pointage est recalculé à son émission avec la dernière navigation ; quand un volume non lié au vaisseau traverse une frontière de face (≈ 0,3 à 1,3 s à 30 °/s), la face voisine doit porter sa charge (charge de la face + charge du secteur ≤ 1, sinon alerte).
- **Resélection de forme d'onde** (4.4) : dans la **table pour le vol** de chaque secteur (5.2, P5) quand le spectre mesuré change : jusqu'à 8 plans de veille précalculés, un par fréquence centrale, répartis sur la bande, les plans dégradés (couverture ou portée moindre) gardés et signalés ; le calculateur prend la meilleure entrée dont la bande est libre et dont la charge tient sur la face, sinon alerte puis délestage.
- **Phase 6, règles déjà posées par l'utilisateur** : le pilotage devient **dynamique** : le radar alloue et reprend les ressources aux fonctions selon la charge réelle et les besoins (le réemploi de la marge vaut en vol, P5 rendu en premier) ; sur une surcharge, **avant tout délestage**, identifier la ressource qui bloque sur la face en limite et la **réallouer** : énergie depuis les faces qui ont de la marge (jusqu'aux plafonds), temps d'émission par resélection de formes d'onde (énergie contre temps) ou par la face voisine dans les **zones de recouvrement**, chaleur, calcul, faisceaux et mémoire par le recouvrement ; **le délestage par priorités n'intervient qu'en dernier recours**.

**10. Réallocation entre veille et poursuite, états de saturation** (règle de l'utilisateur du 26 octobre 2026 ; calcul et sorties en **5.2 P7**, **codé le 26 octobre 2026, V6.24.0** (`core/margin.py`, clé `saturation`) ; état de saturation dans le verdict en **5.3**, règle complète en vol en **phase 6**). La spécification de l'opérateur, n pistes à la revisite x par tranche de poursuite et un volume v à la revisite y, est le **point de dimensionnement** : en deçà, l'optimiseur sert tout à la spécification ; au-delà, veille et poursuite sont tenues dans les limites ci-dessous.

- **Réglages de mission** : `realloc_mode` = `NONE` (défaut), `PRIO_SEARCH` ou `PRIO_TRACK` (qui reçoit la marge, qui garde sa spécification en saturation) ; `load_ceiling` (défaut 0,80, statut D, à recaler en 5.4 avec l'entrelacement imparfait), utilisation maximale de la ressource qui limite, **toujours active** ; la part au-delà est une **réserve de sécurité jamais consommée** ; remplace `auto_alloc_max`.
- **États** (revisite plus courte que la spécification : gain ; plus longue : dégradation) :

| État | `NONE` | `PRIO_SEARCH` | `PRIO_TRACK` |
|---|---|---|---|
| Nominal (tout tient) | n pistes à x, v à y, marge inutilisée | n à x, v à z < y (point 7 : un seul g, jusqu'à `load_ceiling` ou `rr_min_s`, 1 s par défaut) | n à b < x (cadence de poursuite seule accélérée, un seul facteur, jusqu'à `load_ceiling` ou `rr_min_s` de la tranche, **0,1 s par défaut**), volumes à leur RR spécifiée |
| Saturation gérable (pistes au-delà de n, sans dégradation) | n+a à x, v à y | idem | idem |
| Saturation non gérable | comme `PRIO_SEARCH` (rien n'est pris à la veille) | veille tenue à y ; pistes allongées dans l'ordre strict des classes | pistes tenues à x ; veille dégradée dans l'ordre P4_n ; une fois la veille épuisée, pistes allongées par classe |
| Saturation critique (tous les leviers épuisés) | comme `PRIO_SEARCH` | abandon : pistes, puis volumes P4_n, puis P2 | abandon : volumes P4_n, puis pistes, puis P2 |
| P2 inatteignable | **alerte majeure**, le radar continue en P0 et P1 seulement, plus aucune création de piste | idem | idem |

- **Saturation gérable** : a = ressource disponible / coût d'une piste en plus, le plus petit sur toutes les ressources ; disponible = place libre jusqu'à `load_ceiling` plus la marge réemployée (P5), rendue en premier ; au pire, la veille revient exactement à sa spécification. Les pistes en plus prennent les paramètres de leur tranche (classe, revisite, portée, mode de confirmation) ; la marge est reprise avec **un seul g commun** à tous les volumes ; ordre de service KKV > SHIP > DECOY > DEBRIS > autres, puis la plus proche. Permise aussi en `NONE` (on n'utilise que de la ressource libre, ce n'est pas une réallocation).
- **Saturation non gérable**, formules validées : `PRIO_SEARCH`, revisite équivalente des pistes t = x · coût(n+a pistes à x) / ressource restante une fois v assuré à y ; `PRIO_TRACK`, revisite de la veille c = y · coût(v à y) / (ressource de la face − coût(n+a pistes à x) − P0, P1, P2). Les pistes sont allongées dans l'**ordre strict des classes** (DEBRIS jusqu'à son plancher, puis DECOY, SHIP, KKV en dernier) ; la veille est dégradée dans l'**ordre P4_n** (sous-priorité la plus basse d'abord).
- **Intouchables** : P0 et P1. Un volume que l'opérateur ne déclare pas en P1 accepte d'être dégradé.
- **Temps Ku limitant** (ressource propre à une bande ; la veille est en S seulement), leviers dans l'ordre : 1. retirer la percée P3′ ; 2. basculer des confirmations Ku vers le mode S2 (le temps S est pris à la veille dans l'ordre P4_n en `PRIO_TRACK`) ; 3. allonger les pistes par classe. Au-delà : saturation critique. Les modes « hors spécification » par bande viendront plus tard.
- **Planchers** : t_max d'une piste, par classe = revisite au-delà de laquelle la probabilité de perdre la piste par mise à jour dépasse **p_loss_max = 1 %** (D) (chaîne de perte du modèle de piste, `n_miss_lost`), bornée par `latency_max_s` du secteur si elle est donnée ; revisite maximale d'un volume = clé facultative `rr_max_s`, sinon dérivée (revisite pour laquelle la portée de détection cumulée tombe à la portée minimale d'intérêt du volume).
- **Saturation critique** : on abandonne des tâches entières, volume P4_n entier (sous-priorité la plus basse d'abord) ou pistes par classe (la plus lointaine d'abord dans une classe), dans l'ordre du mode, puis P2.
- **Sortie en Pmiss** (le nombre de pistes y est une entrée) : par tranche de poursuite, le nombre de pistes en plus en saturation gérable (tranche seule, puis cumul quand toutes les tranches saturent, dans l'ordre de priorité), le nombre de pistes à partir duquel on entre en saturation critique, et ce qui est abandonné en premier.
- **Mise en œuvre (V6.24.0)** :
  - **saturation gérable, tranche seule** : une sonde (un assemblage) avec max(1, `sat_probe_frac` 10 % de n) pistes de plus dans chaque tranche ; la hausse de chaque ressource est répartie entre les tranches selon leur propre hausse (énergie pour l'énergie et la chaleur, temps Ku pour le temps Ku, cadence des mises à jour sinon) : coût d'une piste, **linéaire** ; a = place jusqu'au plafond (mesurée sur le plan à la spécification, sans réemploi) / coût, le plus petit sur toutes les ressources. C'est une **extrapolation linéaire**, signalée (Q17 c). Les faisceaux simultanés, comptés en entiers, ne bougent pas pour une piste de plus : leur coût est pris sur leur valeur continue (fenêtres ouvertes moyennes × faisceaux de la plus grosse visée), leur place sur l'entier ;
  - **cumul** (Q18 a) : même fraction p de débordement pour toutes les tranches, le plus grand p qui tient, arrondis départagés dans l'ordre de priorité ; **vérifié par un assemblage**, ramené par bissection si une ressource dépasse le plafond ;
  - **t_max** : chaque tranche aux périodes rr_s × `sat_stretch_track` [1, 2, 4, 8, 16] dans le budget de poursuite, en deux temps (jusqu'à × `sat_stretch_stage` = 4, puis × 8 et × 16 seulement pour les tranches encore sous p_loss_max : une période intenable coûte 115 à 163 s de calcul), interpolation log-log de la perte par mise à jour ;
  - **revisite maximale dérivée** (`PRIO_TRACK`) : `ranges_at_rr` à y × `sat_stretch_search` [2, 4, 8, 16], interpolation de 1/portée ; au-delà de la sonde, la valeur × 16 est prise et signalée ;
  - **saturation critique** : coût d'une piste allongée = coût à x × (x / t_max) (**approximation en 1/T, optimiste** : la prédiction plus large d'une piste plus lente n'est pas recalculée) ; en `PRIO_TRACK`, la place rendue par la veille P4_n à sa revisite maximale est mesurée par un assemblage ; leviers Ku quand un temps Ku limite (le budget Ku publie la charge par face de chaque mode de confirmation, `alternatives[mode].faces`) ; cumul vérifié par un assemblage (le temps Ku n'y est pas contrôlé quand les leviers sont comptés : l'assemblage ne les applique pas) ;
  - **sorties** : `saturation.brackets` (pistes en plus gérables, ressource limitante, coût par piste), `saturation.cumulative`, `saturation.critique` (planchers, entrée par tranche et en cumul, leviers, premier abandon) ; section « TRACK SATURATION » du rapport console ;
  - **temps de calcul** : 3 assemblages et 2 budgets de poursuite de plus sur une mission avec poursuite (`mission_synthetic_mix` : 80 → 192 s en `NONE`, 264 s en `PRIO_TRACK`), rien sur une mission sans poursuite ; **dépassement du budget de +30 % accepté par l'utilisateur** (26 octobre 2026).
- **Défaut écarté** : une première version plaçait les pistes au-delà de n sous la veille à sa spécification ; elle cassait la règle KKV > veille (une KKV en plus n'était pas suivie alors que la veille gardait sa RR, une DEBRIS comptée dans la spécification passait devant une KKV en plus). Le choix du mode par l'opérateur la remplace : `PRIO_TRACK` tient KKV > veille, `PRIO_SEARCH` l'abandonne en connaissance de cause.

---

## 8. Paramètres de l'outil (`[tool.*]`)

Ces réglages ne décrivent pas le radar. Ils ont été validés tels quels lors de la revue 0.3, avec des renvois vers les étapes à venir.

| Section | Variable | Valeur | Rôle et renvoi |
|---|---|---|---|
| `tool.cli` | `refresh_s`, `frequency_ghz`, `scan_angle_deg`, `target` | 120 s, 2,8 GHz, 0°, "ST" | Valeurs par défaut de `python core/radar_model.py`. Le rapport donne aussi, pour chaque classe, la SER frontale, la fluctuation, T₀, la dépolarisation et l'accélération (étape 2.4). |
| `tool.optimizer` | `freq_step_mhz`, `freq_coarse_mhz`, `freq_fine_span_mhz`, `excluded_flag_frac`, `tie_rel_tol` | 5 MHz, 20 MHz, 20 MHz, 0,5, 1e-9 | **Étape 3.1, passe P1 (A1, A5, A10, E6)** : la grille des centres S est **dérivée de la bande** (`usable_lo` + k × 5 MHz, signal entier dans la bande et hors des exclusions) ; recherche en deux passes (20 MHz sur les centres valides, plus le centre le plus central de chaque morceau libre, puis 5 MHz sur ±20 MHz autour du meilleur) ; signalement quand les exclusions retirent plus de la moitié des centres valides ; coûts égaux à 1e-9 près départagés par le critère secondaire. `freq_candidates_ghz` est supprimée. |
| | (supprimée) `freq_candidates_ghz` | [2,7, 2,8, 2,9] | À remplacer à l'étape 3.1 par une grille fine au pas de **5 ou 10 MHz**, restreinte au spectre utilisable. Le but est d'éviter les brouilleurs étroits de forte puissance tout en gardant plus de cases fréquence qu'un masquage par multiples de 25 MHz. |
| | `tau_s` | 20 à 800 µs, plus la limite de la réserve (2,0 ms depuis l'étape 1.3) | La plus longue impulsion est **calculée** à partir de la réserve d'énergie. |
| | `n_pulses`, `tx_modes`, `sig_bw_mhz` | {1 … 32}, {single, superposition, partition} (faisceaux de `[multibeam]`, §5.8), {10 … 200} MHz | `b_tx` remplacé par les modes d'émission à l'étape 3.1 (P3) ; bandes de 100 à 200 MHz ajoutées en P2. |
| | `pri_min_s` | 1 ms | PRI des candidats mono-impulsion et ambigus. |
| | `theta_bin_deg`, `bin_deg`, `max_wf_per_sector`, `short_tol`, `n_alphas`, `short_penalty` | 1°, 5, 4, 0,01, 21, 1e6 | Règles de l'optimiseur, revues à l'étape 3.1. |
| | ~~`track_margin_warn`~~ | — | **Retiré à l'étape 5.3 (V6.25.0)** : l'alerte de marge pour la poursuite est le plafond de charge `load_ceiling` du verdict (`core/validation.py`). |
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
| `tool.sky` (étape 3.1, P5) | `refine_rise_db`, `refine_radius_hpbw`, `refine_subsamples` | 0,5 dB, 3,0, 8 | **Étape 3.1, P5** (D5, D6, option 3 de l'utilisateur) : cellules évaluées une à une près des sources chaudes, puis découpées en 8 × 8 points de même poids, chacun aveuglé ou non et compté à part dans le quantile du budget ; la cellule n'est balayée que pour la part restante (les positions la comptent au prorata). Une moyenne du bruit sur 4 × 4 points (une seule valeur par cellule) avait été essayée sans gain. |
| `tool.sky` | `eval_step_hpbw`, `near_r_step_lam_d`, `near_phi_step_deg`, `body_ring_lam_d`, `small_body_lam_d`, `body_rings_max`, `chunk` | 0,5, 0,05, 10°, 2,0, 2,0, 16, 2048 | **Étape 1.5** : pas d'évaluation du T_sys sur un secteur (A5) et précision de l'intégrale du bruit. |
| `tool.display` | `grid_res_deg`, `viz_freq_ghz` | 0,5°, 2,8 GHz | Trame 3D et estimation rapide. |
| `tool.global_opt` | `max_iterations`, `alternatives`, `reuse_corrections` ; `realloc_mode`, `load_ceiling` ; `rr_min_search_s`, `rr_min_track_s` ; `reuse_bisections` ; `sat_probe_frac`, `sat_stretch_track`, `sat_stretch_stage`, `sat_stretch_search` | 3, 6, 2 ; "NONE", 0,80 (D) ; 1 s, 0,1 s ; 8 ; 0,1, [1, 2, 4, 8, 16], 4, [2, 4, 8, 16] | **Étape 5.2** : boucle globale (P3), réemploi de la marge (P4) ; **P7** : valeurs par défaut des réglages de mission de réallocation, planchers par défaut du réemploi (utilisateur), bissection quand une ressource n'est pas linéaire, sondes de la table de saturation (§7.15 point 10). |
| `tool.validation` | `verify_bisections` | 6 (D) | **Étape 5.3, P2** : recalculs de la mission par bissection pour le chiffre vérifié d'un conseil (validé par l'utilisateur le 27 octobre 2026). |
| `tool.mission_defaults` | voir §9 | | Valeurs d'une nouvelle mission. Depuis l'étape 2.1 : `res_max_m` est une résolution **physique**, et `range_min_frac` = 0,1 donne la portée minimale d'intérêt d'un secteur qui n'en fixe pas. |

---

## 9. Modèle de mission (entrées opérateur)

### 9.1 Paramètres globaux

| Clé | Défaut | Contrôle |
|---|---|---|
| `eps_min_deg` / `eps_max_deg` | −90° / +90° (pas de limite) | Fenêtre d'élévation de la mission, envoyée à l'ordonnanceur. Le plancher physique est d'environ −55°. |
| `face_elec_mw` | 2,0 MW | Doit être strictement positive et ne pas dépasser 8 MW (validation provisoire). |
| **`realloc_mode`** (5.2 P7) | `NONE` | `NONE`, `PRIO_SEARCH` ou `PRIO_TRACK` : qui reçoit la marge et qui garde sa spécification en saturation (§7.15 point 10). |
| **`load_ceiling`** (5.2 P7) | 0,80 (D) | Dans ]0, 1] ; utilisation maximale de la ressource qui limite, au-delà réserve de sécurité ; remplace `auto_alloc_max` (§7.15 point 10). |
| **`spectrum_exclusions_ghz`** | absent | **Étape 3.1 (A3, A4).** Spectre interdit : {"S": [[lo, hi], ...], "Ku": [...]}, en GHz ; un signal ne doit recouvrir aucun intervalle (marge de garde 0 : l'opérateur la donne dans l'intervalle). La partie Ku servira à l'étape 3.2. En attendant la mesure en temps réel du spectre (3.2). |
| **`nav`** | absent (attitude identité, sans orbite) | **Étape 3.0.** Liste de messages de navigation, **un seul** en Pmiss (décision de l'utilisateur du 27 octobre 2026 : pas d'enveloppe d'attitudes en Pmiss, elle passe en phase 6) : `label`, `epoch` (ISO 8601, UTC, facultative), `q_ship_to_icrf` [w, x, y, z], orbite facultative (`central_body` de `[sky.surface]`, `r_km` et `v_kms` depuis le centre du corps, axes ICRF), `attitude_ref` (`icrf` ou `rtn`, affichage des angles dans l'interface seulement). §7.14. |
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
| `tracking` | [] | Tranches de poursuite active : r_min et r_max, nombre de cibles (5), période (2 s), zone angulaire (0,2°), σ_R (10 m), σ_V (5 m/s). **5.2 P7** : `rr_min_s` facultatif par tranche (période la plus courte du réemploi en `PRIO_TRACK`, 0,1 s par défaut, ≤ `rr_s`). |
| **`rr_min_s`** | 1 s (`rr_min_search_s`) | **Étape 5.2 (P4, P7).** Revisite la plus courte du réemploi de la marge en `PRIO_SEARCH`, ≤ `rr_target_s`. |
| **`rr_max_s`** | dérivée | **Étape 5.2 (P7).** Revisite la plus longue avant la saturation critique (plancher de la veille en `PRIO_TRACK`), ≥ `rr_target_s` ; absente : la revisite où la pire portée cumulée tombe à `range_min_km`. |
| `max_wf_per_sector`, `short_tol`, `freq_ghz_candidates` | réglages de l'optimiseur | Surcharges possibles par secteur. Depuis l'étape 3.1 (A9), `freq_ghz_candidates` est un **forçage** contrôlé au chargement : fréquences dans la bande S et au moins une où un signal de l'espace de recherche tient hors des exclusions. |

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
| Multiplexage en temps ou en fréquence du multifaisceau | Étape 3.1 (P3) : multiplexage en temps modélisé, hors de l'espace de l'optimiseur (jamais meilleur qu'un faisceau dans le décompte du temps d'émission) ; multiplexage en fréquence écarté. Intérêt réel en chronologie : réévaluer son entrée dans `tx_modes` avec le coût de chronologie (renvoi écrit à l'étape 4.3 de `Feuille_de_route.md`). | 4.3 |
| Résidu du couplage distance-Doppler | Biais corrigé par la mesure du Doppler ; résidu non compté. | 3.3 |
| Affichage du volume utile, édition de `range_min_km` | À terme, les vues ne montrent que le volume utile (sans la zone aveugle) ; l'interface n'édite pas `range_min_km`. | 5.5 |
| PAE en fonction du recul du PA | Plus nécessaire en 3.1 : la superposition retenue est en phase seule (enveloppe constante, PA saturés) et la réduction « pas d'énergie en trop » raccourcit l'impulsion. Loi à établir si un mode à recul (superposition linéaire, réduction de puissance) est rouvert. | — |
| Espaces de recherche | Rien entre 0,8 et 2 ms en S, rien au-delà de 400 µs en Ku. | 3.1, 3.2 |
| Alimentation des TRM | Recharge de la réserve pendant un train, capacité d'alimentation d'un TRM, et **puissance instantanée de la face** pendant une observation (4 900 TRM S en même temps : environ 8,2 MW à d = 0,65, contre 2 MW alloués) ; seule la moyenne sur la période de revisite est vérifiée. | 4.1 |
| Thermique | Chaleur calculée (≈ 370 W par TRM S en pointe) ; capacité des microcanaux et constantes de temps non modélisées. Densité de puissance d'environ 104 kW/m² de RF moyenne. | 4.2 |
| Limite de 70° au-dessus de la fréquence de rupture | Gardée à l'étape 1.4 tant que le diagramme de l'élément n'est pas modélisé (flanc du lobe de réseau à l'horizon). | avec le diagramme de l'élément (à placer) |
| Fréquences candidates de la veille S | 2,7 / 2,8 / 2,9 GHz : depuis l'étape 1.4, aucun signal ne tient aux bords de bande (2,7 et 2,9 GHz), seule 2,8 GHz est jouable. | 3.1 (grille fine dans la bande) |
| Temps de cohérence de la cible | **Par classe et par bande depuis l'étape 2.4** (T₀ = λ / 2Lω). | — |
| Masquage par une cible forte (lobes Doppler et en distance, bruit de phase) | Niveaux en sortie depuis l'étape 2.4 ; OS-CFAR en option. Effet sur une scène dense non compté. | phase 6 (était 5.4, 27 octobre 2026) |
| Table d'aspect de la SER | Clé réservée (SER frontale seule dans le bilan). | 3.3 |
| Tables de confirmation | Cas de Swerling le plus proche (famille ou cas déclaré) : sans décorrélation partielle ni diversité de polarisation, que `solve()` compte pour les observations Ku. | 3.2 |
| Choix de la classe dans l'interface | L'interface conserve `target_type`, montre les valeurs de la classe ; une SER ou un Swerling modifié devient une surcharge. | 5.1 |
| Estimation rapide | Table de coût calculée sur GENERIC en Swerling 1 ; un secteur à classe y entre par sa SER frontale en S. | 5.5 |
| Sensibilité du verdict aux valeurs de la menace | Les classes sont des moyennes. **Non retenue en 5.3** (utilisateur, 27 octobre 2026) : le verdict porte sur les valeurs données (secteur, sinon classe) ; l'opérateur édite la classe pour une autre hypothèse. | développement futur potentiel |
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
| Multifaisceau | Étape 3.1 (P3) : superposition en phase seule, partition de l'ouverture et multiplexage en temps modélisés (§5.8) ; les deux premiers cherchés par l'optimiseur. Fouillis et sources vus par le faisceau d'émission élargi d'une partition non modélisés (diagramme de la face complète), à reprendre avec l'environnement (P4) ; faisceaux de réception par position ×1,85 / ×3,43 pour une partition (charge de calcul, 4.4). | 3.1 (P4), 4.3, 4.4, 5.2 |
| Fréquence Ku | Fixée à 14,0 GHz, provisoire. | 3.2 |
| Masquage par le vaisseau | Modélisé à l'étape 2.5 (tube d'ouverture ; modèle 3D `LP_NISO.obj` depuis le 22 octobre 2026) : aucun pointage masqué avec la géométrie actuelle. Diffraction en champ proche et couplage avec la structure non modélisés. | — |
| Réflexions sur la structure | Soleil, Terre et autres faces réfléchis par la coque ou les radiateurs : non comptés. | hors modèle : développement futur potentiel (27 octobre 2026) |
| Fouillis d'une forme d'onde | Étape 3.1 (P4) : par candidate et par tranche, rejet Doppler progressif, faisceau de partition, Pd moyenné sur les vitesses (§7.13, « Environnement par forme d'onde »). La Ku garde le modèle de l'étape 2.5. | 3.2 (Ku) |
| Portée exigée derrière une surface | Signalée, mais l'exigence n'est pas réduite à la distance de la surface sur ces pointages. | 3.1 |
| Brouilleurs et confirmation Ku | La portée de percée n'est calculée que pour la veille S ; les brouilleurs à distance entrent dans le T_sys Ku. | 3.2 |
| Brouillage trompeur (répéteurs, faux échos) | Non modélisé (décision Q4). | 3.3 |
| Choix du CFAR par pointage | GO conseillé près d'un limbe ; CA partout. | 4.3, 5.2 |
| Coût de l'annulation adaptative | 8 brouilleurs par face au plus. | 4.4 |
| Latence | Règle supprimée, à reconstruire. | 4.3 |
| Répartition de la puissance entre faces | Une seule valeur par mission. | 4.1, 5.2 |
| Consommation hors émission | `overhead_frac` à 3 %. | 4.1, 4.4 |
| Faisceaux de réception par pointage | Calculés (environ 6,1 à −25 dB au pas de 1,4 ; ×1,85 / ×3,43 en partition) mais pas encore comptés dans le bilan de calcul. | 4.4 |
| p_min par volume | Option à creuser. | 5.1 |

### 10.3 Points ouverts

- **(a)** L'optimiseur départage des égalités exactes par le bruit d'arrondi : il faut un critère secondaire (étapes 3.1 et 5.2). La cause principale, le plancher de cohérence, a disparu à l'étape 2.2 ; il reste des égalités entre une impulsion seule étiquetée COHERENT ou NONCOHERENT (identiques).
- **(b)** Le nombre de faces, 4, est écrit en dur dans le code.
- **(c)** Le toucher S2 sort de la règle des rôles S ; les rôles TWS et PROMOTE ne sont pas utilisés (étape 3.2).
- **(d)** `pulse_scale` (était `power_scale`) est arrondi à 3 décimales dans la sortie de l'optimiseur (étape 5.5).
- **(e)** La table de l'estimation rapide n'est calculée qu'à 2,8 GHz (étape 5.5).
- **(f)** **Lobes de jeu des TRSA (étape 1.2), traité en grande partie.** Le jeu est passé de 2 mm à 0,5 mm. Restent en Ku, au haut de la bande : −30 dB tenu jusqu'à 68,5° (14,25 GHz) et 64° (14,5 GHz), −35 dB jusqu'à 62° (14 GHz), 58,5° et 55° ; signalé par secteur. Si la mécanique impose un jeu plus grand, la variante est une maille d'éléments uniforme à travers les jeux (TRM Ku de 10,32 mm ou éléments décalés), qui demande de séparer le pas des éléments de la taille des TRM dans `[aperture]`. Les effets de bord de tuile (couplage mutuel au bord d'un jeu) ne sont pas modélisés.
- **(g)** `rx_taper_edge` = "inscribed" (statut D) à confirmer.
- **(h)** **Masse des réserves d'énergie** (étape 1.3) : 122 kJ par face en S et 147 kJ en Ku, soit environ 270 kg par face à 1 J/g (hypothèse 2050), environ 1,1 t pour les 4 faces ; aucun bilan de masse n'est prévu dans la feuille de route.
- **(i)** Le jeu de 0,5 mm suppose des matériaux appariés en dilatation (étape 4.2).
- **(j)** `mission_detection_envelope.csv` (racine de `V1`), artefact périmé d'avant la feuille de route (modèle V6.0.12), lu par aucun code et sans outil de génération : **supprimé** sur décision de l'utilisateur (20 octobre 2026). Il reste dans l'historique git.
- **(k)** Le couplage mutuel au bord des tuiles et le diagramme réel de l'élément rayonnant ne sont pas modélisés (éléments isotropes dans le facteur de réseau ; l'intégrale du bruit du ciel prend un élément en cos θ, rien vers l'arrière, depuis l'étape 1.5).
- **(l)** Les couplages de l'isolation (étape 1.5 : 90 et 120 dB entre faces, −30 dB dans une TRSA) sont des ordres de grandeur pris au bout optimiste ; les faces sont dans le champ proche l'une de l'autre (limite de Fraunhofer d'environ 325 m en S) : une simulation électromagnétique serait nécessaire.
- **(m)** Les erreurs aléatoires d'amplitude et de phase des TRM ne sont pas modélisées (environ 0,2 % de la puissance hors du lobe pour 0,2 dB et 2° rms : négligeable devant les 2 à 5 % de la pondération pour le bruit du ciel).
- **(o)** **Dérive du test de rotation du Soleil aux grands angles** (étape 3.1, P5) : le budget T_sys varie de 0,8 % à 1,13°, 1,3 % à 1,57° et 1,4 % à 2,9° (part aveuglée sous 0,5 %). Elle croît régulièrement avec l'angle, ce qui fait penser à un effet physique (les faces restent fixes : tourner le ciel et le secteur change l'angle de balayage et le diagramme), pas à la trame. Non vérifié : à confirmer à l'étape 5.6 (`test_sky_rotation_drift_is_measured` l'affiche sans critère).
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

Questions de cadrage (validées) : Q1 clé de secteur `target_type`, valeurs explicites en surcharge ; Q2 SER frontale par bande (table d'aspect réservée à la poursuite) ; Q3 cas de Swerling calculé à partir de la famille et de T₀ ; Q4 T₀ = λ / (2 L ω) ; Q5 perte d'accélération sous une cellule, banc au-delà ; Q6 niveaux de masquage en sortie et OS-CFAR en option, effet sur une scène dense à l'étape 5.4 (passé en phase 6 le 27 octobre 2026) ; Q7 dépolarisation et diversité ; Q8 classes ST, KKV, SHIP, plus débris et leurres (utilisateur), et le code doit accepter de nouveaux types et des changements de valeurs dans le fichier seul.

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
| Q2 attitude | Une seule par mission en Pmiss ; l'enveloppe et le profil dans le temps au simulateur global (phase 6 ; l'enveloppe était prévue en 5.3, retirée le 27 octobre 2026) : l'architecture le prévoit dès 3.0 (liste, une seule fonction de rotation, calcul en aval dans le repère du vaisseau). |
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

### Étape 3.1 : veille S reconstruite (en cours, 22 octobre 2026)

Cadrage validé : cinq blocs, dans l'ordre A grille de fréquences, B espace des formes d'onde, C multifaisceau d'émission, D environnement par forme d'onde (avec le raffinement du bruit du ciel venu de l'étape 3.0), E règles de l'optimiseur. Codage en six passes avec une pause après chacune (demande de l'utilisateur).

| Bloc | Décisions (toutes validées) |
|---|---|
| A | Pas de 5 MHz ; grille **dérivée de la bande** (signal entier dans la bande et hors des exclusions), `freq_candidates_ghz` supprimée ; clé de mission `spectrum_exclusions_ghz` (S, et Ku pour 3.2), marge de garde 0 ; recherche en deux passes (20 MHz puis 5 MHz sur ±20 MHz), chaque morceau libre gardant un centre ; une fréquence par secteur ; agilité renvoyée au bloc B ; forçage par secteur contrôlé ; sorties (fréquence, bande occupée, centres écartés) et signalement au-delà de la moitié des centres écartés. Constat : l'optimum est plat en fréquence ; la grille sert d'abord à éviter le spectre brouillé ou interdit. |
| B | Impulsions ajoutées 5 et 10 µs (autodéfense : portée minimale d'intérêt la plus courte **2 km**, objets de 0,01 à 0,1 m² ; τ ≤ 13,3 µs ; saturation des CAN acceptable, portée non saturée 0,5 à 1,3 km sur cette gamme) et 1,0, 1,2, 1,5 ms ; bandes 100, 150 et 200 MHz, rien sous 10 MHz ; nombres d'impulsions inchangés ; règle de modulation gardée ; **agilité de fréquence** pour les trains non cohérents (tirages indépendants min(n, ⌊Δf / (c / 2L)⌋ + 1)) ; PRI inchangée. Le volume de test B2_AUTOPROT (20 km) n'est pas un besoin réel. |
| C | Émission S seulement (la réception numérique forme déjà ses faisceaux, coût en calcul en 4.4) ; modes proposés : superposition en phase seule (pertes 0,4 / 0,6 / 0,8 dB à 2 / 4 / 8 faisceaux), partition de l'ouverture (2 ou 4), multiplexage en temps (2, 4 ou 8, sans perte) ; superposition linéaire et multiplexage en fréquence écartés ; `l8_db` et `p` supprimés ; réduction « pas d'énergie en trop » par raccourcissement de l'impulsion ; loi de PAE en recul renvoyée. Constat : avec le budget de temps d'émission et l'entrelacement, B faisceaux équivalent à une impulsion B fois plus courte ; l'intérêt réel est en 4.3. Multifaisceau Ku inscrit en 3.2. |
| D | Fouillis par bande et par pointage ; rejet Doppler progressif ; portée ramenée à la distance de la surface ; brouilleur d'autoprotection dans le calcul de la portée (forme exacte) ; raffinement du ciel autour des sources chaudes (hausse > 0,5 dB, brouilleurs à distance ; à moins de 3 largeurs de faisceau) ; test de rotation avec le Soleil à 1 %. |
| E | Règles `bin_deg`, `max_wf`, `short_tol`, `n_alphas`, `short_penalty` gardées ; critère secondaire déterministe ; pas de grille de veille par secteur parmi 1,3 à 1,6 ; pas des faisceaux de réception fixe jusqu'à 4.4 ; élagage exact des candidates dominées ; budget de calcul : `mission_example` ≤ 60 s en séquentiel, suite de tests ≤ 3 min en parallèle. |

Questions de l'utilisateur pendant les blocs : faisceaux de réception simultanés (aucun plafond dans le modèle ; limite physique d'environ 4 900 par polarisation, limite pratique le calcul, étape 4.4) ; comparaison avec les 128 faisceaux d'un radar S actuel (non poursuivie à la demande de l'utilisateur) ; révision de la dalle (supportée par le code dans la famille actuelle ; contrôles à ajouter lors d'une révision réelle, point ouvert) ; élargissement de la bande S (à traiter avant P2).

**Passe P1 (modèle V6.11.1)** : grille dérivée, exclusions, recherche en deux passes, forçage contrôlé, sorties, critère secondaire. Mise au point : le temps de calcul de `mission_example` passait à 91 s avec la grille fine, à cause de la recherche du SNR requis pour la Pd cumulée (25 880 appels, 508 distincts) ; mise en cache exacte : 18 s (16 s avant P1 avec 3 fréquences). Le secteur de test A3_HIBAND forçait 2,9 GHz, où aucun signal ne tient : il force 2,86 GHz (toujours au-dessus de la rupture de 2,85 GHz). Effet sur les missions de test : voir `tests/snapshots/JOURNAL.md`.

**Élargissement de la bande S (modèle V6.11.2, option a de l'utilisateur, W1 à W7 validés)** : bande utile 2,60–2,93 GHz, bande centrale 2,66–2,88 GHz, perte de bord 1,0 dB, limites de balayage, bande instantanée (200 MHz) et `pae_sat` (0,80, plus optimiste) gardées ; valeurs seulement, le code l'absorbe (grille de 67 centres). Gain : agilité jusqu'à 330 MHz moins la bande du signal (KKV et DEBRIS décorrélables), plus de centres libres face aux brouilleurs ; coût : environ 30 à 40 % de calcul en plus pour la Pmiss, perte de bord sur les secteurs qui vont aux bords.

**Passe P2 (modèle V6.11.3)** : impulsions 5 et 10 µs, 1,0, 1,2, 1,5 ms (la liste complète va de 5 µs à 2,0 ms, 13 valeurs) ; bandes 100, 150, 200 MHz ; agilité de fréquence (champ `agility_mhz`, bande occupée = B + excursion, tirages ⌊Δf / (c / 2L)⌋ + 1) ; élagage exact des candidates dominées. 4 368 candidates par secteur. Effet : la plupart des secteurs de test prennent une impulsion de 5 µs à 100 MHz (résolution de 2 m par défaut ; avec la réduction « pas d'énergie en trop », une impulsion courte coûte la même énergie et moins de temps) ; B2_AUTOPROT trouve une forme d'onde ; l'agilité n'est pas choisie sur les missions de test (cibles déclarées SW1, cohérent mono-impulsion meilleur).

Temps de calcul après P2 : `mission_example` 52 s en séquentiel (budget 60 s, après mise en cache exacte de `walk_loss_db` et `oscillator_phase_var`) ; tests de physique 3 min 07 en parallèle (budget de la suite, 3 min, dépassé). P3 multipliera les candidates par environ 2,25. **Décision de l'utilisateur** : appliquer avant P3 les accélérations X1 (retirer avant le calcul les candidates hors bande à la fréquence évaluée, exact), X2 (perte de migration en forme fermée, intégrale exacte), X3 (fréquences d'un secteur évaluées en parallèle, résultats identiques), X5 (allègement de la suite de tests) ; **pas X4** (passe fine limitée aux familles gagnantes : approximation). Le budget de temps est à réexprimer en usage normal de l'outil, parallèle compris.

**Temps de calcul (modèle V6.11.4, décision de l'utilisateur : X1, X2, X3, X5, pas X4)** : la cible de 3 min de la suite de tests est **supprimée** ; on cherche des gains sans valeur cible, et **la précision du modèle ne doit jamais être réduite pour gagner du temps** (règle de l'utilisateur). Mis en œuvre : X1 seules les candidates dont la bande occupée tient à la fréquence évaluée sont calculées ; X2 perte de migration en forme fermée par le sinus intégral (écart avec l'ancienne moyenne sur 401 points < 2·10⁻⁵ dB) ; X3 fréquences d'un secteur évaluées en parallèle depuis le processus principal (résultats identiques ; désactivé dans les processus de secteur, sous pytest-xdist et par `RADAR_FREQ_PARALLEL=0`) ; X5 la fixture `runs` des tests lit les sorties figées au lieu de recalculer les missions (les tests de valeurs figées les recalculent déjà). `mission_example` : 52 → 47 s en séquentiel, 42 s avec X3 ; suite de tests : 7 min 06 → environ 4 min 30.

**Passe P3 (modèle V6.11.5), multifaisceau d'émission S** : questions de mise en œuvre, toutes tranchées par l'utilisateur. Q1 multiplexage en temps modélisé mais retiré de l'espace de l'optimiseur (mêmes énergie et temps par position qu'un faisceau, jamais meilleur : retrait exact) ; hypothèses validées : zone aveugle c·B·τ/2, réserve du TRM, rapport cyclique et PAE sur B·τ. Q2 (b) grille des positions d'une partition sur le faisceau élargi de la sous-ouverture. Q3 les deux coupes des moitiés (horizontale, verticale) évaluées ; sur l'octogone actuel elles sont le même faisceau tourné de 90°, une seule est cherchée (contrôle exact `partition_cuts`). Q4 pertes de l'impulsion entière gardées quand elle est raccourcie (prudent). Q5 (a) plancher de l'impulsion raccourcie τ ≥ 1/B ; PAE de l'impulsion raccourcie calculée exactement. Détail du modèle au §5.8.

Accélérations exactes de P3 (règle : aucune perte de précision) : portée d'une superposition déduite de sa jumelle à un faisceau par l'équation radar ((1 / (B · L))^¼, écart avec `solve()` < 1e-12) ; partie de la loi des pertes de position indépendante du banc de vitesses mise en cache à part (le cache précédent saturait avec les sous-ouvertures) ; Pfa calculée une fois par train d'impulsions ; élagage regroupant les candidates par fonction de coût, le plancher comparé. Temps : voir §1.6.

Effet sur les missions de test (valeurs de test seulement) : temps d'émission S −13 à −99 % par face, énergie 0 à +3,8 % (PAE plus faible de l'impulsion raccourcie) ; la partition est choisie sur les secteurs de test à portée courte (elle répartit le coût fixe d'une impulsion, temps de front et plancher 1/B, sur 3,7 ou 13,7 positions) ; la superposition n'est jamais choisie ; voir `tests/snapshots/JOURNAL.md`. Renvois : fouillis vu par le faisceau élargi d'une partition (P4) ; faisceaux de réception par position ×1,85 / ×3,43 en partition (4.4) ; intérêt du multifaisceau en chronologie (4.3).

**Questions de P4 (23 octobre 2026 ; toutes les recommandations validées, Taylor −30 / −40 dB, garde 10 km)** : Q1 « par bande » = par bande de signal B des candidates S, la Ku en 3.2 (oui) ; Q2 fouillis par candidate et par tranche de 1°, au centile (1 − `short_tol`) des pointages de la tranche, portée × (1 − x)^¼, regroupement exact des candidates (oui) ; Q3 fouillis vu par le diagramme 2D de la sous-ouverture d'une partition, profil lointain à calculer (oui) ; Q4 spectre du fouillis gaussien centré sur `closing_mps`, `closing_spread_mps` = 1 σ (oui ; sinon uniforme en demi-largeur) ; Q5 pondération Doppler de Taylor en option choisie par l'optimiseur là où il y a du fouillis, niveaux −30 / −40 dB à confirmer (oui) ; Q6 plancher du rejet = plancher Doppler de l'oscillateur et mouvement de la plateforme (oui) ; Q7 rejet moyenné sur l'étendue de vitesses dans le Pd moyenné (recommandé) ou au pire point ; Q8 aucun rejet sans intégration cohérente (oui) ; Q9 exigence ramenée à d − garde, garde = max(résolution, `surface_guard_km`), valeur par défaut 10 km à confirmer ; Q10 critère cumulé avec R réduit, sortie des pointages concernés (oui) ; Q11 brouilleur d'autoprotection dans `solve()`, S = SNR_req (N + J), second degré en 1/R² (oui) ; Q12 brouilleur de barrage à densité fixe par MHz (oui ; répondeur et DRFM en 3.3) ; Q13 pas d'annulation dans le lobe principal, le CFAR suit le niveau (oui).

**Passe P4 (modèle V6.11.6), environnement par forme d'onde** : trois choix de mise en œuvre soumis à l'utilisateur et validés : R1 (b1) moyenne exacte du Pd sur les vitesses de la cible (bisection vectorisée, les entrées identiques résolues une fois) ; R2 (c2) classes d'exigence arrondies vers le haut sur 5 % de la portée, critère cumulé recalculé par classe ; R3 aveuglement des seuls pointages qu'aucune candidate ne peut dégager. Modèle au §7.13, « Environnement par forme d'onde ». Constats : sur E_KKV_SPJ (mission de test), aucune candidate n'atteint 500 km avec le brouilleur dans la portée (104 km au mieux) ; la règle du plan prenait alors la moins chère des candidates toutes courtes (environ 1 km). **R4 (b), décision de l'utilisateur** : quand aucune candidate n'atteint la portée sur un segment, la moins de positions courtes, puis la plus grande portée atteinte (pire point du segment rapporté à son exigence), puis le coût ; E_KKV_SPJ passe à 68–79 km (train cohérent de 32 impulsions de 200 µs), toujours signalé ; joué à pleine puissance sur ses 582 positions, il porte la face 2 de 0,5 % à 124 % de son énergie (revisite estimée 20 → 24,9 s) : R4 privilégie la portée même quand le gain coûte cher (point signalé à l'utilisateur). **Concept de l'utilisateur (23 octobre 2026)** : face à un brouilleur d'autoprotection, la veille S n'a pas à percer ; elle identifie l'axe de brouillage et l'ordonnanceur pointe une recherche Ku ciblée de percée dans cet axe (gain Ku environ 25 fois, portée de percée environ 5 fois à énergie égale) ; renvoi écrit à l'étape 4.3 de `Feuille_de_route.md`. **R5 (b), décision de l'utilisateur** : sur un secteur où le brouilleur d'autoprotection met l'exigence hors de portée de la veille (plus de tranches peuplées hors d'atteinte qu'avec le bruit seul), le secteur est planifié contre le bruit seul, au coût normal, et signalé « exigence hors de portée de la veille, axe de brouillage donné à l'ordonnanceur pour une percée Ku » ; R4 (b) reste la règle des autres manques de portée. E_KKV_SPJ revient à 20 / 50 µs (511 à 542 km contre le bruit), face 2 à son coût d'avant P4. Défaut trouvé à la régénération des valeurs figées : dans un processus long, les données de surface S pouvaient être perdues au vidage du cache des résumés du ciel (fouillis alors ignoré pour le secteur) ; corrigé, test de non-régression ajouté. Sur E_EARTH, la grille de 5 % (1 050 km) ramène l'exigence réduite à la portée exigée : D3 n'y change rien. Temps : `mission_environment` 1 min 42 en séquentiel sur le portable après les accélérations exactes (2 min 47 avant).

**Passe P5 (modèle V6.11.7), ciel et pas de grille** : S1 à S4 (cellules évaluées une à une où la hausse du bloc dépasse 0,5 dB et à moins de 3 largeurs de faisceau d'un brouilleur à distance, du centre d'un petit corps ou du limbe d'un grand) ; E7 a (pas de grille de veille par secteur, recherche exacte de 1,3 à 2,0 au pas de 0,1). Test de rotation du Soleil (D7, 1 %) non tenu après S1 à S4 : sur cette machine, 1,05 % et 1,41 % aux angles du test, jusqu'à 3,9 % et 2,7 % sur dix angles de 0,05° à 2,9° (les 2,9 % et 2,0 % notés auparavant venaient d'une autre mesure, non retrouvée). Décision de l'utilisateur (23 octobre 2026) :

| Question | Décision |
|---|---|
| Coût de la précision | Le calcul tourne en préparation de mission (Pmiss) seulement : son coût en mémoire et en temps ne pèse pas sur le calculateur radar ; renvoi aux étapes 4.3, 4.4 et 5.2 pour ce qui serait recalculé en vol. |
| D7 | Option 3 (contre une trame plus fine ou un critère à 3 %) : découpage des cellules en sous-points. |
| Cellules découpées | D'abord les cellules du bord de la région aveuglée ; la mesure a montré que cela règle la part aveuglée (0,40 % au plus) mais pas le budget (3,8 %), dont le quantile tombe dans l'anneau chaud où chaque cellule n'a qu'une valeur ; **toutes les cellules affinées** sont donc découpées (budget 0,15 et 0,81 % aux angles du test). |
| Points par côté | 8 (`tool.sky.refine_subsamples`, sens nouveau : points comptés à part, non plus moyennés). |
| Critère | 1 %, inchangé, tenu ; contributeur principal compté par sous-point aveuglé. |
| Dérive aux grands angles | Test séparé sans critère, point ouvert (o), à l'étape 5.6. |

Coût mesuré : l'évaluation du ciel du secteur de test passe de 0,02 à 0,09 s par fréquence. Le masquage par la structure, l'aveuglement par le fouillis et l'absence de volume devant une surface restent par cellule entière.

### Étape 3.2 : confirmation Ku reconstruite (23 au 25 octobre 2026, modèle V6.13.0)

Cadrage validé : cinq blocs, A fréquence Ku, B espace des observations Ku, C multifaisceau Ku, D passage S → Ku et rôles, E environnement et cible en Ku. Règle de la soirée : le surcoût de calcul de chaque bloc est estimé avant de coder, et aucun test ne tourne sans l'accord de l'utilisateur.

| Bloc A (tout validé) | Décision |
|---|---|
| A1 granularité | une fréquence Ku par secteur |
| A2 grille | dérivée de la bande Ku : 13,5 GHz + k × 10 MHz, signal entier dans la bande et hors des exclusions |
| A3 harmonique 5 des émissions S de la même face | pas une règle de fréquence (elle viderait la bande : 2,8 GHz ± 50 MHz donne 13,75–14,25 GHz) ; recouvrement signalé par secteur avec la part du canal couverte ; la contrainte reste la règle de temps de l'étape 4.3 |
| A4 critère | pour chaque fréquence, le meilleur mode de confirmation et la poursuite ; on garde le couple qui minimise la face la plus chargée (critère actuel des modes) ; à égalité, la fréquence la plus proche du centre de la bande centrale |
| A5 candidates | centres valides tous les 50 MHz, plus le centre le plus central de chaque morceau libre (environ 20), sans passe fine (optimum plat en fréquence) ; surcoût estimé environ +40 % |
| A6 exclusions | `spectrum_exclusions_ghz["Ku"]`, mêmes règles que la S |
| A7 forçage | clé de secteur `ku_freq_ghz_candidates` contrôlée ; `confirmation.ku_freq_ghz` supprimée |
| A8 au-dessus de 14,25 GHz | rien de spécial : la couverture Ku entre déjà dans l'admissibilité des modes |

| Bloc B (tout validé) | Décision |
|---|---|
| B1 impulsions | ajouter 5 µs, 800 µs, 1,5 ms et τ_max (2,08 ms, dérivé de la réserve) à 10–400 µs |
| B2 nombre d'impulsions | inchangé (1 à 64) |
| B3 PRI | ajouter 5 ms (0,5, 1, 2, 5 ms) |
| B4 bandes | 25, 50, 100, 200 MHz |
| B5 observation trop forte | comme la S : impulsion raccourcie à PA saturé (τ' = s τ, plancher τB ≥ 1, PAE exacte), au lieu de la baisse de puissance |
| B6 marge des tables de confirmation | continue, par bissection jusqu'à `p_min` (0,01 dB), au lieu de la grille de 0,5 dB |
| B7 agilité | règle de la S pour les trains non cohérents de n ≥ 2 |
| B8 modulation | règle de la S |
| Coût | élagage exact des candidats dominés (P1) ; pas d'ordre fréquence puis observations (P2, non exact). Surcoût estimé +40 à +60 % par mission. **Précision de l'utilisateur** : le temps de calcul en préparation de mission n'est pas la variable autour de laquelle on développe (une ou deux missions par jour au plus) ; la gêne des tests longs vient de leur fréquence, pas du temps d'une mission. |

| Bloc C (tout validé) | Décision |
|---|---|
| C1 modes de la grappe Ku | superposition à phase seule et partition ; multiplexage en temps modélisé, hors du choix jusqu'à 4.3 |
| C2 grappe de 7 en superposition | entrée de 7 faisceaux ajoutée à `[multibeam]`, interpolée en log₂ entre 4 et 8 : 0,76 dB, statut D |
| C3 partition en Ku | sous-ouvertures Ku calculées comme en S depuis la composition de la face Ku (à vérifier ou à ajouter dans le code) |
| C4 table de pertes | une seule table pour S et Ku |
| C5 latence | sortie seulement : allers-retours de chaque mode publiés par secteur pour 4.3 |
| C6 pistes proches en angle | renvoyé à 4.3 |
| C7 premier temps de KU1+7 | inchangé, un faisceau |

| Bloc D (tout validé) | Décision |
|---|---|
| D1 toucher S2 | vraie forme d'onde S de rôle `CONFIRM` (`CONFIRM` ajouté aux rôles permis en S, statut D) : la forme d'onde de veille du secteur, pointée, avec la marge de la table |
| D2 rôles | `PROMOTE` supprimé ; `TWS` gardé, réservé à la poursuite en veille (3.3) |
| D3 biais S/Ku | inchangé, 1,1 mrad par axe (V) |
| D4 étalonnage | renvoyé à 3.4 |
| D5 banc de vitesses de la confirmation Ku | inchangé (toute la plage) ; resserrement sur la vitesse mesurée en veille renvoyé à 3.3 |
| D6 tables de confirmation | à la fréquence Ku du secteur |
| D7 fausses alarmes | inchangé, (R/R_dét)⁴ moyen 0,2 (V) |

| Bloc E (tout validé) | Décision |
|---|---|
| E1 brouilleur d'autoprotection | compté dans chaque observation Ku (forme exacte de la S, densité Ku de la classe) ; alerte si aucune ne tient |
| E2 recherche Ku de percée | calculée ici (meilleure observation Ku pointée dans l'axe du brouillage, portée de percée, coût) ; déclenchement et chronologie à 4.3 |
| E3 fouillis Ku | modèle de P4 par observation (code généralisé hors des colonnes de veille S) |
| E4 tables de confirmation | vraie loi de fluctuation de la cible (forme Gamma, diversité) |
| E5 intégration Ku | cohérente et non cohérente offertes, l'optimiseur choisit |
| E6 ciel Ku | à la fréquence Ku du secteur, avec le raffinement de P5 |

Coût estimé (information) : mission complète environ 1,5 à 2,5 fois plus longue qu'à la fin de la 3.1.

Codage proposé en passes : Q1 fréquence Ku (A, D6) ; Q2 espace des observations (B) ; Q3 multifaisceau Ku (C) ; Q4 toucher S2 et rôles, loi de fluctuation, intégration (D1, D2, E4, E5) ; Q5 brouilleur, percée et fouillis Ku (E1 à E3, E6) ; Q6 rapport d'impact, valeurs figées et documentation, au N3 choisi par l'utilisateur. Entre les passes, seuls les tests nouveaux ou touchés, sur accord.

**Passes Q1 à Q5 codées (modèle V6.12.0, 23 octobre 2026, non testées à l'écriture)** : `core/tracking.py` reconstruit. Fréquence Ku par secteur (grille de 10 MHz dérivée de la bande, centres tous les 50 MHz et centre de chaque morceau libre, couple (fréquence, mode) choisi sur la face la plus chargée de la mission, à égalité le plus proche du centre de la bande centrale) ; exclusions Ku copiées dans chaque secteur (`ku_spectrum_exclusions_ghz`) ; forçage `ku_freq_ghz_candidates` contrôlé à la validation de la mission. Espace Ku : `[tool.confirmation]` (impulsions 5 µs à τ_max, PRI 0,5 à 5 ms, bandes 25 à 200 MHz, deux intégrations, variante agile des trains non cohérents) ; portée à l'angle θ = portée à 0 × h(f, θ), commune à toutes les formes d'onde (vérifiée au calcul) ; ciel en T^-1/4 ; brouilleur d'autoprotection par la forme exacte ; impulsion raccourcie à PA saturé (`PositionCosts` rendu indépendant de la bande). Marges continues (`margin_db_tol` 0,01 dB ; toucher S2 sur la grille de 0,5 dB d'avant (`s2_touch_step_db`), sa marge Ku par bissection). Grappe : séquentielle, superposition (0,76 dB par interpolation en log₂, la table de la veille n'est pas modifiée) ou partition Ku (moitiés, quarts) ; la moins chère est gardée ; allers-retours publiés. Tables de confirmation avec la loi Gamma de la cible à chaque bande (lois exactes de `core/detection.py`). Fouillis Ku par observation (modèle de P4 : rapport au quantile du groupe, rejet Doppler, gain qui rétablit le Pd), retiré du budget T_sys Ku. Recherche Ku de percée publiée pour un secteur hors de portée sous brouillage. Rôles : `CONFIRM` permis en S, `PROMOTE` supprimé. Faraday de la Ku au centre de la bande centrale (la fréquence Ku est choisie après le plan de veille).

**Tests de Q1 à Q5 (24 octobre 2026)** : auto-test réussi ; tests nouveaux de la 3.2 réussis après deux corrections (quantiles du SNR détecté à k = 1, marge S2 vectorisée, grille du toucher ramenée à 0,5 dB) ; suite physique complète : 171 réussis, 1 échec attendu (`test_cost_table_matches_the_parameter_file`, table de coût non régénérée), environ 21 min en parallèle (3 606 s cumulées). Tests de valeurs figées non lancés : ils échouent par construction tant que Q6 n'est pas faite. Le temps de calcul des missions a crû de ×1,5 à ×4,9 (`test_parallel_run_equals_sequential` 229 → 1 115 s), au-delà de l'estimation. Commit intermédiaire de Q1 à Q5 validé par l'utilisateur (valeurs figées et table de coût non régénérées).

**Reprise : Q6**, dans l'ordre validé : (a) profiler une mission et chercher des accélérations **exactes** (aucune approximation sans accord) ; puis table de coût régénérée, rapport d'impact, valeurs figées regelées (`JOURNAL.md`), documentation (§11, feuille de route, `design/radar_design.tex`), au N3 choisi par l'utilisateur ; avant ce N3, rendre les tests de valeurs figées moins coûteux (parallélisme en fréquence sous pytest).

**Profilage (Q6 a, 24 octobre 2026, arrêt de session ici)** : `mission_example` en tout séquentiel sous cProfile (`RADAR_FREQ_PARALLEL=0`, `optimizer.run(mission, workers=1)`, script dans un `if __name__ == "__main__"` sous Windows), 533 s profilées (le profileur gonfle le code Python très appelé).

| Partie | Temps | Ce qui coûte |
|---|---|---|
| Recherche S (3 secteurs, 57 fréquences × 8 pas = 456 plans) | 361 s (68 %) | `rm.solve` appelé 985 000 fois (environ 2 160 formes d'onde par plan, à θ = 0) : 315 s, dont 117 s d'inversion des lois (`detection.required_snr_db`, 105 000 lois nouvelles) et environ 200 s de pertes, blocs, fluctuation, bancs et diagnostic (`pulse`, `frame`, Pd par observation) dont l'optimiseur ne garde que 5 nombres |
| Budget Ku (57 couples secteur × fréquence) | 172 s (32 %) | marge S2 (`tracking.ConfirmStats.s2_vec`) 99 s : `pd_single` 74 s (dont 70 s de `np.interp` sur de grands tableaux), gains des 7 faisceaux (`_tx_gain`) 23 s recalculés à chaque pas de bissection ; espaces d'observations Ku (`_LookSpace`) 52 s |

Accélérations exactes proposées, **toutes retenues par l'utilisateur le 25 octobre 2026 et codées (modèle V6.12.1)** :

| | Proposition | Exactitude | Gain estimé (temps profilé) |
|---|---|---|---|
| E1 | `solve` en deux parties : une commune aux 8 pas de grille, calculée une fois par (forme d'onde, fréquence) (blocs cohérents, fluctuation, bancs, pertes fixes, gains), et une propre au pas et à la Pfa (pertes de position, SNR requis, CFAR, polarisation) ; voie de l'optimiseur sans diagnostic ; `solve` reste la référence et partage le même code | identique au bit (mêmes opérations, même ordre de sommation), test d'égalité contre `solve` | −120 à −160 s (recherche S), −30 s (espaces Ku) |
| E2 | `s2_vec` : gains des faisceaux gardés par grille de toucher (indépendants de la marge Ku) | identique au bit | −22 s |
| E3 | `pd_single` : interpolation par indice sur la grille SNR régulière (comme `optimizer._pd_at`) au lieu de `np.interp` | écarts d'arrondi (~1e-16), classe de A2 | −40 à −50 s |
| E4 | Budget Ku : fréquences Ku d'un secteur en processus parallèles (comme la recherche S) | identique | temps réel du budget Ku divisé par 3 à 6 |

Écarté : accélérer l'inversion des lois (117 s) demanderait une autre recherche de racine, qui change le résultat à la tolérance (1e-4 dB). Total attendu : environ −250 s profilées sur 533 (−40 à −50 % par mission), plus E4. Vérification proposée après codage, sur accord : test d'égalité E1 / E2 (< 1 min), puis nouveau profilage (~6 min). Question de l'utilisateur avant le choix (25 octobre 2026) : les calculs, notamment au limbe des corps, ne valent-ils que pour une orientation du vaisseau ? Réponse : oui, un seul message de navigation par mission (attitude, date, orbite) ; positions des corps sur les faces, aveuglement, raffinement, fouillis et place des secteurs non liés au vaisseau en dépendent ; le test de rotation (D7) valide le calcul, pas la tenue de la mission quand l'attitude change. Décision : **on garde l'ordre** (enveloppe d'attitudes en 5.3, profil dans le temps en 4.3 et 5.4 ; enveloppe passée en phase 6 le 27 octobre 2026).

Codage (V6.12.1) : E1 `radar_model.solve_base` + `solve_look` (les pertes qui dépendent du regard gardent leur place dans le dictionnaire, total sommé dans le même ordre), `solve` = les deux + diagnostics, l'optimiseur garde `solve_base` en cache (`tool.optimizer.solve_base_cache_size` = 16 384, environ 2 Ko par entrée) ; E2 gains des faisceaux gardés par grille de toucher dans `s2_vec` ; E3 `tracking._interp_grid` (indice sur la grille SNR régulière, plat au-delà des bouts) ; E4 budget Ku de chaque couple (secteur, fréquence Ku) en processus parallèles (`tracking._ku_freq_task`, mêmes conditions que la recherche S). Trois tests ajoutés (égalité E1 au bit contre `solve`, E2 au bit contre le calcul direct, E3 à 1e-12 près de `np.interp`). **Reprise : tests sur accord de l'utilisateur** (tests ciblés, comparaison avec la version précédente, nouveau profilage), puis N3 (table de coût, rapport d'impact, valeurs figées) et documentation de Q6.

**Tests de E1 à E4 (25 octobre 2026)** : tests ciblés passés ; comparaison avec la version précédente sur 129 024 appels de `solve` (mission_example) et les marges S2 des 5 variantes de cluster : **identiques au bit** ; profilage 533 s → 329 s (−38 %), 159 s en parallèle. N3 arrêté pendant le rapport d'impact : **la machine (64 Go) a manqué de mémoire** (processus du pool gardés vivants d'une mission à l'autre, caches sans borne).

**Approximations (25 octobre 2026, règle de précision absolue supprimée par l'utilisateur, modèle V6.13.0).** Tableau effet / impact présenté (recherche S 69 % du calcul, dont inversion des lois 31 % ; budget Ku 31 %). Retenues par l'utilisateur et codées, chacune réglable dans `[tool.*]` (la valeur exacte reste disponible) :

| | Approximation | Réglage | Effet | Gain attendu |
|---|---|---|---|---|
| A1 | Lois des pertes de position arrondies à 0,02 dB. Constat en codant : la Pfa était déjà sur une grille de 1/20 de décade ; ce sont les **lois de position** (part du filtre de vitesse) qui faisaient 19 071 lois par secteur et fréquence | `tool.detection.pos_law_step_db` = 0,02 | SNR requis à 0,003 dB près au plus (mesuré sur 150 lois) | lois distinctes ÷ 6 |
| A2 | Tolérance de l'inversion du critère cumulé 1e-4 → 1e-3 dB | `tool.detection.cum_snr_tol_db` | < 0,001 dB | ~−10 s |
| A4 | Perte de polarisation (GLRT) en cache par (échantillons, fluctuation, Pfa) | — | exact | ~−25 s |
| A5 | Fréquence cherchée au pas de grille par défaut (1,4), puis les 8 pas à la fréquence retenue | `tool.optimizer.spacing_after_frequency` = true | faible (à mesurer) | recherche S ÷ 3 à 4 |
| A6 | Spectre S calme (sans exclusion ni brouilleur) : 3 centres répartis sur la bande centrale | `tool.optimizer.freq_quiet_centres` = 3 | faible (test : moins de 3 % sur l'objectif) | recherche S ÷ 4 environ |
| A8 | Spectre Ku calme : 3 centres Ku sur la bande centrale | `tool.confirmation.ku_freq_quiet_centres` = 3 | faible | budget Ku ÷ 4 à 6 |
| A9 | Marges S2 : tolérance 0,05 dB (0,01), 16 quantiles (32), 8 nœuds de Gauss-Hermite (12) | `tool.confirmation.*` | < 0,05 dB (à mesurer) | −15 à −20 s |
| T1 | Banc de test : sortie de chaque cas gardée avec l'empreinte du code (`tests/.case_cache/`, local) ; un N3 calcule chaque cas une fois | `RADAR_CASE_CACHE=0` force le calcul | aucun | N3 ÷ 2 à 3 |
| T3 | Caches bornés (`tool.detection.snr_cache_size`, `mean_curve_cache_size`, `position_cache_size`) ; `optimizer.release_memory()` arrête le pool et vide les caches après chaque mission du banc | — | aucun | mémoire bornée |

**N3 du 25 octobre 2026 (première passe avec la barre de progression, T1 et T3)** : 18,7 min (plus d'une heure le 23 octobre ; `mission_frames` 2,7 min au lieu de plus de 45) ; 178 tests passent, seuls les 10 tests des valeurs figées échouent (attendu : valeurs de l'étape 3.1, écarts de 3.2 Q1-Q5 et des approximations, 13 178 valeurs). Mesure de l'effet sur `mission_example` et `mission_sky_demo` (V6.12 exact contre V6.13.0) : énergie et temps de veille à +0,25 % au plus, sauf EARTH −5,1 % (trois centres trouvent une meilleure fréquence que la recherche en deux passes, qui n'était pas exhaustive : la fréquence compte pour un secteur qui voit une surface) ; marges Ku +0,02 à +0,07 dB ; 7 à 8 fois plus rapide. Mode S2 → KU1+7 dans 7 secteurs : mesuré sur `mission_example` avec la tolérance S2 remise à 0,01 dB, le mode reste KU1+7 ; les trois modes ont la **même charge de mission à 1e-4 près** (0,0119) : égalité de fait départagée par des écarts minimes, pas un biais à corriger.

Non retenues pour l'instant : A3 (quantiles 16 → 8), A7 (espace de formes d'onde réduit puis affiné), A10 (ciel 4 × 4). **Étape 3.2 close le 25 octobre 2026** : valeurs figées regelées (journal), référence anglaise mise à jour. À revoir : VEILLE_KV de mission_veille500 (+9 % d'énergie de veille, 2,765 → 2,77 GHz), probablement A6 : la fréquence n'est pas plate pour tous les secteurs (options : 5 centres, ou une passe fine autour du meilleur des 3).

### Étape 3.3 : poursuite (cadrage ouvert le 25 octobre 2026)

**Raison d'être** : donner à la poursuite active un coût physique. Aujourd'hui (3.2), une tranche de poursuite coûte une observation Ku par mise à jour à sa portée maximale, SNR = max(détection, précision), période `rr_s` imposée, sans dynamique de piste, sans initialisation ni perte de piste, accélération supposée connue et erreur de prédiction sans perte de pointage. **Entrées** : tranches de la mission (portées, nombre de cibles, précisions, période voulue), classe de cible (a_max, vitesse, SER, fluctuation, dépolarisation), espace des observations Ku et environnement Ku de 3.2, précision angulaire (Cramér-Rao × 1,13), biais S/Ku. **Sorties** : par tranche, période tenue ou calculée, SNR et énergie par mise à jour et leur moteur, erreur de prédiction et perte de pointage, coût d'initialisation, probabilité de perte et coût de réacquisition ; par face, charge de poursuite et alertes. **Hors périmètre** : chronologie (4.3), calcul (4.4), association de données et simulation piste par piste (5.4).

| Question de cadrage | Décision de l'utilisateur |
|---|---|
| Périmètre et blocs A (filtre et cadence), B (pointage et maintien), C (initialisation), D (cibles dans la tranche), E (hypothèses optimistes : accélération inconnue, couplage distance-Doppler, atténuateur) | **validés** |
| Bloc F (polarimétrie : discrimination, classification, Faraday, commutation e_u / e_v ; SER selon l'aspect ; brouillage trompeur ; volume lié à une piste) | **tout en 3.3** |
| Période de mise à jour | **L'opérateur fixe une valeur.** Si elle n'est pas tenable, l'outil donne la valeur calculée en alerte ; l'opérateur décide si elle est acceptable ou s'il bouge d'autres leviers (précision, nombre de cibles, portée, etc.) pour tenir la période voulue. |

**Bloc A, filtre et cadence (validé le 25 octobre 2026, toutes les propositions)** :

| | Variable | Décision |
|---|---|---|
| A1 | Filtre | Kalman en régime établi, 3 états par axe (position, vitesse, accélération), modèle de Singer, Riccati résolue numériquement |
| A2 | Axes et mesures | 3 axes découplés : distance (mesures de distance et de vitesse radiale Doppler), deux axes transverses (R·θ, angle par Cramér-Rao) ; précisions de mesure tirées du SNR de l'observation Ku |
| A3 | Accélération | a_max de la classe sur chacun des 3 axes (la déviation d'un KKV est surtout transverse) |
| A4 | Intensité de Singer | σ_m² = a_max²/3 (1 + 4 P_max − P₀), P_max = 0, P₀ = 0 (médiane, pas le pire cas) |
| A5 | Constante de temps τ_m | KKV 5 s, DECOY comme le KKV, GENERIC 20 s, SHIP 60 s, DEBRIS / ST sans manœuvre. Constat : DECOY a `accel_max_mps2` = 0 (non propulsé), son τ_m est donc sans effet ; un leurre qui ne suit pas la déviation du KKV est un indice de discrimination (bloc F) |
| A6 | Bruit de processus minimal | 0,01 m/s² (dynamique relative mal modélisée) |
| A7 | Sens des précisions | zone angulaire (σ = zone / 4) sur l'erreur **prédite** à la mise à jour suivante ; distance et vitesse radiale sur l'erreur **filtrée** après la mise à jour |
| A8 | « Non tenable » | aucune observation Ku de l'espace ne tient les précisions à la période de l'opérateur ; l'alerte donne la période la plus longue qui les tient, avec l'observation la moins chère ; la surcharge d'une face reste au verdict de charge |

Nouveaux réglages prévus : `[targets.<classe>].maneuver_tau_s`, `[tracking]` `p_max`, `p0`, `process_noise_floor_mps2`.

**Bloc B, pointage et maintien (validé le 25 octobre 2026, toutes les propositions)** :

| | Variable | Décision |
|---|---|---|
| B1 | Perte de pointage | erreur angulaire prédite (2 axes) en loi de pertes de position dans la Pd de l'observation Ku (faisceau Ku réel, méthode de 2.3) |
| B2 | Erreur prédite en distance | sans effet (fenêtre numérique large), contrôlée et signalée au-delà d'un seuil |
| B3 | Banc de vitesses de poursuite | ±3 σ de la vitesse radiale prédite (au lieu d'une hypothèse) |
| B4 | Observation simple ou grappe | le moins cher des deux par mise à jour (modes de grappe de 3.2), sans seuil fixe |
| B5 | Pd par mise à jour | `tracking.pd_update` = 0,9 (réglage de mission plus tard) |
| B6 | Manque | piste extrapolée d'une période, observation suivante recalculée avec le pointage dégradé |
| B7 | Perte de piste | après N = 3 manques de suite (`tracking.n_miss_lost`), probabilité exacte par chaîne de Markov |
| B8 | Coût d'une perte | nouvelle détection de la veille S (déjà couverte) : confirmation (3.2) + initialisation (bloc C) au taux de perte |
| B9 | Coût par période | espérance sur la chaîne : observation nominale + extrapolations + perte |

Nouveaux réglages prévus : `tracking.pd_update`, `tracking.n_miss_lost`, `tracking.vel_bank_sigmas`.

**Bloc C, initialisation (validé le 25 octobre 2026, toutes les propositions)** :

| | Variable | Décision |
|---|---|---|
| C1 | État initial | position = mesure Ku de confirmation, vitesse radiale = Doppler, vitesse transverse σ = v_close / 3 (`tracking.init_vel_sigma_frac`), accélération σ = a_max/√3 |
| C2 | Cadence | à la période de l'opérateur dès la première observation ; nombre d'observations calculé en itérant la covariance ; perte de pointage plus forte au début (B1) |
| C3 | Fin | précisions de A7 à 10 % de leur régime établi (`tracking.init_conv_frac` = 0,1) |
| C4 | Taux | nouvelles cibles du secteur (`new_targets_per_rr`) + pistes perdues (B7), réparti par seconde |
| C5 | Fausses pistes | fausses alarmes passant la confirmation Ku : comptées à leur taux, signalées au-delà d'un seuil |
| C6 | Début de piste | à la portée de confirmation si une tranche la couvre, sinon à l'entrée dans la tranche la plus lointaine (désignée par la veille S) |

**Bloc D, cibles dans la tranche (validé le 25 octobre 2026)** : D1 coût moyenné sur 8 portées de la tranche (cibles uniformes en distance ; nombre de portées dans `[tool.*]`) ; D2 période vérifiée au pire point de la tranche ; D3 vitesse v_close_max du secteur ; D4 `n_targets` inchangé (population instantanée) ; D5 répartition sur faces et dépointages comme les pointages de veille.

**Bloc E, hypothèses optimistes (validé le 25 octobre 2026)** : E1 accélération estimée par le filtre, banc d'hypothèses sur ±3 σ de l'accélération prédite (`tracking.accel_bank_sigmas`) ; E2 résidu du couplage distance-Doppler c·τ·σ_fd / (2B) en quadrature dans la mesure de distance (LFM et NLFM) ; E3 atténuateur commutable par observation après le LNA, jusqu'à 30 dB par pas de 10 dB (`rx.attenuator_max_db`, `rx.attenuator_step_db` par bande), portée non saturée ÷ 10^(A/40) ; E4 à E6 planchers en quadrature : angle 0,05 mrad par axe, distance 0,05 m, vitesse radiale 0,01 m/s (`tracking.angle_floor_mrad`, `range_floor_m`, `vr_floor_mps`). Remarque : ces trois planchers sont des valeurs médianes de leur plage, pas le bout optimiste (0,02 mrad, 0,01 m, 0,001 m/s) que la règle des valeurs technologiques demanderait : signalé à l'utilisateur, qui a gardé les valeurs validées.

**Bloc F (validé le 25 octobre 2026, toutes les propositions)** : F1 discrimination KKV / leurre = probabilité de discrimination correcte par piste et nombre d'observations Ku pour l'atteindre, indices indépendants (dépolarisation par les deux voies, statistique de fluctuation, retour optique, manœuvre vue par le filtre), chacun un test statistique dont la précision dépend du SNR et du nombre d'observations ; F2 `tracking.p_discrimination` = 0,95 (réglage de mission plus tard), alerte avec la portée où elle est atteinte ; F3 même cadre sur les couples de classes : matrice de séparabilité, indicateur sans coût ; F4 Faraday : perte d'estimation sur l'indice de dépolarisation (rotation de 2.5 et son incertitude) ; F5 commutation e_u / e_v évaluée comme option des observations de discrimination (0,15 dB de chaîne d'émission contre matrice de diffusion complète), choisie si moins chère ; F6 table SER(aspect) facultative par classe, aspect tiré de la géométrie d'approche et d'un écart d'aspect de classe, SER frontale sans table ; F7 brouillage trompeur : indicateur de risque de capture si le brouilleur d'autoprotection est déclaré trompeur, vérification comptée au taux `tracking.deception_verify_rate` ; F8 repère de secteur `track` centré sur la direction prédite d'une piste ou d'une désignation, ouverture k σ de l'erreur prédite, mis à jour à chaque période.

**Codage (V6.14.0, 25 octobre 2026, passes R1 à R6 lancées d'un bloc par l'utilisateur)** : `core/trackmodel.py` (filtre, exigences, chaîne de Markov, initialisation, discrimination), `solve()` (loi de pointage, Pd de l'observation, bancs de poursuite, couplage distance-Doppler), budget de poursuite de `core/tracking.py`, atténuateur, table d'aspect, repère `track` (avec l'interface), sorties dans le rapport et l'onglet des résultats ; 12 tests ajoutés, un test adapté (l'accélération d'une observation de poursuite n'est plus nulle d'office). Choix et approximations faits en codant, **à valider** :
- **Exigences découplées** (A7) : l'exigence de distance filtrée est tenue par un filtre qui n'utilise que la mesure de distance, celle de vitesse radiale par un filtre de vitesse seule ; le filtre joint est meilleur, donc c'est prudent. Cela permet une seule bissection par exigence, indépendante de l'observation.
- **Pointage évalué au lobe Ku de face (θ = 0)** pour toutes les tranches de dépointage : prudent (le lobe s'élargit au dépointage) et cela partage les espaces d'observations entre groupes.
- **Grappe de poursuite** : seulement la grappe hexagonale de 7 faisceaux **à la suite** (coût × 7) ; la superposition coûte 7 L, toujours plus ; la partition n'est pas proposée en poursuite.
- **Initialisation** : critère sur les **précisions de l'opérateur** × (1 + 0,1), et non sur les valeurs en régime établi (un débris, presque sans bruit de processus, demandait des centaines d'observations pour s'en approcher) ; les observations d'initialisation mesurent au moins aussi bien que la confirmation (mesure = min(admise, passage de la confirmation)) ; coût = nombre d'observations × une observation pointée à l'erreur moyenne de l'initialisation.
- **Réacquisition** après une perte = une initialisation + une observation de confirmation Ku du groupe principal.
- **Chaîne de perte** pour une observation simple seulement ; la grappe garde sa Pd aux mises à jour extrapolées.
- **Discrimination** faite sur les observations de poursuite (pas d'énergie en plus ; la perte de 0,15 dB de la commutation de polarisation n'est pas comptée en énergie, seulement dans le SNR des indices) ; portée de décision = portée de départ − v·n·T, alerte sous la plus courte portée d'intérêt du secteur.
- **Constat (F3)** : la matrice de séparabilité donne P ≈ 1 entre toutes les classes, parce que l'indice de niveau de SER compare des **moyennes de classe sans dispersion** : c'est optimiste. Proposition : une dispersion de SER par classe (σ en dB) dans la bibliothèque.
- L'estimation rapide (`cost_estimate`) garde l'ancienne observation de poursuite (accélération connue, sans pointage) : recalage renvoyé en 5.5.
- **N3 (25 octobre 2026, 21 min)** : 190 tests passent ; 416 valeurs figées changent (aucune sortie de la veille S ; temps Ku ×3 à ×3,8 sur mission_synthetic_mix, +7 à +9 % sur mission_target_library ; toutes les périodes des missions de test sont tenues ; 32 observations d'initialisation pour le leurre de mission_target_library) ; valeurs regelées (`JOURNAL.md`).

**Cadrage complet (blocs A à F).** Passes de code proposées : R1 paramètres et modèle de piste (A, alerte de période) ; R2 pointage, bancs de vitesse et d'accélération, grappe ou observation simple, chaîne de perte (B, E1) ; R3 initialisation, moyenne sur la tranche, planchers, couplage, atténuateur (C, D, E2-E6) ; R4 discrimination, Faraday, commutation de polarisation, classification (F1-F5) ; R5 SER selon l'aspect, brouillage trompeur, repère `track` (F6-F8) ; R6 rapport d'impact, valeurs figées, documentation (N3 au choix de l'utilisateur).

### Étape 3.4 : fonctions annexes (cadrage ouvert le 25 octobre 2026)

**Raison d'être** : le modèle suppose un radar parfait en permanence (tous les TRM, aucun temps d'étalonnage, résidu d'étalonnage forfaitaire 0,5 dB en S et 0,8 dB en Ku). L'étape rend compte du temps d'étalonnage et des erreurs des TRM. **Entrées** : composition des faces, méthode / cadence / durée d'étalonnage, erreurs résiduelles des TRM (aléatoires et corrélées par TRSA), dérive entre deux étalonnages. **Sorties** : part de temps des faces prise par l'étalonnage, pertes de gain et plancher de lobes secondaires dérivés, contrôles de cohérence du plancher d'angle et du biais S/Ku. **Hors périmètre** : chronologie (4.3), alimentation (4.1), thermique (4.2), priorités (5.1, 5.2).

| Question de cadrage | Décision de l'utilisateur |
|---|---|
| Périmètre, blocs A (étalonnage), B (erreurs des TRM), C (santé des TRM), D (modes dégradés), E (intégration) | validés |
| Modes dégradés (D) et pannes au fil de la mission (C, sauf le coût du BIT) | **reportés en fin de développement** : le MVP est un modèle fonctionnel en situation **nominale** |
| Résidu d'étalonnage, plancher d'angle, biais S/Ku | **option (c), hybride** : perte de gain et plancher de lobes secondaires **dérivés** des erreurs résiduelles (aléatoires et corrélées par TRSA, en fin d'intervalle d'étalonnage) ; plancher d'angle et biais S/Ku gardés en entrée, avec contrôle de cohérence et alerte. **Le résidu `calibration` (0,5 dB S, 0,8 dB Ku) est remplacé par la valeur dérivée** (décision de l'utilisateur sur une valeur V). |

**Variables (validées le 25 octobre 2026, toutes les propositions)** :

| | Variable | Valeur |
|---|---|---|
| A1 | Méthode | étalonnage interne par les coupleurs de chaque TRM (boucle émission et réception), tous les TRM d'une face à la fois (codes orthogonaux) ; couplage mutuel en contrôle ; alignement S/Ku sur objets extérieurs renvoyé à 4.3 |
| A2 | Intervalle `cal_interval_s` | 300 s (S et Ku) |
| A3 | Durée `cal_duration_s` par face | S 10 ms, Ku 50 ms (part du temps 3·10⁻⁵ et 1,7·10⁻⁴) |
| A4 | Énergie | négligée |
| A5 | Dérive corrélée par TRSA | 0,1 °/min, 0,01 dB/min |
| B1 | Erreur aléatoire par TRM | 1°, 0,1 dB (bout optimiste ; aujourd'hui 2 à 3°, 0,2 à 0,3 dB) |
| B2 | Erreur corrélée par TRSA | 0,5°, 0,05 dB |
| B3 | Quantification de phase | nulle (formation numérique, retard vrai par TRM) |
| B4 | Perte dérivée | Ruze (phase e^(−σ²), amplitude 1/(1+σ²)) sur la variance totale en fin d'intervalle, émission et réception : **≈ 0,006 dB** en S comme en Ku, au lieu de 0,5 et 0,8 dB |
| B5 | Plancher de lobes moyens | σ²/(η N) + σ_c²/(η N_TRSA) : **≈ −59 dB** ; alerte s'il dépasse l'option de réception la plus basse − 10 dB |
| B6 | Cohérence | dépointage des erreurs corrélées ≈ 0,015 mrad (S), 0,003 mrad (Ku), sous le plancher d'angle de 0,05 mrad ; biais S/Ku 1,1 mrad au-dessus du dépointage combiné ; alerte sinon |
| BIT | Contrôles intégrés | compris dans l'étalonnage (la boucle par coupleur sert de surveillance), pas de coût à part |
| E1-E3 | Intégration | temps d'étalonnage pris sur le temps d'émission S et Ku de chaque face dans le budget de mission (sorties `calibration_s_time`, `calibration_ku_time`) ; section `[band.*.errors]` qui remplace l'entrée `calibration` ; sortie `array_errors` ; pas de réglage par mission |

**Codage (V6.15.0, 25 octobre 2026)** : `radar_model.array_error_loss_db`, `array_errors`, `calibration_checks` ; perte `calibration` dérivée au chargement ; temps d'étalonnage et alertes dans `tracking.budget` ; 2 tests. **N3 (25 octobre 2026, 20,5 min)** : 190 tests passent ; portées WF-LRS +2,89 %, énergie et temps S des faces −7 à −14 % ; le temps d'étalonnage Ku (1,7·10⁻⁴) domine désormais les petits temps Ku des missions de test ; quatre secteurs de mission_redundancy_demo abandonnent leur impulsion la plus longue. Le test calculé à la main du self-test gardait 0,5 dB : corrigé (Ruze calculé depuis le fichier). Valeurs figées regelées (`JOURNAL.md`).

### Étape 4.1 : puissance électrique (cadrage ouvert le 25 octobre 2026)

**Raison d'être** : aujourd'hui seule la moyenne de l'énergie sur la revisite est vérifiée (`face_elec_mw` × chaîne 0,917) ; ni la puissance instantanée d'une face pendant une observation (≈ 8,2 MW à d = 0,65 contre 2 MW alloués), ni la recharge des réserves des TRM pendant un train, ni la somme des allocations des faces face aux 8 MW du vaisseau (défaut constaté : seule une face est comparée aux 8 MW). **Entrées** : allocation par face (mission), consommation hors émission par élément, capacité continue d'un TRM, stockage par face. **Sorties** : par face, consommation hors émission, puissance d'observation, stockage nécessaire contre disponible, formes d'onde retirées ; somme des allocations ; alertes. **Hors périmètre** : thermique (4.2), chronologie fine (4.3), calculateur (4.4), optimisation de la répartition (5.2).

| Question de cadrage | Décision de l'utilisateur |
|---|---|
| Périmètre, blocs A (répartition des 8 MW), B (consommation hors émission), C (alimentation des TRM), D (puissance instantanée et stockage de face), E (transitoires du bus, sortie seulement) | validés |
| Forme d'onde au-delà de l'alimentation d'un TRM | **retirée** de l'espace de recherche |
| Contrôle du stockage avant l'ordonnanceur | **les deux** : par observation et sur la rafale la plus défavorable de la revisite |

**Variables (validées le 25 octobre 2026)** :

| | Variable | Décision |
|---|---|---|
| A (révisé à la question de l'utilisateur) | Répartition | **réserve commune** : somme des puissances moyennes appelées par les 4 faces (émission + hors émission, à travers la chaîne) ≤ `ship_elec_total_mw` 8 MW ; une face peut dépasser 2 MW si les autres consomment moins ; plafond physique par face `face_feeder_max_mw` = **8 MW** ; `face_elec_mw` de la mission devient un **plafond facultatif réservé par l'opérateur** (nombre ou liste de 4) ; verdict d'énergie au niveau du vaisseau ; l'optimiseur de veille garde une référence de 8 / 4 = 2 MW par face pour arbitrer énergie et temps (arbitrage conjoint en 5.2) ; sorties : puissance appelée par face, marge de la réserve, répartition suggérée |
| B1 | CAN (250 Méch/s, 13 bits) | 0,1 W (aujourd'hui 0,3 à 1 W) : S 1,0 kW, Ku 24,5 kW par face |
| B2 | Voie de réception par polarisation | S 0,5 W, Ku 0,1 W : S 4,9 kW, Ku 24,5 kW |
| B3 | Formation de faisceaux et traitement | 50 kW par face (provisoire, 4.4) |
| B4 | Contrôle et servitudes | 2 kW par face |
| B5 | Pompes du refroidissement | 2 % de la chaleur dissipée moyenne (4.2) |
| B6 | Réception | allumée en permanence ; total ≈ 107 kW par face + pompes ; `overhead_frac` supprimé |
| C1 | Alimentation continue d'un TRM | S 1 700 W, Ku 80 W |
| C2 | Recharge de la réserve pendant un train | à la capacité continue entre deux impulsions ; forme d'onde **retirée** si l'énergie nette tirée sur le train plus une impulsion dépasse `buffer_max_fraction` de la réserve |
| D1-D3 | Stockage de face (S + Ku) | 5 MJ, décharge 15 MW, profondeur utilisable 80 % |
| D4 | Contrôle par observation | énergie de l'observation − puissance de recharge × durée ≤ stockage utilisable ; puissance appelée ≤ 15 MW |
| D5 | Rafale | émissions d'une revisite supposées contiguës : déficit ≤ stockage utilisable, recharge au plafond physique de la face |
| E1 | Transitoires | sortie seulement : puissance appelée au début d'une observation et sa pente sur le front des impulsions |

**Codage (V6.16.0, 25 octobre 2026)** : `radar_model.non_tx_power_w`, `face_bus_draw_w`, `trm_supply_check` (dans `solve_base` : la forme d'onde est retirée), `face_elec_avail_w` net de la consommation permanente ; `optimizer.run` garde la référence de 2 MW, la clé de mission devient un plafond ; `tracking.budget` : appel du bus par face, plafond (alimentation de la face, plafond de l'opérateur), charge du vaisseau et répartition suggérée, contrôles du stockage par observation (pire passage de veille S du plan et pire observation Ku) et par rafale, pointe, transitoires ; interface : plafond par face (0 = partagé). Constats en codant :
- **Rafale (D5)** : regrouper toutes les émissions d'une revisite (≈ 5 s à ≈ 12 MW pour mission_veille500, revisite 120 s) épuise toujours les 4 MJ utilisables (déficit ≈ 24 MJ). Le contrôle reste une alerte et donne en plus une **règle d'ordonnanceur** : au plus ≈ 0,29 s d'émission contiguë à pleine puissance par face, puis recharge (4.3). Le statut de la face n'en dépend pas.
- Aucune forme d'onde S n'est retirée par l'alimentation des TRM ; 68 observations Ku sur 2 356 le sont (trains longs à fort facteur de charge).
- Les missions de test gardent `face_elec_mw` = 2 : elles fonctionnent avec un plafond de 2 MW par face (comportement proche de l'ancien) ; un plafond nul ou absent met les faces en commun.
- **N3 (25 octobre 2026, 22,2 min)** : 194 tests passent ; 1 361 valeurs figées changent, aucun jeu de formes d'onde S ne change ; la charge d'énergie des faces monte (consommation permanente ≈ 5,7 % d'un plafond de 2 MW, visible aussi sur les faces au repos) ; `test_tracking_loads_add_up` mis à jour (temps d'étalonnage de 3.4, charge d'énergie = appel du bus / plafond) ; valeurs regelées (`JOURNAL.md`). Référence anglaise enrichie : glossaire des grandeurs et des phénomènes physiques, tableau final des effets modélisés avec leurs fourchettes d'impact.

### Étape 4.2 : thermique (cadrage ouvert le 25 octobre 2026)

**Raison d'être** : la chaleur n'est aujourd'hui qu'une sortie (≈ 370 W par TRM S en pointe à d = 0,65 ; chaleur moyenne par face), sans capacité d'évacuation vérifiée ; la consommation permanente de 4.1 (106,9 kW par face) n'est pas comptée dans la dissipation ; `duty_max` (0,65, D), la pente de PAE, la dérive d'étalonnage et les pompes tiennent lieu de limites thermiques. **Entrées** : chaleurs (1.3, 4.1), résistances et constantes de temps thermiques, températures maximales, capacité de refroidissement par face, rejet du vaisseau réservé au radar. **Sorties** : montée de température de jonction par forme d'onde, chaleur totale et flux par face avec marge, température de la plaque froide sur une rafale, chaleur du vaisseau contre rejet, alertes et règles d'ordonnanceur. **Hors périmètre** : calcul détaillé (CFD, éléments finis), radiateurs, chronologie (4.3), dérive des performances RF avec la température.

| Question de cadrage | Décision de l'utilisateur |
|---|---|
| Blocs A (bilan de chaleur complet), B (jonction GaN, Z_th transitoire), C (plaque froide à microcanaux, inertie, rafale), D (rejet du vaisseau, commun aux faces), E (couplages) | validés |
| Forme d'onde au-delà de la température de jonction | **retirée**, mais **l'impact doit être quantifié** ; si l'impact sur le modèle est trop fort, retour à `duty_max` fixe |
| Couplages (pompes dérivées du débit, dérive d'étalonnage reliée à la température, contrôle de dilatation du jeu de 0,5 mm) | **rien pour l'instant** : reportés en fin de développement avec les autres effets secondaires (modes dégradés, pannes de TRM) |

**Fluide de refroidissement (donnée de l'utilisateur, 25 octobre 2026)** : dérivé du submer SmartCoolant (diélectrique ; en 2020 : capacité calorifique 2,26 kJ/(kg·K), coefficient d'échange ≈ 2 050 W/(m²·K) en circulation forcée, densité 0,796). **Valeurs 2050-2060 validées** (progrès réaliste) : capacité calorifique 2,5 kJ/(kg·K), coefficient d'échange 3 000 W/(m²·K) en microcanaux, densité 0,8, multiplication de surface des microcanaux ×10, écart plaque–fluide maximal 40 K (plaque ≤ 60 °C pour un fluide à 20 °C), échauffement du fluide 10 K. La capacité de refroidissement par face (C1, ≈ 1,6 MW) et le flux admissible (C2, ≈ 12 W/cm²) deviennent **dérivés** du fluide ; inertie (C3) ≈ 190 kJ/K (50 L de fluide + 100 kg de structure). **Exigence de l'utilisateur** : comme pour toutes les variables, le code doit absorber un changement ultérieur de ces valeurs (tout dérivé du fichier). Impact annoncé d'abord à tort (erreur d'unité de l'assistant, W/m² lus comme W/cm² ÷ 10) : ≈ 1,6 MW par face et 12 W/cm². **Valeurs corrigées en codant** : flux admissible 3 000 × 10 × 40 = 1,2·10⁶ W/m² = **120 W/cm²**, capacité **≈ 15,9 MW par face** (≈ 10,9 MW avec les valeurs 2020) : la plaque froide ne limite jamais une face (au plus 8 MW appelés) ; un TRM S en train dégage ≈ 14 W/cm², loin des 120.

**Variables (validées le 25 octobre 2026, toutes les propositions)** : A1 chaleur d'émission = électrique − RF ; A2 électronique permanente 106,9 kW par face dans le circuit de la face ; A3 pertes d'alimentation et de distribution (5,5 % de l'appel) dans le circuit du vaisseau ; A4 pompes 2 % de la chaleur dans le circuit de la face ; B1 réseau de Foster à 2 pôles + résistance de convection plaque–fluide dérivée du fluide (régime établi) ; B2 S 0,02 K/W (10 µs) + 0,03 K/W (1 ms) ; B3 Ku 0,2 K/W (5 µs) + 0,3 K/W (0,5 ms) ; B4 fluide à 20 °C en entrée ; B5 jonction ≤ 225 °C ; B6 impact mesuré avant adoption (formes d'onde retirées, `duty_max` dérivé contre 0,65, effet sur deux missions), décision ensuite ; C4 échauffement admissible de la plaque 10 K (tampon ≈ 1,9 MJ) ; C5 rafale : alerte + règle d'ordonnanceur ; D1 rejet du vaisseau réservé au radar **6 MW (D, provisoire, donnée du vaisseau)** ; D2 commun aux 4 faces (somme des chaleurs + pertes de conversion).

**Mesure B6 et décision (25 octobre 2026)** : sur 16 128 formes d'onde S et 2 356 observations Ku, jonction au plus 59 °C et 57 °C, ≈ 62 °C même en émission continue : **aucune forme d'onde retirée**, aucune limite thermique du facteur de charge. **Décision de l'utilisateur** : `duty_max` 0,65 gardé fixe (il traduit d'autres limites : PAE, alimentation des TRM) ; contrôle de jonction désactivé (`tool.thermal.junction_check` = false), température de jonction donnée en sortie (`junction_c` de chaque forme d'onde de veille). Seule contrainte thermique active : le rejet du vaisseau (6 MW, D) ; mission_veille500 : 1,0 MW (17 %).

**Codage (V6.17.0)** : `radar_model.face_cooling`, `r_conv_trm`, `junction_temp_c`, `junction_check` ; budget : chaleur du circuit de la face (émission + électronique permanente + pompes) contre la capacité (charge « heat » dans le verdict), rafale thermique contre le tampon de la plaque avec règle d'ordonnanceur, chaleur du vaisseau (faces + pertes de conversion) contre le rejet ; 2 tests ajoutés, 1 adapté. **N3 (25 octobre 2026, 20,7 min)** : 197 tests passent ; 746 valeurs figées changent (nouvelles sorties seulement : aucun chiffre clé, aucune forme d'onde, aucun mode de confirmation ne change) ; valeurs regelées (`JOURNAL.md`).

### Étape 4.3 : ordonnanceur (cadrage ouvert le 25 octobre 2026)

**Raison d'être** : les ressources ne sont comptées qu'en moyenne (parts d'énergie et de temps d'émission), l'entrelacement est supposé parfait (`frame_interleaved`, D), la latence n'existe plus depuis la revue 0.3 et les règles d'ordonnancement accumulées (cadence commune par bande, harmonique S sur la Ku, décalage de grille, émission contiguë maximale électrique et thermique, étalonnages, percée Ku, CFAR par pointage, vérification face au brouilleur trompeur, réévaluation du multiplexage en temps et de la superposition) sont publiées sans être appliquées. L'étape vérifie que les tâches tiennent **dans le temps** et chiffre la **latence**. **Hors périmètre** : calcul (4.4), optimisation globale (5.2), Monte-Carlo à arrivées aléatoires (5.4), réaffectation des secteurs aux faces avec l'attitude (5.2, 5.4).

| Question de cadrage | Décision de l'utilisateur |
|---|---|
| Blocs A (modèle des tâches), B (approche), C (entrelacement et éclipse), D (règles entre faces et bandes), E (latence), F (surcharge), G (règles accumulées) | validés |
| Approche | **(a) bilan statistique** (occupation, files d'attente par priorité, latences moyennes et quantiles, pertes d'entrelacement) **+ (b) chronologie synthétique déterministe** par face sur un cycle (algorithme glouton, priorités et échéances) pour appliquer les règles ; la simulation à événements discrets reste en 5.4 |
| Exigence de latence d'un secteur | **les deux** : clé de mission par secteur, prioritaire ; sinon dérivée du temps avant impact |
| Priorités (corrigées par l'utilisateur le 25 octobre 2026 : faute de frappe dans la première version) | **P0 étalonnage** (sans lui le radar ne marche pas) ; **P1 volumes d'autoprotection** (une **case à cocher** sur un volume, qui le passe en P1 ; sinon c'est un volume de veille ordinaire) ; **P2 confirmation et initialisation** (ne pas perdre une détection de la veille : confirmer et initialiser au plus tôt) ; **P3 mises à jour de piste** (quelques manques restent gérables) ; **P4 veille**, avec une **sous-priorité par volume choisie par l'opérateur** (P4_1, P4_2, P4_3, …) |

**Percée Ku dans l'axe d'un brouilleur (décision de l'utilisateur, 25 octobre 2026)** : **interdite par défaut** ; elle ne peut être autorisée que par l'opérateur **en vol** (« en live »), face à un brouillage réel, **jamais en préparation de mission**. Autorisée, la tâche de percée est de niveau **P3'** : planifiée en queue de P3 (après les mises à jour de piste), avant la veille P4. En Pmiss, l'observation de percée de l'étape 3.2 reste une **sortie d'information** (ce qu'elle coûterait si l'opérateur l'autorisait), hors budget et hors chronologie.

**Variables (validées le 25 octobre 2026, toutes les propositions, avec les priorités corrigées)** :

| | Variable | Décision |
|---|---|---|
| A1 | Veille | une tâche par position et par trame (émission + écoute 2R/c), période = revisite ; P4_x par la clé de secteur `search_priority` (1 = la plus prioritaire, défaut 1) ; P1 si `self_protection` = true |
| A2-A3 | Confirmation, initialisation | P2 ; arrivées de Poisson aux taux de la face ; durées et allers-retours des modes |
| A4 | Mise à jour de piste | P3, périodique, tolérance ±10 % de la période, au-delà un manque (chaîne de perte de 3.3) |
| A5 | Étalonnage | P0, toutes les 300 s, un bloc de 10 ms (S) / 50 ms (Ku), retard ≤ 10 % de l'intervalle |
| A6 | Percée, vérification | percée hors Pmiss (P3' en vol si autorisée) ; vérification face au brouilleur trompeur attachée à la mise à jour (P3) |
| B1-B3 | Chronologie synthétique | durée = plus longue revisite de la face, plafonnée à 60 s ; glouton par priorité puis échéance, sans préemption ; arrivées tirées une fois avec une graine fixe ; quantiles de latence par le bilan statistique |
| C1-C3 | Entrelacement et éclipse | perte −10 log(1 − ρ_bande), ρ = part de temps des créneaux d'émission de la bande ; calculée et signalée au-delà de 0,5 dB sans relancer le plan (bouclage en 5.2) ; `frame_interleaved` vérifié par la chronologie |
| D1-D2 | Règles entre faces et bandes | créneaux d'émission communs par bande, part fixée par la face la plus chargée (coût d'alignement) ; observations Ku hors des créneaux S quand l'harmonique couvre le canal Ku |
| E1-E3 | Latence | file à priorités non préemptive + allers-retours + traitement 1 ms (provisoire, 4.4) ; quantile 95 % ; exigence `latency_max_s` du secteur, sinon 10 % du temps avant impact (`latency_frac`) |
| F1-F2 | Surcharge | délestage : veille P4 de la sous-priorité la plus basse vers la plus haute (revisite étirée), puis mises à jour P3 (période étirée, alerte) ; P0, P1, P2 jamais délestés (mission non faisable sinon) ; sorties : revisite et période effectives, latence au quantile 95 %, ce qui glisse et pourquoi |
| G1-G5 | Règles accumulées | émission contiguë maximale (électrique et thermique) appliquée dans la chronologie ; étalonnages en P0 ; décalage de grille : règle publiée ; GO-CFAR automatique près d'un limbe (0,05 dB compté) ; gain de trame du multiplexage en temps et de la superposition chiffré et publié, hors optimiseur (5.2) |

Nouvelles clés de secteur : `self_protection`, `search_priority`, `latency_max_s` ; réglages `scheduler.latency_frac`, `update_jitter_frac`, `processing_s`, `[tool.scheduler]` durée de chronologie.

**Codage (V6.18.0, 25 octobre 2026)** : nouveau module `core/scheduler.py`, appelé par `optimizer.run` après le budget de confirmation et de poursuite (clé de rapport `scheduler`). Classes de tâches par face et par bande tirées du plan de veille et du budget Ku (veille par secteur, S2, confirmations, initialisations, mises à jour, étalonnages) ; bilan statistique (attentes de Cobham, quantile 95 % par queue exponentielle au-delà de la probabilité d'occupation, délestage P4 puis P3) ; chronologie synthétique par face (glouton, sans préemption, graine fixe, horizon = plus longue revisite S de la face plafonnée à 60 s, pauses de recharge du stockage et de la plaque froide) ; créneaux communs par bande et éclipse ; règle de l'harmonique S sur le temps Ku ; latence de confirmation contre l'exigence ; interface : case « self-protection (P1) » et sous-priorité de veille. Réglages : `[scheduler]` latency_frac 0,1, update_jitter_frac 0,1, processing_s 1 ms (D), calibration_max_delay_frac 0,1, eclipse_flag_db 0,5 ; `[tool.scheduler]` horizon_max_s 60, seed 1 ; 4 tests. Écarts au cadrage, **à signaler** :
- **G4** (GO-CFAR automatique près d'un limbe, 0,05 dB compté) : seulement **conseillé en sortie** (`go_cfar_advised`), le coût n'entre pas encore dans le bilan (écart de 0,01 dB avec le CA-CFAR) ;
- **G5** (gain de trame du multiplexage en temps et de la superposition) : **non chiffré** ; avec l'entrelacement vérifié, l'attente des échos est déjà cachée et le gain est nul dans ce modèle ; à reprendre en 5.2 si l'entrelacement devient imparfait ;
- essai sur mission_synthetic_mix : créneaux S 2,4 % (éclipse 0,10 dB), toutes les latences tenues (7,7 ms pour KV contre 2 s), aucun délestage, aucune mise à jour en retard.
- **N3 (25 octobre 2026, 19,6 min)** : 200 tests passent ; 103 valeurs figées changent (nouvelles sorties seulement) ; sur les missions de test, créneaux S de 0,1 à 4,3 % du temps (éclipse 0,004 à 0,19 dB), toutes les latences de confirmation tenues (1,4 ms à 0,11 s au quantile 95 %), aucun délestage ; `test_every_parameter_is_read_by_the_code` a trouvé des entrées lues seulement à l'appel de leur fonction (consommation permanente, inertie thermique) : lues désormais au chargement ; valeurs regelées (`JOURNAL.md`).

### Étape 4.3 bis : ordre des pointages dans un volume (ouverte le 25 octobre 2026)

Insérée par l'utilisateur entre 4.3 et 4.4 (le rythme des changements de pointage et les écoutes simultanées alimentent 4.4 ; l'ordre réel sert surtout en 5.4). Script de référence analysé à la demande de l'utilisateur : `RADARv5/geom_v1_scan_visualizer_realtime_PLANE_PROJ_PSCLB.py` (RADARv5 obsolète comme source : idées seulement). Il pose des anneaux dans un plan angulaire approché et ordonne les pointages par PSCLB (cases de 5° × 5°, toujours la moins visitée : uniformité progressive ; puis proximité du pointage précédent). Retenu : le principe d'**uniformité progressive** (une trame étirée ou interrompue par le délestage se dégrade uniformément) et la revisite régulière ; écartés : la grille du script (le modèle a la grille hexagonale exacte en (u, v)), le biais de proximité (changer de pointage ne coûte rien avec le retard vrai numérique par TRM ; au plus un départage). Rappel donné à l'utilisateur : l'élargissement du faisceau aux grands dépointages est physique (ouverture projetée D cos θ) et exactement rendu en (u, v) (nombre de pointages, gain en cos θ, désadaptation active, précision en 1/cos θ) ; seul manque le diagramme réel de l'élément (point ouvert).

| Question | Décision de l'utilisateur |
|---|---|
| Option d'ordre | **(2) ordre à faible discordance** (Halton ou renversement de bits) sur la grille (u, v) du plan de veille, départage par proximité ou numéro de position ; avec décalage de la grille à chaque trame, durée par tranche de dépointage, groupes multifaisceau, insertion par priorité, revisite par direction et uniformité en sortie |

**Variables (validées le 25 octobre 2026)** : V1 ordre de Morton à bits renversés (van der Corput à deux dimensions) sur les pointages (u, v), départage par numéro ; V2 cycle de 3 décalages de la grille, 0, (a + b)/3, 2(a + b)/3 ; V3 durée de chaque pointage = temps d'émission de la forme d'onde de sa tranche (impulsion raccourcie du plan) ; V4 les pointages suivants groupés par « positions par passage » en multifaisceau ; V5 revisite par direction mesurée dans la chronologie, alerte au-delà de 10 % ; V6 uniformité à 10, 25 et 50 % de la trame (part des 5 × 5 cases grossières visitées contre l'idéal) ; V7 la chronologie de 4.3 sert les vrais pointages dans l'ordre, le bilan statistique ne change pas.

**Codage (V6.19.0)** : `core/scanorder.py` (grille hexagonale du plan en (u, v) par face, décalages, ordre, durées par tranche, groupes multifaisceau, mesures) ; l'ordonnanceur l'utilise dans sa chronologie et publie par volume et par face le nombre de pointages, le temps d'émission d'une trame, l'uniformité, le saut moyen et le débit de pointages (entrées de 4.4), et la dispersion de la revisite ; réglages `[tool.scanorder]` (n_shifts 3, coarse_cells 5, fractions 0,1 / 0,25 / 0,5, revisit_flag_frac 0,1) ; 2 tests. Mise au point : la quantification des positions doit être **plus grossière que l'écart entre pointages** (2^L cases par côté, la plus grande puissance de 2 sous l'étendue / l'écart), sinon les bits fins brouillent l'ordre. Essais : pointages réels à 1-4 % des positions du plan (discrétisation) ; uniformité 0,76 à 0,96 dès 10 % de la trame, 0,92 à 1 à 50 % ; dispersion de la revisite ≈ 10 ms sur 120 s ; saut moyen 14 à 50° (sans coût avec le retard vrai numérique).

**N3 (25 octobre 2026, 21,6 min)** : 203 tests passés ; seuls les snapshots ont changé (189 valeurs : nouvelles sorties `scan` et `revisit_spread_s`, délais de départ de la chronologie déplacés de quelques ms par les vrais pointages), regelés. Pointages réels : 0 à +20 % au-dessus de l'estimation par l'aire du plan (effets de bord des petits volumes) ; uniformité à 25 % de la trame 0,92 à 1,0 sur les grands volumes, jusqu'à 0,36 sur les petits volumes découpés. Correction du banc : le cas `model` gardait l'objet interne d'espace des visées Ku (son adresse mémoire changeait à chaque exécution) ; il est retiré de la sortie.

### Étape 4.4 : calcul et données (ouverte et codée le 25 octobre 2026)

**Cadrage validé par l'utilisateur** (toutes les propositions A1 à F3) ; consigne : toutes les valeurs du bloc C (et les autres) doivent pouvoir changer à mesure que l'architecture s'affine, donc rien n'est écrit dans le code (section `[compute]` de `radar_system.toml`).

- **A, architecture** : chaîne hiérarchique. Chaque TRSA ramène ses voies à la bande du signal (changement de fréquence numérique, échantillons complexes à `adc_oversampling` × la bande, I et Q sur `iq_bits` = 16). En S elle envoie ses 50 voies brutes au processeur de face (`transport_S = raw` : former 43 faisceaux sur place enverrait plus de données) ; en Ku elle forme des faisceaux partiels (`transport_Ku = subarray_beams`, 1 250 voies pour quelques faisceaux). Formateur de faisceaux : 64 faisceaux simultanés par face en S, 16 en Ku. Anti-brouillage sur les 196 sorties de TRSA (`nulling_dof = trsa`). Les CAN Ku sont toujours alimentés, leurs données ne sont traitées que pendant les visées Ku.
- **B, charge** par visée (veille S, confirmation Ku, mise à jour de piste) : formation de faisceaux 8 opérations par multiplication-accumulation complexe × voies × faisceaux × fs × durée d'écoute ; compression 10 N log₂N par impulsion, faisceau et polarisation ; bancs de vitesse et d'accélération 8 × hypothèses × cellules (×4) × impulsions, plus la FFT Doppler (×4) ; CFAR 10 opérations par cellule et hypothèse ; anti-brouillage N² par échantillon d'apprentissage (2 N échantillons), N³/3 par inversion, application des poids, sur une face qu'un brouilleur éclaire ; pistage 10⁴ opérations par mise à jour. Moyennes = somme des cadences × coûts ; fenêtres d'écoute ouvertes simultanément par la loi de Little (somme des cadences × durées d'écoute), faisceaux simultanés = ce nombre arrondi au-dessus × les faisceaux du plus gros dwell (majorant).
- **C, technologie 2050-2060** (bout optimiste) : 0,01 pJ par opération (mémoire comprise), 0,05 pJ par bit transporté ; `power.dbf_kw` (50 kW) devient le **plafond** de puissance du calcul, dont 20 % toujours consommés ; capacité = (plafond × 0,8 − puissance de transport) / énergie par opération ; liens 4 Tbit/s par TRSA, 400 Tbit/s en entrée du processeur de face.
- **D, latence** : les données arrivent au processeur pendant l'écoute ; après la fin du dwell restent le retard d'un lien plus lent que le flux (nul ici) et le calcul à la capacité laissée par la charge moyenne. Elle remplace `scheduler.processing_s` (1 ms provisoire, clé retirée).
- **E, en vol** : le calculateur applique le plan Pmiss, l'ordonnanceur et le pistage, et resélectionne une forme d'onde dans des tables précalculées quand le spectre mesuré change : `reselect_ops_per_entry` (10³ opérations par couple forme d'onde × tranche, **D** : valeur proposée gardée par l'utilisateur, à revoir en fin de développement du modèle, étape 5.7) ; la réoptimisation complète reste en Pmiss ou en 5.2 ; la part « attitude » est en 4.5.
- **F, branchements** : latence calculée dans l'ordonnanceur ; alertes faisceaux, liens, entrée de face et charge de calcul ; puissance du calcul rapportée par face.

**Codage (V6.20.0, 25 octobre 2026)** : nouveau module `core/compute.py`, appelé par `optimizer.run` après le budget Ku et avant l'ordonnanceur (clé de rapport `compute`) ; le budget Ku exporte les nombres de ses visées (`ku_confirm_wf` par secteur, `wf_params` par tranche de poursuite). Faisceaux d'un dwell = faisceaux de réception par position × positions du dwell (le champ d'une partition compte déjà les positions de son sous-faisceau : × nombre de faisceaux émis). 3 tests. Écart au cadrage, **à signaler** :
- **F1** : les bilans de puissance et de chaleur qui dimensionnent le plan (optimiseur, budget Ku) gardent le **plafond** de 50 kW comme consommation permanente : la consommation calculée dépend de la charge Ku, calculée après eux, et une boucle demanderait de refaire le plan. La consommation réelle (≈ 10 kW par face sur les missions de test) et le tirage et la chaleur corrigés (`non_tx_kW_actual`, `bus_draw_MW_actual`, `face_heat_kW_actual`) sont rapportés à côté ; la boucle complète est à faire avec l'optimiseur global (5.2). **Accepté par l'utilisateur le 25 octobre 2026.**
- Essais : charge de calcul 10¹¹ à 2·10¹⁶ op/s par face (le maximum : bancs de vitesse d'un secteur à fouillis de surface) contre une capacité ≈ 4·10¹⁸ op/s (≤ 0,6 %) ; faisceaux simultanés jusqu'à 42 en S (sur 64) et 7 en Ku (sur 16) ; liens 0,2 Tbit/s par TRSA en S ; tampons cohérents jusqu'à 167 Go sur une face (secteur à fouillis : aucune capacité mémoire n'est fixée, sortie seulement) ; latence de traitement 0,01 µs à 5 ms.

**N3 (25 octobre 2026, 20,7 min)** : 206 tests passés ; seuls les snapshots et le test des invariants de calcul (qui les lit) ont changé, regelés (92 valeurs : nouvelle sortie `compute`, nombres des visées Ku, latences de confirmation avec la latence de traitement calculée). **Nouvelles alertes** sur deux missions de test : des formes d'onde S en émission partitionnée (4 sous-faisceaux sur 13,7 positions) demandent 155 à 171 faisceaux de réception simultanés contre 64 au formateur. L'optimiseur ne connaît pas encore cette capacité : à faire entrer dans le choix des formes d'onde (avec la boucle de 5.2, ou plus tôt sur décision de l'utilisateur).

**Après le N3 (décisions de l'utilisateur, 25 octobre 2026)** : formateur de faisceaux porté à **512** faisceaux simultanés en S et **32** en Ku (les formes d'onde partitionnées des missions de test en demandent jusqu'à 171 ; coût de calcul négligeable) ; **mémoire de 4 To par face** pour les tampons cohérents (bout optimiste de 0,5 à 4 To) et **débit mémoire de 50 To/s par face** (provisoire), deux nouvelles ressources contrôlées (`compute.memory_tb`, `compute.memory_bw_tbps`) ; trafic mémoire compté avec **une seule lecture par dwell** (les bancs de vitesse et d'accélération travaillent dans la mémoire interne du processeur ; une relecture par hypothèse donnait 10 800 To/s sur un secteur à fouillis). Capacité de faisceaux dans l'optimiseur : avec la boucle de 5.2 (plus aucun dépassement sur les missions de test).

**N3 (25 octobre 2026, 20,5 min)** après ces décisions : 207 tests passés ; seuls les snapshots et le test mémoire (qui les lit) ont changé, regelés (112 valeurs, toutes sous `compute` et les alertes) ; plus aucune alerte de calcul sur les missions de test.

### Étape 4.5 : re-pointage en vol (ouverte et codée le 25 octobre 2026)

**Cadrage validé par l'utilisateur** : blocs B, C, D, E, F tels que proposés ; vitesse angulaire max **30 °/s** et accélération angulaire max **15 °/s²** (valeurs de l'utilisateur, éditables dans `[repointing]` de `radar_system.toml`) ; navigation à 100 Hz, retard 5 ms, connaissance d'attitude 1″ et de vitesse 10⁻⁴ °/s (1 σ par axe), tolérance 0,05 lobe, 100 opérations par pointage.

- **B, boucle** : la grille d'un volume est fixée une fois par trame dans son propre repère ; chaque pointage est converti en (u, v) de face à son émission avec le dernier message de navigation prolongé par la vitesse mesurée, et émis par la face qui le voit alors ; les faisceaux de réception sont formés avec l'attitude de la réception ; les volumes `ship` ne bougent pas, le limbe suit l'altitude à chaque message.
- **C, erreurs** : à une émission, la navigation a au plus retard + un intervalle = 15 ms ; erreur quadratique moyenne E[r²] = 2 (σ_att² + (σ_vitesse × âge)²) + (½ α âge²)² (l'accélération non mesurée, pire cas, domine : 6,1″ sur 15 ms). Veille : perte moyenne aller-retour 2 × 12,04 E[r²] / lobe², ajoutée aux pertes de chaque bande (`nav_pointing`, au haut de la bande, lobe le plus fin) : 3·10⁻⁵ dB en S, 8·10⁻⁴ dB en Ku ; erreur 0,0012 lobe en S et 0,0059 en Ku contre 0,05 ; accélération tenable 648 °/s² en S, 131 °/s² en Ku. Poursuite : l'erreur par axe (4,4″) s'ajoute quadratiquement à l'erreur de pointage du filtre (3.3).
- **D, migration** : un volume qui n'est pas lié au vaisseau traverse une frontière de face en (son étendue sur la face) / 30 °/s, soit 0,3 à 1,3 s sur les missions de test ; pendant ce temps la face voisine doit porter sa charge (charge de la face + charge du secteur ≤ 1, sinon alerte). La vérification sur toute l'enveloppe d'attitudes passe en phase 6 (décision du 27 octobre 2026 : pas d'enveloppe en Pmiss) ; le verdict de la 5.3 reprend la migration à l'attitude de la mission (état transitoire).
- **E, calcul** : 100 opérations par pointage et, une fois par trame, par direction du raster pour le masque d'un volume non lié au vaisseau : fonction `repointing` du budget de la 4.4 (≈ 10⁷ op/s par face, négligeable).
- **F** : perte et erreur de poursuite branchées dans le bilan ; alertes « accélération non tenable » et « migration au-delà de la marge de la face » ; vaisseau en rotation dans la vue 3D reporté à la phase 6.

**Codage (V6.21.0, 25 octobre 2026)** : nouveau module `core/repoint.py` (clé de rapport `repointing`), perte `nav_pointing` dans les pertes de bande, erreur de navigation dans le pointage du modèle de piste, fonction de calcul `repointing` ; 2 tests. Écart au cadrage, **à signaler** : la vitesse et l'accélération max sont des réglages du fichier de paramètres (éditables), pas des clés de mission ; une valeur par mission demanderait une perte de bande propre à la mission (à reprendre en 5.1 si besoin). La vitesse angulaire n'entre pas dans l'erreur de pointage (elle est mesurée et compensée) : elle ne joue que sur la migration.

**N3 (25 octobre 2026, 23,8 min)** : 207 tests passés. Snapshots changés (3 774 valeurs : la perte `nav_pointing` déplace toutes les portées d'au plus 0,04 %, nouvelles sorties de re-pointage ; aucun jeu de formes d'onde, fréquence, mode de confirmation ni statut ne change), regelés. Deux autres échecs, corrigés : le test interne du modèle lancé en script (import relatif du nouveau module, repli ajouté) et le test « repère inertiel à attitude identité = repère vaisseau » (un volume inertiel re-pointe en vol quelle que soit l'attitude du moment : le travail de re-pointage et ses totaux peuvent différer, test adapté).

### Étape 5.1 : modèle de mission enrichi (ouverte et codée le 25 octobre 2026)

**Cadrage validé par l'utilisateur** : A (réglages opérationnels dans la mission) validé ; B tables de SER par présentation, **12 azimuts × 7 élévations** (tous les 30°, 84 valeurs par classe), remplies à partir des valeurs connues ; tous les aspects possibles ; **détection au pire aspect** (`tool.targets.aspect_policy = worst`), **sauf le KKV, pris de face** (il vise le vaisseau : nez et autodirecteur vers le radar) ; dispersion d'un objet à l'autre et cône optique gardés ; C une seule phase, **interrupteur de mission autorisant la percée brouilleur** (alors budgétée et ordonnancée en P3′) ; D pas de volumes types ; E pas de plafond d'observations ; F choix de la classe et nouveaux réglages dans l'interface, **interface provisoire** (une IHM complète la remplacera en phase 6).

- **P1, réglages** : mission `confirm_p_min`, `ship_rate_max_dps`, `ship_accel_max_dps2` (la perte de bande `nav_pointing` reste dimensionnée par le fichier, une mission plus sévère est signalée avec la perte manquante) ; volume `confirm_p_min`, `pd` (Pd cumulée de veille, portée par le critère de recherche jusqu'à `solve_look`), `fa_rate_per_s` (à la place de `fa_per_rr`) ; tranche `pd_update`, `p_discrimination`. Sorties `pd_cumulative`, `fa_per_rr`, `confirm_p_min` par secteur.
- **P2, bibliothèque** : table `aspect_rcs_db` par classe (D) : KKV 0 / +2 / +5 / +10 / +15 dB à 0 / 15 / 30 / 60 / 90° du nez, arrière symétrique ; SHIP 0 → +10 dB de profil ; DECOY, DEBRIS, GENERIC, ST plates. Interpolation bilinéaire sur une grille de 2° qui contient le nez. Politiques : `worst` (moindre SER totale, optique comprise dans son cône), `frontal` (au décalage d'aspect de la 3.3), `mean` (moyenne par angle solide). La moyenne aurait donné KKV × 1,9 et SHIP × 1,5 en portée : écartée par l'utilisateur. Avec les tables actuelles, **aucune portée de détection ne change**. Dispersions (D) : SER 3 dB (KKV, DECOY, SHIP), 5 dB (DEBRIS, GENERIC), 0 (ST, référence du banc), dépolarisation 2 dB (0 pour ST) ; cône optique du KKV ±5°. Discrimination : dispersion d'aspect (sauf classe vue de face) et dispersion d'objet, termes systématiques des indices de SER et de dépolarisation (KKV / DECOY après 5 visées : indice de SER 20,5 → 6,4).
- **P3, percée** : `burn_through_allowed` (faux par défaut) ; autorisée, la visée Ku de percée d'un secteur hors de portée de veille à cause de son brouilleur d'autoprotection entre dans le budget des faces (une visée par revisite, répartie comme les positions du secteur) et dans l'ordonnanceur en P3′ (classe (3, 1), après les mises à jour, avant la veille).
- **P4, interface** : groupe « Mission settings » ; dans le formulaire du volume, classe de cible (valeurs affichées, surcharges signalées), Pd cumulée, fausses alarmes par revisite ou par seconde, `confirm_p_min` du volume ; colonnes `pd_update` et `p_discrimination` des tranches. Une valeur égale au fichier n'est pas écrite dans la mission.
- Écarts à signaler : les dispersions par classe sont un choix de l'outil dans la plage validée (3 à 5 dB) ; la dispersion d'aspect ne s'applique pas à une classe vue de face.

**N3 (25 octobre 2026, 22,2 min)** : 211 tests passés ; seuls les snapshots ont changé (291 valeurs : nouvelles sorties de mission, de percée et de présentation, séparabilité des classes avec les dispersions), regelés ; aucun jeu de formes d'onde, fréquence, mode de confirmation, statut ni portée ne change.

### Étape 5.2 : optimiseur global (25 et 26 octobre 2026, modèle V6.23.0)

**Cadrage validé par l'utilisateur** : A boucle de point fixe sur les optimiseurs existants ; B objectif « face la plus chargée de la mission », puis somme des charges ; C les 8 MW du vaisseau répartis au prorata du besoin ; en cas de manque ou d'égalité, priorités par niveaux, toutes faces confondues : P0 étalonnage et permanent, P1 autoprotection, P2 confirmations et initialisations, P3 poursuite par classe de cible (KKV > SHIP > DECOY > DEBRIS > autres), P3′ percée, P4_n veille (un P1 de l'antenne 2 passe avant un P3 de l'antenne 3) ; points traités en 5.2 : 1 coût des fausses alarmes, 2 répartition des 8 MW, 3 éclipse, 4 puissance réelle du calcul, 5 capacité de faisceaux, 7 critère secondaire, 8 tables pour le vol ; point 6 (entrelacement imparfait, GO-CFAR) reporté en 5.4 ; au plus 3 passes, +30 % de temps au plus.

- **P1, répartition** (`core/allocation.py`, clé `power_allocation`) : besoin de chaque face décomposé par niveau (permanent avec la puissance **réelle** du calcul de la 4.4, pompes ; veille P1 ; confirmations ; poursuite par classe ; percée ; veille P4_n), égal à la puissance réelle tirée par la face (à 0,1 kW près) ; puissance suffisante : prorata du besoin sous les plafonds (alimentation 8 MW, plafond opérateur), l'excédent d'une face plafonnée reporté sur les autres ; manque : niveaux servis dans l'ordre, le dernier en partie, RR tenable (veille) ou nombre de pistes tenables (poursuite), alerte « mission not tenable » si P0 à P2 n'est pas servi en entier. Vérifié sur les missions de test (besoin 0,73 à 0,90 MW) et sur un manque simulé (vaisseau ramené en mémoire à 0,5 MW, fichier inchangé).
- **P2, couplages dans le choix des formes d'onde de veille** (clés internes de secteur remplies par la boucle) : `_fa_energy_J` (énergie Ku de confirmation d'une fausse alarme, ajoutée à l'énergie de chaque position au prorata des fausses alarmes du candidat) ; `_eclipse_db` (perte d'éclipse des créneaux communs, appliquée aux portées comme le facteur de T_sys) ; filtre permanent des candidats dont un dwell demande plus de faisceaux que le formateur (512 en S). Sans ces clés, les sorties sont identiques aux snapshots. Constat sur le point 1 : avec la politique `fa_rate` (2.3, défaut), le nombre de fausses alarmes par revisite est fixe, leur coût Ku ne dépend pas de la forme d'onde (≈ 25 W pour une face de 1,8 MW) ; il ne joue qu'en mode `pfa_cell`.
- **P3, boucle de point fixe** (`optimizer.run` → `_assemble`, `_couplings`, `_mission_choice`, clé `global_opt`) : chaque secteur garde jusqu'à 6 alternatives (fréquence, pas de grille) de même couverture et même portée ; après chaque assemblage des budgets, l'éclipse n'est réinjectée qu'au-delà de `scheduler.eclipse_flag_db` (0,5 dB), le coût des fausses alarmes qu'en mode `pfa_cell` ; le choix global retient, secteur par secteur, l'alternative qui baisse la face la plus chargée de la mission (énergie et temps S, confirmations et poursuites comprises), puis la somme ; seuls les secteurs touchés sont replanifiés (`_force_plan` : une seule fréquence évaluée) ; arrêt quand rien ne change, au plus `tool.global_opt.max_iterations` = 3 passes. Sur les missions de test : aucun changement (les charges s'additionnent face par face et la répartition entre faces est fixée par la géométrie : le meilleur choix propre est aussi le meilleur pour la mission), temps inchangé. Réglages `[tool.global_opt]` max_iterations 3, alternatives 6 ; 3 tests.
- **P4, réemploi de la marge** (règle ajoutée par l'utilisateur le 26 octobre 2026, `core/margin.py`, clé `margin_reuse`) : la RR demandée d'un volume est un **minimum** ; si les ressources ont de la marge (mission « facile »), le radar raccourcit les revisites jusqu'à l'utilisation **X = `auto_alloc_max`** (« allocation auto max », **une valeur par mission**, défaut **0 = désactivé**, `[tool.global_opt]`). Exemple de l'utilisateur : un volume à RR 20 s peut tourner à RR 2 s si X est assez haut. Décisions de l'utilisateur :
  - **niveau de priorité** : la marge réaffectée est le niveau **P5**, sous la veille P4_n (la première rendue en cas de surcharge, avant tout délestage) ;
  - **X porte sur la ressource dimensionnante** : énergie du vaisseau (besoin / 8 MW, partagée par les faces), et par face énergie (besoin / plafond d'alimentation ou opérateur), temps S et Ku, chaleur, calcul, mémoire, débit mémoire, faisceaux ;
  - **même facteur g pour tous les volumes**, P1 (autoprotection) compris, monté ensemble (remplissage) : les volumes d'une face qui atteint X s'arrêtent, les autres continuent ; un volume à cheval sur plusieurs faces est borné par la plus chargée ;
  - **RR minimale optionnelle par volume** (`rr_min_s`), **sans valeur par défaut : c'est à l'opérateur de la spécifier** (utilisateur, 26 octobre 2026 ; une RR crédible est de l'ordre de 1 à 2 s, sans plancher le réemploi peut descendre à des valeurs inutiles, 0,09 s à l'essai) ;
  - **fausses alarmes figées par seconde** (politique `fa_rate` : la Pfa par case baisse comme 1 / g, la charge de confirmation ne bouge pas) ;
  - **seulement la RR pour l'instant** (« veille plus loin », « volumes de moindre priorité mieux servis » : à affiner plus tard) ;
  - **plan gardé** (formes d'onde, pas de grille, échelle d'impulsion par tranche) : seule la revisite s'accélère, RR réelle = RR / g ; la **portée de détection cumulée à la RR réelle** est calculée et publiée par tranche (`range_km_at_rr_real`, à pleine impulsion comme `range_km_at_bin_max`) ;
  - le verdict reste porté sur la RR demandée ; la RR réelle est une sortie ;
  - **la règle vaut aussi en vol** (phase 6) : le radar alloue et reprend dynamiquement aux différentes fonctions selon la charge, les besoins et les ressources.

  Mise en œuvre : une sonde (toutes les veilles à g = 2) donne la pente de chaque ressource, répartie entre secteurs au prorata de leur énergie sur la face (énergie, chaleur), de leur temps S (temps S) ou de leur cadence de pointage (les autres) ; remplissage ; assemblage complet des budgets aux RR réelles ; une ressource non linéaire (ordonnanceur, chaleur) est corrigée par la sécante des deux derniers points et le remplissage refait, au plus `reuse_corrections` = 2 fois (tolérance 0,5 point) ; la part de veille due au raccourcissement est un niveau P5 de la répartition de la P1 (`power_allocation` aux RR réelles publiée dans `margin_reuse`) ; les alertes nouvelles aux RR réelles sont publiées (préfixe « at the real revisits »). Interface provisoire : « Allocation auto max » dans Mission settings, « RR min » dans le formulaire du volume ; RR réelle affichée dans le résultat du secteur. 4 tests.
  - Essai, `mission_example` à X = 80 % : utilisation nominale 22 % sur chaque face (faisceaux Ku : 7 sur 32, ressource qui ne dépend pas de la RR), énergie 4 % du vaisseau ; RR réelles S01 20 → 0,09 s (g = 224, bornée par l'énergie de F1, plafond opérateur 2 MW), S02 5 → 0,068 s et S03 120 → 1,6 s (g = 74, bornées par l'énergie de F2) ; portées cumulées +56 % (S01), +89 % (S02), +66 % (S03) ; vaisseau à 60 % ; 4,4 MW en P5. Avec `rr_min_s` = 5 s sur S01 : g = 4, portée +19 %. Nouvelle alerte aux RR réelles : éclipse des créneaux communs 0,54 dB (non réinjectée dans le plan). Temps : deux assemblages de plus et une résolution des portées par secteur (59 → 119 s à froid).
  - **Écarts à signaler** : en politique `pfa_cell`, la Pfa par case est un réglage de l'opérateur, gardée : les fausses alarmes par seconde montent avec g (comptées dans la charge) ; les faisceaux simultanés entrent dans X mais ne dépendent pas de la RR (ils ne bornent jamais g, ils peuvent seulement fixer l'utilisation nominale publiée) ; l'ordonnanceur ne sépare pas la part P5 de la veille (une seule classe par volume, la séparation en vol est en phase 6).
- **P5, tables pour le vol** (26 octobre 2026, cadrage validé par l'utilisateur, toutes les propositions) :
  - **A, contenu** : par secteur, un plan de veille par fréquence centrale évaluée par la recherche (son meilleur pas de grille) : fréquence, pas, bande occupée, couverture, part des positions en deçà de la portée, pire portée, charges d'énergie et de temps de la face la plus chargée, nombre de groupes de tranches ; le plan retenu d'abord ; plans déjà calculés pendant la recherche, aucun calcul de plus ;
  - **sélection** : au plus **8 entrées** (`compute.flight_table_entries_max`, D), réparties sur la bande (la fréquence la plus éloignée de celles déjà gardées, à égalité le classement de la recherche) ;
  - **entrées dégradées gardées** (question 1 : oui) : un plan qui laisse plus du secteur non balayé ou plus de positions en deçà de la portée que le plan retenu est marqué `degraded` ;
  - **B, règle en vol** : si la bande occupée du plan courant devient brouillée ou interdite, le calculateur prend la meilleure entrée dont la bande est libre et dont la charge tient sur la face (une entrée non dégradée d'abord), sinon alerte puis délestage selon les priorités ;
  - **C, taille** : **64 octets par entrée et par groupe de tranches** (`compute.flight_table_bytes_per_entry`, D) ; mémoire de la face = somme des tables des secteurs qu'elle sert, ajoutée aux tampons cohérents contre les 4 To (`flight_tables_TB`) et dans la ressource « mémoire » du réemploi de la marge ;
  - **D, coût** : `reselect_ops` = 10³ opérations × nombre d'entrées × groupes de tranches de chacune (au lieu de formes d'onde × tranches du plan retenu) ;
  - **E, sortie** : clé `flight_table` de chaque secteur (entrées, octets, dégradées, pire repli) et ligne du rapport console ;
  - **Ku** (question 4) : veille S seulement ; le repli de la fréquence Ku de confirmation est renvoyé en 5.7.

  Codage : `optimizer._flight_table` (dans `_best_plan`, avant la conversion du plan en tranches), `_merge_flight_table` (un secteur replanifié par la boucle globale sur une seule fréquence imposée garde la table de sa recherche, l'entrée du plan imposé remplaçant l'entrée la plus proche en fréquence) ; `compute` (mémoire, coût de resélection) ; 2 tests. Essais : `mission_example` (spectre calme : 3 centres évalués, approximation A6) 3 entrées par secteur, 0,8 à 2,9 ko ; `mission_environment` (spectre encombré) 8 entrées sur 18 à 19 centres, 4,5 à 6,1 ko par secteur ; le secteur E_EARTH a 5 entrées dégradées (2,6 à 5,5 % de positions en deçà de la portée, une à 15,5 % non balayé au-delà de 2,85 GHz). Mémoire négligeable (≈ 10⁻⁸ To par face). **Écart à signaler** : en spectre calme, la table n'a que 3 entrées (les centres évalués par l'approximation A6) ; un repli plus riche demanderait d'évaluer plus de centres en Pmiss.
- **P6, verdict et clôture** (26 octobre 2026, formule validée par l'utilisateur) : la charge d'énergie d'une face dans le verdict est **besoin / allocation** de la P1 (`optimizer._energy_verdict`, après la répartition dans l'assemblage) ; ressource limitante, statut et marge restante recalculés, `alloc_MW` ajouté ; conseil de relâchement quand l'énergie dépasse 1 ; l'alimentation de la face et le plafond opérateur restent contrôlés à part. Modèle V6.23.0.
- **N3 (26 octobre 2026, 23,0 min)** : 220 tests passés ; seuls les tests de snapshots et `test_tracking_loads_add_up` (qui les lit) ont échoué avant le regel. 311 valeurs changent : nouvelles sorties `power_allocation`, `global_opt` (une passe sur chaque mission de test : aucun couplage au-dessus de son seuil, aucune alternative meilleure pour la mission) et `flight_table` ; coût de resélection compté sur les tables (par exemple 48 000 → 96 000 opérations) ; charge d'énergie des faces ≈ −0,02 (le besoin prend la consommation réelle du calcul, ≈ 10 kW au lieu du plafond de 50 kW ; avec le plafond opérateur de 2 MW des missions de test, charge = besoin / 2 MW ; la mission sans plafond a la même charge sur les 4 faces, besoin total / 8 MW). Aucun jeu de formes d'onde, fréquence, mode de confirmation, portée, statut de face ni ressource limitante ne change. Regelés (`JOURNAL.md`). Le réemploi de la marge, désactivé par défaut, ne produit aucune sortie sur les missions de test.
- **P7, réallocation et saturation** (26 octobre 2026, après la clôture de la 5.2 ; règle de l'utilisateur, synthèse au §7.15 point 10 ; diagramme de la logique d'allocation fait le même jour ; codé en V6.24.0). Point de départ (utilisateur) : la marge va à la veille par défaut ; en saturation de poursuite, si la ressource qui limite est partagée entre S et Ku, les pistes au-delà de la spécification sont prises en reprenant la marge de la veille sans la faire tomber sous sa spécification (exemple : X = 80 %, spécification à 50 %, 30 points à la veille ; 105 pistes pour 100 spécifiées : les 5 en plus reprennent une partie des 30 points).

| Question | Décision de l'utilisateur |
|---|---|
| Q1 portée, nombre de pistes en plus | a = ressource disponible / coût d'une piste ; au pire, la veille retombe juste sur sa spécification |
| Q2 paramètres des pistes en plus | ceux de leur tranche |
| Q3 où reprendre la marge | un seul g commun à tous les volumes |
| Q4 ordre de service | KKV > SHIP > DECOY > DEBRIS > autres, puis la plus proche |
| Q5 au-delà de la marge | « saturation non gérable » ; en deçà, « saturation gérable » |
| Q6 ressource propre à une bande | saturation non gérable pour l'instant (puis leviers Ku, Q16) ; modes hors spécification par bande plus tard |
| Q7 `auto_alloc_max` = 0 | aucune réallocation |
| Q8 sortie Pmiss | par tranche : seule, puis cumul |
| Q9 place | calcul en 5.2 P7, état dans le verdict en 5.3, règle de vol en phase 6 |
| Défaut KKV > veille | constaté (pistes en plus sous la veille) ; remplacé par la règle de l'utilisateur à deux modes `PRIO_SEARCH` / `PRIO_TRACK` (§7.15 point 10), formules t et c validées |
| Ordre des classes | pistes allongées par classe, en `PRIO_SEARCH` comme en `PRIO_TRACK` quand la veille ne peut plus rien rendre |
| Ordre de la veille | dégradée dans l'ordre P4_n ; P0 et P1 intouchables (un volume non déclaré P1 accepte d'être dégradé) |
| Q10 `PRIO_TRACK` nominal | volumes à leur RR spécifiée, seule la cadence de poursuite change |
| Q11 réallocation interdite | la saturation gérable utilise la ressource libre ; la saturation non gérable n'est pas une réallocation permise |
| Q12 allongement des pistes | ordre strict des classes (t commun et pondération écartés) |
| Q14 part au-delà du plafond | réserve de sécurité jamais consommée → `load_ceiling` séparé de `realloc_mode` |
| Q16 temps Ku limitant | leviers : retirer P3′, confirmations Ku vers S2, puis pistes par classe |
| V1 `load_ceiling` par défaut | 0,80 (D), à recaler en 5.4 |
| C1 plancher des pistes | t_max par classe à p_loss_max = 1 % par mise à jour (D), borné par `latency_max_s` |
| C2 plancher de la veille | `rr_max_s` si donnée, sinon dérivée de la portée cumulée |
| C3 ordre d'abandon | `PRIO_TRACK` : volumes, pistes, P2 ; `PRIO_SEARCH` : pistes, volumes, P2 |
| C4 P2 inatteignable | alerte majeure, P0 et P1 seulement, plus de création de piste |
| C5 sortie Pmiss | ajouter l'entrée en saturation critique et le premier abandon |
| `NONE` en saturation | comme `PRIO_SEARCH` sans réemploi de la marge |
| V2 à V7 (cadrage de la P7) | `rr_min_s` par tranche ; `auto_alloc_max` retiré (refus avec message) ; alerte entre `load_ceiling` et 1, défaut au-delà de 1 ; sonde de max(1, 10 % n) pistes ; budget de temps +30 % ; pistes en plus réparties entre faces comme leur tranche |
| Planchers du réemploi | **1 s pour la veille, 0,1 s pour la poursuite** par défaut (la cadence de poursuite montait à 1 ms à l'essai) |
| Q17, Q18 | valeurs « tranche seule » en extrapolation linéaire signalée, seul le cumul est vérifié ; cumul = même fraction de n pour toutes les tranches |
| D1 à D5 | méthode des planchers, de la saturation critique (approximation en 1/T), des leviers Ku et du premier abandon validée |
| Temps de calcul | dépassement accepté (mission avec poursuite : +140 % en `NONE`) |

  Codage en 5 passes (P7a réglages, P7b `PRIO_TRACK`, P7c table de saturation gérable, P7d saturation critique, P7e rapport et documents), modèle **V6.24.0** : `core/margin.py` (`realloc_mode`, `load_ceiling`, `ceiling_flags`, remplissage commun `_solve` aux volumes et aux tranches, `_back_off`, `reuse_tracks`, `saturation`, `_cumulative`, `track_floors`, `search_floors`, `_ku_levers`, `critique`), `core/allocation.py` (part accélérée d'une tranche en P5), `core/mission.py` (contrôles), `core/tracking.py` (charges par mode de confirmation), interface provisoire (« Reallocation », « Load ceiling », colonne « RR min » des tranches), entrée `tracking.p_loss_max` (D). 13 tests nouveaux ou adaptés.
  - Essais (`mission_synthetic_mix`, valeurs de mission de test, pas une performance) : `PRIO_TRACK` sans plancher montait la cadence à 1 ms (faisceaux Ku à 656 % avant la bissection) ; avec le plancher, KV/0 et KV/1 à 0,1 s, 2 assemblages (+30 %). Table : t_max 3,37 s (KV/0) et 2,48 s (KV/1) ; pistes en plus gérables 5 403 et 10 807 seules (faisceaux Ku de F1), cumul p = 831 vérifié ; entrée en saturation critique +18 229 et +13 401 seules, cumul p = 2 008 vérifié ; premier abandon : pistes de KV/1 en `NONE`, volume SHIPW en `PRIO_TRACK` ; revisites maximales dérivées au-delà de la sonde (× 16).
  - **N3 (26 octobre 2026, 34,6 min)** : 232 tests passés ; seuls les 9 tests de snapshots ont échoué avant le regel. 121 valeurs changent, toutes des ajouts : `alternatives[mode].faces` du budget Ku, clé `saturation` sur les trois missions avec poursuite, une alerte nouvelle sur `mission_sky_demo` (cumul de la saturation critique ramené par bissection, énergie de F2) ; aucun jeu de formes d'onde, fréquence, mode de confirmation, portée, statut, charge ni ressource limitante ne change. Regelés (`JOURNAL.md`).
  - **Écarts à signaler** : extrapolation linéaire des valeurs « tranche seule » ; approximation en 1/T du coût d'une piste allongée (optimiste) ; leviers Ku non appliqués dans l'assemblage de vérification ; la mémoïsation des budgets Ku par secteur, essayée pour le temps de calcul, n'apportait rien et a été retirée.

### Étape 5.3 : moteur de validation (ouverte le 27 octobre 2026)

**Portée** (question de l'utilisateur) : la 5.3 porte sur la **Pmiss**, pas sur le temps réel. Elle juge le plan calculé hors ligne sur l'instantané de navigation de la mission (« dans cet état, le radar tient-il la mission, sinon pourquoi et que relâcher ? »). La règle de saturation en vol, la réallocation dynamique et le délestage restent en phase 6 ; le verdict ne reprend que l'**état** de saturation calculé par la 5.2 (P7).

**Pas d'enveloppe d'attitudes en Pmiss** (décision de l'utilisateur, 27 octobre 2026) : la Pmiss calcule sur **un seul** message de navigation, au modèle complet. L'outil temps réel (phase 6) reprendra une version simplifiée et recalée du modèle de la Pmiss, moins précise mais robuste, jouée en direct sur les évolutions prévues par le scénario (attitude, objets). La passe P5 « enveloppe » proposée est retirée ; les renvois de l'enveloppe (3.0 Q2, migration sur l'enveloppe, navigation tirée de l'enveloppe du scénario) passent en phase 6.

**Cadrage validé** (« ok pour les suggestions », 27 octobre 2026) :

| Question | Décision |
|---|---|
| Q1 états | trois états par contrainte : OK, ALERTE, NOK ; pour une ressource, OK jusqu'à `load_ceiling`, ALERTE jusqu'à 1 (réserve de sécurité entamée), NOK au-delà |
| Q2 `track_margin_warn` | retiré (remplacé par `load_ceiling` et la répartition par niveaux) |
| Q3 verdict global | NOK dès qu'une contrainte est NOK (la spécification de l'opérateur n'est pas tenue) ; chaque contrainte porte son niveau de priorité, les niveaux en défaut sont listés ; « P2 inatteignable » reste l'alerte majeure de la P7 |
| Q4 portée courte | ALERTE dans la tolérance `short_tol` du secteur, NOK au-delà, avec la fraction de positions courtes |
| Q5 conseil chiffré | (c) estimation analytique par défaut, calcul vérifié (bissection) sur demande ; c'est le résultat vérifié qui fait foi |
| Q6 leviers | portée, RR, SER de dimensionnement, pistes par tranche, précisions de poursuite, plafonds de puissance des faces, mode de confirmation |
| Q7 redondances | « le plan de A tient le besoin de B » (`core/redundancy.py`) ; recouvrement entre faces en information chiffrée (charge reprenable par la face voisine), sans changer le plan |
| Q8 sensibilité | ~~SER : deux chiffres (plan figé, plan réoptimisé) ; vitesse et Swerling : plan figé seulement~~ **retirée** avec la P4 (voir plus bas) |
| Q10 présentation | clé JSON `verdict`, rapport texte de `optimizer`, tableau dans l'onglet Résultats (interface définitive en 5.5) |

**Découpage** : P1 table des contraintes et verdict ; P2 conseil chiffré ; P3 redondances et recouvrements ; ~~P4 sensibilité à la menace~~ (retirée).

**P4 sensibilité à la menace retirée** (décision de l'utilisateur, 27 octobre 2026) : le moteur valide la mission **avec les valeurs qu'on lui donne** (celles du secteur si l'opérateur les a saisies, sinon celles de la classe choisie) ; il ne cherche pas de combien la menace peut s'écarter de ces valeurs. Pour une autre hypothèse, l'opérateur édite la classe ou surcharge le secteur et relance le calcul. La décision Q8 et le renvoi de l'étape 2.4 sont fermés ; à revisiter plus tard si le temps le permet.

- **P1, verdict** (codée le 27 octobre 2026, modèle **V6.25.0**, `core/validation.py`, clé `verdict`, appelée à la fin de `optimizer.run`) : toute contrainte est une **charge = besoin / limite**, de l'un de trois genres :
  - **ressource** (énergie besoin / allocation de la P6, temps S et Ku, chaleur, calcul, mémoire, débit mémoire, faisceaux S et Ku, lien TRSA, entrée du processeur, puissance du calcul, stockage par observation ; énergie du vaisseau besoin / 8 MW, rejet de chaleur du vaisseau) : OK jusqu'à `load_ceiling`, ALERTE jusqu'à 1, NOK au-delà ;
  - **exigence** (plan trouvé, portée, revisite dans la mission, mode de confirmation admissible, P de confirmation (charge = (1 − P) / (1 − `p_min`)), latence de confirmation, période et portée de chaque tranche de poursuite, discrimination, niveau de la répartition des 8 MW servi en entier, ordonnancement faisable, délestage) : OK jusqu'à 1, NOK au-delà ; ALERTE pour une portée courte dans `short_tol` (charge = fraction courte / tolérance) et une discrimination atteinte sous la plus courte portée d'intérêt ;
  - **transitoire** (migration entre faces quand le vaisseau tourne, erreur de pointage de navigation, accélération au-delà de celle du dimensionnement des pertes) : OK jusqu'à 1, ALERTE au-delà.
  Le stockage et la chaleur d'une rafale de revisite tassée sont des **règles de l'ordonnanceur** (il étale les émissions, étapes 4.1 et 4.2) : état OK avec la note ; la frise synthétique de la 4.3 passe en ALERTE s'il reste des pauses de recharge ou des mises à jour en retard. Les harmoniques S dans le canal Ku restent des règles de l'ordonnanceur (alertes texte). Sorties : état de la mission (NOK, ALERTE, OK ; INCOMPLETE si le calcul est annulé), décompte, **ressource limitante** de la mission (la plus forte charge de ressource) et de chaque face, niveaux de priorité en défaut, **saturation** (état à la spécification : nominal, réserve entamée, sur-souscrit ou P2 inatteignable ; mode ; par tranche, pistes en plus gérables seule et en cumul, seuil critique, t_max ; premier abandon), et la table complète.
  - Essais sur les neuf missions de test (valeurs figées V6.24.0, valeurs de mission de test, pas une performance) : 88 à 136 contraintes par mission ; `mission_sky_demo` **NOK** (EARTH : aucun mode de confirmation admissible, P2 ; portée courte 0,7 % en ALERTE) ; `mission_environment` **NOK** (E_EARTH : 1,3 % de positions courtes pour une tolérance de 1 %) ; `mission_veille500` ALERTE (VEILLE_KV : 0,3 % de positions courtes) ; les six autres OK. Ressource limitante : faisceaux Ku de F1 à 21,9 % sur la plupart des missions (pic entier de faisceaux), énergie de F4 à 24,8 % sur `mission_frames`, débit mémoire de F1 à 51,5 % sur `mission_environment`.
  - Passe intermédiaire : N2 seulement, valeurs figées non regelées (règle du §1.3 : une passe sur deux) ; le test qui compare le verdict figé au verdict recalculé ne compare qu'une fois la clé figée. Commit `37bcd0c`.
- **P2, conseil chiffré** (cadrage validé le 27 octobre 2026 : « suggestions validées », Q1 à Q8 ; codée le 27 octobre 2026, modèle **V6.26.0**, clé `verdict.advice`, `validation.verify`) :
  - **cible** : `load_ceiling` pour une ressource (état OK), la valeur à 1 en second chiffre ; 1 pour une exigence ; conseil sur les NOK **et** les ALERTE ;
  - **ressource linéaire** (énergie, temps S, temps Ku) d'une face au-dessus de sa cible de ΔL : un élément (veille, confirmation, tranche de poursuite, percée) de part s > ΔL garde la fraction f = 1 − ΔL/s de sa charge ; lois : **veille** charge'/charge = (R'/R)⁴ · SNR_req(q')/SNR_req(q) · RR/RR', q = v_close_max · RR / R (**la portée d'un volume est la portée « au pire »**, règle rappelée par l'utilisateur le 27 octobre 2026 : un objet raté en début de cycle doit encore être détecté à R, les observations qui comptent sont espacées de v · RR au-delà de R ; pour 20 km/s et une RR de 20 s, un volume à 500 km doit assurer la détection entre 500 et 900 km ; c'est le critère cumulé de l'étape 2.3, `detection.required_snr_db`, ici sur la forme d'onde principale du volume), R' et RR' résolus par bissection, pas de levier de revisite quand une revisite plus longue ne soulage pas (RR du minimum en note) ; la loi simple 1/RR, validée d'abord (Q3), a été **réfutée par la vérification** (voir les essais) et remplacée avec l'accord de l'utilisateur ; SER' = SER / f (hypothèse de menace, q inchangé) ; poursuite n' = n f, T' = T / f, précision du facteur limitant (ANG : zone, RNG, DOP) / √f ; temps d'émission pris limité en énergie (mêmes lois) ; valeurs arrondies à 3 chiffres du côté qui soulage ; les **3 plus gros éléments seuls**, plus un **relâchement uniforme** (même facteur sur les revisites et périodes de toute la face) ; alternatives de confirmation (exactes, étape 3.2) ; plafond opérateur `face_elec_mw` d'une face plafonnée (besoin / plafond de charge, dans la limite de l'alimentation) ; percée retirée ;
  - **énergie** : quand les 8 MW suffisent, l'allocation est au prorata des besoins, la charge de chaque face est celle du vaisseau (besoin / 8 MW) : le conseil est donné **une fois, sur l'énergie du vaisseau** (part d'un élément = son tirage total / 8 MW) ; une face plafonnée pèse son propre tirage sur son plafond ;
  - **plusieurs ressources d'une face en défaut** (Q8) : le conseil vise la plus forte et contrôle les autres ressources linéaires par la même loi (`others_left`) ;
  - **ressources en marches** (faisceaux, calcul, débit mémoire, stockage) et **chaleur** (Q6) : facteur proportionnel uniforme sur les revisites, signalé approché ; la part statique de la puissance du calcul est retirée (aucun levier si elle dépasse seule la cible) ; les **crêtes par observation** (lien TRSA, entrée du processeur, tampons mémoire) dépendent des formes d'onde (bande, faisceaux par observation), pas des revisites : pas de levier de revisite, note seulement (constaté à l'essai) ;
  - **aucun élément ne suffit** (par exemple une charge faite surtout de consommation permanente) : une entrée « pas de levier » donne la part variable et la part fixe (constaté à l'essai) ;
  - **exigence en défaut** (Q5) : portée courte → `short_tol` porté à la fraction courte (exact), et portée abaissée à celle des tranches d'angle entières courtes (si une tranche l'est ; sinon les positions courtes sont des directions isolées, ciel ou fouillis, sans levier de portée) ; P de confirmation → `confirm_p_min` à la valeur atteinte ; latence → `latency_max_s` à la latence atteinte ; période de poursuite → la plus longue tenable ; portée de poursuite → pistes dans la portée Ku ; niveau de puissance servi en partie → revisite ou pistes tenables de la répartition ;
  - **vérification** (Q7) : `verify()` recalcule la mission avec les changements d'un conseil ; si l'estimation tient, bissection géométrique entre les valeurs actuelles et l'estimation (`tool.validation.verify_bisections` = 6 recalculs, D) pour le plus petit relâchement qui tient ; sinon le changement est doublé au plus deux fois d'abord ; tient = la contrainte du conseil à sa cible ou dessous (absente compte pour tenue) ; c'est le chiffre vérifié qui fait foi ; `python -m core.validation <mission> [--verify n] [rapport.json]` et bouton « Verify the selected advice » de l'onglet Résultats (fil d'exécution en arrière-plan) ;
- **États renommés** : « KO » devient **« NOK »** partout (code, interface, documents ; demande de l'utilisateur, 27 octobre 2026).
  - **Vérification : un relâchement ne tient que s'il n'ajoute aucune autre contrainte NOK** (constaté à l'essai : une revisite très longue faisait tomber le plan du secteur, ce qui baissait l'énergie) ; après doublement, la bissection repart du dernier pas en échec.
  - Essais de bout en bout (`mission_veille500`, `load_ceiling` abaissé à 0,2 pour créer des alertes, valeurs de mission de test, pas une performance ; un recalcul de mission ≈ 28 s sur le portable) : énergie du vaisseau 31,2 % pour une cible de 20 %. **Levier RR de VEILLE_KV** (25 km/s, RR 20 s, 500 km : q = 1) estimé par 1/RR à 33,8 s : **ne tient pas**, l'énergie monte à 58,9 % (33,8 s), 142,7 % (57 s) ; la loi corrigée ne propose plus de levier de revisite (minimum de charge vers 21 s). **Levier portée** : 438 km par R⁴ (optimiste), **403 km vérifié** (8 recalculs, 185 s), 381 km par la loi corrigée (prudente).
  - Passe paire : table de coût régénérée (paramètre ajouté), N3, rapport d'impact et regel des valeurs figées (clé `verdict` ajoutée sur chaque mission) : **N3 34,8 min, 238 tests passés**, seuls les 9 tests de valeurs figées échouaient avant le regel ; 9 valeurs changent, une par mission (la clé ajoutée), aucune autre ; `model.json` gardé (bruit de dernier chiffre du portable) ; journal `tests/snapshots/JOURNAL.md`. Le test du verdict figé compare les contraintes par identifiant et les conseils sans leur numéro (les valeurs figées gardent leurs clés triées, l'ordre des lignes n'est que de la présentation).
- **P3, redondances et recouvrements** (cadrage validé le 27 octobre 2026 : « suggestions validées », V1 à V8 ; codée le 27 octobre 2026, modèle **V6.27.0**, information seulement, le plan ne change pas) :
  - **V1 plans** : le test « le plan de A tient le besoin de B » (`core/redundancy.py`) se fait **à la fin du calcul, avec les plans retenus** (fréquence, formes d'onde par tranche de dépointage, échelle d'impulsion, Pfa, après l'optimiseur global) ; un secteur sans plan est planifié seul, à la puissance de référence. La règle de la portée au pire s'applique (revisite de A, cible et portée de B, critère cumulé). Sortie `redundancy` du calcul, reprise dans `verdict.redundancy` avec le coût du secteur redondant par face.
  - **V2 seuils** : `cover_full` 0,995 et `cover_min` 0,01 gardés (`[tool.redundancy]`).
  - **V3 conseil** : une ressource de face (énergie, temps S) en ALERTE ou NOK reçoit d'abord le retrait (redondance totale) ou la découpe (morceaux en boîtes) d'un secteur redondant qui pèse sur elle (genre `redundancy`, « spécification gardée » ; gain = part du secteur dans la charge × part de son besoin tenue, estimation pour une découpe) ; `verify()` applique le retrait ou les morceaux et recalcule.
  - **V4 coût sur la voisine** (`optimizer.face_overlap`) : coût d'une position du plan du secteur à l'angle de dépointage vu depuis la voisine (exact là où le plan a cette tranche ; au-delà, forme d'onde de la tranche la plus proche, portée mise à l'échelle par `range_at()` entre les deux angles, échelle d'impulsion (R / r)⁴ et `PositionCosts`, inatteignable au-delà de l'impulsion pleine), densité de positions de la voisine (w dΩ / cellule) ; l'aveuglement par le ciel est laissé de côté.
  - **V5, V6** : veille S seulement ; limite de dépointage de la bande à la fréquence du plan du secteur.
  - **V7** : `verdict.overlap` par face et voisine : angle solide de la zone (et la part atteignable), temps S et énergie de la face dans la zone et sur la voisine, marge de la voisine jusqu'à `load_ceiling` (temps S ; énergie de la face si elle est plafonnée, sinon du vaisseau), part reprenable et gain ; une ligne de conseil (genre `overlap`, sans changement : les zones de responsabilité restent celles du plan, la réallocation en vol est en phase 6).
  - **V8** : clés `verdict.redundancy` et `verdict.overlap`, lignes du rapport texte, tableau « redondances et recouvrements » de l'onglet Résultats.
  - Essais (valeurs de mission de test, pas une performance) : `mission_veille500` (500 km, 25 km/s), chaque face voit 0,97 sr de la zone de sa voisine, 0,90 sr atteignable : le temps S y passe de 1,06 % à 2,93 % sur la voisine (dépointage plus fort) et l'énergie de 0,12 à 0,36 MW ; reprise possible en temps, mais l'énergie du vaisseau monte de 3 %. `mission_redundancy_demo` : A1_TRIM entièrement tenu par B1_TRIM, B1_TRIM tenu à 50 % par A1_TRIM (cas mutuel, l'un ou l'autre suffit), B5_LONG entièrement tenu par A5_SHORT. Temps de calcul : 70 s et 41 s sur le portable. **N2 (28,9 min)** : 237 tests passés ; seul le test du verdict figé échouait sur les deux clés ajoutées (passe intermédiaire, valeurs figées non regelées) : il compare désormais les clés figées, les nouvelles le seront au regel de la P4.
- **Fermeture de l'étape** (27 octobre 2026) : la P4 étant retirée, le regel est fait après la P3. Table de coût régénérée (empreinte seulement), **N3 34,3 min sur le bureau, 241 tests passés**, seuls les 9 tests de valeurs figées échouaient ; 66 valeurs changent, toutes des ajouts (`redundancy`, `face_overlap` par secteur, `verdict.redundancy`, `verdict.overlap`) ; le reste ne diffère que par le bruit de dernier chiffre entre le portable et le bureau (au plus 5,7·10⁻¹¹ en relatif) ; journal `tests/snapshots/JOURNAL.md`. **Étape 5.3 fermée.**

### Étape 5.4 : simulation Monte-Carlo à événements discrets (cadrage validé le 27 octobre 2026)

**Raison d'être** : le modèle de la Pmiss est analytique (moyennes, quantiles, entrelacement parfait). La 5.4 **joue la mission par tirages** pour le plan calculé et l'instantané de navigation de la mission, et dit où le modèle analytique est optimiste. C'est l'arbitre final du bilan statistique ; le moteur à événements sert ensuite de base au simulateur de la phase 6.

**Sorties du périmètre** (utilisateur, 27 octobre 2026) : **scène dense** (masquage par les lobes en distance et en Doppler et par le bruit de phase, CFAR CA, OS, GO pollués par les objets voisins) → **phase 6**, car l'effet ne compte que dans les grappes du scénario ; **réflexions sur la structure** (Soleil, Terre, émission renvoyée) → **sorties du modèle**, développement futur potentiel (calcul déterministe, pas un tirage).

**Cadrage validé** (« appliquer les propositions », 27 octobre 2026) :

| Question | Décision |
|---|---|
| A1 quantités | Pd cumulée au pire par secteur ; fausses alarmes par revisite et par seconde contre `fa_per_rr` ; P et latence de confirmation contre `confirm_p_min` et `latency_max_s` ; initialisation des pistes et perte par mise à jour contre `p_loss_max` ; ordonnanceur : latences au quantile 95 %, mises à jour en retard, pertes d'entrelacement |
| A2 déclenchement | sur demande seulement (`python -m core.montecarlo <mission>`, bouton de l'onglet Résultats), jamais dans le calcul normal |
| A3 verdict | exigence non tenue (borne haute de l'intervalle sous l'exigence) : NOK « selon Monte-Carlo » ; exigence tenue mais valeur analytique hors de l'intervalle du côté optimiste : ALERTE « modèle optimiste » ; le plan ne change jamais |
| A4 `load_ceiling` | charge maximale qui tient latences et mises à jour, **proposée** sans être appliquée (D, change seulement avec l'accord de l'utilisateur) |
| A5 faces | une chronologie par face et par bande ; énergie des 8 MW à la répartition moyenne de la 5.2 |
| B1 moteur | `core/montecarlo.py`, file d'événements datés, classes de tâches et règles de la chronologie 4.3, ordre de balayage 4.3 bis ; réutilisé en phase 6 |
| B2 durée d'un tirage | traversée de la fenêtre de la portée au pire par la cible test + préchauffe de 2 revisites ; `horizon_max_s` 600 s **pour la chronologie de l'ordonnanceur seulement** (révisé le 27 octobre 2026 à l'essai de M1 : les cibles test comptent toutes les visées au-dessus de `pd_look_floor`) |
| B3 SNR d'une visée | lu dans le plan (SNR(R)·(R/r)⁴ et pertes de la position), pas de recalcul RF |
| B4 pertes de position | tirées : position dans le faisceau, dans la cellule de distance et dans le filtre Doppler |
| B5 fluctuation | Swerling tiré (SW1/3 constant sur une visée, SW2/4 d'impulsion à impulsion) ; entre visées, même valeur si Δt < T₀, nouvelle sinon ; tirages d'agilité du modèle |
| B6 détection | statistique de test tirée exactement (loi du χ²) contre le seuil de la Pfa du plan (vérifie l'ajustement de Shnidman) ; CFAR en bruit homogène |
| B7 fausses alarmes | nombre par visée tiré (loi binomiale, cellules réelles de l'étape 2.2) ; chacune déclenche une confirmation P2 qui consomme du temps et échoue avec la Pfa de confirmation |
| B8 éclipse | écho tombant dans un créneau d'émission commun perdu (tiré) |
| B9 pauses | règles de stockage (4.1) et de plaque froide (4.2) de la chronologie 4.3 |
| C1 populations | (i) cibles test, une par tirage et par secteur (Pd cumulée, confirmation) ; (ii) charge de fond au dimensionnement : n pistes par tranche entretenues + fausses alarmes |
| C2 direction | uniforme en angle solide dans le volume, positions masquées exclues |
| C3 entrée | à R + v·RR, phase de balayage uniforme, trajectoire radiale à vitesse constante |
| C4 vitesse | `v_close_max` ; option `v_draw` = "uniform" entre `v_close_min` et `v_close_max` |
| C5 manœuvre | aucune en veille ; Singer en poursuite |
| C6 valeurs | celles du secteur, sinon de la classe (règle de la 5.3) ; SER frontale, aspect dans le temps en phase 6 |
| D1 chaîne | détection → confirmation (mode du plan) → initialisation (critère 3.3) → mises à jour à la période de la tranche → perte selon la règle de ratés de la 3.3 |
| D2 filtre | (a) succès ou raté par mise à jour, erreur de pointage tirée de la covariance stationnaire de Singer ; Kalman complet et précisions en phase 6 |
| D3 association | parfaite |
| D4 SNR de mise à jour | plan Ku à la portée de la piste, piste placée uniformément dans la portée de sa tranche |
| D5 discrimination | hors du Monte-Carlo (phase 6) |
| G1 graine | `seed` 1, écrite dans la sortie ; même graine, même résultat |
| G2 tirages | 200 à 5 000 cibles test par secteur, arrêt à une demi-largeur de 0,01 à 95 % (`n_min` 200, `n_max` 5000, `ci_tol` 0,01) |
| G3 intervalles | Wilson à 95 % (`ci_level` 0,95) ; Poisson pour les nombres de fausses alarmes |
| G4 événements rares | Pfa vérifiée par le nombre de fausses alarmes sur toutes les cellules visitées |
| G5 temps de calcul | pas de budget, mesuré et rapporté ; parallèle par secteur et par face |
| H1 clé | `montecarlo` dans le résultat quand le calcul est lancé ; lignes ajoutées au verdict, source « MC » |
| H2 tableau | quantité, analytique, Monte-Carlo, intervalle, exigence, état |
| H3 présentation | rapport texte, tableau et bouton de l'onglet Résultats (interface définitive en 5.5) |
| H4 tests | reproductibilité, visée seule contre Shnidman, nombre de fausses alarmes contre Pfa, petite mission à graine fixe dans les valeurs figées |
| H5 version | modèle V6.28.0 |

Réglages dans `[tool.montecarlo]` (statut D) : `seed` 1, `n_min` 200, `n_max` 5000, `ci_tol` 0,01, `ci_level` 0,95, `horizon_max_s` 600, `warmup_revisits` 2, `v_draw` "max".

**Découpage** : M1 moteur et cibles test (B, C), Pd cumulée ; M2 fausses alarmes, confirmation, latences de l'ordonnanceur, éclipse tirée ; M3 poursuite (D) ; M4 verdict, proposition de `load_ceiling`, rapport et interface ; M5 impact, N3, regel, documents. N2 sur les passes intermédiaires, N3 et regel en fin d'étape, chaque test sur accord de l'utilisateur.

- **M1, cibles test de la veille** (codée le 27 octobre 2026, modèle **V6.28.0**, `core/montecarlo.py`, `python -m core.montecarlo <mission> [résultat.json]`) :
  - **plan** : `optimizer.run(keep_plans=True)` garde le plan de chaque secteur (`_plan`, objets Python, absent d'un calcul normal, dont les sorties ne changent pas), avec la portée à impulsion pleine et le nombre de positions de chaque tranche de 1° ;
  - **SNR d'une visée lu dans le plan** (B3) : SNR moyen avant les pertes de position à la portée d'exigence = SNR requis de la tranche (critère cumulé, loi des positions, même calcul que l'optimiseur) + 40 log₁₀(portée atteinte / R), portée atteinte = portée à impulsion pleine × `pulse_scale`^(1/4) (ciel, brouilleurs et éclipse moyenne compris, comme dans le plan) ; entre visées, loi en (R / r)⁴ (bruit) ; un secteur avec brouilleur d'autoprotection est signalé (la percée va en 1 / r²) ; une tranche limitée par une surface prend sa distance comme portée d'exigence ;
  - **cible test** (C1 à C6) : tranche tirée au prorata de ses positions, visées à R (1 + (k + φ) q), q = v · RR / R, φ uniforme, v = `v_close_max` (ou uniforme, `v_draw`) ;
  - **position tirée** (B4) : un point uniforme dans la cellule du réseau d'émission, **fixe pendant la vie de la cible**, vu à travers les décalages du réseau d'une trame à l'autre (étape 4.3 bis, cycle de `n_shifts`) : décalage d'émission = le point ramené dans la cellule décalée, décalage de réception = ce décalage ramené dans le réseau de réception du pointage ; repliement Doppler et écart au filtre de vitesse fixes par cible ; repliement en distance tiré à chaque visée ; une partition (sous-ouverture) tire sa perte d'émission à chaque visée (signalé). **Le modèle analytique, lui, moyenne chaque perte de position indépendamment d'une visée à l'autre : l'écart est ce que mesure M1** ;
  - **fluctuation** (B5) : puissance Gamma de la forme de la visée, même tirage entre visées si RR < T₀ ; **décision** (B6) : statistique tirée exactement (χ² non centré) contre le seuil de la Pfa de la tranche ;
  - **arrêt** (G1 à G3) : graine par secteur, lots de `batch` cibles, arrêt entre `n_min` et `n_max` quand la demi-largeur de Wilson passe sous `ci_tol` ; les visées dont la Pd moyenne reste sous `pd_look_floor` ne sont pas tirées, **toutes les autres comptent, sans borne de temps**, comme dans le critère analytique (décision de l'utilisateur, 27 octobre 2026 : à l'essai, `horizon_max_s` = 600 s ne laissait que 5 visées à une revisite de 120 s et coupait la fenêtre de la portée au pire des cibles lentes et lointaines, d'où un faux NOK ; `horizon_max_s` ne borne plus que la chronologie de l'ordonnanceur en M2) ; une borne de sécurité de 20 000 visées est signalée si elle coupe ;
  - **état par secteur** (A3, sur les positions dans la portée) : NOK si la borne haute de l'intervalle est sous la Pd demandée ; ALERTE si la Pd analytique dépasse la borne haute (modèle optimiste) ; OK sinon. La Pd sur toutes les positions (portée courte comprise) est donnée à part ;
  - **écart au cadrage B1, à signaler** : en M1 les visées d'une cible sont périodiques (RR exacte, pas d'ordonnanceur), elles sont tirées en bloc (vectorisé) ; la file d'événements datés arrive avec la M2, quand l'ordonnanceur rend les instants de visée aléatoires ;
  - tests : réduction dans la cellule hexagonale, intervalle de Wilson, visée seule contre la loi exacte (SW0, SW1), reproductibilité et accord avec le critère analytique quand les pertes sont tirées indépendamment.
  - Essai (`mission_veille500`, valeurs de mission de test, pas une performance ; 46 s dont 0,1 à 0,4 s de Monte-Carlo par secteur) : pertes tirées indépendamment (hypothèse analytique), le Monte-Carlo retrouve 0,9 (CONE_D1 0,898 [0,888 ; 0,908], VEILLE_KV 0,902) ; position fixe par cible : **CONE_D1 à D4 0,922 à 0,929** (8 000 km, RR 120 s, 5 km/s, q = 0,075 : le modèle analytique est **prudent**, la cible repasse près du centre d'un faisceau au fil des décalages du réseau), **VEILLE_KV 0,907** [0,897 ; 0,916] (q = 1 : pas d'écart) ; tous OK.
  - **N2 (12,3 min)** : 240 tests passés ; deux échecs corrigés puis repassés seuls : table de coût périmée (régénérée, empreinte seulement) et paramètres `tool.montecarlo` lus par aucun module importé par le test (le test importe désormais `core.montecarlo` ; `warmup_revisits` lu dès M1, utilisé en M2).
