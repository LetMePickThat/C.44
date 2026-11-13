# Migration vers une architecture logicielle rapide (calcul exact en temps réel)

Nov 12, 2026 · @Jean Valjean

## Contexte et objectif

Objectif : faire tourner le calcul exact de l'outil (optimiseur, règle d'allocation, relief, verdict) en temps réel, soit un plan en ≤ 1 s sur événement et 60 plans par seconde de manœuvre, contre 56 à 540 s par instant aujourd'hui. Le gain visé est d'environ ×10⁴, sans changer un seul résultat.

Le système réel devra planifier ses ressources avec le modèle exact, pas avec l'estimation rapide : sur `MOONTEST_VELOCITY`, l'estimation dit OK là où le calcul exact dit NOK (bande passante mémoire à 119–149 %). Le même code doit servir à la préparation de mission au sol et à bord.

**Dans le périmètre :** le calcul exact d'un instant (`core/optimizer.py`, `core/allocation.py`, `core/redundancy.py`, `core/margin.py`, `core/validation.py`, `core/tracking.py`, `core/radar_model.py` et leurs dépendances) et sa reprise sur une suite d'instants (`core/time_profile.py`).

**Hors périmètre :** le traitement du signal embarqué (formation de faisceaux, bancs, CFAR, environ 5·10¹⁸ op/s par face), qui relève du matériel de 2050 ; le Monte-Carlo de vérification, qui reste hors de la boucle temps réel ; l'interface graphique et le simulateur de scénario, déjà largement compilés.

## État actuel

Le calcul exact d'un instant prend 56 à 540 s sur 8 processus (cas de non-régression N3), soit 1,5 à 9 min selon `performance-risks.md`. Aucun profil détaillé n'existe : on ne sait pas où part ce temps.

| Élément | Aujourd'hui | Conséquence pour la vitesse |
| --- | --- | --- |
| Langage du calcul exact | Python pur pour l'optimiseur, l'allocation, la poursuite, le relief ; numpy pour les rasters | boucles et appels de fonction interprétés, coût par appel de l'ordre de la microseconde |
| Code compilé | numba seulement dans `core/sky.py`, `sim/fast.py`, `sim/propagate.py` et un noyau de `core/radar_model.py` | l'essentiel du calcul exact n'en profite pas |
| Parallélisme | `ProcessPoolExecutor` dans `core/optimizer.py`, moitié des cœurs par défaut | copie des données entre processus, un cache par processus |
| Caches | mémoïsation de `solve()` et `solve_base()` par processus (≈ 50 Mo), masques, ciel, `antenna_cache.json`, `cost_table.json`, `.calibration_cache.json` | perdus à chaque lancement d'outil, rechauffés à chaque calcul |
| Données de mission | dictionnaires Python (`sec: dict`) passés partout | accès par clé lents, pas de disposition mémoire contiguë |
| Enchaînement | tout l'instant est recalculé, jusqu'à 4 re-planifications par volume dégradé (sécante) | pas de reprise du plan précédent, part séquentielle incompressible |
| Référence | valeurs figées à rel 1e-9 (`tests/snapshots/`, 3 606 valeurs dans le dernier gel), `JOURNAL.md` | base de non-régression déjà en place |

Points chauds supposés, à confirmer par le profilage : l'évaluation des candidats de l'optimiseur (formes d'onde × fréquences × mailles × secteurs de dépointage), les rasters de volumes et de ciel, et les re-planifications de la règle d'allocation.

## Exigences cibles

L'exigence dure est le débit en manœuvre : 60 plans exacts par seconde quand le vaisseau tourne à 30°/s, puisque les volumes sont reconstruits tous les 0,5°. La latence, elle, est relâchée par la planification anticipée sur l'attitude commandée.

| Exigence | Valeur proposée | Origine |
| --- | --- | --- |
| Plan sur événement (nouvelle cible, brouillage, panne, écart de navigation) | ≤ 1 s | proposition, à valider |
| Débit en manœuvre | 60 plans/s à 30°/s | `repointing.rate_max_dps` 30°/s, seuil de reconstruction 0,5° (passe G1) |
| Avance de la planification anticipée | ≥ 1 s sur l'attitude commandée | proposition |
| Profil de mission (`core/time_profile.py`) | tout le plan de vol en calcul exact en < 1 min | proposition ; aujourd'hui seuls 8 instants au plus passent en exact |
| Fidélité | résultats identiques à rel 1e-9 aux valeurs figées, sur toutes les missions de test | règle actuelle des snapshots |
| Déterminisme | même entrée, même plan, quel que soit le nombre de cœurs | requis pour la certification et pour comparer sol et bord |
| Source des valeurs | `radar_system.toml` reste l'unique source | règle du fichier de paramètres global |
| Machine de référence | serveur bi-processeur actuel, 64–128 cœurs, GPU facultatif | pour démontrer la faisabilité aujourd'hui ; la cible embarquée de 2050 est à part |

Les valeurs « proposition » sont à trancher par vous (règle « ask for every variable »).

## Architecture logicielle cible

Le calcul exact devient un service permanent, appelé par tous les outils, qui s'appuie sur un noyau compilé et des caches gardés en mémoire. Le code Python actuel reste la référence de non-régression.

&#91;embedded content: architecture logicielle cible · service, noyau, référence\]

L'état d'entrée passe par le recalcul incrémental et la planification anticipée avant d'atteindre le pool de threads, qui appelle le noyau ; `radar_system.toml` alimente tout le service.

- **Noyau compilé** : `solve_base`, `solve_look`, `detection.required_snr_db`, pertes de position, rasters de volumes et de ciel. Entrées en tableaux contigus, sans dictionnaire.
- **Pool de threads** : un seul processus, mémoire et caches partagés, au lieu de `ProcessPoolExecutor` et d'un cache par processus.
- **Recalcul incrémental** : chaque volume porte la clé de ses entrées (géométrie, ciel, cible, ressources) ; seuls les volumes dont la clé change sont replanifiés, la règle d'allocation repart du plan précédent.
- **Planification anticipée** : une file de plans futurs calculés le long de l'attitude commandée (plan de vol, nœuds de manœuvre) ; un écart de navigation ou un événement déclenche un recalcul.
- **Interfaces** : la même API sert l'outil de préparation, le profil de mission et, plus tard, l'ordonnanceur de bord.

## Leviers de gain et estimation

Le gain combiné est estimé entre ×10³ et ×3·10⁴, soit un plan en 5 ms à 0,5 s au lieu de 56 à 540 s. Ces facteurs sont des ordres de grandeur génériques, non mesurés sur ce code : la phase 0 les remplace par des chiffres.

| Levier | Gain estimé | Condition | Limite |
| --- | --- | --- | --- |
| Noyau compilé (équation radar, pertes, SNR requis, rasters) | ×20–100 | boucles Python remplacées par du code natif, données en tableaux contigus | gain plus faible là où numpy vectorise déjà |
| Caches persistants en mémoire | ×1–3 | un service qui vit entre deux calculs, caches partagés entre threads | mémoire : ≈ 50 Mo par processus aujourd'hui, à borner |
| Recalcul incrémental | ×3–10 | ne refaire que les volumes dont l'entrée a changé ; repartir du plan précédent | un virage touche tous les volumes non liés au vaisseau |
| Parallélisme (8 → 64–128 cœurs, threads) | ×5–10 | candidats, fréquences et volumes évalués en parallèle, sans copie entre processus | parties séquentielles : allocation, jusqu'à 4 re-planifications |
| Planification anticipée | ne réduit pas le calcul, change l'exigence | plans calculés à l'avance sur l'attitude commandée | ne couvre pas l'imprévu (cible, panne) : reste le ≤ 1 s |

La formule de passage, avec t₀ le temps actuel et gᵢ les gains :

```latex
t_{\text{plan}} = \frac{t_0}{g_{\text{compilé}} \, g_{\text{caches}} \, g_{\text{incrémental}} \, g_{\text{parallèle}}}, \qquad t_0 = 56\text{ à }540\text{ s}
```

En manœuvre, 60 plans/s avec des plans de 50–500 ms demandent 3 à 30 planificateurs en parallèle, soit un ou quelques serveurs.

## Plan de migration par phases

Sept phases, chacune livrée par un commit et validée sur les valeurs figées avant la suivante. On ne compile que ce que la phase 0 désigne comme coûteux.

| Phase | Contenu | Livrable | Critère de sortie |
| --- | --- | --- | --- |
| 0. Profilage | profiler les cas N3 et `MOONTEST_VELOCITY` avec `tools/exact_compare.py` (compteur d'étapes) et un échantillonneur (py-spy ou cProfile) | répartition du temps par fonction ; nombre d'appels à `solve()` par volume ; part séquentielle | tableau des gains révisé avec des chiffres mesurés |
| 1. Modèle de données | remplacer les dictionnaires de mission et de volume par des structures typées et des tableaux contigus, sans changer les calculs | `core` lisible par un noyau compilé | valeurs figées identiques à rel 1e-9 |
| 2. Noyau compilé | `solve_base` / `solve_look`, `detection.required_snr_db`, pertes de position, rasters de volumes et de ciel | noyau natif + chemin Python gardé en référence | identique à rel 1e-9 ; gain ≥ ×20 mesuré |
| 3. Optimiseur et allocation | boucle des candidats, élagage des dominés, règle d'allocation, sécante des re-planifications, en threads | `core/optimizer.py` et `core/allocation.py` sur le noyau | identique ; plan N3 en < 5 s sur 16 cœurs |
| 4. Service de planification | processus permanent, caches gardés, entrée = état (attitude, cibles, santé), sortie = plan ; recalcul incrémental | service appelé par l'outil de préparation et par le profil | plan sur événement ≤ 1 s sur la machine de référence |
| 5. Planification anticipée | file de plans futurs le long de l'attitude commandée ; profil de mission entièrement exact | `core/time_profile.py` sans estimation rapide | 60 plans/s soutenus ; profil complet < 1 min |
| 6. Portage embarqué (option) | code déterministe, mémoire bornée, partitionnement temps réel | noyau certifiable | hors périmètre de cette étude |

Le passage de la phase 2 dépend du choix de langage (question ouverte) : numba garde tout en Python mais ne s'embarque pas ; C++ ou Rust en extension s'embarquent mais coûtent plus cher à écrire.

## Validation et non-régression

Une migration n'est acceptée que si le nouveau code reproduit l'ancien : mêmes verdicts, mêmes plans, valeurs à rel 1e-9. Le banc existant (`tests/test_snapshots.py`, `tests/update_snapshots.py`, `JOURNAL.md`) sert de juge.

1. **Double exécution.** Le chemin Python reste dans le dépôt comme référence. Un indicateur choisit le chemin ; un test compare les deux sur toutes les missions de `tests/missions/` et sur `scenarios/`.
2. **Tolérance.** rel 1e-9 sur les valeurs figées. Un écart venant de l'ordre des sommes flottantes en parallèle est corrigé par une réduction déterministe, pas par un relâchement de la tolérance.
3. **Aucun regel pendant la migration.** Les valeurs figées ne sont pas re-gélées par une phase de migration : un écart est un défaut du portage. Si un regel est inévitable, il est noté dans `JOURNAL.md` avec sa cause.
4. **Mesure de vitesse.** Chaque phase enregistre le temps par cas N3, avec le compteur d'étapes et la machine utilisée, pour suivre le gain réel.
5. **Protocole des tests.** Selon vos notes : demander avant chaque test, la batterie complète au plus une fois par jour ou tous les deux jours, les longs calculs sur le PC principal.

## Risques et questions ouvertes

Le risque principal est que le gain réel reste sous ×10³ : la phase 0 le dira avant tout investissement lourd.

| Risque | Effet | Parade |
| --- | --- | --- |
| Gain du compilé plus faible que prévu (numpy déjà vectorisé) | plan au-delà de 1 s | profiler d'abord ; viser l'algorithmique (élagage, incrémental) |
| Part séquentielle de l'allocation | le parallélisme plafonne | revoir la sécante des re-planifications ; allocation par face en parallèle |
| Scènes denses (beaucoup de volumes, pistes, brouilleurs) | coût plus que linéaire, cas de test trop petits (4 volumes) | ajouter une mission de charge au banc |
| Écarts flottants entre Python et natif | faux échecs de non-régression | réductions déterministes, mêmes fonctions mathématiques |
| Modèle encore mouvant (V6.42–V6.44 non testées) | portage d'un code qui change | commencer par les parties stables (équation radar, détection) |
| Deux implémentations à tenir | double travail à chaque évolution | garder Python comme référence seulement le temps de la migration |

Questions à trancher :

- [ ] Langage du noyau : numba (rapide à écrire, non embarquable) ou C++ / Rust en extension (embarquable) ?
- [ ] Valeurs cibles : plan sur événement ≤ 1 s, 60 plans/s en manœuvre, avance de 1 s, profil < 1 min : à garder ou à changer ?
- [ ] Machine de référence : le PC principal (16 cœurs logiques) ou un serveur de 64–128 cœurs ?
- [ ] Ordre : faire la migration avant ou après les étapes 5.5–5.7 et l'étape 6.4 (radar dans la boucle) ?
- [ ] La mémoire et le calcul doivent-ils entrer dans l'estimation rapide en attendant, ou l'estimation disparaît-elle avec la migration ?

## Prochaines étapes

La première étape est le profilage : sans lui, tous les gains de ce document restent des estimations.

- [ ] Trancher les questions ouvertes ci-dessus.
- [ ] Lancer sur le PC principal `tools/exact_compare.py` sur `MOONTEST_VELOCITY` (déjà en attente) avec un profileur, puis un cas N3 court et un cas long.
- [ ] Remplir le tableau des leviers avec les chiffres mesurés et confirmer ou revoir l'objectif ×10⁴.
- [ ] Décider du découpage de la phase 1 (modèle de données) selon les points chauds trouvés.
