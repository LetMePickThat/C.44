# SPEC — Optimiseur pré-mission v2 (à implémenter dans le repo)
Cible : `mission_optimizer.py` (+ adaptations `cost_estimate.py`, `gen_cost_table.py`, `pm_window.py`, `results_window.py`, `optimize_worker.py`). Modèle radar : `core/radar_model.py` (inchangé). Prototype de référence : `proto_per_bin_reference.py` (une dalle, non production). Fichiers de test : `mission_veille500.json`, `report_veille500.json`.

## 1. Pourquoi (défauts constatés sur v1)
1. **Une seule forme d'onde (WF) par secteur**, dimensionnée sur le dépointage le plus défavorable, puis puissance réduite aux autres angles. L'énergie est bonne mais le **temps d'émission est surestimé** : veille KV 500 km, RR 20 s → 65 % énergie / **102 %** temps. Avec une WF par tranche de dépointage : 75 % énergie / **13 %** temps.
2. **Critère = énergie seule** (temps rapporté sans être optimisé) : il faut minimiser max(énergie, temps).
3. **Zones non balayées ignorées dans le choix de fréquence** : l'optimiseur choisit 2,9 GHz (limite de balayage 70° au-dessus de 2,85 GHz) parce qu'il y a plus de pointages « gratuits », et laisse 14,6 % du secteur non balayé sans pénalité. Reproduction : `mission_veille500.json` sans `freq_ghz_candidates`.
4. **Départage lexicographique trop strict** : `round(short/total, 6)` : 0,4 % de pointages sous la portée l'emporte sur 2× moins de temps.
5. **Pas de sortie « marge poursuite »**.
6. Lenteur (corrigée en partie par le cache de `sector_area_uv`) : prévoir cache et parallélisme (§5).

## 2. Modèle (par dalle k, par secteur i, par tranche de dépointage θ_b)
Notations : `P_ikb` = nombre de pointages de la dalle k dans la tranche b (déjà calculé par `positions_table`, tranches de 1°, à regrouper en tranches de `BIN_DEG` = 5° en production, 10° en prototype). `R` = portée demandée, `RR` = RR cible, `Ppk` = `BAND_S.p_pk_total_w` (9,8 MW), `Pavg` = `elec_mw·1e6·CHAIN_EFF·PAE` (1,375 MW pour 2 MW).

Pour une WF candidate w (intégration, Btx, B, N, τ, PRI) et une tranche b évaluée à son **dépointage maximal** θ_b⁺ :
- `R_w(θ_b⁺) = solve(w, f, θ_b⁺, cible).range_km` (puissance crête max).
- Réduction de puissance : `s = min(1, (R / R_w)^4)` (R ∝ P^¼). Candidate **admissible** si `R_w ≥ R`.
- Énergie par pointage : `E = Ppk · N · τ / Btx · s` (J).
- Temps d'émission par pointage : `T = N · τ / Btx` (s), indépendant de s.
- Fractions de la dalle : `f_e = Σ_b P·E / (Pavg·RR)` ; `f_t = Σ_b P·T / RR`.
- Bornes physiques : `f_t ≥ f_e·Pavg/Ppk` (≈ 0,14·f_e).

**Objectif secteur** : choisir f ∈ {fréquences autorisées}, et pour chaque tranche b une WF w_b, minimisant `max(f_e, f_t)`.
Résolution : scalarisation `coût_b(w) = α·E/Pavg + (1−α)·T`, balayer α ∈ {0, 0,05 … 1}, retenir l'α qui minimise `max(f_e, f_t)` (suffisant ; bisection possible).
Critère de comparaison entre fréquences/α, **dans cet ordre** (tuple) :
1. `unscanned(f)` − min_f unscanned (fraction de secteur hors balayage supplémentaire due à la fréquence ; 0 si même résultat)
2. `short` = fraction de pointages sans WF admissible, arrondie à 0,01 (tolérance 1 %)
3. `max(f_e, f_t)`

Tranche sans WF admissible : prendre la WF de portée maximale (s = 1), compter ses pointages dans `short`, signaler (drapeau existant).

**Fréquences** : défaut `(2.7, 2.8)` ; **n'autoriser f > 2,85 GHz que si aucun pointage du secteur n'a θ > 70°** (sinon critère 1 les exclut). Conserver `freq_ghz_candidates` si fourni par l'utilisateur.

**Plusieurs WF par secteur** : une WF par tranche donne jusqu'à 15 WF. Ajouter `max_wf_per_sector` (défaut 4, optionnel dans le secteur) : fusionner les tranches voisines en gardant, pour chaque groupe, la WF admissible au θ le plus défavorable du groupe (recalculer s par tranche). Sortie : liste `wf_by_bin` + liste dédupliquée `wf_set`.

**Totaux et RR** (par dalle k) :
- `F_e(k) = Σ_i f_e,ik`, `F_t(k) = Σ_i f_t,ik`
- `RR_est(i) = RR_i · max(1, max_k∈faces(i) F_e(k), max_k F_t(k))` (remplace la dilatation sur l'énergie seule)
- `RR_seul(i) = RR_i · max(f_e,i, f_t,i)` (secteur seul sur la dalle, au max des dalles)

**Marge poursuite** (nouveau, par dalle) :
- `marge_puissance = 1 − F_e` ; `P_dispo_kW = (1 − F_e)·Pavg/1e3`
- `marge_temps_S = 1 − F_t` (informatif : le Ku est un réseau séparé, son temps n'est pas consommé par la veille S)
- Le **seul couplage S↔Ku est la puissance électrique** (alimentation commune, 2 MW/dalle). Rapport cyclique Ku équivalent : `duty_Ku = P_dispo / (BAND_KU.p_pk_total_w)` (9,19 MW crête). Faire apparaître `P_dispo_kW` et `duty_Ku_pct`.
- Alerte si `F_e > 0,9` (« marge poursuite < 10 % ») et si `F_e > 1` ou `F_t > 1` (sur-souscription).

## 3. Sortie JSON (compatible ascendante : ne rien renommer)
Ajouter par secteur : `wf_by_bin: [{theta_lo_deg, theta_hi_deg, positions, wf:{freq_ghz, integration, tau_us, n_pulses, pri_ms, sig_bw_mhz, b_tx, dwell_ms}, range_km_at_bin_max, power_scale}]`, `wf_set`. Garder `wf` (= WF de la tranche la plus peuplée) pour ne pas casser `results_window.py`. Dans `faces[k]` : `time_fraction` devient le temps **exact** du modèle ci-dessus. Dans `faces_total[k]` : `marge_puissance`, `P_dispo_kW`, `duty_Ku_pct`, `marge_temps_S`. Ajouter `global.objective: "max(energy,time)"` et `global.bin_deg`.

## 4. Interface
- `results_window.py` : (a) échelle de couleur de la métrique « % énergie » **propre** (vert < 70 %, orange 70–100 %, rouge > 100 % ; l'actuelle `_metric_color` est un ratio RR et reste verte pour toute fraction ≤ 1) ; (b) colonne « marge poursuite (kW / % duty Ku) » dans « Totaux par dalle » ; (c) tableau WF par tranche pour le secteur sélectionné ; (d) message si `cancelled` (résultat partiel).
- `pm_window.py` : jauge « marge poursuite » ; alerte dérivée de l'estimation rapide.
- `mission_data.validate_*` : refuser `v_close_max_mps <= 0`, `phi_min == phi_max` (secteur d'un seul azimut) ; avertir si les secteurs ne couvrent pas tout le volume balayé (volume « non alloué »).
- `cost_estimate.py` / `gen_cost_table.py` : stocker aussi `T` par pointage ; borne basse du temps = `E_tot/(Ppk·RR)` (déjà exposée), ne pas présenter comme exacte ; la table est insuffisante au-delà de ~5 000 km (voir §6, écart connu S03).

## 5. Performance / robustesse
- Précalculer **une fois par secteur** la table `(candidate × tranche)` de `R_w` (~3 000 candidates × 8–15 tranches ≈ 40 k `solve`, quelques s) ; la réutiliser pour tous les α et toutes les fréquences ; `functools.lru_cache` sur `(paramètres WF, f, θ, rcs, swerling, v)`.
- `multiprocessing` par secteur (compatible `OptimizeWorker` : progression par secteur terminé, annulation entre secteurs).
- Un seul chargement de `core/radar_model.py` partagé par `geometry.py` et `mission_optimizer.py` (aujourd'hui deux instances).
- Ne pas modifier les formules du modèle radar.

## 6. Tests d'acceptation (valeurs de référence, tolérance ±5 % relatif sauf mention)
Fichier : `mission_veille500.json` (cône avant 30° = 4 secteurs CONE_D1..D4 boîtes ε 75–90, ST 1000 m² 8000 km RR 120 s ; VEILLE_KV boîte ε ≤ 75, 0,135 m², 25 km/s, 500 km, RR 20 s). Par dalle :
| Grandeur | v1 (une WF) | v2 attendu |
|---|---|---|
| VEILLE_KV énergie | 65 % | ≈ 75 % (70–80 %) |
| VEILLE_KV temps d'émission | 102 % | ≈ 13 % (10–20 %) |
| Cône (par volume) énergie | 4,1 % | 4–7 % |
| Cône temps | 0,9 % | ≤ 1,5 % |
| Total énergie par dalle | 69,5 % | ≈ 80 % |
| Marge puissance poursuite | ≈ 30 % | ≈ 20 % (≈ 275 kW) |
| Portée cône au pire dépointage (inchangé) | 7 299 km | 7 299 km (±1 %), meilleur 9 726 km |
| Pointages KV | 4 254 (f 2,8) | 4 254 ± 1 % |
- Non-régression : `selftest()` du modèle passe ; `mission_example.json` (S01, S02, S03) : RR_seul S01 ≈ 6 s, S02 ≈ 0,5 s, S03 ≈ 46 s (v1) — le v2 doit donner des **énergies ≤ celles de v1** (le v2 peut seulement faire mieux sur l'objectif max(E,t)).
- Le v2 ne doit **jamais** choisir f > 2,85 GHz sur `mission_veille500.json` (régression du défaut 3).
- 11,3 % du secteur VEILLE_KV est « hors zone scannée » : normal (ε < −55° côté −Z, limite physique), à signaler sans pénaliser.
- Test d'unité : pour une WF donnée, `E = Ppk·N·τ/Btx·s` et `T = N·τ/Btx` ; `f_t ≥ f_e·Pavg/Ppk`.

## 7. Hypothèses [D] et points ouverts
- Temps d'émission = temps de la **chaîne TX S** (un faisceau à la fois si Btx = 1) ; l'entrelacement masque l'attente d'écho (`FRAME_INTERLEAVED`), la réception numérique n'est pas une ressource limitante.
- Détection seule (pas de confirmation Ku) : la poursuite et la confirmation consomment la marge de puissance, non modélisées ici.
- Les formes d'onde changent par tranche de dépointage : implique que le calculateur central accepte une table de commande par direction (cohérent avec l'architecture numérique verrouillée).
- Swerling ≠ 1 non pris en compte dans la table précalculée de `cost_estimate` (l'optimiseur le gère via `solve`).
- Pénalité « portée non atteinte » : tolérance 1 %, à rendre paramétrable (`short_tol`).
