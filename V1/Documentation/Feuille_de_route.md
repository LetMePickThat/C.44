# Feuille de route

Cette feuille de route mène de l'outil actuel à un **outil complet de préparation de mission**. L'opérateur y prépare sa mission, et celle-ci est validée contre **toutes** les contraintes et ressources du système radar : radiofréquence, ordonnancement, puissance, thermique, calcul, etc., et pas seulement le bilan de liaison.

L'utilisateur l'a acceptée le 20 octobre 2026. Elle se suit dans l'ordre, une étape à la fois, de session en session, avec un commit par étape, fait seulement sur demande.

Règles appliquées à chaque étape :
- rappeler la raison d'être de l'étape, ses entrées et ses sorties, puis obtenir l'accord avant de coder ;
- demander à l'utilisateur la valeur de **chaque** variable, sans jamais en choisir une en silence ;
- garder toutes les variables système dans le fichier unique `radar_system.toml` ;
- ne jamais présenter les missions de test et les valeurs figées comme des références ou des performances ;
- traiter au même endroit tout ce qui touche un même sujet, et fixer une valeur dépendante après celles dont elle dépend ;
- pour les technologies, retenir des valeurs crédibles mais optimistes pour 2050-2060, et le bout optimiste de chaque plage proposée ;
- le simulateur radar travaille dans le repère du vaisseau ; la position des objets lui viendra d'un futur simulateur global de scénario.

La procédure détaillée d'une étape et la manière de reprendre le travail dans une nouvelle session sont décrites au §1.4 de `Choix_architecturaux.md`.

(D) signale une étape qui demande une décision ou des données de l'utilisateur. Le détail des choix déjà faits est dans `Choix_architecturaux.md`.

## État d'avancement (25 octobre 2026)

| Étape | État |
|---|---|
| 0.1 | ✔ fait (commit cc14375) |
| 0.2 | ✔ fait (commit 1fb4bc7) |
| 0.3 | ✔ fait (commits c5c2400 et e2ebb90) |
| 1.1 | ✔ fait (commit fc511b3) |
| 1.2 | ✔ fait (commit 32df57b) |
| 1.3 | ✔ fait (commit c814182) |
| 1.4 | ✔ fait (commit 7099b38) |
| 1.5 | ✔ fait (commit 4bcaa5d) |
| 2.1 | ✔ fait (commit d1654ac) |
| 2.2 | ✔ fait (commit 90573c9) |
| 2.3 | ✔ fait (commit 102ea6f) |
| 2.4 | ✔ fait (commit 008d583) |
| 2.5 | ✔ fait (commit baeadcf) |
| 3.0 | ✔ fait (commit 22114fd ; ajoutée le 22 octobre 2026) |
| 3.1 | ✔ fait : P1 à P6, bande S élargie, accélérations (`90555d8`, `b4be91e`, `cd97216`, `f6a526c`, `b0dbc09`, `f8fa433`, `d19829e`, puis le commit de P6) |
| 3.2 | ✔ fait le 25 octobre 2026 : Q1 à Q5 (`8720343`, V6.12.0), Q6 : accélérations exactes E1 à E4 (V6.12.1), règle de précision absolue supprimée et approximations A1, A2, A4, A5, A6, A8, A9, banc T1 / T3 et barre de progression du N3 (V6.13.0), valeurs figées regelées (`JOURNAL.md`) |
| 3.3 | codée le 25 octobre 2026 (V6.14.0, passes R1 à R6) : modèle de piste (`core/trackmodel.py`), budget de poursuite reconstruit, discrimination, atténuateur, aspect, repère `track` ; N3 du 25 octobre (21 min, 190 tests passent, valeurs figées regelées, `JOURNAL.md`) ; trois valeurs D à confirmer (`faraday_uncert_frac`, `deception_verify_rate`, `track_frame_k`) ; commit sur demande |
| 3.4 | codée le 25 octobre 2026 (V6.15.0) : erreurs du réseau et étalonnage en vol (perte `calibration` dérivée ≈ 0,006 dB au lieu de 0,5 / 0,8 dB, plancher de lobes, contrôles de cohérence, temps d'étalonnage dans le budget des faces) ; modes dégradés et pannes reportés en fin de développement ; N3 et commit sur demande |
| 4.1 | codée le 25 octobre 2026 (V6.16.0) : réserve commune des 8 MW (plafond physique 8 MW, plafond facultatif de l'opérateur), consommation permanente 106,9 kW par face, alimentation des TRM (formes d'onde retirées), stockage de face (observation, rafale, pointe), règle d'émission contiguë pour 4.3, transitoires ; N3 et commit sur demande |
| 4.2 | ✔ fait le 25 octobre 2026 (V6.17.0) : fluide et plaque froide dérivés (capacité ≈ 15,9 MW par face, jamais limitante), jonction mesurée sans effet (contrôle désactivé, `duty_max` gardé), rejet du vaisseau 6 MW (D) commun aux faces |
| 4.3 | codée le 25 octobre 2026 (V6.18.0) : `core/scheduler.py` (priorités P0 étalonnage, P1 autoprotection, P2 confirmation / initialisation, P3 mises à jour, P4_n veille ; bilan statistique + chronologie synthétique ; créneaux communs et éclipse ; harmonique ; latence) ; GO-CFAR seulement conseillé, gain du multiplexage en temps non chiffré ; N3 et commit sur demande |
| 4.3 bis | codée le 25 octobre 2026 (V6.19.0) : ordre des pointages dans un volume (`core/scanorder.py` : grille réelle en (u, v), ordre de Morton à bits renversés, 3 décalages de grille, durées par tranche, multifaisceau, uniformité, revisite par direction) ; N3 passé (21,6 min, snapshots regelés) ; commit sur demande |
| 4.4 | codée le 25 octobre 2026 (V6.20.0) : `core/compute.py` (débits, faisceaux simultanés, charge par fonction, tampons, puissance du calcul plafonnée par `dbf_kw`, latence de traitement à la place de `processing_s`, coût d'une resélection en vol) ; bilans de dimensionnement gardés au plafond (boucle en 5.2, accepté) ; N3 passé (20,7 min, snapshots regelés) ; formateur porté à 512 faisceaux en S et 32 en Ku, mémoire 4 To et débit mémoire 50 To/s par face (décisions de l'utilisateur, N3 20,5 min) ; capacité de faisceaux dans l'optimiseur avec la boucle de 5.2 |
| 4.5 | codée le 25 octobre 2026 (V6.21.0) : `core/repoint.py` (pointage converti à l'émission, erreur de navigation 30 °/s / 15 °/s², perte `nav_pointing` ≈ 10⁻³ dB au plus, erreur de poursuite, migration entre faces, coût de calcul) ; N3 passé (23,8 min, snapshots regelés) |
| 5.1 | codée le 25 octobre 2026 (V6.22.0) : réglages de mission et de volume, tables de SER 12 × 7 (détection au pire aspect, KKV de face), dispersions dans la discrimination, percée brouilleur autorisable, interface provisoire ; N3 passé (22,2 min, snapshots regelés) |
| 5.2 et suivantes | à faire (5.7 et phase 6 ajoutées le 25 octobre 2026) |

---

## Phase 0 : fondations

- **0.1 Dépôt de paramètres unique et versionné.** Les valeurs sont chargées depuis un fichier, avec leur statut ([V], [D] ou [?]) et leur source. Plus aucune modification de `globals()`, et un seul chargement de `radar_model`. ✔
- **0.2 Banc de tests pytest.** Il reprend le self-test actuel et les missions de test (mission_example, mission_veille500, mission_redundancy_demo, synthétique). Il compare les valeurs figées avec une tolérance relative de 1e-9, vérifie des invariants physiques et produit un rapport d'impact à chaque étape. Ces valeurs ne sont pas des références. ✔
- **0.3 Revue de toutes les valeurs [?]** (D). ✔

## Phase 1 : antenne et RF

- **1.1 Géométrie cohérente de l'antenne.** ✔ (commit fc511b3)
  - Calculer le **vrai diagramme** à partir de la composition des TRSA (rangées de l'octogone, jeux, pas de 52 mm en S et 10,4 mm en Ku), au lieu de HPBW = 0,886 λ/D_env, qui est la valeur d'une ouverture rectangulaire.
  - Séparer le faisceau d'émission, uniforme et qui sert à la grille de veille, du faisceau de réception, défini par son niveau de lobes secondaires, dont on déduit `eta_illum_rx`.
  - Largeur de lobe sur deux axes en (u, v).
  - (D) Ouverture Ku partagée ou séparée.

  Décisions prises :
  - précision angulaire par la borne de Cramér-Rao × 1,13 ;
  - lobes de réception à −25, −30 ou −35 dB, réglés par volume, −25 par défaut ;
  - grappe de faisceaux de réception au pas de 0,5 largeur de lobe ;
  - ouverture Ku partagée.

- **1.2 Diagramme de rayonnement.** ✔ (commit 32df57b)

  Décisions prises :
  - feuille de courant avec WAIM adaptée à 45° (S et Ku), plus un terme de lobe de réseau exp(−g/0,02) dans le pire azimut ;
  - perte appliquée à l'émission et à la réception ; limites de balayage verrouillées, alerte au-delà de 3 dB par passage ;
  - polarisation d'émission linéaire selon e_v ; 0,5 dB gardés pour la recombinaison jusqu'à 1.5 ; pureté calculée en sortie ;
  - lobes secondaires calculés sur tout l'espace visible, avec alerte par secteur ;
  - jeu entre TRSA ramené de 2 mm à 0,5 mm, après la découverte des lobes de jeu.

  Contenu prévu à l'origine :
  - Vrai lobe principal issu de la pondération, lobes secondaires, lobes de réseau, perte physique au dépointage (réflexion active).
  - **Doit apporter un modèle unique** de la désadaptation active |Γ(θ, f)|, pour S et Ku : WAIM, approche des lobes de réseau en haut de bande. Ce modèle doit expliquer la limite de 70° au-dessus de 2,85 GHz.
  - Depuis la revue 0.3, le gain ne suit que la projection cos θ : l'exposant 1,1 de V5.6.0 et la loi de perte au dépointage de V5.4.0 (0,6 dB à 70°) ont été retirés, car ils comptaient la même perte deux fois. Les résultats sont donc **optimistes** aux grands dépointages tant que cette étape n'est pas faite : 1,53 dB aller-retour à 70° (portée +9,2 %), 2,0 dB à 75° (+12,2 %).
  - **Polarisation** (sujet transverse) :
    - décider la polarisation d'émission (linéaire, et laquelle, ou circulaire) ;
    - modéliser la pureté de polarisation de l'antenne (rapport axial, polarisation croisée) en fonction du dépointage et de la fréquence, avec le WAIM.

    Ce modèle remplacera la perte fixe `pol_loss_dual_db` de 0,5 dB (S et Ku).
  - Marge de lobe de réseau de 1,1 % à 2,9 GHz pour 75°.
  - Lobes du réseau périodique de jeux entre TRSA (hors de la fenêtre de 8 λ/D évaluée à l'étape 1.1).

- **1.3 TRM et chaîne d'émission S et Ku.** ✔ (commit c814182)

  Décisions prises :
  - TRM GaN sur diamant avec microcanaux ; valeurs crédibles mais optimistes pour 2050-2060 ;
  - bilan de face **électrique** ; PAE variable : S 0,80, Ku 0,65 à d = 0,10, fronts 0,2 / 0,1 µs, baisse 0,05 / 0,08 par unité de facteur de forme ;
  - facteur de forme maximal 0,65 sur les trains ; réserve 25 J / 1,2 J, fraction 0,2, d'où τ_max 2,0 / 2,08 ms ;
  - dissipation calculée en sortie ; recharge pendant le train et alimentation des TRM déplacées en 4.1 ; PAE en fonction du recul renvoyée en 3.1.

  Contenu prévu à l'origine :
  - PAE en fonction de l'impulsion et du facteur de forme, facteur de forme maximal, impulsion maximale, réserve d'énergie dans les deux bandes, dissipation.
  - (D) Données des TRM Ku.
  - **Doit remplacer** la constante `power.pae` = 0,75 (commune à S et Ku) par une **PAE variable** par bande, en fonction de la longueur d'impulsion et du facteur de forme.
  - Revoir `e_buf_trm_j` (5 J, statut ?), `buffer_max_fraction` et `duty_max`.
  - Densité de puissance d'environ 104 kW/m² de RF moyenne (avec l'étape 4.2).
  - Recul du PA en multifaisceau par superposition (pour l'étape 3.1).

- **1.4 Bande.** ✔

  Décisions prises :
  - loi de bord en cosinus surélevé, bande centrale en GHz par bande (S 2,74 à 2,86 GHz, Ku 13,7 à 14,3 GHz), 0,5 dB au bord, perte totale moyennée sur le spectre occupé ;
  - le signal doit tenir dans la bande utile ; bande instantanée S 200 MHz confirmée ; retard vrai numérique explicite ; limite de 70° gardée ;
  - principe : les valeurs restent ajustables (bande plus large si les TRM progressent), le code l'encaisse (tout calculé, contrôles au chargement).

  Contenu prévu à l'origine :
  - Loi de dégradation aux bords propre à chaque bande, au lieu de la fraction de bande copiée de l'ancienne bande Ku (l'ancienne table `rf_layer_s` de RADARv5 est obsolète et ne sert pas de source).
  - Bande instantanée en S (aujourd'hui 200 MHz, « pour l'instant »).
  - Retard vrai (numérique par TRM en S et en Ku, décidé lors de la revue 0.3).
  - Question : relâcher ou non la limite de 70° au-dessus de 2,85 GHz (tranchée : gardée).

- **1.5 Réception.** ✔

  Décisions prises (bout optimiste de chaque plage) :
  - le bruit de phase de l'oscillateur passe **en entier** à l'étape 2.2, avec la cohérence (règle « regrouper par sujet ») ;
  - ciel dans le **repère du vaisseau**, donné par la mission (clé `sky`), plus tard par le simulateur global de scénario ; le modèle ne calcule jamais où sont les corps ;
  - T_ant = ciel froid (S 3,2 K, Ku 2,8 K) + fond galactique (profil en latitude et longitude, pire cas 8 K à 2,8 GHz sans repère) + corps (table des T_b par type et par bande, surcharge possible par le scénario) intégrés sur le diagramme de réception réel moyenné en azimut ;
  - pointages **aveuglés** au-delà de +10 dB ; T_sys du bilan d'un secteur = centile (1 − `short_tol`) des autres pointages ;
  - CAN I/Q à 250 Méch/s, 13 bits, +10 dBm : facteur de bruit dérivé (20 dB) ; T_sys froid S 144,5 K, Ku 230,8 K ; dynamique, portée minimale non saturée, alerte des tranches de poursuite ; récupération du limiteur 50 / 25 ns ;
  - isolation : matrice des verdicts ; **synchronisation** des faces par bande (`interface_mode` sync) ; harmonique 5 de la S dans la bande Ku : règle de fréquence sur la même face ;
  - polarisation : détecteur GLRT sur la polarisation inconnue (perte calculée, 0,54 dB sur un échantillon cohérent), 0,05 dB en poursuite ; remplace les 0,5 dB provisoires.

  Contenu prévu à l'origine :
  - T_sys selon la direction (Terre, Lune, Soleil, plan galactique).
  - LNA ; dynamique et saturation du CAN.
  - Isolation entre S et Ku et entre faces.
  - Modèle du bruit de phase de l'oscillateur : spectre et effet sur le temps d'aller-retour (**passé à l'étape 2.2**).
  - **Polarisation** : deux voies de polarisation en réception par TRM et leur recombinaison (cohérente ou optimale), avec le gain de diversité de polarisation face à la perte (le gain de diversité reste à l'étape 2.4).

## Phase 2 : signal et détection

- **2.1 Formes d'onde paramétriques.** ✔

  Décisions prises (bout optimiste de chaque plage) :
  - LFM et NLFM, choix automatique (NLFM si f_d,max / B ≤ 2 %, perte 0,1 dB ; sinon LFM avec Taylor n̄ = 5 à la réception) ; lobes en distance = option du volume ; perte `compression` calculée ; `DSP` décomposé (précision finie 0,3 dB, chevauchement 0,7 dB jusqu'à 2.2) ;
  - k_r dérivé du spectre pondéré ; résolution physique (élargissement compris), signalée si elle n'est pas tenue ;
  - loi exacte de perte d'un filtre adapté à une autre vitesse et nombre de filtres pour 1 dB, en sortie pour l'étape 2.2 ; biais de couplage distance-Doppler et dérive AM/PM en sortie ;
  - nouvelle clé de secteur `range_min_km` (défaut 10 % de la portée) ; zone aveugle stricte ; PRI des trains non ambigus ≥ 2R/c + τ + récupération ; trains ambigus échelonnés (3 PRI, ±10 %) avec la perte d'éclipsage moyenne ;
  - multiplexage du multifaisceau passé en 3.1 ; espace de recherche au-delà de 75 MHz ouvert en 3.1 ;
  - à terme, les vues afficheront seulement le volume utile, sans la zone aveugle (5.5).

  Contenu prévu à l'origine :
  - **Renvois de l'étape 1.3** : zone aveugle c·τ/2 à vérifier **par rapport au besoin du secteur** (300 km pour les impulsions de 2 ms, sans importance à 10 000 km) ; éclipse au facteur de forme élevé (jusqu'à 0,65) ; migration et Doppler dans l'impulsion (50 m à 25 km/s sur 2 ms) ; dérive de phase et d'amplitude dans l'impulsion.
  - **Renvoi de l'étape 1.5** : le récepteur est sourd pendant τ + la récupération du limiteur (50 ns en S, 25 ns en Ku ; sortie `rx_deaf_us` des formes d'onde), à prendre dans l'éclipsage.
  - Modulation, compression, lobes en distance, tolérance au Doppler, couplage distance-Doppler.
  - Éclipsage, ambiguïté en distance et en Doppler, PRF échelonnée.
  - Pondération de Taylor déduite de la modulation réelle (pour k_r).
  - Bandes de signal au-delà de 75 MHz.
  - Multiplexage en temps ou en fréquence pour le multifaisceau.

- **2.2 Traitement.** ✔

  Décisions prises (bout optimiste de chaque plage) :
  - blocs cohérents de taille optimale puis intégration non cohérente ; plancher η_min et plafond de 149 ms supprimés ; temps de cohérence de la cible provisoire (t₀ 0,25 s) jusqu'à l'étape 2.4 ;
  - oscillateur photonique commun, perte par l'aller-retour calculée (négligeable), plancher Doppler en sortie, plateforme 10 µm ;
  - banc d'hypothèses de vitesse (pas à 1 dB, perte moyenne au bilan, résidu 0,1 dB) sur l'étendue de vitesses du secteur, une hypothèse en poursuite ;
  - chevauchement calculé (grilles ×4 en distance et en Doppler, pas de fenêtre), CA-CFAR à 1 024 cellules ; entrées `straddle` et `CFAR` supprimées ;
  - fausses alarmes comptées sur les cases indépendantes (distance depuis `range_min_km`, Doppler d'un bloc, hypothèses de vitesse, angle ≈ 0,85), Pfa 1e-8 inchangée ;
  - **constat** : les bancs de vitesses des trains multiplient les fausses alarmes (×50 à ×535) et l'optimiseur de veille ne voit pas ce coût (sujet des fausses alarmes traité en 2.3, 5.1 et 5.2, décision de l'utilisateur).

  Contenu prévu à l'origine :
  - Migration traitée par bancs d'hypothèses de vitesse : perte en fonction du nombre de cases traversées, au lieu d'un forfait de 1 dB.
  - Chevauchement de cases, CFAR, nombre réel de cases.
  - **Doit aussi corriger le modèle de cohérence.** Le plancher η_min = 0,70 est atteint vers 149 ms, puis fige la perte à 1,55 dB quelle que soit la durée d'intégration : ce n'est pas physique, et c'est la cause des égalités exactes dans l'optimiseur. Il faut plafonner le temps cohérent et intégrer de façon non cohérente au-delà.
  - t₀ = 0,25 s et p = 2 n'ont **aucune source connue** et sont à établir.
  - **Disposition provisoire** depuis la revue 0.3 : `radar_model.COH_T_MAX_S` = t₀(−ln η_min)^(1/p) = 149 ms. Les générateurs de formes d'onde de l'optimiseur et de la poursuite écartent les intégrations cohérentes plus longues. À retirer quand les blocs cohérents suivis d'une intégration non cohérente seront modélisés.
  - **Renvoi de l'étape 2.1** : le pas du banc de vitesses se déduit de la loi de perte calculée à l'étape 2.1 (`velocity_mismatch_loss_db` : 1 dB à n_c = 1,016, soit 13 filtres pour 2 ms à 75 MHz sur 0 à 25 km/s) ; le forfait de chevauchement de 0,7 dB (`straddle`) est à remplacer par la grille réelle des cases.
  - **Bruit de phase de l'oscillateur, sujet entier (décidé à l'étape 1.5)** : modèle du spectre de l'oscillateur et effet sur l'aller-retour (53 ms à 8 000 km, décalages d'environ 19 Hz). Il décorrèle l'émission et la réception (lobes Doppler, cohérence) ; ce n'est pas seulement une gigue à court terme. À traiter avec la refonte de la cohérence ci-dessus.

- **2.3 Critère de détection** (D). ✔

  Décisions prises :
  - veille : `detection.pd` 0,9 devient une **Pd cumulée** atteinte à `range_km` sur les observations d'une cible qui se rapproche à v_close_max (espacées de v·RR, phase moyennée, sans plafond) ; Ku : une observation ;
  - Pd de chaque observation **moyennée** sur la position aléatoire de la cible (grilles, chevauchement, filtre de vitesse) : une perte « positions » ; lois de détection **exactes** (Swerling 0 à 4, toute Pfa) ;
  - politique de fausses alarmes « fa_rate » : 10 par secteur et par revisite, Pfa par case déduite (bornée 1e-14 à 1e-4) ; mode « pfa_cell » conservé ;
  - grille de veille **1,04 → 1,4** lobe, **décalée à chaque trame** (exigence pour l'ordonnanceur) ; T_sys du ciel gardé au centile ; redondance au même critère ;
  - constat : deux régimes selon q = v·RR / R (longue portée : énergie divisée par 3 à 25 ; anti-KKV à q = 1 : énergie ×4).

  Contenu prévu à l'origine :
  - Pd moyenne ou au pire point de la cellule de la grille.
  - Pd cumulée sur plusieurs trames pour une cible qui se rapproche.
  - Revoir `beam_spacing_hpbw` et `enable_grid_loss`.
  - **Renvois de l'étape 2.2** : moyenne ou pire point aussi pour le chevauchement et le banc de vitesses (pertes moyennes aujourd'hui) ; **politique de fausses alarmes** : Pfa par case fixe (aujourd'hui) ou taux de fausses alarmes fixé par position ou par période, la Pfa par case s'en déduisant (le seuil monte avec le nombre de cases au lieu des fausses alarmes).
  - **Renvoi de l'étape 1.5** : T_sys du bilan d'un secteur au centile (1 − `short_tol`) des pointages non aveuglés, c'est-à-dire le pire point à 1 % près ; le choix entre moyenne et pire point est à revoir ici.

- **2.4 Modèles de cibles** (D). ✔

  Décisions prises :
  - **bibliothèque de classes** dans `radar_system.toml` (`[targets.<NOM>]` et `[targets.<NOM>.band.S|Ku]`), schéma contrôlé au chargement ; **une classe s'ajoute ou se modifie dans le fichier seul** (exigence de l'utilisateur) ; classes ST (référence), GENERIC (complète un secteur sans classe), KKV, SHIP, DEBRIS, DECOY ;
  - les classes sont des **valeurs moyennes, pas absolues** (l'armement de l'adversaire n'est pas connu) : milieu de chaque plage, toute valeur surchargeable par secteur (`target_type` + surcharges) ;
  - SER copolaire **frontale** par bande ; SHIP 300 m² de face ; KKV : corps 0,135 m² + **autodirecteur IR à miroir** (D 0,3 m, f/D 1, œil de chat η_r 0,1), soit 1,27 m² en S et 13,8 m² en Ku ;
  - T₀ = λ / (2 L ω) par classe et par bande (remplace t₀ 0,25 s) ; fluctuation en forme Gamma : famille × min(n, 1 + T_obs / T₀), ou cas déclaré ; diversité de polarisation (1 + ρ)² / (1 + ρ²) et énergie croisée 1 + ρ ;
  - accélération : perte (phase quadratique, courbure) sous 1 dB, sinon banc d'hypothèses qui multiplie celles de vitesse ;
  - masquage par une cible forte et OS-CFAR (rang 3/4, option) en sortie ; alerte si la revisite est sous 3 T₀.

  Contenu prévu à l'origine :
  - SER selon l'aspect et la bande, cas de Swerling par bande, cinématique et manœuvre.
  - Un **temps de cohérence T₀ par type de cible** (rotation, accélération, décorrélation de scintillation), au lieu du `coherence.t0_s` = 0,25 s global (provisoire depuis l'étape 2.2, qui a mis en place les blocs cohérents).
  - **Renvois de l'étape 2.2** : accélération et manœuvre dans le banc de vitesses (courbure en distance : 10 g sur 1,7 s font 140 m) ; masquage d'une cible faible par les lobes Doppler (plancher de bruit de phase, transformée sans fenêtre) et par les lobes en distance ; CFAR robuste (OS, GO) en présence de cibles voisines.
  - **Polarisation** : matrice de diffusion par type de cible (SER copolaire et croisée, dépolarisation selon l'aspect) et effet de la diversité de polarisation sur le cas de Swerling (perte de fluctuation). Depuis l'étape 1.5, la recombinaison des deux voies est le détecteur GLRT, sans gain de diversité : ce gain est à établir ici.

- **2.5 Environnement.** ✔

  Décisions prises :
  - **structure du vaisseau** dans `[ship]`, d'après le **modèle 3D `assets/models/LP_NISO.obj` depuis le 22 octobre 2026** (référence de tous les calculs de masquage et de position des éléments ; auparavant des primitives) (coque de révolution, télescopes, ailettes, tourelles, mâts, rails, ailes de radiateurs en 4 étages, d'après le dessin validé ; pas de repliement) ; **la géométrie se modifie dans le fichier seul** (exigence de l'utilisateur), et la vue 3D est construite sur les mêmes primitives ;
  - masquage par la fraction bloquée du tube d'ouverture (champ proche), perte −20 log(1 − f) par passage, masqué au-delà de 3 dB ; constat : aucun pointage masqué et radiateurs jamais vus avec l'inclinaison de 25° ;
  - bruit de la structure : coque ε 0,01 / 290 K, radiateurs ε 0,05 / **750 K** (médiane d'un vaisseau nucléaire) ; réflexions non comptées (5.4) ;
  - surfaces (`[sky.surface]`, γ Terre −12 / −10 dB, Lune −11,5 dB) : masque derrière la surface, fouillis par l'anneau d'égale distance traité comme un bruit (limité par le fouillis au-delà), rejet Doppler si le scénario donne la bande de la surface, alerte de saturation des CAN, trains ambigus retirés face à une surface ;
  - brouilleurs de bruit en classes moyennes (`[jammers.*]` : SOJ_SHIP 50, SOJ_ESCORT 40, SPJ_SHIP 27, SPJ_POD 10, SPJ_KKV 5 dBW/MHz), à distance dans le ciel de la mission, d'autoprotection par secteur (portée de percée) ; annulation 40 dB × 8 par face ;
  - objets non menaçants (`objects_per_rr`), rotation de Faraday en sortie (pas de perte avec le GLRT à deux voies), GO-CFAR en option et alerte de limbe ; point « bruit de fond » clos.

  Contenu prévu à l'origine :
  - Bruit de fond.
  - **Masquage des champs de vue de l'antenne par le vaisseau** (ailes de radiateurs de 76,5 m, coque), d'après la géométrie `[ship]`, pour le calcul de la couverture. Aujourd'hui, seul le plancher arrière ε ≈ −55° est modélisé. **Avec le bruit thermique de la structure du vaisseau** dans le champ des faces (ailes de radiateurs chaudes, coque), même objet (décidé à l'étape 1.5) : la structure deviendra un « corps » du bruit du ciel (`core/sky.py`).
  - Fouillis terrestre : la Terre est déjà une source de bruit (étape 1.5) ; son écho (fouillis) et la saturation des CAN qu'il peut causer (numérisation à l'élément, avant la formation de faisceau) sont à traiter ici, comme le brouillage.
  - (D) Débris, fouillis lunaire ou terrestre, brouillage ?
  - **Rotation de Faraday** (gardée par l'utilisateur), seulement pour les lignes de visée qui traversent l'ionosphère ou la plasmasphère :
    - orbite terrestre basse : plusieurs radians en S dans l'ionosphère ;
    - plasmasphère : environ 0,3 rad à 2,8 GHz sur 2 × 8 000 km ;
    - interplanétaire : environ 1e-12 rad, négligeable ;
    - négligeable en Ku et en polarisation circulaire.

    Elle dépend des positions du vaisseau et de la cible.

## Phase 3 : fonctions radar

- **3.0 Repère de mission et attitude** (ajoutée le 22 octobre 2026, à la demande de l'utilisateur). ✔ Constat : les volumes, les tranches de poursuite et le ciel sont donnés dans le repère du vaisseau ; si le vaisseau tourne, la région veillée tourne avec lui, ce qui n'a pas de sens opérationnel. Décisions de cadrage (validées par l'utilisateur) :
  - **repère des volumes** choisi par mission : inertiel centré sur le vaisseau (ICRF / J2000), orbital local (LVLH / RTN) ou défini par deux directions (axe de menace, corps) ; chaque secteur peut rester dans le repère du **vaisseau** (volumes liés au vaisseau) ;
  - **attitude** : une seule par mission maintenant (a), puis une enveloppe d'attitudes validée au pire cas par face et par contrainte (b, étape 5.3), puis le profil d'attitude dans le temps fourni par le simulateur global (c, étapes 4.3 et 5.4). L'architecture le prévoit dès 3.0 : attitude rangée dans une liste (quaternion, date facultative, étiquette), une seule fonction de passage au repère du vaisseau, tout le calcul en aval reste dans le repère du vaisseau, résultat calculé pour une attitude donnée ; le chargeur refuse plusieurs attitudes tant que b n'est pas fait ;
  - **le radar applique la rotation** des volumes à partir de l'attitude fournie (rotation seule, sans éphéméride) ; le simulateur global reste la source de l'attitude et des positions des objets ;
  - **le ciel** (corps, repère galactique) est donné dans le **même repère de mission** que les secteurs ;
  - les régions **non centrées sur le vaisseau** (parallaxe) sont renvoyées au simulateur global : en préparation de mission, les volumes restent centrés sur le vaisseau (direction et portée).

  Architecture de la simulation globale (D1 à D6, validés par l'utilisateur le 22 octobre 2026), fondée sur la séparation entre **vérité** (connue de la seule simulation) et **connaissance** (ce que le vaisseau peut savoir) :

  | # | Couche | Côté | Connaît | Produit |
  |---|---|---|---|---|
  | 1 | Simulation globale | simulation (vérité) | orbites, objets et leurs caractéristiques (attitude, SER selon l'aspect, température, manœuvres), propagation newtonienne | états vrais au cours du temps |
  | 2 | Convertisseur | simulation (vérité) | états vrais | (a) pour la simulation RF : positions et vitesses relatives des objets en repère vaisseau, corps du ciel ; (b) pour le calculateur : un **message de navigation** (attitude, état d'orbite propre, corps), plus tard avec les erreurs de navigation |
  | 3 | Calculateur radar | vaisseau (connaissance) | profil de mission (Pmiss), message de navigation, détections reçues | conversion des volumes en repère vaisseau, ordonnancement, formes d'onde, ordres de pointage, pistes |
  | 4 | Simulation RF | simulation | ordres du calculateur, vérité en repère vaisseau (2a) | émission, échos, bruit, ciel, détections et fausses alarmes renvoyées au calculateur |

  - le calculateur ne lit **jamais** la vérité des cibles : il ne les connaît que par ses détections ; l'attitude, l'orbite propre et les éphémérides des grands corps sont des connaissances de bord légitimes (navigation) ;
  - le glissement des volumes et des poursuites avec l'orientation est un **pilotage par le calculateur**, à partir de la navigation (la décision Q3 se lit ainsi) ;
  - calculateur et simulation RF partagent les mêmes modèles (`radar_model`, `radar_system.toml`) : le premier pour prévoir, la seconde pour jouer ;
  - l'outil actuel est la couche 3 en préparation de mission, sur un instantané du message de navigation saisi par l'opérateur ; la liste d'attitudes de Q2 devient une **liste de messages de navigation** (un seul tant que 5.3 n'est pas faite) ;
  - en préparation de mission, le ciel est **prévu** par le calculateur (saisie de l'opérateur, en repère inertiel ou de mission) ; dans la simulation, sa vérité passe par 2a vers la couche 4 ;
  - **repères des volumes** (D4) : vaisseau (protection), inertiel (« coin de ciel »), orbital local RTN, **limbe du corps** (veille de découverte sur l'horizon : bande d'élévation par rapport au limbe, qui suit l'altitude), deux directions (axe de menace) ; le volume lié à une piste ou à une désignation va en 3.3. Le type de repère entre dans le calcul (repères dynamiques) ;
  - **périmètre de la 3.0** (D5) : l'architecture écrite dans la documentation (la simulation n'est pas construite), le message de navigation, la bibliothèque de repères et la conversion des volumes dans la couche calculateur ; les couches 1, 2 et 4 restent à faire.

  Fait (modèle V6.11.0, `core/frames.py`, détail dans `Choix_architecturaux.md` §7.14) : clé de mission `nav` (un message : quaternion vaisseau → ICRF, date, orbite facultative), clé de secteur `frame` (`ship`, `inertial`, `rtn`, `limb` avec `limb_alt_km` à 0 km par défaut, `two_dir`), clé `frame` du ciel ; passage au repère du vaisseau une seule fois avant tout calcul ; contrôles et alerte ciel / navigation ; interface (groupe Navigation, repère du secteur, vue 3D « ship » ou « ICRF ») ; mission de test `mission_frames.json`. Les missions existantes ne changent pas.

  Renvois : enveloppe d'attitudes au pire cas par face → **5.3** ; profil d'attitude dans le temps et réaffectation des secteurs aux faces en vol → **4.3**, **5.2** ; volume lié à une piste ou à une désignation → **3.3** ; régions non centrées sur le vaisseau (parallaxe) et erreurs de navigation → simulateur global ; l'interface ne saisit pas encore le ciel (JSON seulement, comme avant).

- **3.1 Veille S reconstruite sur les nouveaux modèles** (optimizer.py). En cours : cadrage et variables des blocs A à E validés (détail dans `Choix_architecturaux.md` §11) ; codage en six passes avec pause (P1 grille de fréquences ✔, P2 formes d'onde ✔, P3 multifaisceau ✔, P4 environnement ✔, P5 ciel et pas de grille ✔, P6 rapport et documentation ✔) ; avant P2, élargissement de la bande S (décision de l'utilisateur).
  - **Renvois de l'étape 1.3** : loi de PAE en fonction du recul du PA (multifaisceau par superposition, et réduction de puissance « pas d'énergie en trop », aujourd'hui à PAE constante) ; impulsions candidates entre 0,8 et 2 ms. *Levé en P3 : la superposition est en phase seule et la réduction raccourcit l'impulsion, les PA restent saturés ; la loi n'est plus nécessaire.*
  - **Multifaisceau en émission** : tous les modes de formation sont possibles, et l'optimiseur doit les évaluer et choisir selon le contexte :
    - (a) superposition de B faisceaux par TRM : enveloppe non constante, donc recul du PA, qui demande le modèle de PA de l'étape 1.3 ;
    - (b) partition de l'ouverture : gain divisé par B, faisceau plus large, d'après le diagramme de l'étape 1.1 ;
    - (c) multiplexage en temps ou en fréquence dans l'impulsion : énergie par faisceau, résolution, Doppler (étape 2.1).

    Cela remplace la loi unique `multibeam.l8_db` / `p` (statut ?). À reprendre aussi à l'étape 5.2.

    **Fait en P3 (modèle V6.11.5)** : (a) en phase seule, enveloppe constante, PA saturés, pertes 0,4 / 0,6 / 0,8 dB à 2 / 4 / 8 faisceaux ; (b) moitiés (coupe `h` ou `v`) et quadrants, grille des positions sur le faisceau élargi (×1,85 / ×3,43), les deux moitiés équivalentes sur l'octogone actuel ; (c) multiplexage en temps modélisé, hors de l'espace de l'optimiseur, et multiplexage en fréquence écarté ; « pas d'énergie en trop » par raccourcissement de l'impulsion (PAE exacte, plancher τ ≥ 1/B). Détail : `Choix_architecturaux.md` §5.8 et §11. Reste pour P4 : fouillis vu par le faisceau d'émission élargi d'une partition.
  - **Grille de fréquences S** : remplacer `freq_candidates_ghz` (2,7, 2,8 et 2,9 GHz) par une grille fine au pas de 5 ou 10 MHz. Depuis l'étape 1.4, aucun signal ne tient à 2,7 ni à 2,9 GHz (bords de bande) : la grille doit se placer dans la bande, en tenant compte de la largeur du signal. Le but est d'éviter les brouilleurs étroits de forte puissance tout en gardant plus de cases fréquence qu'un masquage par multiples de 25 MHz. La grille est restreinte au spectre utilisable (connaissance du spectre, voir 3.2).
  - Bandes de signal au-delà de 75 MHz (bi_max 200 MHz) : plus de cases, donc plus de fausses alarmes. **Depuis l'étape 2.1**, le modèle est valide jusqu'à 200 MHz et la résolution est physique : 2 m demandent 89 MHz à −25 dB, et les secteurs qui demandent 2 m sont signalés tant que l'espace de recherche s'arrête à 75 MHz.
  - **Renvois de l'étape 2.5** : fouillis évalué par forme d'onde et par pointage (aujourd'hui à la portée exigée, avec la bande admissible la plus étroite, et un rejet Doppler en tout ou rien) ; exigence de portée ramenée à la distance de la surface sur les pointages qui la regardent (aujourd'hui seulement signalé) ; réponse au brouillage d'autoprotection (formes d'onde plus énergétiques). *Fait en P4 (V6.11.6) : fouillis par candidate et par tranche avec rejet Doppler progressif et Pd moyenné sur les vitesses, exigence ramenée à d − garde (10 km), brouilleur d'autoprotection dans la portée (forme exacte) ; la Ku garde le modèle de 2.5 jusqu'à 3.2.*
  - **Renvois de l'étape 2.1** : multiplexage en temps ou en fréquence du multifaisceau (tout le sujet ici) ; impulsions plus courtes que 20 µs pour les secteurs à courte portée (B2_AUTOPROT, 20 km, n'a plus de forme d'onde : 20 µs laissent 3 km aveugles) ; choix de la famille de modulation par l'optimiseur si utile.
  - Règles de l'optimiseur (`bin_deg`, `max_wf`, `short_tol`, `n_alphas`, `short_penalty`) et défaut des égalités exactes : il faut un critère secondaire (ici et en 5.2).
  - La table de coût n'est calculée qu'à 2,8 GHz (optimiste pour les secteurs à 2,7 ou 2,9 GHz) : à recaler à l'étape 5.5. `track_margin_warn` → 5.3.
  - La plus longue impulsion S est déjà **calculée** à partir de la réserve des TRM (`radar_model.tau_max_s`).
  - Le pas des faisceaux de réception (0,5) pourra être réglé par l'optimiseur.
  - **Renvoi de l'étape 3.0** (option 1 validée par l'utilisateur) : le bilan de bruit du ciel d'un secteur (`sector_sky`) est évalué au centre de blocs de la trame (φ, ε) du vaisseau (pas d'environ 0,5 lobe) ; avec un corps chaud ponctuel, il varie de 1 à 4 % selon l'attitude ou la position du corps (artefact de calcul). Correction : **raffinement local** (T_sys calculé à chaque pointage là où la hausse dépasse un seuil, blocs ailleurs) ; ensuite, le test de rotation inclura le Soleil avec une tolérance serrée.
  - **Renvoi de l'étape 2.3** : pas de la grille de veille propre à chaque secteur (1,4 en commun ; l'optimum va de 1,3 à 1,6 selon la Pd par observation).

- **3.2 Confirmation Ku reconstruite** (tracking.py). ✔ (25 octobre 2026 ; les points ci-dessous sont traités, détail au §11 de `Choix_architecturaux.md`, sauf la connaissance du spectre en temps réel, renvoyée à 4.3 et 5.2)
  - **Renvoi de l'étape 1.3** : impulsions Ku au-delà de 400 µs (τ_max Ku = 2,08 ms).
  - **Renvoi de l'étape 3.1 (multifaisceau Ku)** : la grappe KU7 / KU1+7 est aujourd'hui comptée comme 7 observations successives à pleine puissance. Étudier les modes de multifaisceau d'émission de la 3.1 (phase seule, partition, multiplexage en temps ; code indépendant de la bande, table de pertes partagée à confirmer ou spécialiser pour la Ku). Pas de gain d'énergie attendu (même énergie pour un même SNR par case) ; gains en **latence** (une seule observation pour la grappe), pour les observations à énergie en trop (impulsion minimale), et pour plusieurs pistes proches en angle (chronologie, 4.3).
  - Passage de S à Ku, biais S/Ku, étalonnage.
  - Élargir l'espace des observations Ku (`ku_tau`, `ku_n`, `ku_pri`, `ku_sig_bw` : 2 bandes et 3 PRI aujourd'hui) et revoir `margin_db_step` = 0,5 dB, qui crée des sauts.
  - **L'optimiseur doit choisir la fréquence Ku**, comme il le fait déjà pour la S. Aujourd'hui `confirmation.ku_freq_ghz` est fixée à 14,0 GHz, à titre provisoire.
  - **Temps réel** : connaissance du spectre. Il faut mesurer quelles parties des bandes S et Ku sont brouillées ou perturbées et restreindre le choix des fréquences au spectre utilisable (liens avec 2.5 pour le brouillage, 4.3 pour l'ordonnanceur et 5.2 pour l'optimiseur).
  - Le toucher S2 de confirmation est une observation S hors de la règle des rôles S (non vérifiée, sans objet Waveform). Les rôles TWS et PROMOTE existent mais aucune forme d'onde ne les utilise.
  - **Renvoi de l'étape 1.5** : le choix de la fréquence Ku doit éviter l'harmonique 5 des émissions S de la même face, [5(f − B/2), 5(f + B/2)] (règle de fréquence de la matrice d'isolation, signalée par secteur dans `scheduler_rules`).
  - **Renvoi de l'étape 2.5** : portée de percée des brouilleurs d'autoprotection pour les observations Ku (calculée seulement pour la veille S).
  - **Renvoi de l'étape 2.4** : les tables de confirmation gardent le cas de Swerling le plus proche (sans décorrélation partielle ni diversité, que `solve()` compte pour les observations Ku).

- **3.3 Poursuite.**
  - **Rappel à faire à l'utilisateur (décidé en 1.2)** : la polarisation d'émission est linéaire selon e_v ; la commutation e_u / e_v à chaque observation (un seul PA aiguillé vers les deux ports de l'élément, environ 0,1 à 0,2 dB de plus dans TX_chain) reste une **option à évaluer** pour la polarimétrie.
  - Filtre de Kalman ou IMM ; cadence de mise à jour selon la précision et la manœuvre ; initialisation, maintien et perte de piste.
  - **Polarisation** (avec 3.2) : usage polarimétrique des deux voies pour la confirmation, la discrimination des leurres et la classification ; le déséquilibre entre les deux voies (non modélisé, sans effet sur le GLRT) y comptera.
  - **Renvoi de l'étape 2.5** : brouillage trompeur (répéteurs, faux échos) ; effet de la rotation de Faraday sur l'usage polarimétrique.
  - **Renvois de l'étape 2.4** : table d'aspect de la SER (clé réservée, SER frontale seule aujourd'hui) ; accélération supposée connue par la piste sur une observation de poursuite (optimiste) ; discrimination KKV / leurre (fluctuation SW3/4 contre SW0, dépolarisation, retour optique).
  - **Renvoi de l'étape 2.1** : résidu du couplage distance-Doppler (c·τ·σ_fd / 2B) dans la précision en distance.
  - **Renvoi de l'étape 1.5** : atténuateur commutable par observation (ou STC) pour la poursuite rapprochée ; aujourd'hui le gain est fixe et une alerte signale les tranches de poursuite sous la portée non saturée Ku.

- **3.4 Fonctions annexes** (D) : étalonnage en vol, santé des TRM, mode dégradé.
  - **Décision du 25 octobre 2026** : (complétée à l'étape 4.2 : s'y ajoutent les couplages thermiques — pompes dérivées du débit, dérive d'étalonnage reliée à la température, contrôle de dilatation du jeu de 0,5 mm entre TRSA) le MVP est un modèle fonctionnel en situation nominale ; les **modes dégradés** (face perdue, fraction de TRM perdus, puissance réduite, ouverture Ku perdue, TRSA ou chaîne de CAN perdue) et les **pannes de TRM au fil de la mission** sont reportés en fin de développement (après la phase 5).

## Phase 4 : ressources du système

- **4.1 Puissance électrique** (D).
  - **Venu de l'étape 1.3 (option A)** : tout le sujet de l'alimentation des TRM est traité ici : recharge de la réserve pendant un train, capacité d'alimentation continue d'un TRM (propositions de 1 700 W en S et 80 W en Ku pour d = 0,65), et **puissance instantanée de la face** pendant une observation (tous les TRM en même temps : environ 8,2 MW à d = 0,65, contre 2 MW alloués), avec le stockage de face. Aujourd'hui, seule la moyenne sur la période de revisite est vérifiée.
  - Bus de 8 MW, stockage, transitoires.
  - Les 8 MW du vaisseau sont **répartissables à la demande entre les faces**, par exemple 3 MW sur F1 et 5 MW sur F2 à F4. Le logiciel radar doit gérer cette répartition avec les autres paramètres pour tenir la mission (aussi en 5.2). Aujourd'hui, une mission a une seule valeur `face_elec_mw` appliquée aux 4 faces, et la validation vérifie seulement qu'une face ne dépasse pas 8 MW.
  - Remplacer `overhead_frac` = 3 % par un vrai bilan de consommation hors émission (réception numérique et CAN par TRM, environ 130 kW par face, contrôle, refroidissement), avec 4.4.
  - ✔ **Fait le 25 octobre 2026 (V6.16.0)** ; renvois : chronologie réelle des émissions et du stockage (règle d'émission contiguë) → **4.3** ; chiffres de la formation de faisceaux → **4.4** ; pompes → **4.2** ; optimisation conjointe de la répartition entre faces → **5.2**.

- **4.2 Thermique** (D) : dissipation, capacité de refroidissement, constantes de temps, régime permanent face au régime crête.
  - Entrées de l'étape 1.3 : chaleur calculée par TRM et par face (≈ 370 W par TRM S en pointe à d = 0,65) ; capacité des microcanaux à vérifier ; matériaux appariés en dilatation exigés par le jeu de 0,5 mm.

- **4.3 Ordonnanceur** (D). C'est l'étape la plus structurante : il faut décider tôt entre un bilan statistique et une simulation.
  - Modèle des tâches, priorités, entrelacement contraint par les allers-retours, chronologies S et Ku par face, latence, politique de surcharge.
  - **Latence reconstruite de zéro.** L'ancienne règle `confirmation.latency_max_frac` (allers-retours comparés à 1 % du temps avant impact) a été supprimée lors de la revue 0.3 : elle ne limitait jamais rien (2nv/c) et ignorait les vraies causes. Il faut construire une vraie latence (attente dans la file de l'ordonnanceur, traitement, allers-retours) et la comparer à une exigence de délai par secteur.
  - Vérifier `frame_interleaved`.
  - **Renvoi de l'étape 3.1 (P4, brouilleur d'autoprotection), concept de l'utilisateur** : la veille S n'a pas à percer le brouillage d'autoprotection ; qu'elle ne le passe que tard est accepté. Elle sert à **identifier l'axe de brouillage** (le secteur et la direction où le bruit du brouilleur apparaît) ; l'ordonnanceur pointe alors une **recherche Ku ciblée dans cet axe** pour la percée (burn-through). Ordre de grandeur : à énergie égale, gain d'émission Ku environ 25 fois celui de la S, donc une portée de percée environ 5 fois plus grande (R_bt ∝ √(P G_t σ τ N / (J₀ SNR L)), densité du brouilleur égale dans les deux bandes pour SPJ_KKV). À modéliser : détection et localisation angulaire du brouilleur par la veille (rapport J/N, sans percée), déclenchement et coût de la recherche Ku de percée dans le budget de la face (avec 3.2 pour la forme d'onde Ku et 3.3 pour la piste).
  - **Renvoi de l'étape 3.1 (P3, multifaisceau S)** : réévaluer l'entrée du multiplexage en temps (et de la superposition) dans `tool.optimizer.tx_modes` une fois le coût de chronologie modélisé. Sans ce terme, le multiplexage est exactement dominé : mêmes énergie et temps d'émission par position qu'un faisceau, avec une zone aveugle, un rapport cyclique et un affaissement de la réserve sur B·τ. Avec lui (entrelacement imparfait, attente des échos partagée par B positions, occupation du séquenceur), il peut diviser la trame par B. Le modèle est prêt (`tx_mode` = "tdm") ; il reste à ajouter le terme de coût et le mode à `tx_modes`.
  - **Renvoi de l'étape 2.5** : choix du CFAR (CA, OS, GO) par pointage, GO près des limbes.
  - **Renvoi de l'étape 3.1 (P5)** : ce que le calculateur radar recalcule en vol (carte du ciel et aveuglement quand l'attitude change, formes d'onde), à quelle cadence ; le budget de calcul est à l'étape 4.4.
  - **Renvoi de l'étape 2.3** : décalage de la grille de veille à chaque trame (`grid_dithering`), supposé par le critère cumulé.
  - **Renvois de l'étape 1.5** : (1) **cadence commune d'émission et de réception de toutes les faces, par bande** (`isolation.interface_mode` = sync) : aucune face ne reçoit une bande pendant qu'une autre l'émet ; coût estimé de 0 à 1 dB si les émissions sont alignées, jusqu'à −10 log(1 − Σ des facteurs d'émission des autres faces) sinon (0,5 à 1,5 dB en forte charge), aucune perte comptée avant cette étape ; (2) pas de réception Ku sur une face pendant une émission S dont l'harmonique couvre le canal Ku ; (3) **annulation active à revoir ici** (coûts : perte avant le LNA versée dans T_sys, puissance injectée égale à la fuite, temps d'adaptation à chaque pointage, calcul dans l'espace des faisceaux ; solution mixte proposée : synchronisation entre faces voisines, annulation entre faces opposées) ; (4) pointages aveuglés par le Soleil à sortir du plan de balayage ; (5) **renvoi de l'étape 2.1** : éclipsage par les autres observations (positions entrelacées) et par les autres faces synchronisées.

- **4.4 Calcul et données** (D).
  - Débits des CAN, faisceaux de réception numériques simultanés (dont les quelque 3,4 faisceaux par pointage de l'étape 1.1), charge de traitement, capacité du calculateur central.
  - **Entrée de l'étape 2.1** : nombre de filtres de vitesse par observation (`velocity_hypotheses_1db`).
  - **Entrées des étapes 2.4 et 2.5** : hypothèses d'accélération (elles multiplient celles de vitesse) ; annulation adaptative des brouilleurs (8 par face).
  - **Entrées de l'étape 2.2** : hypothèses de vitesse par forme d'onde (`velocity_hypotheses`, jusqu'à plusieurs centaines), grilles de détection ×4 en distance et en Doppler, transformées par bloc cohérent. **Capacité du formateur de faisceaux** (faisceaux simultanés par face, débit) à fixer ici comme ressource et à comparer au besoin calculé par l'ordonnanceur (4.3) : question de l'utilisateur à l'étape 2.2.
  - **Entrées de l'étape 1.5** : deux CAN I/Q par TRM (deux voies de polarisation) à 250 Méch/s et 13 bits, soit 9 800 CAN en S et 245 000 en Ku par face, et une formation de faisceau doublée ; leur consommation est à budgéter.
  - Leur consommation électrique alimente le bilan de puissance de l'étape 4.1, à la place de `overhead_frac`.
  - **Renvoi de l'étape 3.1 (P5)** : aujourd'hui, l'optimiseur de veille (ciel, aveuglement, formes d'onde) ne tourne qu'en préparation de mission (Pmiss) ; son coût n'est pas celui du calculateur radar. Décider ce que le calculateur recalcule en vol (attitude qui change, spectre mesuré en temps réel) et avec quel budget de calcul (avec 4.3 et 5.2). La part liée à l'attitude est traitée en 4.5.

- **4.5 Re-pointage en vol** (D) (ajoutée le 25 octobre 2026, à la demande de l'utilisateur, après le point sur le maintien des pointages quand la géométrie change). Constat : `core/frames.py` (étape 3.0) tourne chaque volume dans le repère du vaisseau **une seule fois**, pour un instantané de navigation ; aucune partie du modèle ne maintient les pointages quand l'attitude change ou que le vaisseau avance sur son orbite. Cette étape définit la boucle de re-pointage du calculateur radar :
  - cadence des messages de navigation et réapplication de la rotation aux volumes (`inertial`, `rtn`, `limb`, `two_dir`, `track`) et au ciel inertiel ;
  - veille horizon (`limb`) : suivi du limbe quand l'altitude et l'attitude changent ;
  - rotation admissible pendant une trame (déformation de la couverture : les pointages du début et de la fin de trame ne sont pas pris dans la même attitude), recalcul de la grille de pointages (étape 4.3 bis) et de l'ordre de balayage ;
  - migration d'un secteur d'une face à l'autre (réaffectation, perte de balayage, budget de chaque face) ;
  - poursuite : compensation de la rotation entre la prédiction du filtre et l'émission, cadence de mise à jour de l'attitude, effet de l'erreur de connaissance d'attitude sur la perte de pointage (étape 3.3) ;
  - coût de calcul de ce recalcul en vol, sur le budget de l'étape 4.4 ;
  - **renvois regroupés ici** : profil d'attitude dans le temps et réaffectation des secteurs aux faces en vol (étapes 3.0, 4.3, 5.2) ; part « attitude » du renvoi P5 de l'étape 3.1. Restent ailleurs : enveloppe d'attitudes au pire cas en préparation de mission (5.3), parallaxe et erreurs de navigation (simulateur global).

## Phase 5 : mission et validation

- **5.1 Modèle de mission enrichi.**
  - **Renvoi de l'étape 3.3** : dispersion de SER par classe dans la bibliothèque (la matrice de séparabilité donne aujourd'hui P ≈ 1 entre toutes les classes, moyennes sans dispersion) ; `tracking.pd_update` et `tracking.p_discrimination` comme réglages de mission.
  - Menaces, priorités, phases temporelles et modes, avec des fichiers qui restent compatibles.
  - **Volumes par défaut** (idée de l'utilisateur, étape 2.3) : des volumes types prêts à l'emploi pour les besoins récurrents, par exemple l'autoprotection du vaisseau contre un KKV ; leurs paramètres standards (portée, revisite, etc.) seront discutés à ce moment-là, pas pendant la construction de l'outil.
  - **Bibliothèque de menaces** : l'opérateur choisit un type de cible par secteur au lieu de ressaisir SER, Swerling et vitesse. **Depuis l'étape 2.4**, la bibliothèque existe (`[targets.*]`, clé de secteur `target_type`) et entre dans le bilan ; reste l'interface : liste des classes, affichage des valeurs, surcharges (aujourd'hui l'interface conserve la clé, et une SER ou un Swerling modifié devient une surcharge).
  - **Renvoi de l'étape 2.2 (sujet des fausses alarmes, décision de l'utilisateur : 2.3, 5.1, 5.2)** : réglage de la politique de fausses alarmes dans la mission (taux toléré par secteur ou par période).
  - **Renvois de l'étape 2.3** : Pd cumulée et taux de fausses alarmes **par secteur** ; **plafond du nombre d'observations** pour une cible surgissante (sortie d'horizon, lancement), aujourd'hui sans plafond (bout optimiste) ; la Pd cumulée suppose que la cible est présente sur toute son approche.
  - `confirmation.p_min` (0,98) comme réglage de mission, avec en **option à étudier** un réglage par volume (secteur).

- **5.2 Optimiseur global.** Répartition conjointe de la veille, de la confirmation et de la poursuite sur toutes les ressources et toutes les faces, **y compris la répartition à la demande des 8 MW du vaisseau entre les faces** (voir 4.1). **Renvoi de l'étape 2.2** : l'optimiseur doit compter le **coût de confirmation des fausses alarmes** dans le choix des formes d'onde de veille (les bancs de vitesses des trains multiplient les fausses alarmes par 50 à 535). **Renvoi de l'étape 3.1 (P5)** : répartir le calcul entre la préparation de mission (Pmiss) et le vol (replanification en vol, avec 4.3 et 4.4).

- **5.3 Moteur de validation.** Verdict par contrainte, ressource limitante, conseil chiffré de relâchement, redondances et recouvrements.
  - **Renvoi de l'étape 2.4** : les classes de cibles étant des moyennes, **sensibilité du verdict aux valeurs de la menace** (de combien la SER et les autres valeurs peuvent baisser avant que le secteur ne tienne plus).

- **5.4 Simulation Monte-Carlo à événements discrets.** C'est l'arbitre final du bilan statistique.
  - **Note de l'utilisateur (25 octobre 2026, étape 3.2)** : hors préparation de mission, le simulateur global de scénario demandera probablement des **approches simplifiées (moins précises)** que la Pmiss, pour garder un calcul en temps réel ou quasi réel. La règle « jamais de précision perdue pour le temps de calcul » vaut pour la Pmiss ; les modèles simplifiés du simulateur seront à définir (avec 4.3 et 4.4 pour ce que le calculateur recalcule en vol) et à recaler sur le modèle complet.
  - **Renvoi de l'étape 2.5** : réflexions sur la structure du vaisseau (Soleil, Terre, autres faces).
  - **Renvoi de l'étape 2.4** : effet réel du masquage par les cibles voisines (lobes Doppler et en distance, bruit de phase) et du CFAR (CA ou OS) sur une scène dense.

- **5.5 Interface opérateur.**
  - Préparation, estimation rapide recalée, diagnostic par contrainte, chronologies, export du rapport.
  - Onglet d'édition des paramètres.
  - Arrondi de `pulse_scale` (était `power_scale`).
  - **Renvois de l'étape 2.1** : les vues (3D, interface) n'affichent que le **volume utile**, sans la zone aveugle (demande de l'utilisateur) ; édition de `range_min_km` dans l'interface.
  - **Renvoi de l'étape 2.3** : l'estimation rapide reste au critère d'une observation à `detection.pfa` ; à recaler sur la Pd cumulée et le taux de fausses alarmes.
  - **Renvoi de l'étape 3.3** : l'estimation rapide garde l'ancienne observation de poursuite (accélération connue, sans loi de pointage, sans initialisation ni perte de piste) ; à recaler sur le modèle de piste.
  - **Renvoi de l'étape 1.5** : l'estimation rapide (`cost_table.json`) est au ciel froid, sans le bruit du ciel ni l'aveuglement ; l'interface n'édite pas le ciel de la mission (il viendra du simulateur de scénario), elle le conserve.
  - **Renvoi de l'étape 2.4** : la table de coût est calculée sur GENERIC en Swerling 1 ; un secteur à classe y entre par sa SER frontale en S.

- **5.6 Vérification, validation et documentation.** Cas de référence figés, traçabilité des paramètres, manuel de l'opérateur, note de modèle.

- **5.7 Fin de développement de la Pmiss** (ajoutée le 25 octobre 2026). Les éléments reportés après le MVP nominal (décision de l'étape 3.4, complétée en 4.2) :
  - **modes dégradés** : face perdue, fraction de TRM perdus, puissance réduite, ouverture Ku perdue, TRSA ou chaîne de CAN perdue ;
  - **pannes de TRM au fil de la mission** ;
  - **couplages thermiques** : pompes dérivées du débit, dérive d'étalonnage reliée à la température, contrôle de dilatation du jeu de 0,5 mm entre TRSA ;
  - les points ouverts encore « à placer » en fin de feuille de route, à trancher un par un (faits, reportés en phase 6 ou abandonnés) ;
  - **renvoi de l'étape 4.4** : coût d'une resélection de forme d'onde en vol (`compute.reselect_ops_per_entry`, 10³ opérations, D) à revoir.

  **Critère de fin de la Pmiss** (décision de l'utilisateur, 25 octobre 2026) : phase 5 entière, 5.7 comprise. La Pmiss est finie **avant** de commencer la phase 6, pour que tout son code serve à construire le simulateur global.

## Phase 6 : simulateur global de scénario

Commence quand la Pmiss est finie (critère de l'étape 5.7). Cadrage à faire le moment venu ; ce qui est déjà acquis :
  - **interface fixée depuis l'étape 3.0** : le simulateur fournit les messages de navigation (attitude, orbite, corps central) et la position des objets dans le repère du vaisseau ; le radar ne calcule jamais où sont les objets ; parallaxe et erreurs de navigation sont de son ressort ;
  - le radar en vol exécute ce que définissent les étapes 4.3 (ordonnanceur), 4.3 bis (ordre de balayage), 4.4 (calcul en vol) et 4.5 (re-pointage en vol) ;
  - le moteur à événements et les modèles simplifiés recalés sur la Pmiss partent de l'étape 5.4 (note de l'utilisateur de l'étape 3.2 : temps réel ou quasi réel, précision moindre que la Pmiss) ;
  - y trouvent leur place : profil d'attitude dans le temps, aspect réel des cibles par observation (variabilité de la SER), pannes au fil de la mission vues dans le temps.

## Jalons

| Jalon | Contenu |
|---|---|
| A | phases 0 à 2 |
| B | phase 3 |
| C | étapes 4.1 à 4.5 |
| D | étapes 5.1 à 5.6 (Pmiss en situation nominale, MVP) |
| Pmiss finie | étape 5.7 (fin de développement) : critère de passage à la phase 6 |
| 6 | phase 6 : simulateur global de scénario |

## Points ouverts à placer dans les étapes

| Point | Étape |
|---|---|
| Égalités exactes de l'optimiseur départagées par le bruit d'arrondi (candidats cohérents au plancher de cohérence) : il faut un critère secondaire. | 3.1 (ou plus tôt si l'utilisateur le souhaite) |
| Le nombre de faces (4) est écrit en dur dans le code. | à placer |
| **Révision de la dalle** (22 octobre 2026) : la composition (`[aperture]`) est entièrement dérivée par le code, dans la famille actuelle (TRSA carrées d'un seul type, rangées centrées et symétriques, ouverture S / Ku partagée). À faire lors d'une révision réelle (décision de l'utilisateur) : contrôle au chargement que la dalle tient sur le panneau du modèle 3D (aujourd'hui seul le centre est contrôlé) et test de bout en bout d'une composition modifiée ; revoir alors les valeurs par TRM et par face (puissance, réserve, allocation électrique, masse). | à la révision de la dalle |
| Densité de puissance d'environ 104 kW/m² de RF moyenne. | 4.2 |
| Masse des réserves d'énergie des TRM : environ 270 kg par face à 1 J/g (hypothèse 2050), 1,1 t pour les 4 faces. | à placer (pas de bilan de masse dans la feuille de route) |
| `pulse_scale` (était `power_scale`) arrondi à 3 décimales dans la sortie de l'optimiseur. | 5.5 |
| `rx_taper_edge` = "inscribed" (statut D) à confirmer. | à placer |
| **Lobes de jeu des TRSA** (étape 1.2) : traité en grande partie en ramenant le jeu à 0,5 mm. Restent en Ku au haut de la bande −30 dB tenu jusqu'à 64-68,5° et −35 dB jusqu'à 55-62°. Variante si la mécanique impose plus de jeu : maille d'éléments uniforme à travers les jeux. | à placer (2.5 faite : les lobes lointains entrent déjà dans le bruit de structure et le fouillis) |
| Inclinaison des faces et géométrie : changer `geometry.tilt_deg` demande de modifier aussi le cône porte-antennes et `ship.array_radius_m` (étape 2.5 : la géométrie n'est plus déduite de l'inclinaison). | à placer |
| Matériaux appariés en dilatation, exigés par le jeu de 0,5 mm (à confirmer en mécanique). | 4.2 |
| Limite de 70° au-dessus de la fréquence de rupture : gardée à l'étape 1.4 tant que le diagramme de l'élément n'est pas modélisé. | avec le diagramme de l'élément (à placer) |
| Couplage mutuel au bord des tuiles et diagramme réel de l'élément rayonnant non modélisés (l'intégrale du bruit du ciel prend un élément en cos θ, rien vers l'arrière, depuis l'étape 1.5). | à placer |
| Couplages entre faces (90 et 120 dB) et dans une TRSA (−30 dB) de l'étape 1.5 : ordres de grandeur au bout optimiste, une simulation électromagnétique serait nécessaire. | à placer (avec 4.3) |
| Erreurs aléatoires d'amplitude et de phase des TRM non modélisées (environ 0,2 % de la puissance hors du lobe pour 0,2 dB et 2° : négligeable pour le bruit du ciel). | à placer |
| **Variabilité de la SER par classe** (question de l'utilisateur, 25 octobre 2026, après l'étape 3.3 ; reportée à sa demande, l'architecture du radar passe avant) : tables SER(aspect) par classe (propositions : KKV 0 / +2 / +5 / +10 / +15 dB à 0 / 15 / 30 / 60 / 90° ; DECOY plate (ballon) ; SHIP 0 à +10 dB ; DEBRIS, GENERIC, ST plates), plage d'aspect pendant le scénario (KKV et DECOY ±10°, SHIP ±30°, DEBRIS ±90°, GENERIC ±20°), dispersion d'un objet à l'autre (SER 3 à 5 dB, dépolarisation 2 dB, 1 σ), retour optique du KKV limité à un cône de ±5° ; effets : coût sur la SER moyenne de la plage, variance d'aspect (décroît avec les observations) et dispersion (systématique) dans les indices de discrimination, ce qui corrige la séparabilité aujourd'hui ≈ 1. Aspect réel par observation fourni plus tard par le simulateur global (5.4). Clés prévues : `aspect_span_deg`, `rcs_spread_db`, `depol_spread_db`, `optics_cone_deg`. | à placer (avec 5.1 pour la bibliothèque, 5.4 pour l'aspect dans le temps) |
