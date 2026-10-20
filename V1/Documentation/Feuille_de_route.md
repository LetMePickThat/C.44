# Feuille de route

Cette feuille de route mène de l'outil actuel à un **outil complet de préparation de mission**. L'opérateur y prépare sa mission, et celle-ci est validée contre **toutes** les contraintes et ressources du système radar : radiofréquence, ordonnancement, puissance, thermique, calcul, etc., et pas seulement le bilan de liaison.

L'utilisateur l'a acceptée le 20 octobre 2026. Elle se suit dans l'ordre, une étape à la fois, de session en session, avec un commit par étape, fait seulement sur demande.

Règles appliquées à chaque étape :
- rappeler la raison d'être de l'étape, ses entrées et ses sorties, puis obtenir l'accord avant de coder ;
- demander à l'utilisateur la valeur de **chaque** variable, sans jamais en choisir une en silence ;
- garder toutes les variables système dans le fichier unique `radar_system.toml` ;
- ne jamais présenter les missions de test et les valeurs figées comme des références ou des performances.

(D) signale une étape qui demande une décision ou des données de l'utilisateur. Le détail des choix déjà faits est dans `Choix_architecturaux.md`.

## État d'avancement (20 octobre 2026)

| Étape | État |
|---|---|
| 0.1 | ✔ fait (commit 90a5fc4) |
| 0.2 | ✔ fait (commit c776688) |
| 0.3 | ✔ fait (commits f205a23 et 7f6bd35) |
| 1.1 | ✔ codé et testé, pas encore commité |
| 1.2 et suivantes | à faire, la prochaine étant 1.2 |

---

## Phase 0 : fondations

- **0.1 Dépôt de paramètres unique et versionné.** Les valeurs sont chargées depuis un fichier, avec leur statut ([V], [D] ou [?]) et leur source. Plus aucune modification de `globals()`, et un seul chargement de `radar_model`. ✔
- **0.2 Banc de tests pytest.** Il reprend le self-test actuel et les missions de test (mission_example, mission_veille500, mission_redundancy_demo, synthétique). Il compare les valeurs figées avec une tolérance relative de 1e-9, vérifie des invariants physiques et produit un rapport d'impact à chaque étape. Ces valeurs ne sont pas des références. ✔
- **0.3 Revue de toutes les valeurs [?]** (D). ✔

## Phase 1 : antenne et RF

- **1.1 Géométrie cohérente de l'antenne.** ✔ (pas encore commité)
  - Calculer le **vrai diagramme** à partir de la composition des TRSA (rangées de l'octogone, jeux, pas de 52 mm en S et 10,4 mm en Ku), au lieu de HPBW = 0,886 λ/D_env, qui est la valeur d'une ouverture rectangulaire.
  - Séparer le faisceau d'émission, uniforme et qui sert à la grille de veille, du faisceau de réception, défini par son niveau de lobes secondaires, dont on déduit `eta_illum_rx`.
  - Largeur de lobe sur deux axes en (u, v).
  - (D) Ouverture Ku partagée ou séparée.

  Décisions prises :
  - précision angulaire par la borne de Cramér-Rao × 1,13 ;
  - lobes de réception à −25, −30 ou −35 dB, réglés par volume, −25 par défaut ;
  - grappe de faisceaux de réception au pas de 0,5 largeur de lobe ;
  - ouverture Ku partagée.

- **1.2 Diagramme de rayonnement.**
  - Vrai lobe principal issu de la pondération, lobes secondaires, lobes de réseau, perte physique au dépointage (réflexion active).
  - **Doit apporter un modèle unique** de la désadaptation active |Γ(θ, f)|, pour S et Ku : WAIM, approche des lobes de réseau en haut de bande. Ce modèle doit expliquer la limite de 70° au-dessus de 2,85 GHz.
  - Depuis la revue 0.3, le gain ne suit que la projection cos θ : l'exposant 1,1 de V5.6.0 et la loi de perte au dépointage de V5.4.0 (0,6 dB à 70°) ont été retirés, car ils comptaient la même perte deux fois. Les résultats sont donc **optimistes** aux grands dépointages tant que cette étape n'est pas faite : 1,53 dB aller-retour à 70° (portée +9,2 %), 2,0 dB à 75° (+12,2 %).
  - **Polarisation** (sujet transverse) :
    - décider la polarisation d'émission (linéaire, et laquelle, ou circulaire) ;
    - modéliser la pureté de polarisation de l'antenne (rapport axial, polarisation croisée) en fonction du dépointage et de la fréquence, avec le WAIM.

    Ce modèle remplacera la perte fixe `pol_loss_dual_db` de 0,5 dB (S et Ku).
  - Marge de lobe de réseau de 1,1 % à 2,9 GHz pour 75°.
  - Lobes du réseau périodique de jeux entre TRSA (hors de la fenêtre de 8 λ/D évaluée à l'étape 1.1).

- **1.3 TRM et chaîne d'émission S et Ku.**
  - PAE en fonction de l'impulsion et du facteur de forme, facteur de forme maximal, impulsion maximale, réserve d'énergie dans les deux bandes, dissipation.
  - (D) Données des TRM Ku.
  - **Doit remplacer** la constante `power.pae` = 0,75 (commune à S et Ku) par une **PAE variable** par bande, en fonction de la longueur d'impulsion et du facteur de forme.
  - Revoir `e_buf_trm_j` (5 J, statut ?), `buffer_max_fraction` et `duty_max`.
  - Densité de puissance d'environ 104 kW/m² de RF moyenne (avec l'étape 4.2).
  - Recul du PA en multifaisceau par superposition (pour l'étape 3.1).

- **1.4 Bande.**
  - Table de dégradation aux bords (rf_layer_s), propre à chaque bande, au lieu de la fraction de bande copiée de l'ancienne bande Ku.
  - Bande instantanée en S (aujourd'hui 200 MHz, « pour l'instant »).
  - Retard vrai.

- **1.5 Réception.**
  - T_sys selon la direction (Terre, Lune, Soleil, plan galactique).
  - LNA ; dynamique et saturation du CAN.
  - Isolation entre S et Ku et entre faces.
  - Modèle du bruit de phase de l'oscillateur : spectre et effet sur le temps d'aller-retour (sert à l'étape 2.2).
  - **Polarisation** : deux voies de polarisation en réception par TRM et leur recombinaison (cohérente ou optimale), avec le gain de diversité de polarisation face à la perte.

## Phase 2 : signal et détection

- **2.1 Formes d'onde paramétriques.**
  - Modulation, compression, lobes en distance, tolérance au Doppler, couplage distance-Doppler.
  - Éclipsage, ambiguïté en distance et en Doppler, PRF échelonnée.
  - Pondération de Taylor déduite de la modulation réelle (pour k_r).
  - Bandes de signal au-delà de 75 MHz.
  - Multiplexage en temps ou en fréquence pour le multifaisceau.

- **2.2 Traitement.**
  - Migration traitée par bancs d'hypothèses de vitesse : perte en fonction du nombre de cases traversées, au lieu d'un forfait de 1 dB.
  - Chevauchement de cases, CFAR, nombre réel de cases.
  - **Doit aussi corriger le modèle de cohérence.** Le plancher η_min = 0,70 est atteint vers 149 ms, puis fige la perte à 1,55 dB quelle que soit la durée d'intégration : ce n'est pas physique, et c'est la cause des égalités exactes dans l'optimiseur. Il faut plafonner le temps cohérent et intégrer de façon non cohérente au-delà.
  - t₀ = 0,25 s et p = 2 n'ont **aucune source connue** et sont à établir.
  - **Disposition provisoire** depuis la revue 0.3 : `radar_model.COH_T_MAX_S` = t₀(−ln η_min)^(1/p) = 149 ms. Les générateurs de formes d'onde de l'optimiseur et de la poursuite écartent les intégrations cohérentes plus longues. À retirer quand les blocs cohérents suivis d'une intégration non cohérente seront modélisés.
  - Bruit de phase de l'oscillateur sur l'aller-retour (53 ms à 8 000 km, décalages d'environ 19 Hz). Il décorrèle l'émission et la réception (lobes Doppler, cohérence) ; ce n'est pas seulement une gigue à court terme (voir 1.5).

- **2.3 Critère de détection** (D).
  - Pd moyenne ou au pire point de la cellule de la grille.
  - Pd cumulée sur plusieurs trames pour une cible qui se rapproche.
  - Revoir `beam_spacing_hpbw` et `enable_grid_loss`.

- **2.4 Modèles de cibles** (D).
  - SER selon l'aspect et la bande, cas de Swerling par bande, cinématique et manœuvre.
  - Un **temps de cohérence T₀ par type de cible** (rotation, accélération, décorrélation de scintillation), au lieu du `coherence.t0_s` = 0,25 s global.
  - **Polarisation** : matrice de diffusion par type de cible (SER copolaire et croisée, dépolarisation selon l'aspect) et effet de la diversité de polarisation sur le cas de Swerling (perte de fluctuation).

- **2.5 Environnement.**
  - Bruit de fond.
  - **Masquage des champs de vue de l'antenne par le vaisseau** (ailes de radiateurs de 76,5 m, coque), d'après la géométrie `[ship]`, pour le calcul de la couverture. Aujourd'hui, seul le plancher arrière ε ≈ −55° est modélisé.
  - (D) Débris, fouillis lunaire ou terrestre, brouillage ?
  - **Rotation de Faraday** (gardée par l'utilisateur), seulement pour les lignes de visée qui traversent l'ionosphère ou la plasmasphère :
    - orbite terrestre basse : plusieurs radians en S dans l'ionosphère ;
    - plasmasphère : environ 0,3 rad à 2,8 GHz sur 2 × 8 000 km ;
    - interplanétaire : environ 1e-12 rad, négligeable ;
    - négligeable en Ku et en polarisation circulaire.

    Elle dépend des positions du vaisseau et de la cible.

## Phase 3 : fonctions radar

- **3.1 Veille S reconstruite sur les nouveaux modèles** (optimizer.py).
  - **Multifaisceau en émission** : tous les modes de formation sont possibles, et l'optimiseur doit les évaluer et choisir selon le contexte :
    - (a) superposition de B faisceaux par TRM : enveloppe non constante, donc recul du PA, qui demande le modèle de PA de l'étape 1.3 ;
    - (b) partition de l'ouverture : gain divisé par B, faisceau plus large, d'après le diagramme de l'étape 1.1 ;
    - (c) multiplexage en temps ou en fréquence dans l'impulsion : énergie par faisceau, résolution, Doppler (étape 2.1).

    Cela remplace la loi unique `multibeam.l8_db` / `p` (statut ?). À reprendre aussi à l'étape 5.2.
  - **Grille de fréquences S** : remplacer `freq_candidates_ghz` (2,7, 2,8 et 2,9 GHz) par une grille fine au pas de 5 ou 10 MHz. Le but est d'éviter les brouilleurs étroits de forte puissance tout en gardant plus de cases fréquence qu'un masquage par multiples de 25 MHz. La grille est restreinte au spectre utilisable (connaissance du spectre, voir 3.2).
  - Bandes de signal au-delà de 75 MHz (bi_max 200 MHz) : plus de cases, donc plus de fausses alarmes (avec 2.1).
  - Règles de l'optimiseur (`bin_deg`, `max_wf`, `short_tol`, `n_alphas`, `short_penalty`) et défaut des égalités exactes : il faut un critère secondaire (ici et en 5.2).
  - La table de coût n'est calculée qu'à 2,8 GHz (optimiste pour les secteurs à 2,7 ou 2,9 GHz) : à recaler à l'étape 5.5. `track_margin_warn` → 5.3.
  - La plus longue impulsion S est déjà **calculée** à partir de la réserve des TRM (`radar_model.tau_max_s`).
  - Le pas des faisceaux de réception (0,5) pourra être réglé par l'optimiseur.

- **3.2 Confirmation Ku reconstruite** (tracking.py).
  - Passage de S à Ku, biais S/Ku, étalonnage.
  - Élargir l'espace des observations Ku (`ku_tau`, `ku_n`, `ku_pri`, `ku_sig_bw` : 2 bandes et 3 PRI aujourd'hui) et revoir `margin_db_step` = 0,5 dB, qui crée des sauts.
  - **L'optimiseur doit choisir la fréquence Ku**, comme il le fait déjà pour la S. Aujourd'hui `confirmation.ku_freq_ghz` est fixée à 14,0 GHz, à titre provisoire.
  - **Temps réel** : connaissance du spectre. Il faut mesurer quelles parties des bandes S et Ku sont brouillées ou perturbées et restreindre le choix des fréquences au spectre utilisable (liens avec 2.5 pour le brouillage, 4.3 pour l'ordonnanceur et 5.2 pour l'optimiseur).
  - Le toucher S2 de confirmation est une observation S hors de la règle des rôles S (non vérifiée, sans objet Waveform). Les rôles TWS et PROMOTE existent mais aucune forme d'onde ne les utilise.

- **3.3 Poursuite.**
  - Filtre de Kalman ou IMM ; cadence de mise à jour selon la précision et la manœuvre ; initialisation, maintien et perte de piste.
  - **Polarisation** (avec 3.2) : usage polarimétrique des deux voies pour la confirmation, la discrimination des leurres et la classification.

- **3.4 Fonctions annexes** (D) : étalonnage en vol, santé des TRM, mode dégradé.

## Phase 4 : ressources du système

- **4.1 Puissance électrique** (D).
  - Bus de 8 MW, stockage, transitoires.
  - Les 8 MW du vaisseau sont **répartissables à la demande entre les faces**, par exemple 3 MW sur F1 et 5 MW sur F2 à F4. Le logiciel radar doit gérer cette répartition avec les autres paramètres pour tenir la mission (aussi en 5.2). Aujourd'hui, une mission a une seule valeur `face_elec_mw` appliquée aux 4 faces, et la validation vérifie seulement qu'une face ne dépasse pas 8 MW.
  - Remplacer `overhead_frac` = 3 % par un vrai bilan de consommation hors émission (réception numérique et CAN par TRM, environ 130 kW par face, contrôle, refroidissement), avec 4.4.

- **4.2 Thermique** (D) : dissipation, capacité de refroidissement, constantes de temps, régime permanent face au régime crête.

- **4.3 Ordonnanceur** (D). C'est l'étape la plus structurante : il faut décider tôt entre un bilan statistique et une simulation.
  - Modèle des tâches, priorités, entrelacement contraint par les allers-retours, chronologies S et Ku par face, latence, politique de surcharge.
  - **Latence reconstruite de zéro.** L'ancienne règle `confirmation.latency_max_frac` (allers-retours comparés à 1 % du temps avant impact) a été supprimée lors de la revue 0.3 : elle ne limitait jamais rien (2nv/c) et ignorait les vraies causes. Il faut construire une vraie latence (attente dans la file de l'ordonnanceur, traitement, allers-retours) et la comparer à une exigence de délai par secteur.
  - Vérifier `frame_interleaved`.

- **4.4 Calcul et données** (D).
  - Débits des CAN, faisceaux de réception numériques simultanés (dont les quelque 3,4 faisceaux par pointage de l'étape 1.1), charge de traitement, capacité du calculateur central.
  - Leur consommation électrique alimente le bilan de puissance de l'étape 4.1, à la place de `overhead_frac`.

## Phase 5 : mission et validation

- **5.1 Modèle de mission enrichi.**
  - Menaces, priorités, phases temporelles et modes, avec des fichiers qui restent compatibles.
  - **Bibliothèque de menaces** : l'opérateur choisit un type de cible par secteur au lieu de ressaisir SER, Swerling et vitesse. Aujourd'hui, `[targets.*]` ne sert qu'au rapport en ligne de commande et aux tests.
  - `confirmation.p_min` (0,98) comme réglage de mission, avec en **option à étudier** un réglage par volume (secteur).

- **5.2 Optimiseur global.** Répartition conjointe de la veille, de la confirmation et de la poursuite sur toutes les ressources et toutes les faces, **y compris la répartition à la demande des 8 MW du vaisseau entre les faces** (voir 4.1).

- **5.3 Moteur de validation.** Verdict par contrainte, ressource limitante, conseil chiffré de relâchement, redondances et recouvrements.

- **5.4 Simulation Monte-Carlo à événements discrets.** C'est l'arbitre final du bilan statistique.

- **5.5 Interface opérateur.**
  - Préparation, estimation rapide recalée, diagnostic par contrainte, chronologies, export du rapport.
  - Onglet d'édition des paramètres.
  - Arrondi de `power_scale`.

- **5.6 Vérification, validation et documentation.** Cas de référence figés, traçabilité des paramètres, manuel de l'opérateur, note de modèle.

## Jalons

| Jalon | Contenu |
|---|---|
| A | phases 0 à 2 |
| B | phase 3 |
| C | étapes 4.1 à 4.3 |
| D | phase 5 |

## Points ouverts à placer dans les étapes

| Point | Étape |
|---|---|
| Égalités exactes de l'optimiseur départagées par le bruit d'arrondi (candidats cohérents au plancher de cohérence) : il faut un critère secondaire. | 3.1 (ou plus tôt si l'utilisateur le souhaite) |
| Le nombre de faces (4) est écrit en dur dans le code. | à placer |
| Marge de lobe de réseau S de 1,1 % à 2,9 GHz et 75°, qui explique les 70° au-dessus de 2,85 GHz. | 1.2 |
| Densité de puissance d'environ 104 kW/m² de RF moyenne. | 1.3, 4.2 |
| `power_scale` arrondi à 3 décimales dans la sortie de l'optimiseur. | 5.5 |
| `rx_taper_edge` = "inscribed" (statut D) à confirmer. | 1.1 / 1.2 |
