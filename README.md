# surface_reconstruction

Nodo 2 della pipeline descritta in [pcl-surface-reconstruction-pipeline.md](pcl-surface-reconstruction-pipeline.md):
**unione di più acquisizioni e ricostruzione della superficie**.

Il nodo `cloud_merger_node` riceve la point cloud filtrata dal nodo 1 e, a ogni chiamata di `acquire`, la porta nel frame comune del robot con la posa della camera letta da TF (cinematica diretta + calibrazione mano-occhio). Le acquisizioni vengono tenute in una lista. Con `compute_surface` vengono unite e da esse si ricostruisce una mesh, che viene pubblicata per Rviz e per il nodo 3 (generazione del percorso).

Per una guida passo-passo, anche senza robot né camera, vedi il **[tutorial](docs/TUTORIAL.md)**.

## Requisiti

- Ubuntu 24.04, ROS 2 Jazzy
- PCL 1.14 (`libpcl-dev`), `ros-jazzy-pcl-conversions`, `ros-jazzy-tf2-eigen`
- Pacchetto `surface_reconstruction_msgs` (servizio `SaveMesh`), in un repository separato: va clonato nello stesso workspace

```bash
cd ~/projects/ros2_ws/src
git clone <url>/surface_reconstruction.git
git clone <url>/surface_reconstruction_msgs.git
cd ..
rosdep install --from-paths src/surface_reconstruction src/surface_reconstruction_msgs --ignore-src -y
colcon build --packages-up-to surface_reconstruction
source install/setup.bash
```

## Avvio

```bash
ros2 launch surface_reconstruction cloud_merger.launch.py \
    input_cloud:=/<topic_del_nodo_1> \
    params_file:=/percorso/mio_config.yaml   # opzionale
```

| Argomento del launch | Default | Descrizione |
|---|---|---|
| `input_cloud` | `filtered_cloud` | topic della cloud filtrata pubblicata dal nodo 1 |
| `params_file` | `config/cloud_merger.yaml` | file dei parametri |

## Pipeline

```
acquire (per ogni posa)                  compute_surface
───────────────────────                  ─────────────────────────────────────────────
cloud del nodo 1                         unione di tutte le acquisizioni
 → normali (se assenti le stima)          → voxel grid (opzionale)
 → normali orientate verso la camera      → taglio del tavolo (PassThrough su Z globale)
 → TF world ← camera al timestamp         → Statistical Outlier Removal
 → transformPointCloudWithNormals         → Moving Least Squares (+ riorientamento normali)
 → push nella lista                       → Poisson
                                          → trimming dei triangoli non supportati dai dati
                                          → pubblicazione Marker + shape_msgs/Mesh
```

Scelte implementative:

- **Orientamento delle normali.** Poisson richiede normali orientate in modo coerente. Ogni acquisizione ha le normali rivolte verso la propria camera. Dopo MLS, che non garantisce il verso, ogni normale viene riallineata a quella del punto originale più vicino.
- **Trimming.** Poisson restituisce sempre una superficie *chiusa*, che avvolgerebbe anche la parte sotto l'oggetto mai vista dalla camera. Vengono quindi eliminati i triangoli con almeno un vertice più lontano di `poisson_trim_distance` dai dati.
- **Taglio del tavolo dopo l'unione**, nel frame comune, dove il tavolo è a Z costante (vedi la motivazione nel documento della pipeline).
- **Controlli in `acquire`.** Vengono rifiutate le cloud più vecchie di `max_cloud_age`, per evitare acquisizioni con il robot ancora in movimento, e quelle con meno di `min_points` punti.

## Interfacce

Tutti i nomi sono relativi al nodo (`/cloud_merger_node/...`), tranne il topic di ingresso.

### Topic

| Topic | Tipo | Dir. | Descrizione |
|---|---|---|---|
| `filtered_cloud` | `sensor_msgs/PointCloud2` | sub | cloud filtrata dal nodo 1, con o senza normali |
| `~/mesh_marker` | `visualization_msgs/Marker` | pub, latched | mesh per Rviz (`TRIANGLE_LIST`) |
| `~/mesh` | `shape_msgs/Mesh` | pub, latched | mesh per il nodo 3 |
| `~/accumulated_cloud` | `sensor_msgs/PointCloud2` | pub, latched | acquisizioni unite, prima dei filtri |
| `~/processed_cloud` | `sensor_msgs/PointCloud2` | pub, latched | cloud dopo filtri e MLS, con normali (input di Poisson) |

I publisher "latched" usano QoS `transient_local`: chi si sottoscrive dopo riceve comunque l'ultimo messaggio. In Rviz impostare *Durability Policy = Transient Local*.

### Servizi

| Servizio | Tipo | Descrizione |
|---|---|---|
| `~/acquire` | `std_srvs/Trigger` | acquisisce l'ultima cloud ricevuta e la aggiunge alla lista |
| `~/remove_last` | `std_srvs/Trigger` | rimuove l'ultima acquisizione (es. robot non ancora fermo) |
| `~/compute_surface` | `std_srvs/Trigger` | unisce le acquisizioni e ricostruisce la mesh |
| `~/save` | `surface_reconstruction_msgs/SaveMesh` | salva la mesh; formato dall'estensione: `.ply`, `.vtk`, `.obj` |
| `~/reset` | `std_srvs/Trigger` | svuota acquisizioni e mesh |

Il campo `message` della risposta riporta sempre l'esito, con il numero di punti, la dimensione della mesh e i tempi.

### TF

Serve la catena `world_frame` ← `frame_id` della cloud, disponibile al timestamp della cloud. Normalmente la forniscono:
- `robot_state_publisher` per la cinematica diretta (`world` → … → flangia);
- uno `static_transform_publisher` con la calibrazione mano-occhio (flangia → frame della camera).

## Parametri

I parametri vengono **riletti a ogni chiamata di servizio**, quindi si possono tarare a runtime con `ros2 param set` e rilanciare `compute_surface` senza riacquisire.

| Parametro | Default | Descrizione |
|---|---|---|
| `world_frame` | `world` | frame comune in cui vengono unite le cloud |
| `tf_timeout` | `0.5` | [s] attesa massima per la TF |
| `max_cloud_age` | `1.0` | [s] età massima della cloud in `acquire`; ≤ 0 disabilita il controllo |
| `min_points` | `100` | punti validi minimi per accettare un'acquisizione |
| `normal_k_search` | `20` | vicini per la stima delle normali (solo se la cloud non le ha) |
| `merge_voxel_leaf` | `0.0` | [m] voxel grid sulla cloud unita, utile per uniformare le zone di sovrapposizione; ≤ 0 disabilita |
| `table_z` | `0.0` | [m] quota del tavolo in `world_frame` |
| `table_margin` | `0.005` | [m] si tengono i punti con `z > table_z + table_margin` |
| `z_max` | `10.0` | [m] quota massima tenuta |
| `sor_mean_k` | `30` | vicini per la Statistical Outlier Removal |
| `sor_stddev_mul` | `1.0` | soglia in deviazioni standard per la SOR |
| `mls_search_radius` | `0.01` | [m] raggio MLS (≈ 3–5 volte la distanza tra i punti) |
| `mls_polynomial_order` | `2` | ordine del polinomio MLS |
| `mls_upsampling` | `none` | `none` \| `sample_local_plane` \| `voxel_grid_dilation` |
| `mls_upsampling_radius` / `mls_upsampling_step` | `0.005` / `0.002` | [m] per `sample_local_plane` |
| `mls_dilation_voxel_size` / `mls_dilation_iterations` | `0.002` / `1` | per `voxel_grid_dilation` |
| `poisson_depth` | `8` | profondità dell'octree: più alta = più dettaglio (e più tempo) |
| `poisson_point_weight` | `4.0` | peso dei punti (interpolazione); valori alti seguono di più i dati |
| `poisson_samples_per_node` | `1.5` | campioni minimi per nodo; alzarlo con dati rumorosi |
| `poisson_scale` | `1.1` | rapporto tra il cubo di ricostruzione e il bounding box dei dati |
| `poisson_threads` | `4` | thread usati da MLS e Poisson |
| `poisson_trim_distance` | `0.005` | [m] scarta i triangoli lontani dai dati (≈ 1–3 volte la distanza tra i punti); ≤ 0 disabilita |
| `default_mesh_file` | `surface.ply` | file usato da `save` se `filename` è vuoto |
| `mesh_color` | `[0.2, 0.6, 0.9, 1.0]` | colore RGBA del Marker |

## Struttura

```
surface_reconstruction/
├── CMakeLists.txt, package.xml
├── config/cloud_merger.yaml          parametri di default
├── launch/cloud_merger.launch.py
├── include/surface_reconstruction/cloud_merger_node.hpp
├── src/cloud_merger_node.cpp         implementazione del nodo
├── src/cloud_merger_main.cpp         main (MultiThreadedExecutor)
├── scripts/fake_camera.py            camera simulata per test/tutorial
└── docs/TUTORIAL.md
```

Il nodo usa un `MultiThreadedExecutor` con due callback group: la sottoscrizione continua ad aggiornare l'ultima cloud anche mentre `compute_surface` è in esecuzione. I servizi sono serializzati tra loro.

## Autori

Sviluppato da **CARI-JRL**.

Sviluppatore e maintainer: Manuel Beschi (<manuel.beschi@unibs.it>)

## Licenza

Apache License 2.0, vedi [LICENSE](LICENSE).

Copyright 2026 CARI-JRL
