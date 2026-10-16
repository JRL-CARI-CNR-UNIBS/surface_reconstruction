# Tutorial — ricostruzione di una superficie con `cloud_merger_node`

Il tutorial ha tre parti:

1. **[Prova in simulazione](#1-prova-in-simulazione)**: nessun hardware; una camera simulata vede un oggetto sul tavolo da quattro pose diverse.
2. **[Uso con robot e RealSense](#2-uso-con-robot-e-realsense)**: cosa collegare sul sistema reale.
3. **[Taratura dei parametri e problemi comuni](#3-taratura-dei-parametri-e-problemi-comuni)**

Tempo richiesto per la parte 1: circa 10 minuti.

---

## 0. Compilazione

```bash
cd ~/projects/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/surface_reconstruction src/surface_reconstruction_msgs --ignore-src -y
colcon build --packages-up-to surface_reconstruction
source install/setup.bash
```

In ogni nuovo terminale ricordarsi di eseguire `source ~/projects/ros2_ws/install/setup.bash`.

---

## 1. Prova in simulazione

### 1.1 La camera simulata

`fake_camera.py` simula una camera che guarda verso il basso una scena composta da un tavolo a `z = 0` con un "dosso" alto 6 cm. Pubblica:

- la point cloud vista dalla camera (solo i punti nel campo visivo, con un po' di rumore) sul topic `filtered_cloud`, nel frame `fake_camera_optical_frame`;
- la TF `world → fake_camera_optical_frame`, che qui sostituisce cinematica del robot + calibrazione mano-occhio.

La posizione della camera è data dai parametri `cam_x`, `cam_y` e `cam_z`. Cambiandoli a runtime si simula il robot che si sposta in una nuova posa di acquisizione.

### 1.2 Avvio

**Terminale 1**, la camera simulata:

```bash
ros2 run surface_reconstruction fake_camera.py
```

**Terminale 2**, il nodo di ricostruzione:

```bash
ros2 launch surface_reconstruction cloud_merger.launch.py
```

Dovresti vedere:

```
[cloud_merger_node]: cloud_merger_node ready: listening on '/filtered_cloud', common frame 'world'
```

**Terminale 3**, Rviz:

```bash
rviz2
```

In Rviz:

1. *Global Options → Fixed Frame* = `world`
2. *Add → By topic*:
   - `/filtered_cloud` → PointCloud2: è la vista "live" della camera.
   - `/cloud_merger_node/accumulated_cloud` → PointCloud2: le acquisizioni accumulate.
   - `/cloud_merger_node/mesh_marker` → Marker: la mesh ricostruita.
3. Per i display `accumulated_cloud` e `mesh_marker` impostare *Topic → Durability Policy* = **Transient Local**. Sono topic "latched": altrimenti Rviz non riceve i messaggi pubblicati prima della sua sottoscrizione.
4. Facoltativo: *Add → TF* per vedere la posa della camera.

### 1.3 Acquisizioni da più pose

Nel **terminale 4** si muove la camera e si acquisisce. Prima posa:

```bash
ros2 param set /fake_camera cam_x -0.15
ros2 param set /fake_camera cam_y -0.10
ros2 service call /cloud_merger_node/acquire std_srvs/srv/Trigger
```

```
response: success=True, message='Acquisition #1 stored (24000 points)'
```

In Rviz `accumulated_cloud` mostra la prima porzione di scena. Ripeti per le altre tre pose:

```bash
ros2 param set /fake_camera cam_x 0.15
ros2 service call /cloud_merger_node/acquire std_srvs/srv/Trigger

ros2 param set /fake_camera cam_y 0.10
ros2 service call /cloud_merger_node/acquire std_srvs/srv/Trigger

ros2 param set /fake_camera cam_x -0.15
ros2 service call /cloud_merger_node/acquire std_srvs/srv/Trigger
```

A ogni chiamata `accumulated_cloud` cresce. Le nuvole si sovrappongono correttamente perché ognuna è stata trasformata in `world` con la TF valida al momento dell'acquisizione.

### 1.4 Annullare un'acquisizione sbagliata

Supponiamo di aver acquisito per errore una posa in più:

```bash
ros2 param set /fake_camera cam_z 0.35
ros2 service call /cloud_merger_node/acquire std_srvs/srv/Trigger     # acquisizione #5 (sbagliata)
ros2 service call /cloud_merger_node/remove_last std_srvs/srv/Trigger
```

```
response: success=True, message='Last acquisition removed, 4 remaining'
```

Non serve ricominciare da capo: si rimuove solo l'ultima acquisizione.

### 1.5 Ricostruzione della superficie

```bash
ros2 service call /cloud_merger_node/compute_surface std_srvs/srv/Trigger
```

```
response: success=True, message='Surface computed from 4 acquisitions: 95838 merged points,
  21891 after filtering, 21891 after MLS, mesh with 51839 vertices and 103229 triangles (2.4 s)'
```

Il messaggio riassume cosa è successo:
- **95838 merged points**: tutte le acquisizioni unite, compreso il tavolo;
- **21891 after filtering**: il tavolo è stato tagliato (`table_z + table_margin`) e gli outlier rimossi;
- **mesh**: risultato di Poisson dopo il trimming.

In Rviz compare la mesh azzurra del dosso, senza il tavolo e senza la "pancia" chiusa sotto l'oggetto.

Per vedere la cloud che entra in Poisson, con le normali calcolate da MLS, aggiungi in Rviz `/cloud_merger_node/processed_cloud` (Transient Local).

### 1.6 Salvataggio

```bash
ros2 service call /cloud_merger_node/save surface_reconstruction_msgs/srv/SaveMesh "{filename: '/tmp/surface.ply'}"
```

Il formato dipende dall'estensione: `.ply` (binario), `.vtk` o `.obj`. Il file si può aprire con MeshLab, CloudCompare o `pcl_viewer /tmp/surface.ply`. Con `filename` vuoto si usa il parametro `default_mesh_file`; un percorso relativo è risolto rispetto alla cartella da cui è stato lanciato il nodo.

### 1.7 Taratura a runtime

I parametri vengono riletti a ogni chiamata, quindi si può tarare la ricostruzione **senza riacquisire**:

```bash
ros2 param set /cloud_merger_node poisson_depth 9            # più dettaglio
ros2 param set /cloud_merger_node merge_voxel_leaf 0.003     # uniforma le zone sovrapposte
ros2 service call /cloud_merger_node/compute_surface std_srvs/srv/Trigger
```

Quando i valori ti soddisfano, salvali con `ros2 param dump /cloud_merger_node > mio_config.yaml` e riusali con `params_file:=mio_config.yaml`.

### 1.8 Nuova scansione

```bash
ros2 service call /cloud_merger_node/reset std_srvs/srv/Trigger
```

Svuota le acquisizioni e la mesh; in Rviz la mesh scompare.

---

## 2. Uso con robot e RealSense

Sul sistema reale `fake_camera.py` viene sostituita da tre componenti:

| Componente | Fornisce |
|---|---|
| driver RealSense + **nodo 1** (acquisizione live) | cloud filtrata, cluster più grande con normali |
| `robot_state_publisher` (driver del robot) | TF `world → … → tool0` (cinematica diretta) |
| `static_transform_publisher` | TF `tool0 → camera_link` (calibrazione mano-occhio) |

### 2.1 Calibrazione mano-occhio come TF statica

Con i valori della calibrazione (traslazione in metri, quaternione):

```bash
ros2 run tf2_ros static_transform_publisher \
    --x 0.032 --y -0.045 --z 0.061 \
    --qx 0.0 --qy 0.0 --qz 0.7071 --qw 0.7071 \
    --frame-id tool0 --child-frame-id camera_link
```

Il driver RealSense pubblica poi `camera_link → camera_depth_optical_frame`. Verifica che la catena sia completa:

```bash
ros2 run tf2_ros tf2_echo world camera_depth_optical_frame
```

Il frame da usare come `child-frame-id` dipende da quale frame è stato calibrato: se la calibrazione dà direttamente la posa dell'*optical frame*, collega quello e non `camera_link`.

### 2.2 Configurazione

Copia il file di parametri e adattalo:

```bash
cp $(ros2 pkg prefix surface_reconstruction)/share/surface_reconstruction/config/cloud_merger.yaml ~/mio_config.yaml
```

Da controllare per primi:

- `world_frame`: il frame base della cella (es. `world` o `base_link`);
- `table_z`: quota del piano del tavolo in `world_frame`. Per misurarla, acquisisci una volta, guarda `accumulated_cloud` in Rviz e usa *Publish Point* su un punto del tavolo;
- `table_margin`: 3–10 mm, in base al rumore della camera;
- `mls_search_radius` e `poisson_trim_distance`: vedi la [sezione 3](#3-taratura-dei-parametri-e-problemi-comuni).

### 2.3 Avvio

```bash
ros2 launch surface_reconstruction cloud_merger.launch.py \
    input_cloud:=/<topic_cloud_filtrata_del_nodo_1> \
    params_file:=$HOME/mio_config.yaml
```

### 2.4 Procedura di scansione

1. Porta il robot nella prima posa e **attendi che sia fermo**.
2. `acquire`. Se la risposta segnala una cloud troppo vecchia, il nodo 1 non sta pubblicando oppure gli orologi non sono sincronizzati (vedi la sezione 3).
3. Ripeti per ogni posa, con il 30–50% di sovrapposizione tra viste adiacenti.
4. Controlla `accumulated_cloud` in Rviz: se una vista è disallineata o sporca, `remove_last` e riacquisisci.
5. `compute_surface`, controlla la mesh, poi `save`.

Il nodo 4 (autoscan) automatizzerà i passi 1–3 chiamando `acquire` su una griglia di pose.

---

## 3. Taratura dei parametri e problemi comuni

### Regole pratiche

Sia `d` la distanza media tra punti vicini (per la D435 a 40–50 cm, con il voxel grid del nodo 1, tipicamente 2–4 mm):

| Parametro | Valore indicativo |
|---|---|
| `mls_search_radius` | 3–5 · `d` |
| `poisson_trim_distance` | 1–3 · `d` |
| `merge_voxel_leaf` | ≈ `d`, se le zone sovrapposte risultano più dense o più "spesse" |
| `poisson_depth` | 8 per oggetti di 20–30 cm; +1 raddoppia la risoluzione, con circa 4–8 volte il tempo |

### Problemi comuni

| Sintomo | Causa probabile | Rimedio |
|---|---|---|
| `No point cloud received yet` | topic di ingresso sbagliato | `ros2 topic list`; lancia con `input_cloud:=...` |
| `Latest point cloud is too old` | nodo 1 fermo, oppure orologi non sincronizzati tra PC | controlla `ros2 topic hz`; sincronizza con chrony; in emergenza `max_cloud_age: 0.0` |
| `TF lookup world <- ... failed` | manca la calibrazione mano-occhio o `robot_state_publisher` | `ros2 run tf2_tools view_frames` e verifica la catena |
| Acquisizioni disallineate in `accumulated_cloud` | calibrazione mano-occhio imprecisa, robot non fermo, TF in ritardo | ricalibra; attendi più a lungo prima di `acquire` |
| `no point left after the table cut` | `table_z` errato o di segno sbagliato | misura `table_z` come al punto 2.2 |
| Pezzi di tavolo nella mesh | `table_margin` troppo piccolo per il rumore | aumenta `table_margin` |
| Mesh con buchi o a brandelli | `poisson_trim_distance` troppo piccolo | aumentalo, oppure aumenta `mls_search_radius` |
| Mesh "gonfia" ai bordi o con lembi in eccesso | `poisson_trim_distance` troppo grande | riducilo |
| Mesh troppo liscia, spigoli arrotondati | `mls_search_radius` troppo grande, `poisson_depth` bassa | riduci il raggio, aumenta la profondità |
| Mesh rugosa o con increspature | rumore residuo | aumenta `mls_search_radius` o `poisson_samples_per_node` |
| Superficie "rovesciata" o con bolle | normali incoerenti, ad es. cloud del nodo 1 con normali sbagliate | in Rviz mostra `processed_cloud` con *Color Transformer = Intensity* e *Channel Name = normal_z*: le zone con normali capovolte hanno un colore diverso |
| `compute_surface` lento | troppi punti, `poisson_depth` alta | `merge_voxel_leaf > 0`; riduci `poisson_depth` |
