# 🏎️ LKAS — Lane Keeping Assist System per Ferrari SF90 Stradale

Progetto universitario per il corso di **Controlli Automatici** — progettazione di un sistema di controllo **SISO** per il mantenimento della corsia (**Lane Keeping Assist System**) modellato sui parametri dinamici reali della **Ferrari SF90 Stradale**, implementato in MATLAB con la Control System Toolbox.

---

## 📌 Descrizione

Il progetto implementa un sistema di controllo in retroazione per il mantenimento laterale del veicolo rispetto all'asse della corsia. Partendo dal modello bicicletta linearizzato della dinamica laterale del veicolo, si progetta un controllore che soddisfi specifiche di prestazione in termini di tempo di assestamento, errore a regime e reiezione di disturbi sinusoidali.

---

## 🚗 Parametri del Veicolo

I parametri dinamici sono modellati sulla Ferrari SF90 Stradale:

| Parametro | Valore | Descrizione |
|-----------|--------|-------------|
| `m` | 1570 kg | Massa del veicolo |
| `wheelBase` | 2.65 m | Passo |
| `lf` | 1.4575 m | Distanza CdG–asse anteriore (55% posteriore) |
| `lr` | 1.1925 m | Distanza CdG–asse posteriore |
| `c_af` | 65000 N/rad | Rigidità di deriva anteriore |
| `c_ar` | 70000 N/rad | Rigidità di deriva posteriore |
| `Iz` | 1573 kg·m² | Momento di inerzia in imbardata |
| `v_long` | 8.33 m/s | Velocità longitudinale (30 km/h) |

> La scelta `c_af < c_ar` configura il veicolo in leggero **sovrasterzo**, coerente con il carattere sportivo della SF90 Stradale.

<img width="697" height="348" alt="image" src="https://github.com/user-attachments/assets/a716be59-4766-4786-9364-20ccdca232d9" />


---

## 🧠 Modello Dinamico

Il sistema è descritto dal **modello bicicletta linearizzato** con stato:

```
x = [e1, ė1, e2, ė2]
```

dove `e1` è l'errore laterale del centro di massa rispetto all'asse corsia ed `e2` è l'errore di orientamento (imbardata) rispetto alla strada. L'ingresso di controllo è l'**angolo di sterzo** δ.

La funzione di trasferimento della **pianta** è:

```
P(s) = W1e(s) + ds · W2e(s)
```

dove `ds = 7 m` è la distanza di previsione del punto di riferimento ahead del veicolo (preview distance).

---

## 🎯 Specifiche di Progetto

| Specifica | Valore richiesto |
|-----------|-----------------|
| Tempo di assestamento | < 5 s |
| Errore a regime (gradino) | < 2% |
| Sovraelongazione | < 5% |
| Reiezione disturbo | `n(t) = (4π/180)·sin(2π·3·t)` rad |

---

## 🔧 Progettazione del Controllore

Il controllore finale è composto da più stadi progettati in sequenza:

### 1. Analisi della pianta
Verifica della stabilità con criterio di Bode e luogo delle radici. La pianta presenta guadagno statico positivo e nessun polo a parte reale positiva: il criterio di Bode è applicabile.

### 2. Compensatore `C1`
Aggiunta di uno zero reale e due poli veloci per attirare i rami del luogo delle radici nel semipiano sinistro:

```matlab
C1 = (s + 1/0.1) / (s + 1/0.01)^2
```

### 3. Rete di ritardo (delay network)
Miglioramento del margine di fase per la robustezza:

```matlab
delayNet = (1 + alpha·tau·s) / (1 + tau·s)    % alpha=0.7, tau=0.1
```

### 4. Filtro notch
Filtro arresta-banda centrato a `ω = 2π·3` rad/s per la reiezione del disturbo sinusoidale:

```matlab
H = (s² + ωn²) / (s² + 2·ζ·ωn·s + ωn²)    % ζ=0.6
```

### 5. Filtro passa-basso sul riferimento `Fr`
Riduzione della sovraelongazione e miglioramento del transitorio:

```matlab
Fr = 1 / (τ·s + 1)
```

### 6. Guadagno
`gain = 110`, tarato tramite luogo delle radici per garantire stabilità e margini adeguati.

---

## 📊 Funzioni di Sensitività Analizzate

| Funzione | Descrizione |
|----------|-------------|
| `Wyr` | Anello chiuso riferimento → uscita |
| `Wyr1` | `Wyr` con filtro passa-basso sul riferimento |
| `Wur` | Sensitività di controllo (sforzo di controllo) |
| `Wer` | Sensitività (robustezza ai disturbi) |

<img width="696" height="220" alt="image" src="https://github.com/user-attachments/assets/4f83cf4f-9fe8-4373-984f-da95be68daec" />

---

## 🛠️ Requisiti

- MATLAB R2020b o superiore
- Control System Toolbox

---

## ▶️ Esecuzione

Aprire il file in MATLAB ed eseguire dall'inizio:

```matlab
run('lkas_sf90.m')
```

Lo script è strutturato in sezioni (`%%`) eseguibili separatamente con **Run Section** (`Ctrl+Enter`):

| Sezione | Contenuto |
|---------|-----------|
| Parametri veicolo | Definizione del modello dinamico |
| Spazio di stato | Matrici `Ae`, `B1e`, `B2e`, `C1e`, `C2e` |
| Funzione di trasferimento | Calcolo di `Pe(s)` |
| Stabilità | Bode e root locus della pianta |
| Progetto controllore | Sintesi di `Ce` con `C1`, `delayNet`, notch, gain |
| Simulazioni | Risposta al gradino, reiezione disturbo, `lsim` |
| Prestazioni | Analisi di `Wur`, `Wer`, `stepinfo` |

---

## 📁 Struttura del Repository

```
.
└── lkas_sf90.m     # Script MATLAB principale
```

---

## 📈 Risultati

- Tutti i poli della `Wyr` a parte reale negativa — sistema **asintoticamente stabile**
- Reiezione efficace del disturbo sinusoidale `n(t)` grazie al filtro notch centrato a 3 Hz
- Specifiche di assestamento, errore a regime e sovraelongazione soddisfatte con il filtro `Fr` attivo
