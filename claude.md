
# System Prompt & Context: AgOpenGPS - Keya Motor Virtual WAS Firmware

## 🤖 Regole di Comportamento dell'Assistente

* **Estrema Sintesi:** Sii il meno verboso possibile. Fornisci risposte dirette e chirurgiche (es. mostra solo le righe di codice da modificare). Non fornire spiegazioni didattiche a meno che non ti venga esplicitamente richiesto.
* **Gestione della Documentazione:** Se devi generare un README, scrivere documentazione estesa o file di testo lunghi, **NON** stamparli a video nella chat. Conferma semplicemente di aver compreso e attendi istruzioni su come l'utente preferisce riceverli, oppure fornisci solo comandi per generarli.

## 📌 Panoramica del Progetto

Questo firmware è una variante per **AgOpenGPS** progettata per girare su piattaforma **Arduino** (specificamente su scheda **Teensy 4.1**).
La particolarità del sistema è l'eliminazione del sensore di sterzo fisico (WAS - Wheel Angle Sensor) sull'assale anteriore del trattore. L'angolo delle ruote viene calcolato matematicamente in tempo reale sfruttando esclusivamente i dati dell'**encoder integrato nel motore di sterzo Keya**.

## 📂 Struttura del Repository e Sorgente di Verità

Nel repository è presente un file chiamato **`repomix-output.xml`**.
Questo file contiene **l'intera codebase esportata in formato XML tramite Repomix** e deve essere considerato l'unica rappresentazione completa della struttura del progetto e dei file sorgente.

### Regole di utilizzo di `repomix-output.xml`

* Utilizzare `repomix-output.xml` come riferimento assoluto per comprendere architettura, relazioni tra file e flusso del firmware.
* Valutare sempre il contesto completo della codebase prima di proporre modifiche: non ragionare mai su singoli file isolati.
* Non dedurre o inventare dipendenze mancanti: se qualcosa non è chiaro, cercalo all'interno di `repomix-output.xml`.
* Le modifiche suggerite devono rimanere retrocompatibili con l'attuale architettura.

## 🛠️ Stack Tecnologico

* **Hardware:** Teensy 4.1, Motore Keya (Autosteer / Encoder integrato).
* **Ambiente di Sviluppo:** Arduino IDE / Teensyduino.
* **Software di Riferimento:** AgOpenGPS.

## 🎯 Regole di Sviluppo e Vincoli

* **Preservare la Struttura:** La logica portante del codice originale funziona. **VIETATO** riscrivere il codice da zero o stravolgerne l'architettura.
* **Interventi Mirati (Chirurgici):** Concentrati solo sulla risoluzione dei bug segnalati e sul perfezionamento della matematica dell'angolo virtuale.
* **Massima Efficienza:** Il Teensy 4.1 è veloce, ma il codice deve restare non bloccante. Non introdurre ritardi (`delay()`, loop bloccanti o polling inefficaci) nella comunicazione seriale/CAN con il motore Keya o con AgOpenGPS.

## 🐛 Analisi dei Problemi (Bug & Feedback sul campo)

I test sul campo hanno dimostrato che il comportamento percepito è inaffidabile. I tentativi di *tuning* dei parametri (sensibilità, PID, ecc.) sono stati inefficaci, il che suggerisce un difetto logico-matematico nel calcolo dell'angolo e non un semplice problema di regolazione. I test con GPS e BNO disattivati confermano che l'errore è intrinseco alla logica di lettura dell'encoder.

I problemi da analizzare e risolvere si dividono in 3 categorie principali:

### 1. Deriva e Falsi Azzeramenti (Center & Zero Drift)

* Falso Centro: In alcune situazioni il sistema considera erroneamente un angolo di ~15° come direzione di marcia dritta e "azzera" scorrettamente il riferimento.
* Errore Residuo: Durante la marcia in rettilineo, rimane spesso un errore di orientamento costante (es. 7° o 9°) senza che l'algoritmo effettui l'azzeramento per correggerlo.

### 2. Latenza e Confusione in Sterzata (Lag & Tracking Issues)

* Riallineamento Lento: Il sistema impiega troppo tempo per ritrovare la linea e rientrare a zero dopo manovre ampie (es. inversione a U).
* Disorientamento: Il sistema tende ad "accumulare errore" o a confondersi durante l'esecuzione delle curve.

### 3. Inconsistenza tra Macchine (Geometry Mismatch)

* Il firmware è stato testato su due trattori con esiti diametralmente opposti: su un John Deere il comportamento era discreto, mentre su un Valtra è risultato pessimo. Questo suggerisce che l'algoritmo non sta gestendo o compensando correttamente i diversi rapporti meccanici di sterzo o la geometria di Ackermann specifica di ogni veicolo.