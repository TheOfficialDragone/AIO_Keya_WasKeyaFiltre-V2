# Context: AgOpenGPS - Keya Motor WAS Firmware

## 📌 Panoramica del Progetto

Questo firmware è una variante per **AgOpenGPS** progettata per girare su piattaforma **Arduino** (specificamente su scheda **Teensy 4.1**).
La particolarità del sistema è l'eliminazione del sensore di sterzo fisico (WAS - Wheel Angle Sensor) sull'assale anteriore del trattore. L'angolo delle ruote viene calcolato matematicamente in tempo reale sfruttando i dati dell'**encoder integrato nel motore di sterzo Keya**.

## 📂 Struttura del Repository e Sorgente di Verità

Nel repository è presente un file chiamato **`repomix-output.xml`**.

Questo file contiene **l'intera codebase esportata in formato XML tramite Repomix** e deve essere considerato una rappresentazione completa della struttura del progetto e dei file sorgente.

### Regole di utilizzo di `repomix-output.xml`

* Utilizzare `repomix-output.xml` come riferimento principale per comprendere architettura, relazioni tra file e flusso del firmware.
* Quando si analizza il comportamento del sistema o si propongono modifiche, considerare sempre il contesto completo della codebase presente nel file XML e non solo singoli file isolati.
* Evitare di dedurre struttura o dipendenze mancanti: se qualcosa non è chiaro, cercarlo all'interno di `repomix-output.xml`.
* Le modifiche suggerite devono rimanere compatibili con l'organizzazione attuale del progetto rappresentata nel file.

## 🛠️ Stack Tecnologico

* **Hardware:** Teensy 4.1, Motore Keya (Autosteer).
* **Ambiente di Sviluppo:** Arduino IDE / Teensyduino.
* **Software di Riferimento:** AgOpenGPS.

## 🎯 Regole di Sviluppo e Vincoli

* **Preservare la Struttura:** La struttura portante e la logica del codice originale funzionano bene nella maggior parte dei contesti. **NON** è consentito riscrivere il codice da zero o stravolgerne l'architettura.
* **Interventi Mirati:** Le modifiche devono essere chirurgiche, focalizzate esclusivamente sulla risoluzione dei bug segnalati e sul miglioramento della stabilità dell'algoritmo di calcolo dell'angolo.
* **Efficienza su Teensy 4.1:** Sebbene il Teensy 4.1 offra elevate prestazioni, il codice deve rimanere snello, reattivo e non deve introdurre ritardi (polling/loop block) nella comunicazione seriale/CAN con il motore Keya o AgOpenGPS.

## 🐛 Feedback e Problemi da Risolvere

* Il sistema è ancora troppo lento nel riallinearsi sulla linea e ritrovare lo zero dopo un’inversione a U.
* In alcuni casi il sistema non torna allo zero abbastanza rapidamente e le regolazioni effettuate non sembrano migliorare il comportamento.
* Sono state eseguite prove con BNO disattivato e GPS disattivato, ma il sistema continua a non risultare sufficientemente affidabile.
* Sono stati aumentati e diminuiti diversi parametri di sensibilità, senza trovare una configurazione soddisfacente.
* Sono stati testati numerosi valori di configurazione: su un trattore JD il comportamento sembrava abbastanza buono, mentre sul Valtra l’esperienza è stata molto frustrante.
* In alcune situazioni il sistema considera erroneamente un angolo di circa 15° come direzione corretta e azzera il riferimento.
* In altri casi il veicolo procede diritto con un errore residuo di circa 7° senza che venga effettuato l’azzeramento.
* È stato osservato che il sistema tende a confondersi durante le curve.
* Dopo il ritorno in rettilineo rimane spesso un errore residuo di orientamento (ad esempio circa 9°) invece di ritornare correttamente allo zero.
* Nel complesso il comportamento percepito non risulta ancora sufficientemente stabile e affidabile nelle diverse condizioni operative.
