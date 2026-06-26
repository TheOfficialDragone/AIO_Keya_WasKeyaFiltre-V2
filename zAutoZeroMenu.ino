// =============================================================
// MENU SERIE - Reglage parametres Auto-Zero WAS (encodeur Keya)
// =============================================================
// Utilisation :
//   - Ouvrir le moniteur serie (115200 baud)
//   - Taper  'z'  puis ENTREE pour afficher le menu
//   - Taper le numero du parametre, ENTREE, puis la valeur, ENTREE
//   - Les valeurs sont sauvegardees en EEPROM (adresse 90)
// =============================================================

#define EEPROM_ADDR_AZ_PARAMS  90

// Structure AutoZeroParams definie dans Autosteer.ino (avant ce fichier dans l'ordre de compilation)

// Instance globale - valeurs par defaut
AutoZeroParams azParams = {
  .speedMin    = 2.5f,
  .yawRateMax  = 0.3f,
  .gpsHdgMax   = 0.3f,
  .timeSlowMs  = 500,
  .timeFastMs  = 200,
  .speedSlow   = 3.0f,
  .speedFast   = 12.0f,
  .useBno      = 1,      // BNO actif par defaut
  .useGps      = 1,      // GPS actif par defaut
  .beta        = 0.05f,  // correction douce : 5% de l'erreur par cycle
  .ident       = 0xA202  // bumpe pour forcer reinit EEPROM avec nouveaux champs
};

// -----------------------------------------------------------------
// Appeler dans autosteerSetup() apres les autres EEPROM.get()
// -----------------------------------------------------------------
void azMenuSetup()
{
  AutoZeroParams saved;
  EEPROM.get(EEPROM_ADDR_AZ_PARAMS, saved);
  if (saved.ident == 0xA202) {
    azParams = saved;
    Serial.println("[AZ-MENU] Parametres charges depuis EEPROM.");
  } else {
    EEPROM.put(EEPROM_ADDR_AZ_PARAMS, azParams);
    Serial.println("[AZ-MENU] Premiere utilisation - valeurs par defaut sauvegardees.");
  }
  azMenuPrint();
}

// -----------------------------------------------------------------
// Affichage du menu
// -----------------------------------------------------------------
void azMenuPrint()
{
  Serial.println();
  Serial.println("======= MENU AUTO-ZERO WAS =======");
  Serial.print("1. Vitesse mini       : "); Serial.print(azParams.speedMin,    1); Serial.println(" km/h");
  Serial.print("2. Yaw rate maxi (BNO): "); Serial.print(azParams.yawRateMax,  2); Serial.println(" deg/s  (bas=strict)");
  Serial.print("3. Variation GPS maxi : "); Serial.print(azParams.gpsHdgMax,   2); Serial.println(" deg    (bas=strict)");
  Serial.print("4. Duree basse vit.   : "); Serial.print(azParams.timeSlowMs      ); Serial.println(" ms");
  Serial.print("5. Duree haute vit.   : "); Serial.print(azParams.timeFastMs      ); Serial.println(" ms");
  Serial.print("6. Seuil basse vit.   : "); Serial.print(azParams.speedSlow,   1); Serial.println(" km/h");
  Serial.print("7. Seuil haute vit.   : "); Serial.print(azParams.speedFast,   1); Serial.println(" km/h");
  Serial.print("8. Source BNO         : "); Serial.println(azParams.useBno ? "ACTIF" : "INACTIF");
  Serial.print("9. Source GPS         : "); Serial.println(azParams.useGps ? "ACTIF" : "INACTIF");
  Serial.print("10. Beta correction   : "); Serial.print(azParams.beta,        3); Serial.println("  (0.01=lent .. 0.2=rapide)");
  Serial.println("--- EKF params ---");
  Serial.print("11. Wheelbase (m)     : "); Serial.print(ekfWheelBase, 2); Serial.println(" m  (MEASURE ON TRACTOR)");
  Serial.print("12. Rkin (meas.noise) : "); Serial.print(ekfRkin, 6);      Serial.println("  (1.5deg)^2 = 6.8e-4");
  Serial.print("13. Qdelta (proc.noise): "); Serial.print(ekfQdelta, 6);   Serial.println("  default 1e-4");
  Serial.print("14. Vmin (m/s)        : "); Serial.print(ekfVmin, 2);      Serial.println(" m/s min speed for kinematic update");
  Serial.print("17. Max angle (deg)   : "); Serial.print(ekfMaxAngleDeg, 2); Serial.println(" deg  (physical lock-to-lock half-range)");
  Serial.println("18. Reset valeurs defaut");
  Serial.println("19. Quitter");
  Serial.println("--- Taper 'c' pour assistant calibration butee-a-butee (CPD) ---");
  Serial.println("==================================");
  Serial.println("Taper numero + ENTREE :");
}

// -----------------------------------------------------------------
// Boucle du menu - appeler dans autosteerLoop() ou loop()
// Retourne true si le menu est actif (bloque les autres traitements)
// -----------------------------------------------------------------
static bool    azMenuActive  = false;
static uint8_t azMenuStep    = 0;  // 0=attente choix, 1=attente valeur
static uint8_t azMenuChoice  = 0;

// Lock-to-lock CPD calibration wizard state (AGCO US8583312 pattern)
static bool    calWizActive = false;
static uint8_t calWizStep   = 0;
static int32_t calLeftTicks = 0;

bool azMenuLoop()
{
  // -----------------------------------------------------------------
  // Assistant calibration butee-a-butee (CPD) — actif meme hors menu
  // -----------------------------------------------------------------
  if (calWizActive && Serial.available()) {
    String calInput = Serial.readStringUntil('\n');
    calInput.trim();
    if (calWizStep == 1) {
      calLeftTicks = keyaEncoderRaw;
      calWizStep   = 2;
      Serial.print("[CAL] LEFT  recorded: "); Serial.println(calLeftTicks);
      Serial.println("[CAL] Now steer FULL RIGHT to mechanical stop, then press Enter.");
    } else if (calWizStep == 2) {
      int32_t calRightTicks = keyaEncoderRaw;
      Serial.print("[CAL] RIGHT recorded: "); Serial.println(calRightTicks);
      int32_t newZero    = (calLeftTicks + calRightTicks) / 2;
      float   totalTicks = fabsf((float)(calRightTicks - calLeftTicks));
      float   newCPD     = totalTicks / (2.0f * ekfMaxAngleDeg);
      // Apply
      keyaZeroTicks   = newZero;
      keyaTicksPerDeg = newCPD;
      // Persist
      EEPROM.put(EEPROM_ADDR_KEYA_TICKS, keyaTicksPerDeg);
      EEPROM.put(EEPROM_ADDR_KEYA_ZERO,  keyaZeroTicks);
      ekfFullReset();
      Serial.print("[CAL] New center (zeroTicks): "); Serial.println(newZero);
      Serial.print("[CAL] New ticksPerDeg: ");        Serial.println(newCPD, 2);
      Serial.println("[CAL] EKF reset. Drive straight to verify.");
      calWizActive = false;
      calWizStep   = 0;
    }
    return calWizActive;
  }

  // Detection touche 'z' hors menu
 if (!azMenuActive) {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        // Commandes filtres EMA BNO (EY / ER)
        if (handleEmaSerialCommand(input)) return false;

        // Lancement assistant calibration butee-a-butee (CPD)
        if (input == "c" || input == "C") {
            calWizActive = true;
            calWizStep   = 1;
            Serial.println("[CAL] Lock-to-lock wizard. Steer FULL LEFT to mechanical stop, then press Enter.");
            return false;
        }

        // Activation menu auto-zero
        if (input == "z" || input == "Z") {
            azMenuActive = true;
            azMenuStep   = 0;
            azMenuPrint();
        }
    }
    return false;
}

  // Menu actif
  if (!Serial.available()) return true;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return true;

  if (azMenuStep == 0)
  {
    // Lecture du choix
    azMenuChoice = input.toInt();

    if (azMenuChoice == 18) {
      // Reset defaut
      azParams = { 1.0f, 0.8f, 1.0f, 500, 200, 3.0f, 12.0f, 1, 1, 0.05f, 0xA202 };
      EEPROM.put(EEPROM_ADDR_AZ_PARAMS, azParams);
      Serial.println("[AZ-MENU] Valeurs par defaut restaurees et sauvegardees.");
      azMenuPrint();
      return true;
    }

    if (azMenuChoice == 19) {
      Serial.println("[AZ-MENU] Menu ferme. Taper 'z' pour rouvrir.");
      azMenuActive = false;
      azMenuStep   = 0;
      return false;
    }

    if ((azMenuChoice >= 1 && azMenuChoice <= 14) || azMenuChoice == 17) {
      if (azMenuChoice == 8 || azMenuChoice == 9) {
        Serial.print("Nouvelle valeur (0=inactif, 1=actif) pour parametre ");
      } else {
        Serial.print("Nouvelle valeur pour parametre ");
      }
      Serial.print(azMenuChoice);
      Serial.println(" :");
      azMenuStep = 1;
    } else {
      Serial.println("Choix invalide.");
      azMenuPrint();
    }
  }
  else if (azMenuStep == 1)
  {
    // Lecture de la valeur
    float val = input.toFloat();

    switch (azMenuChoice) {
      case 1: azParams.speedMin    = val; break;
      case 2: azParams.yawRateMax  = val; break;
      case 3: azParams.gpsHdgMax   = val; break;
      case 4: azParams.timeSlowMs  = (uint32_t)val; break;
      case 5: azParams.timeFastMs  = (uint32_t)val; break;
      case 6: azParams.speedSlow   = val; break;
      case 7: azParams.speedFast   = val; break;
      case 8: azParams.useBno      = (val >= 1.0f) ? 1 : 0; break;
      case 9: azParams.useGps      = (val >= 1.0f) ? 1 : 0; break;
      case 10:
        if (val >= 0.001f && val <= 1.0f) azParams.beta = val;
        else Serial.println("Beta hors plage (0.001 - 1.0), ignore.");
        break;
      case 11:
        ekfWheelBase = val;
        ekfSaveParams();
        Serial.print("[EKF] wheelBase="); Serial.println(ekfWheelBase, 2);
        break;
      case 12:
        ekfRkin = val;
        ekfSaveParams();
        Serial.print("[EKF] Rkin="); Serial.println(ekfRkin, 6);
        break;
      case 13:
        ekfQdelta = val;
        ekfSaveParams();
        Serial.print("[EKF] Qdelta="); Serial.println(ekfQdelta, 6);
        break;
      case 14:
        ekfVmin = val;
        ekfSaveParams();
        Serial.print("[EKF] Vmin="); Serial.println(ekfVmin, 2);
        break;
      case 17:
        if (val >= 5.0f && val <= 90.0f) {
          ekfMaxAngleDeg = val;
          ekfSaveParams();
          Serial.print("[EKF] maxAngleDeg="); Serial.println(ekfMaxAngleDeg, 2);
        } else {
          Serial.println("Max angle hors plage (5 - 90), ignore.");
        }
        break;
    }

    // Only persist azParams for AZ cases (1-10); EKF cases (11-14) save via ekfSaveParams() above
    if (azMenuChoice >= 1 && azMenuChoice <= 10) {
      EEPROM.put(EEPROM_ADDR_AZ_PARAMS, azParams);
      Serial.println("[AZ-MENU] Sauvegarde OK.");
    }
    azMenuStep = 0;
    azMenuPrint();
  }

  return true;
}
