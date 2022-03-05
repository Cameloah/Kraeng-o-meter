//
// Created by koorj on 19.01.2022.
//


#include "Arduino.h"

#include "user_interface.h"
#include "version.h"
#include "module_memory.h"
#include "device_manager.h"
#include "linalg_core.h"
#include "wifi_debugger.h"

bool enable_serial_stream = false;
bool enable_serial_verbose = false;
bool enable_measurements = false;

void ui_config() {
    MODULE_MEMORY_ERROR_t retVal = MODULE_MEMORY_ERROR_UNKNOWN;
    // extract next word
    char* sub_key = strtok(nullptr, " \n");

    if (sub_key == nullptr) {
        Serial << "\nUngültiger Parameter. Mindestens einer der folgenden Parameter fehlt:\n" <<
            "konfiguriere -r [Koordinatensystem]              - Kalibrieren des Koordinatensystems. '1' für Gehäusekalibrierung und '2' für Schiffskalibrierung\n" <<
            "             -s ['b' 'h' 'bb' or 'stb'] [Wert]   - Setzen des eingegebenen Werts als Neigungswinkel-Schwellwert für Bug,\n" <<
            "                                                   Heck, Backbord oder Steuerbord des Schiffes\n" <<
            "                                        --auto   - speichere aktuellen Neigungswinkel als Schwellwert für Bug, Heck\n" <<
            "                                                   Backbord oder Steuerbord des Schiffes\n" <<
            "             --extern [Wert]                     - aktiviere/deaktiviere externes Alarmsignal\n" <<
            "             --filter [Wert]                     - Filterhärte, ein kleinerer Wert verstärkt den Tiefpassfilter und erzeugt mehr Robustheit,\n"
            "                                                   aber verlangsamt die Reaktionszeit des Geräts. Standardwert: 1.0\n\n";
        return;
    }

    // calibration of coordinate frames
    if (!strcmp(sub_key, "-r")) {
        // convert to int
        sub_key = strtok(nullptr, " \n");
        auto calib_mode = (int8_t) atof(sub_key);
        switch (calib_mode) {
            case 1:
                calibrate_device();
                break;
            case 2:
                calibrate_ship();
                break;
            default:
                Serial.println("\nUnbekannter Modus. Kalibrierungsmodi sind '1' für Gehäusekalibrierung und '2' für Schiffskalibrierung.");
        }
    }

    // set thresholds
    else if (!strcmp(sub_key, "-s")) {
        float user_input;
        char* axis_key = strtok(nullptr, " \n");
        sub_key = strtok(nullptr, " \n");

        if(sub_key == nullptr)
        {
            Serial.println("\nEs muss ein Wert übergeben werden.");
            return;
        }

        // import float
        user_input = atof(sub_key);

        if (!strcmp(sub_key, "--auto")) {
            // if option auto is provided copy data from main angles array
            if ((!strcmp(axis_key, "b")) || (!strcmp(axis_key, "h")))
                user_input = angles_x_y[0];
            else user_input = angles_x_y[1];
        }

        if(!strcmp(axis_key, "b"))
            config_data.threshold_angle_x[0] = user_input;

        else if (!strcmp(axis_key, "h"))
            config_data.threshold_angle_x[1] = user_input;

        else if (!strcmp(axis_key, "bb"))
                config_data.threshold_angle_y[0] = user_input;

        else if (!strcmp(axis_key, "stb"))
            config_data.threshold_angle_y[1] = user_input;

        else {
            Serial << "\nUngültiger Richtungsparameter. Der Syntax ist:\n" <<
                   "konfiguriere -s ['b' 'h' 'bb' or 'stb'] [Wert]   - Setzen des eingegebenen Werts als Neigungswinkel-Schwellwert für Bug,\n"
                   <<
                   "                                                   Heck, Backbord oder Steuerbord des Schiffes\n" <<
                   "                                        --auto   - speichere aktuellen Neigungswinkel als Schwellwert für Bug, Heck\n"
                   <<
                   "                                                   Backbord oder Steuerbord des Schiffes\n\n";
            return;
        }

        Serial << "Neigungswinkel-Schwellwert für '" << axis_key << "' auf " << user_input << " gesetzt.\n";
        if((retVal = module_memory_save_config()) != MODULE_MEMORY_ERROR_NO_ERROR) {
            Serial << "Fehler beim Speichern: " << retVal << "\n";
            return;
        };
    }

    // toggle external warning signal
    else if(!strcmp(sub_key, "--extern")) {
        sub_key = strtok(nullptr, " \n");
        uint8_t user_input = *sub_key - '0';

        switch (user_input) {
            case 1:
                config_data.flag_external_warning = true;
                Serial << "Externes Warnsignal eingeschaltet.\n";
                break;

            case 0:
                config_data.flag_external_warning = false;
                Serial << "Externes Warnsignal ausgeschaltet.\n";
                break;

            default:
                Serial.println("Ungültiger Parameter. Wert '1' oder '0' zum Einschalten bzw. Ausschalten.");
                return;
        }
        // save user data
        if((retVal = module_memory_save_config()) != MODULE_MEMORY_ERROR_NO_ERROR) {
            Serial << "Fehler beim Speichern: " << retVal << "\n";
            return;
        };
    }

    // set filter parameter for sensor data
    else if(!strcmp(sub_key, "--filter")) {
        sub_key = strtok(nullptr, " \n");
        // import float
        float user_input = atof(sub_key);

        config_data.filter_mavg_factor = user_input;
        Serial << "Filterhärte auf " << user_input << " gesetzt.\n";
        if((retVal = module_memory_save_config()) != MODULE_MEMORY_ERROR_NO_ERROR) {
            Serial << "Fehler beim Speichern: " << retVal << "\n";
            return;
        };
    }

    // configure wifi access
    else if(!strcmp(sub_key, "--wifi")) {
        Serial.println("Name/SSID des Netzwerks?");

        // flush serial buffer
        Serial.readString();

        // listen for user input
        while (!Serial.available())
        // wait a bit for transfer of all serial data
        delay(50);

        String wifi_user_input = Serial.readString();
        wifi_user_input.trim(); // remove trailing new line character

        if (wifi_user_input.length() > (sizeof (config_data.wifi_ssid) / sizeof (char))) {
            Serial.println("SSID zu lang!");
            return;
        }
        // save ssid
        wifi_user_input.toCharArray(config_data.wifi_ssid, (sizeof (config_data.wifi_ssid) / sizeof (char)));

        // ask for password
        Serial.println("Passwort?");

        // flush serial buffer
        Serial.readString();

        // listen for user input
        while (!Serial.available())
            // wait a bit for transfer of all serial data
            delay(50);

        wifi_user_input = Serial.readString();
        wifi_user_input.trim(); // remove trailing new line character

        if (wifi_user_input.length() > (sizeof (config_data.wifi_pw) / sizeof (char))) {
            Serial.println("Passwort zu lang!");
            return;
        }
        wifi_user_input.toCharArray(config_data.wifi_pw, (sizeof (config_data.wifi_pw) / sizeof (char)));

        Serial << "WiFi-Daten gespeichert. SSID: '" << config_data.wifi_ssid << "', PW: '" << config_data.wifi_pw << "'\n";
        if((retVal = module_memory_save_config()) != MODULE_MEMORY_ERROR_NO_ERROR) {
            Serial << "Fehler beim Speichern: " << retVal << "\n";
            return;
        };
    }

    // handle fw updates
    else if(!strcmp(sub_key, "--update")) {
        sub_key = strtok(nullptr, " \n");

        if (sub_key == nullptr)
            strcpy(sub_key, "");

        if(!strcmp(sub_key, "--auto")) {
            sub_key = strtok(nullptr, " \n");
            if (sub_key == nullptr)
                strcpy(sub_key, "");

            auto user_input = (int8_t) atof(sub_key);

            switch (user_input) {
                case 1:
                    config_data.flag_auto_update = true;
                    Serial << "Automatische Updates eingeschaltet.\n";
                    break;

                case 0:
                    config_data.flag_auto_update = false;
                    Serial << "Automatische Updates ausgeschaltet.\n";
                    break;

                default:
                    Serial.println("Ungültiger Parameter. Wert '1' oder '0' zum Einschalten bzw. Ausschalten.");
                    return;
            }
            if((retVal = module_memory_save_config()) != MODULE_MEMORY_ERROR_NO_ERROR) {
                Serial << "Fehler beim Speichern: " << retVal << "\n";
                return;
            };
            return;
        }

        config_data.flag_check_update = true;
        if((retVal = module_memory_save_config()) != MODULE_MEMORY_ERROR_NO_ERROR) {
            Serial << "Fehler beim Speichern: " << retVal << "\n";
            return;
        };
        Serial.println("ESP32 wird neu gestartet und auf Updates überprüft.");
        esp_restart();
    }

    else {
        Serial << "\nUngültiger Parameter. Mindestens einer der folgenden Parameter fehlt:\n" <<
        "konfiguriere -r [Koordinatensystem]              - Kalibrieren des Koordinatensystems. '1' für Gehäusekalibrierung und '2' für Schiffskalibrierung\n" <<
        "             -s ['b' 'h' 'bb' or 'stb'] [Wert]   - Setzen des eingegebenen Werts als Neigungswinkel-Schwellwert für Bug,\n" <<
        "                                                   Heck, Backbord oder Steuerbord des Schiffes\n" <<
        "                                        --auto   - speichere aktuellen Neigungswinkel als Schwellwert für Bug, Heck\n" <<
        "                                                   Backbord oder Steuerbord des Schiffes\n" <<
        "             --extern [Wert]                     - aktiviere/deaktiviere externes Alarmsignal\n" <<
        "             --filter [Wert]                     - Filterhärte, ein kleinerer Wert verstärkt den Tiefpassfilter und erzeugt mehr Robustheit,\n"
        "                                                   aber verlangsamt die Reaktionszeit des Geräts. Standardwert: 1.0\n\n";
    }
}

void ui_mode() {
    // extract next word
    char* sub_key = strtok(nullptr, " \n");
    // convert to int
    auto new_mode = (int8_t) atof(sub_key);
    // check if within boundaries
    if (new_mode >= 0 && new_mode <= 2) {
        put_state_mode(new_mode);
        Serial << "Ändere Ausgabemodus auf " << config_data.state_mode << "\n";
    }
    else Serial.println("\nUnbekannter Ausgabemodus. Modi sind '0' für Sensor-, '1' für Geräte- und '2' für Schiffskoordinatensystem.");
}

void ui_stream() {
    // extract next word
    char* sub_key = strtok(nullptr, " \n");

    if (sub_key == nullptr) {
        Serial.println("\nFehlender Parameter. Optionen sind '--start' und '--stop'.");
        return;
    }

    if (!strcmp(sub_key, "--start")) {
        sub_key = strtok(nullptr, " \n");
        enable_serial_stream = true;

        if (sub_key == nullptr)
            return;

        if (!strcmp(sub_key, "--erweitert"))
            enable_serial_verbose = true;

        else
            Serial.println("Unbekannte Option.");
    }

    else if (!strcmp(sub_key, "--stop")) {
        enable_serial_stream = false;
        enable_serial_verbose = false;
    }

    else
        Serial.println("\nUnbekannter Parameter. Optionen sind '--start' und '--stop'.");
}

void ui_memory() {
    char* sub_key = strtok(nullptr, " \n");

    if (sub_key == nullptr) {
        Serial << "\nUngültiger Parameter. Mindestens einer der folgenden Parameter fehlt:\n" <<
        "speicher [Option]       - Zugriff auf gespeicherte Einstellungen über '--alles', zurücksetzen ALLER Einstellungen mit '--löschen'\n\n";
        return;
    }

    if (!strcmp(sub_key, "--alles")) {
        Serial << "\n";
        switch(module_memory_load_config()) {
            case MODULE_MEMORY_ERROR_READ:
                Serial.println("Keine Daten gefunden.");
                break;

            case MODULE_MEMORY_ERROR_NO_ERROR:
                Serial << "Konfigurationsdaten aus Speicher: \n" <<
                "WiFi SSID:                               " << config_data.wifi_ssid << "\n" <<
                "WiFi passwort:                           " << config_data.wifi_pw << "\n" <<
                "Automatische Updates:                    " << config_data.flag_auto_update << "\n" <<
                "Bei Neustart auf Updates überprüfen:     " << config_data.flag_check_update << "\n" <<
                "Externes Warnsignal:                     " << config_data.flag_external_warning << '\n' <<
                "Gehäusesystem kalibriert:                " << config_data.flag_device_calibration_state << '\n' <<
                "Schiffssystem kalibriert:                " << config_data.flag_ship_calibration_state << '\n' <<
                "Schwellwert für Bug und Heck             " << config_data.threshold_angle_x[0] << "°, " << config_data.threshold_angle_x[1] << "°\n" <<
                "Schwellwert für Backbord und Steuerbord  " << config_data.threshold_angle_y[0] << "°, " << config_data.threshold_angle_y[1] << "°\n" <<
                "Filterhärte:                             " << config_data.filter_mavg_factor << '\n' <<
                "Ausgabemodus:                            " << config_data.state_mode << '\n' <<
                "Gehäuserotationsmatrix R_1_0:            " << config_data.rot_mat_1_0 << '\n' <<
                "Schiffsrotationsmatrix R_2_1:            " << config_data.rot_mat_2_1 << '\n';
                break;

            default:
                Serial.println("Unbekannter Speicherfehler.");
        }
    }

    else if (!strcmp(sub_key, "--löschen")) {

        Serial.println("Lösche Flash-Speicher des ESP32...");
        module_memory_erase_namespace();
        // print error if function returns
        Serial.println("Fehler beim Löschen!");
    }

    else Serial << "\nUngültiger Parameter. Mindestens einer der folgenden Parameter fehlt:\n" <<
                "speicher [Option]       - Zugriff auf gespeicherte Einstellungen über '--alles', zurücksetzen ALLER Einstellungen mit '--löschen'\n\n";
}

void ui_info() {
    Serial << "Kräng-o-meter Version " << FW_VERSION_MAJOR << "." << FW_VERSION_MINOR << "." << FW_VERSION_PATCH << "\n";
}

void ui_debug() {
    char* sub_key = strtok(nullptr, " \n");

    if (sub_key == nullptr) {
        Serial << "\nUngültiger Befehl. Mindestens einer der folgenden Parameter fehlt:\n" <<
        "debug --einzel              - starte einzelne Messung\n" <<
        "      --reinit              - Reinitialisiere den Sensor\n" <<
        "      --reboot              - Neustarten des Geräts\n" <<
        "      --aktiviere           - Aktivieren oder Deaktivieren des Sensors\n\n";
    }

    if (!strcmp(sub_key, "--einzel"))
        Serial << "Einzelne Messung: " << device_manager_get_accel_raw() << "\n";

    else if (!strcmp(sub_key, "--reinit"))
        device_manager_init_imu();

    else if (!strcmp(sub_key, "--reboot"))
        esp_restart();

    else if (!strcmp(sub_key, "--aktiviere")) {
        sub_key = strtok(nullptr, " \n");

        if(sub_key == nullptr)
            return;
        
        // convert to int
        uint8_t user_int = *sub_key - '0';
        enable_measurements = user_int;
    }

    else {
        Serial << "\nUngültiger Befehl. Mindestens einer der folgenden Parameter fehlt:\n" <<
               "debug --einzel              - starte einzelne Messung\n" <<
               "      --reinit              - Reinitialisiere den Sensor\n" <<
               "      --reboot              - Neustarten des Geräts\n" <<
               "      --aktiviere           - Aktivieren oder Deaktivieren des Sensors\n\n";
    }
}

void ui_serial_comm_handler() {
    // listen for user input
    if (Serial.available())
        delay(50); // wait a bit for transfer of all serial data
    uint8_t rx_available_bytes = Serial.available();
    if (rx_available_bytes > 0) {
        // import entire string until "\n"
        char rx_user_input[rx_available_bytes];
        Serial.readBytes(rx_user_input, rx_available_bytes);

        // extract first word as command key
        char* rx_command_key = strtok(rx_user_input, " \n");

        // catch exception where no token was sent
        if (rx_command_key == nullptr)
            return;

        if (!strcmp(rx_command_key, "modus"))
            ui_mode();

        else if (!strcmp(rx_command_key, "konfiguriere"))
            ui_config();

        else if (!strcmp(rx_command_key, "stream"))
            ui_stream();

        else if (!strcmp(rx_command_key, "speicher"))
            ui_memory();

        else if (!strcmp(rx_command_key, "info"))
            ui_info();

        else if (!strcmp(rx_command_key, "debug"))
            ui_debug();

        else if (!strcmp(rx_command_key, "hilfe")) {
            Serial << "\nListe der verfügbaren Befehle:\n" <<
                    "konfiguriere -r [Koordinatensystem]              - Kalibrieren des Koordinatensystems. '1' für Gehäusekalibrierung und '2' für Schiffskalibrierung\n" <<
                    "             -s ['b' 'h' 'bb' or 'stb'] [Wert]   - Setzen des eingegebenen Werts als Neigungswinkel-Schwellwert für Bug,\n" <<
                    "                                                   Heck, Backbord oder Steuerbord des Schiffes\n" <<
                    "                                        --auto   - speichere aktuellen Neigungswinkel als Schwellwert für Bug, Heck\n" <<
                    "                                                   Backbord oder Steuerbord des Schiffes\n" <<
                    "             --extern [Wert]                     - aktiviere/deaktiviere externes Alarmsignal\n" <<
                    "             --filter [Wert]                     - Filterhärte, ein kleinerer Wert verstärkt den Tiefpassfilter und erzeugt mehr Robustheit,\n"
                    "                                                   aber verlangsamt die Reaktionszeit des Geräts. Standardwert: 1.0\n\n" <<
                    "stream [Option]                                  - Starten und Stoppen des Datenstreams über USB mit '--start' oder '--stop'\n\n" <<
                    "modus [Koordinatensystem]                        - Ändern des Ausgabemodus. Modi sind '0' für Sensor-, '1' für Geräte- und '2' für Schiffskoordinatensystem\n\n" <<
                    "speicher [Option]                                - Zugriff auf gespeicherte Einstellungen über '--alles', zurücksetzen ALLER Einstellungen mit '--löschen'\n\n" <<
                    "info                                             - Rückgabe der Geräteinformationen wie z.b. der Firmware-Version\n\n";
        }

        else {
            // unknown command
            Serial.println("\nUnbekannter Befehl. Nutzen Sie 'hilfe' um eine Liste aller verfügbaren Befehle und deren Syntax zu erhalten.");
        }

        // flush serial buffer
        Serial.readString();

        if (enable_serial_stream)
            delay(1000); // for readability when data stream is active
            Serial << '\n';
    }
}