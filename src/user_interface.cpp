//
// Created by Cameloah on 19.01.2022.
//


#include "Arduino.h"

#include "user_interface.h"
#include "version.h"
#include "device_manager.h"
#include "linalg_core.h"

#include "github_update.h"
#include "ram_log.h"
#include "network_manager.h"

bool enable_serial_stream = false;
bool enable_serial_verbose = false;
bool enable_measurements = true;

String matrixToString(const Matrix<3, 3>& matrix) {
    String result = "[";
    for (int i = 0; i < 3; ++i) {
        result += "[";
        for (int j = 0; j < 3; ++j) {
            result += String(matrix(i, j));
            if (j < 2) result += ", ";
        }
        result += "]";
        if (i < 2) result += ", ";
    }
    result += "]";
    return result;
}

void ui_config() {
    // extract next word
    char* sub_key = strtok(nullptr, " \n");

    if (sub_key == nullptr) {
        DualSerial << "\nUngültiger Parameter. Mindestens einer der folgenden Parameter fehlt:\n" <<
            "konfiguriere -r [Koordinatensystem] --auto         - Kalibrieren des Koordinatensystems. '1' für Gehäusekalibrierung und '2' für Schiffskalibrierung\n" <<
            "                                    [matrix]       - Manuelles eingeben der Rotationsmatrizen im Format [[x,x,x],[x,x,x],[x,x,x]]\n" <<
            "             -s ['b' 'h' 'bb' oder 'stb'] [Wert]   - Setzen des eingegebenen Werts als Neigungswinkel-Schwellwert für Bug,\n" <<
            "                                                     Heck, Backbord oder Steuerbord des Schiffes\n" <<
            "                                          --auto   - speichere aktuellen Neigungswinkel als Schwellwert für Bug, Heck\n" <<
            "                                                     Backbord oder Steuerbord des Schiffes\n" <<
            "             --extern [Wert]                       - aktiviere/deaktiviere externes Alarmsignal\n" <<
            "             --filter [Wert]                       - Filterhärte, ein kleinerer Wert verstärkt den Tiefpassfilter und erzeugt mehr Robustheit,\n"
            "                                                     aber verlangsamt die Reaktionszeit des Geräts. Standardwert: 1.0\n"
            "             --wifi                                - konfigurieren des WLAN Netzwerknamens (SSID) und des Passworts\n"
            "             --update                              - einmaliges Überprüfen auf Updates\n"
            "                      --auto ['1' oder '0']        - Automatisches Überprüfen auf Updates beim Starten ein oder ausschalten\n"
            "                      --version [v#.#.#]           - Auf spezifische Version updaten, wenn verfügbar\n\n";
        return;
    }

    // calibration of coordinate frames
    if (!strcmp(sub_key, "-r")) {
        // convert to int
        sub_key = strtok(nullptr, " \n");
        auto index_rotmat = (int8_t) atof(sub_key);

        // get user intention
        sub_key = strtok(nullptr, " \n");
        if(sub_key == nullptr)
        {
            DualSerial.println("\nEs muss ein Wert übergeben werden.");
            return;
        }

        // do auto calibration
        if (!strcmp(sub_key, "--auto")) {
            switch (index_rotmat) {
                case 1:
                    calibrate_device();
                    break;
                case 2:
                    calibrate_ship();
                    break;
                default:
                    DualSerial.println(
                            "\nUnbekannter Modus. Kalibrierungsmodi sind '1' für Gehäusekalibrierung und '2' für Schiffskalibrierung.");
            }
            return;
        }

        char* input_rotmat = strtok(sub_key, "[], \n");
        float rot_mat[9];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if(input_rotmat == nullptr) {
                    DualSerial.println("Unzulässige Werte");
                    return;
                }
                if (index_rotmat == 1)
                    rot_mat[i*3 + j] = atof(input_rotmat);
                else
                    rot_mat[i*3 + j] = atof(input_rotmat);
                input_rotmat = strtok(nullptr, "[], \n");
            }
        }

        esp_err_t retval;

        if (index_rotmat == 1) {
            retval = config_data.set("rot_mat_1_0", (uint8_t*) rot_mat, true);
        } else {
            retval = config_data.set("rot_mat_2_1", (uint8_t*) rot_mat, true);
        }

        if (retval == ESP_OK)
            DualSerial << "Rotationsmatrix gespeichert.\n";
        else {
            ram_log_notify(RAM_LOG_ERROR_MEMORY, (uint32_t) retval);
            DualSerial << "Fehler beim Speichern.\n";
        }
    }

    // set thresholds
    else if (!strcmp(sub_key, "-s")) {
        float user_input;
        char* axis_key = strtok(nullptr, " \n");
        sub_key = strtok(nullptr, " \n");

        if(sub_key == nullptr)
        {
            DualSerial.println("\nEs muss ein Wert übergeben werden.");
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

        float* threshold_angle_x = static_cast<float*>(config_data.get("th_angle_x"));
        float* threshold_angle_y = static_cast<float*>(config_data.get("th_angle_y"));

        if(!strcmp(axis_key, "b"))
            threshold_angle_x[0] = user_input;

        else if (!strcmp(axis_key, "h"))
            threshold_angle_x[1] = user_input;

        else if (!strcmp(axis_key, "bb"))
                threshold_angle_y[0] = user_input;

        else if (!strcmp(axis_key, "stb"))
            threshold_angle_y[1] = user_input;

        else {
            DualSerial << "\nUngültiger Richtungsparameter. Der Syntax ist:\n" <<
                   "konfiguriere -s ['b' 'h' 'bb' oder 'stb'] [Wert]   - Setzen des eingegebenen Werts als Neigungswinkel-Schwellwert für Bug,\n" <<
                   "                                                     Heck, Backbord oder Steuerbord des Schiffes\n" <<
                   "                                          --auto   - speichere aktuellen Neigungswinkel als Schwellwert für Bug, Heck\n" <<
                   "                                                     Backbord oder Steuerbord des Schiffes\n\n";
            return;
        }

        DualSerial << "Neigungswinkel-Schwellwert für '" << axis_key << "' auf " << user_input << " gesetzt.\n";

        esp_err_t retval = config_data.save("th_angle_x");
        if (retval != ESP_OK) {
            ram_log_notify(RAM_LOG_ERROR_MEMORY, (uint32_t) retval);
            DualSerial << "Fehler beim Speichern.\n";
            return;
        }

        retval = config_data.save("th_angle_y");
        if (retval != ESP_OK) {
            ram_log_notify(RAM_LOG_ERROR_MEMORY, (uint32_t) retval);
            DualSerial << "Fehler beim Speichern.\n";
            return;
        }
    }

    // toggle external warning signal
    else if(!strcmp(sub_key, "--extern")) {
        sub_key = strtok(nullptr, " \n");
        uint8_t user_input = *sub_key - '0';

        switch (user_input) {
            case 1:
                config_data.set("ext_warning", true);
                DualSerial << "Externes Warnsignal eingeschaltet.\n";
                break;

            case 0:
                config_data.set("ext_warning", false);
                DualSerial << "Externes Warnsignal ausgeschaltet.\n";
                break;

            default:
                DualSerial.println("Ungültiger Parameter. Wert '1' oder '0' zum Einschalten bzw. Ausschalten.");
                return;
        }
        // save user data
        esp_err_t retval = config_data.save("ext_warning");
        if (retval != ESP_OK) {
            ram_log_notify(RAM_LOG_ERROR_MEMORY, (uint32_t) retval);
            DualSerial << "Fehler beim Speichern.\n";
            return;
        }
    }

    // set filter parameter for sensor data
    else if(!strcmp(sub_key, "--filter")) {
        sub_key = strtok(nullptr, " \n");
        // import float
        float user_input = atof(sub_key);

        esp_err_t retval = config_data.set("filter_factor", user_input, true);
        if (retval != ESP_OK) {
            ram_log_notify(RAM_LOG_ERROR_MEMORY, (uint32_t) retval);
            DualSerial << "Fehler beim Speichern.\n"; 
        }
    }

    else {
        DualSerial << "\nUngültiger Parameter. Mindestens einer der folgenden Parameter fehlt:\n" <<
        "konfiguriere -r [Koordinatensystem] --auto         - Kalibrieren des Koordinatensystems. '1' für Gehäusekalibrierung und '2' für Schiffskalibrierung\n" <<
        "                                    [matrix]       - Manuelles eingeben der Rotationsmatrizen im Format [[x,x,x],[x,x,x],[x,x,x]]\n" <<
        "             -s ['b' 'h' 'bb' oder 'stb'] [Wert]   - Setzen des eingegebenen Werts als Neigungswinkel-Schwellwert für Bug,\n" <<
        "                                                     Heck, Backbord oder Steuerbord des Schiffes\n" <<
        "                                          --auto   - speichere aktuellen Neigungswinkel als Schwellwert für Bug, Heck\n" <<
        "                                                     Backbord oder Steuerbord des Schiffes\n" <<
        "             --extern [Wert]                       - aktiviere/deaktiviere externes Alarmsignal\n" <<
        "             --filter [Wert]                       - Filterhärte, ein kleinerer Wert verstärkt den Tiefpassfilter und erzeugt mehr Robustheit,\n"
        "                                                     aber verlangsamt die Reaktionszeit des Geräts. Standardwert: 1.0\n";
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
        DualSerial << "Ändere Ausgabemodus auf " << *config_data.getInt("state_mode") << "\n";
    }
    else DualSerial.println("\nUnbekannter Ausgabemodus. Modi sind '0' für Sensor-, '1' für Geräte- und '2' für Schiffskoordinatensystem.");
}

void ui_stream() {
    // extract next word
    char* sub_key = strtok(nullptr, " \n");

    if (sub_key == nullptr) {
        DualSerial.println("\nFehlender Parameter. Optionen sind '--start' und '--stop'.");
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
            DualSerial.println("Unbekannte Option.");
    }

    else if (!strcmp(sub_key, "--stop")) {
        enable_serial_stream = false;
        enable_serial_verbose = false;
    }

    else
        DualSerial.println("\nUnbekannter Parameter. Optionen sind '--start' und '--stop'.");
}

void ui_memory() {
    char* sub_key = strtok(nullptr, " \n");

    if (sub_key == nullptr) {
        DualSerial << "\nUngültiger Parameter. Mindestens einer der folgenden Parameter fehlt:\n" <<
        "speicher [Option]       - Zugriff auf gespeicherte Einstellungen über '--alles', zurücksetzen ALLER Einstellungen mit '--löschen'\n\n";
        return;
    }

    if (!strcmp(sub_key, "--alles")) {
        DualSerial << "\n";
        String payload;

        switch(config_data.loadAll()) {
            case ESP_ERR_NOT_FOUND:
                DualSerial.println("Keine Daten gefunden.");
                break;

            case ESP_OK:
                payload = "Konfigurationsdaten aus Speicher: \n";
                payload += "Externes Warnsignal:                     " + String(*config_data.getBool("ext_warning")) + '\n' +
                "Gehäusesystem kalibriert:                " + String(*config_data.getBool("calib_device")) + '\n' +
                "Schiffssystem kalibriert:                " + String(*config_data.getBool("calib_ship")) + '\n' +
                "Schwellwert für Bug und Heck             " + String(static_cast<float*>(config_data.get("th_angle_x"))[0]) + "°, " + String(static_cast<float*>(config_data.get("th_angle_x"))[1]) + "°\n" +
                "Schwellwert für Backbord und Steuerbord  " + String(static_cast<float*>(config_data.get("th_angle_y"))[0]) + "°, " + String(static_cast<float*>(config_data.get("th_angle_y"))[1]) + "°\n" +
                "Filterhärte:                             " + String(*config_data.getFloat("filter_factor")) + '\n' +
                "Ausgabemodus:                            " + String(*config_data.getInt("state_mode")) + '\n' +
                "Gehäuserotationsmatrix R_1_0:            " + matrixToString(*static_cast<Matrix<3, 3>*>(config_data.get("rot_mat_1_0"))) + '\n' +
                "Schiffsrotationsmatrix R_2_1:            " + matrixToString(*static_cast<Matrix<3, 3>*>(config_data.get("rot_mat_2_1"))) + '\n';
                DualSerial.print(payload);
                break;

            default:
                DualSerial.println("Unbekannter Speicherfehler.");
        }
    }

    else if (!strcmp(sub_key, "--löschen")) {

        DualSerial.println("Funktion nicht verfügbar.");
    }

    else DualSerial << "\nUngültiger Parameter. Mindestens einer der folgenden Parameter fehlt:\n" <<
                "speicher [Option]       - Zugriff auf gespeicherte Einstellungen über '--alles', zurücksetzen ALLER Einstellungen mit '--löschen'\n\n";
}

String ui_info() {
    String fw_version;
    String info;

    // Construct the firmware version string
    fw_version += "\nFirmware version:   v"
    + String(FW_VERSION_MAJOR) + "." + String(FW_VERSION_MINOR) + "." + String(FW_VERSION_PATCH) + "\n";

    // get reset reason
    String boot_msg = String(esp_reset_reason(), HEX);
    if (boot_msg.length() == 1)
        boot_msg = "0" + boot_msg;
    boot_msg = "0x" + boot_msg;

    // Append additional system information to the info string.
    info = "Wifi mode:          " + network_manager_get_mode() + "\n"
    + "Wifi connected to:  " + (WiFi.isConnected() ? WiFi.SSID() : "not connected") + "\n"
    + "IP-address:         " + (WiFi.isConnected() ? WiFi.localIP().toString() : "") + "\n"
    + "Uptime:             " + ram_log_time_str(esp_timer_get_time()) + "\n"
    + "Boot message:       " + boot_msg + "\n\n";

    DualSerial.print(fw_version + info);

    ram_log_print_log();

    return fw_version;
}

void ui_debug() {
    char* sub_key = strtok(nullptr, " \n");

    if (sub_key == nullptr) {
        DualSerial << "\nUngültiger Befehl. Mindestens einer der folgenden Parameter fehlt:\n" <<
        "debug --einzel              - starte einzelne Messung\n" <<
        "      --reinit              - Reinitialisiere den Sensor\n" <<
        "      --reboot              - Neustarten des Geräts\n" <<
        "      --aktiviere           - Aktivieren oder Deaktivieren des Sensors\n\n";
    }

    if (!strcmp(sub_key, "--einzel"))
        DualSerial << "Einzelne Messung: " << device_manager_get_accel_raw() << "\n";

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
        DualSerial << "\nUngültiger Befehl. Mindestens einer der folgenden Parameter fehlt:\n" <<
               "debug --einzel              - starte einzelne Messung\n" <<
               "      --reinit              - Reinitialisiere den Sensor\n" <<
               "      --reboot              - Neustarten des Geräts\n" <<
               "      --aktiviere           - Aktivieren oder Deaktivieren des Sensors\n\n";
    }
}



void ui_serial_comm_handler() {
    // listen for user input
    if (DualSerial.available())
        delay(50); // wait a bit for transfer of all serial data
    uint8_t rx_available_bytes = DualSerial.available();
    if (rx_available_bytes > 0) {
        // import entire string until "\n"
        char rx_user_input[rx_available_bytes];
        DualSerial.readBytes(rx_user_input, rx_available_bytes);

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
            DualSerial << "\nListe der verfügbaren Befehle:\n" <<
                    "konfiguriere -r [Koordinatensystem] --auto         - Kalibrieren des Koordinatensystems. '1' für Gehäusekalibrierung und '2' für Schiffskalibrierung\n" <<
                    "                                    [matrix]       - Manuelles eingeben der Rotationsmatrizen im Format [[x,x,x],[x,x,x],[x,x,x]]\n" <<
                    "             -s ['b' 'h' 'bb' oder 'stb'] [Wert]   - Setzen des eingegebenen Werts als Neigungswinkel-Schwellwert für Bug,\n" <<
                    "                                                     Heck, Backbord oder Steuerbord des Schiffes\n" <<
                    "                                          --auto   - Speichere aktuellen Neigungswinkel als Schwellwert für Bug, Heck\n" <<
                    "                                                     Backbord oder Steuerbord des Schiffes\n" <<
                    "             --extern [Wert]                       - aktiviere/deaktiviere externes Alarmsignal\n" <<
                    "             --filter [Wert]                       - Filterhärte, ein kleinerer Wert verstärkt den Tiefpassfilter und erzeugt mehr Robustheit,\n"
                    "                                                     aber verlangsamt die Reaktionszeit des Geräts. Standardwert: 1.0\n"
                    "stream [Option]                                    - Starten und Stoppen des Datenstreams über USB mit '--start' oder '--stop'\n\n" <<
                    "modus [Koordinatensystem]                          - Ändern des Ausgabemodus. Modi sind '0' für Sensor-, '1' für Geräte- und '2' für Schiffskoordinatensystem\n\n" <<
                    "speicher [Option]                                  - Zugriff auf gespeicherte Einstellungen über '--alles', zurücksetzen ALLER Einstellungen mit '--löschen'\n\n" <<
                    "info                                               - Rückgabe der Geräteinformationen und Logbuch\n\n";
        }

        else {
            // unknown command
            DualSerial.println("\nUnbekannter Befehl. Nutzen Sie 'hilfe' um eine Liste aller verfügbaren Befehle und deren Syntax zu erhalten.");
        }

        // flush serial buffer
        DualSerial.readString();

        DualSerial << '\n';
        if (enable_serial_stream)
            delay(1000); // for readability when data stream is active
    }
}