#include "FS.h"

extern QueueHandle_t mqttAckQueue;
static const char *TAG = "FAT";

// Function to initialize and mount FAT filesystem
esp_err_t init_fatfs(void)
{
    esp_err_t ret;
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 10,
        .format_if_mount_failed = true,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE};

    // Mount FATFS partition
    ret = esp_vfs_fat_spiflash_mount(MOUNT_POINT, "storage", &mount_config, &s_wl_handle);
    if (ret != ESP_OK)
    {
        // ESP_LOGE("init_fatfs", "Failed to mount FATFS (%s)", esp_err_to_name(ret));
        return ret;
    }
    // ESP_LOGI("init_fatfs", "FATFS mounted successfully");

    return ESP_OK;
}

static void create_directory(const char *dir_path)
{
    struct stat st;
    if (stat(dir_path, &st) == 0)
    {
        if (S_ISDIR(st.st_mode))
        {
            ESP_LOGI("FAT", "Directory %s already exists", dir_path);
        }
        else
        {
            ESP_LOGE("FAT", "%s exists but is not a directory", dir_path);
        }
    }
    else
    {
        if (mkdir(dir_path, 0777) != 0 && errno != EEXIST)
        {
            ESP_LOGE("FAT", "Failed to create directory: %s", dir_path);
        }
        else
        {
            ESP_LOGI("FAT", "Directory created: %s", dir_path);
        }
    }
}
void setup_directories(void)
{
    create_directory(LOG_PATH);
    create_directory(CREDENTIAL_PATH);
}

esp_err_t savePayload(const M_payload_t *payload)
{
    char file_path[128];
    struct stat st;
    // Build file path using timestamp
    snprintf(file_path, sizeof(file_path), "/fatfs/log/%llu.bin", payload->timestamp);
    // Check if file already exists
    if (stat(file_path, &st) == 0)
    {
        ESP_LOGW("FAT", "File %s already exists. Skipping save.", file_path);
        return ESP_OK; // Already saved
    }

    // Create file and write binary payload
    FILE *f = fopen(file_path, "wb");
    if (!f)
    {
        ESP_LOGE("FAT", "Failed to open file for writing: %s", file_path);
        return ESP_FAIL;
    }

    size_t written = fwrite(payload, 1, sizeof(M_payload_t), f);
    fclose(f);

    if (written != sizeof(M_payload_t))
    {
        ESP_LOGE("FAT", "Failed to write full payload to file");
        return ESP_FAIL;
    }

    ESP_LOGI("FAT", "Payload saved to %s (%u bytes)", file_path, sizeof(M_payload_t));
    return ESP_OK;
}

static esp_err_t processPendingLogs(void)
{
    DIR *dir = opendir(LOG_PATH);
    if (!dir)
    {
        ESP_LOGE("FAT", "Failed to open directory: %s", LOG_PATH);
        return ESP_FAIL;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL)
    {
        if (!strstr(entry->d_name, ".bin"))
            continue;

        char file_path[128];
        strncpy(file_path, LOG_PATH, sizeof(file_path) - 1);
        file_path[sizeof(file_path) - 1] = '\0'; 
        strncat(file_path, "/", sizeof(file_path) - strlen(file_path) - 1);
        strncat(file_path, entry->d_name, sizeof(file_path) - strlen(file_path) - 1);

        FILE *f = fopen(file_path, "rb");
        if (!f)
        {
            ESP_LOGE("FAT", "Failed to open file: %s", file_path);
            continue;
        }

        M_payload_t payload;
        size_t read = fread(&payload, 1, sizeof(M_payload_t), f);
        fclose(f);

        if (read != sizeof(M_payload_t))
        {
            ESP_LOGW("FAT", "Corrupt file: %s", file_path);
            continue;
        }

        if (xQueueSend(modbus_payload_queue, &payload, pdMS_TO_TICKS(100)) == pdPASS)
        {
            ESP_LOGI("FAT", "Queued payload from %s", file_path);

            // Wait for file path ack
            char ack_path[128] = {0};
            if (xQueueReceive(mqttAckQueue, &ack_path, pdMS_TO_TICKS(10000)) == pdPASS)
            {
                if (strlen(ack_path) == 0)
                {
                    ESP_LOGW("FAT", "Publish failed for %s, keeping file", file_path);
                }
                else if (strcmp(ack_path, file_path) == 0)
                {
                    ESP_LOGI("FAT", "ACK received for %s, deleting", file_path);
                    f_unlink(file_path);
                }
            }
            else
            {
                ESP_LOGW("FAT", "No ACK for %s, keeping", file_path);
            }
        }
        else
        {
            ESP_LOGW("FAT", "Queue full, skipping %s", file_path);
        }
    }

    closedir(dir);
    return ESP_OK;
}
void checkPendingLogs(void)
{
    DIR *dir = opendir(LOG_PATH);
    if (!dir) {
        ESP_LOGE("FAT", "Failed to open log directory: %s", LOG_PATH);
        return;
    }

    struct dirent *entry;
    bool has_logs = false;

    while ((entry = readdir(dir)) != NULL) {
        if (strstr(entry->d_name, ".bin")) {
            has_logs = true;
            break;  // Found at least one .bin file
        }
    }

    closedir(dir);

    if (has_logs) {
        ESP_LOGI("FAT", "Pending log files found. Processing...");
        processPendingLogs();
    } else {
        ESP_LOGI("FAT", "No pending log files to process.");
    }
}
