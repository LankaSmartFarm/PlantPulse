#include <inttypes.h>
#include <dirent.h> 
#include <stdint.h>  
#include <stdbool.h> 
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "esp_partition.h"
#include <errno.h>
#include "esp_system.h"
#include "sys/stat.h"
#include "wear_levelling.h"
#include "mqtt.h"


static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
#define PARTITION_LABEL "storage"  // Replace "storage" with the actual partition label if it's different
// Define the path where the partition is mounted
#define BASE_PATH "/fatfs"
#define MOUNT_POINT "/fatfs"
#define LOG_PATH "/fatfs/log"
#define CREDENTIAL_PATH "/fatfs/credential"

esp_err_t init_fatfs(void);
void setup_directories(void);
void checkPendingLogs(void);

extern esp_err_t savePayload(const M_payload_t *payload);



