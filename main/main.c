#include <stdbool.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "Fusion.h" // Assume Fusion.h was also refactored to fusion.h
#include "mpu6050.h"
#include "i2c.h"
#include "epmfsm.h"

/* macros */

/* enums */
typedef enum {
    StateIndex_MPU_Initialization,
    StateIndex_MPU_Calibration,
    StateIndex_MPU_Running,
    StateIndex_GPS_Initialization,
    StateIndex_GPS_Running,
    StateIndex_NUM_States
} StateIndex;

typedef enum {
    RelationIndex_MPU_Initialized,
    RelationIndex_MPU_Calibrated,
    RelationIndex_GPS_Initialized,
    RelationIndex_NUM_Relations
} RelationIndex;

/* structs */
typedef struct {
    bool initialized;
    bool calibrated;
    bool running;
    float ax, ay, az; // Accelerometer data
    float gx, gy, gz; // Gyroscope data
    float oax, oay, oaz; // Accelerometer offsets
    float ogx, ogy, ogz; // Gyroscope offsets
    float temp;       // Temperature
    FusionAhrs ahrs;
    int64_t startup_time;
    int64_t last_fetch_time; // Last data fetch timestamp
    int64_t delta_time;
    int sample_count; // Number of samples taken during calibration
} MpuEssence;

typedef struct {
    bool initialized;
    bool running;
    int64_t startup_time;
} GpsEssence;

/* unions */

/* Function definitions */
static void manifest_mpu_startup(void *essence, epmfsm_state *states, epmfsm_relation *relations);
static void manifest_mpu_calibration(void *essence, epmfsm_state *states, epmfsm_relation *relations);
static void manifest_mpu_running(void *essence, epmfsm_state *states, epmfsm_relation *relations);
static void manifest_gps_startup(void *essence, epmfsm_state *states, epmfsm_relation *relations);
static void manifest_gps_running(void *essence, epmfsm_state *states, epmfsm_relation *relations);
static void transition_mpu_calibration_state(epmfsm_relation *relation, epmfsm_state *initialize, epmfsm_state *calibration, epmfsm_state *states, epmfsm_relation *relations);
static void transition_mpu_running_state(epmfsm_relation *relation, epmfsm_state *calibration, epmfsm_state *running, epmfsm_state *states, epmfsm_relation *relations);
static void transition_gps_running_state(epmfsm_relation *relation, epmfsm_state *calibration, epmfsm_state *running, epmfsm_state *states, epmfsm_relation *relations);
void app_main(void);

/* variables */

/* configuration, allows nested code to access above variables */
#include "config.h"

/* function implementations */
static void
manifest_mpu_startup(void *essence, epmfsm_state *states, epmfsm_relation *relations)
{
    MpuEssence *mpu_essence = (MpuEssence *)essence;
    mpu6050_init();

    ESP_LOGI("MPU6050", "Initialization Successful");
    mpu_essence->startup_time = esp_timer_get_time();
    mpu_essence->initialized = true;
}

static void
manifest_mpu_calibration(void *essence, epmfsm_state *states, epmfsm_relation *relations)
{
    MpuEssence *mpu_essence = (MpuEssence *)essence;
    int64_t current_time = esp_timer_get_time();
    int64_t delta_time = current_time - mpu_essence->last_fetch_time;
    int64_t calibration_time = current_time - mpu_essence->startup_time;

    if (delta_time >= (MPU6050_SAMPLE_PERIOD * 1000000)) {
        mpu_essence->delta_time = delta_time;
        mpu_essence->last_fetch_time = current_time;

        float ax, ay, az, gx, gy, gz;
        mpu6050_get_accel(&ax, &ay, &az);
        mpu6050_get_gyro(&gx, &gy, &gz);

        mpu_essence->ogx += gx;
        mpu_essence->ogy += gy;
        mpu_essence->ogz += gz;
        mpu_essence->oax += ax;
        mpu_essence->oay += ay;
        mpu_essence->oaz += az;
        mpu_essence->sample_count++;

        if(calibration_time >= (MPU6050_CALIBRATION_DURATION_MS * 1000)) {
            // Calculate offsets
            mpu_essence->ogx /= mpu_essence->sample_count;
            mpu_essence->ogy /= mpu_essence->sample_count;
            mpu_essence->ogz /= mpu_essence->sample_count;
            mpu_essence->oax /= mpu_essence->sample_count;
            mpu_essence->oay /= mpu_essence->sample_count;
            mpu_essence->oaz /= mpu_essence->sample_count;
            
            mpu_essence->calibrated = true;
            ESP_LOGI("MPU6050", " ogx %0.1f, ogy %0.1f, ogz %0.1f,\n oax %0.1f, oay %0.1f, oaz %0.1f",
                mpu_essence->ogx, 
                mpu_essence->ogy, 
                mpu_essence->ogz,
                mpu_essence->oax,
                mpu_essence->oay,
                mpu_essence->oaz);
        }
    }
}

static void
manifest_mpu_running(void *essence, epmfsm_state *states, epmfsm_relation *relations)
{
    MpuEssence *mpu_essence = (MpuEssence *)essence;
    int64_t current_time = esp_timer_get_time();
    int64_t delta_time = current_time - mpu_essence->last_fetch_time;

    if (delta_time >= (MPU6050_SAMPLE_PERIOD * 1000000)) {
        mpu_essence->delta_time = delta_time;
        mpu_essence->last_fetch_time = current_time;

        mpu6050_get_accel(&mpu_essence->ax, &mpu_essence->ay, &mpu_essence->az);
        mpu6050_get_gyro(&mpu_essence->gx, &mpu_essence->gy, &mpu_essence->gz);
        mpu_essence->temp = mpu6050_get_temp();

        // Apply offsets
        mpu_essence->gx -= mpu_essence->ogx;
        mpu_essence->gy -= mpu_essence->ogy;
        mpu_essence->gz -= mpu_essence->ogz;
        mpu_essence->ax -= mpu_essence->oax;
        mpu_essence->ay -= mpu_essence->oay;
        mpu_essence->az -= mpu_essence->oaz - 1; // Correct for gravity

        // Convert delta_time from microseconds to seconds
        float delta_time_seconds = delta_time / 1000000.0f;

        FusionVector gyroscope = {.axis={mpu_essence->gx, mpu_essence->gy, mpu_essence->gz}};
        FusionVector accelerometer = {.axis={mpu_essence->ax, mpu_essence->ay, mpu_essence->az}};

        FusionAhrsUpdateNoMagnetometer(&mpu_essence->ahrs, gyroscope, accelerometer, delta_time_seconds);
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&mpu_essence->ahrs));
        FusionVector earth = FusionAhrsGetEarthAcceleration(&mpu_essence->ahrs);

        printf("\033[2J\033[H");
        ESP_LOGI("MPU6050", "Roll %0.1f, Pitch %0.1f, Yaw %0.1f",
            euler.angle.roll, 
            euler.angle.pitch, 
            euler.angle.yaw);
        ESP_LOGI("MPU6050", "X %0.1f, Y %0.1f, Z %0.1f",
            earth.axis.x,
            earth.axis.y,
            earth.axis.z);
        ESP_LOGI("MPU6050", "Temperature: %.2f °C", mpu_essence->temp);
    }
}

static void
manifest_gps_startup(void *essence, epmfsm_state *states, epmfsm_relation *relations)
{
    GpsEssence *gps_essence = (GpsEssence *)essence;

    ESP_LOGI("BN220", "Initialization Successful");
    gps_essence->startup_time = esp_timer_get_time();
    gps_essence->initialized = true;
}

static void
manifest_gps_running(void *essence, epmfsm_state *states, epmfsm_relation *relations)
{
    return;
}

static void
transition_mpu_calibration_state(epmfsm_relation *relation, epmfsm_state *initialize, epmfsm_state *calibration, epmfsm_state *states, epmfsm_relation *relations)
{
    if (initialize->is_active && ((MpuEssence *)initialize->essence)->initialized) {
        initialize->is_active = false;
        calibration->is_active = true;
        relation->is_active = false;
        ESP_LOGI("MPU6050", "Transitioning to Calibration State");
    }
}

static void
transition_mpu_running_state(epmfsm_relation *relation, epmfsm_state *calibration, epmfsm_state *running, epmfsm_state *states, epmfsm_relation *relations)
{
    if (calibration->is_active && ((MpuEssence *)calibration->essence)->calibrated) {
        calibration->is_active = false;
        running->is_active = true;
        relation->is_active = false;
        ((MpuEssence *)running->essence)->running = true;
        ESP_LOGI("MPU6050", "Transitioning to Running State");
    }
}

static void
transition_gps_running_state(epmfsm_relation *relation, epmfsm_state *initialize, epmfsm_state *running, epmfsm_state *states, epmfsm_relation *relations)
{
    if (initialize->is_active && ((GpsEssence *)initialize->essence)->initialized) {
        initialize->is_active = false;
        running->is_active = true;
        relation->is_active = false;
        ((GpsEssence *)running->essence)->running = true;
        ESP_LOGI("BN220", "Transitioning to Running State");
    }
}

/* Main function */
void
app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI("GENERAL", "I2C initialized successfully");

    MpuEssence mpu_essence = {0};
    GpsEssence gps_essence = {0};

    FusionAhrsInitialise(&mpu_essence.ahrs);

    epmfsm_state states[StateIndex_NUM_States] = {
        [StateIndex_MPU_Initialization] = {
            .essence = &mpu_essence,
            .purpose = "MPU Initialization",
            .manifest = manifest_mpu_startup,
            .is_active = true
        },
        [StateIndex_MPU_Calibration] = {
            .essence = &mpu_essence,
            .purpose = "MPU Calibration",
            .manifest = manifest_mpu_calibration,
            .is_active = false
        },
        [StateIndex_MPU_Running] = {
            .essence = &mpu_essence,
            .purpose = "MPU Running",
            .manifest = manifest_mpu_running,
            .is_active = false
        },
        [StateIndex_GPS_Initialization] = {
            .essence = &gps_essence,
            .purpose = "GPS Initialization",
            .manifest = manifest_gps_startup,
            .is_active = true,
        },
        [StateIndex_GPS_Running] = {
            .essence = &gps_essence,
            .purpose = "GPS Running",
            .manifest = manifest_gps_running,
            .is_active = false,
        }
    };

    epmfsm_relation relations[RelationIndex_NUM_Relations] = {
        [RelationIndex_MPU_Initialized] = {
            .state1 = &states[StateIndex_MPU_Initialization],
            .state2 = &states[StateIndex_MPU_Calibration],
            .purpose = "MPU Startup to Calibration",
            .manifest = transition_mpu_calibration_state,
            .is_active = true
        },
        [RelationIndex_MPU_Calibrated] = {
            .state1 = &states[StateIndex_MPU_Calibration],
            .state2 = &states[StateIndex_MPU_Running],
            .purpose = "MPU Calibration to Running",
            .manifest = transition_mpu_running_state,
            .is_active = true
        },
        [RelationIndex_GPS_Initialized] = {
            .state1 = &states[StateIndex_GPS_Initialization],
            .state2 = &states[StateIndex_GPS_Running],
            .purpose = "GPS Startup to Running",
            .manifest = transition_gps_running_state,
            .is_active = true
        }
    };

    while(true) {
        epmfsm_scheduler(states, StateIndex_NUM_States, relations, RelationIndex_NUM_Relations);
    }
}
