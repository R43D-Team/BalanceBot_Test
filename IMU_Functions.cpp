#include "IMU_Functions.h"

ICM_20948_I2C icm;
icm_20948_DMP_data_t data;

extern bool controlLoopIntervalOverride;

void setupIMU() {
  bool initialized = false;
  while (!initialized) {

    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.

    icm.begin(WIRE_PORT, AD0_VAL);


    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(icm.statusString());

    if (icm.status != ICM_20948_Stat_Ok) {
      Serial.println(F("Trying again..."));

      delay(500);
    } else {
      initialized = true;
    }
  }
  Serial.println("\n\nConnected to IMU\n\n");
  bool success = true;
  success &= (icm.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (icm.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
  // Enable the FIFO
  success &= (icm.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (icm.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (icm.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (icm.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success) {
    Serial.println("DMP enabled!");
  } else {
    Serial.println("Enable DMP failed!");
    Serial.println("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h...");
    while (1)
      ;  // Do nothing more
  }
}


bool newData() {
  icm.readDMPdataFromFIFO(&data);
  controlLoopIntervalOverride = false;
  if ((icm.status == ICM_20948_Stat_Ok) || (icm.status == ICM_20948_Stat_FIFOMoreDataAvail))  // Was valid data available?
  {
    if (icm.status == ICM_20948_Stat_FIFOMoreDataAvail) {
      controlLoopIntervalOverride = true;
    }
    return true;
  }
  return false;
}

double readPitch() {

  double pitch = 0;

  if ((data.header & DMP_header_bitmap_Quat6) > 0)  // We have asked for GRV data so we should receive Quat6
  {
    // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
    // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
    // The quaternion data is scaled by 2^30.

    //Serial.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

    // Scale to +/- 1
    double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;  // Convert to double. Divide by 2^30
    double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;  // Convert to double. Divide by 2^30
    double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;  // Convert to double. Divide by 2^30

    // Convert the quaternions to Euler angles (roll, pitch, yaw)
    // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

    double q2sqr = q2 * q2;

    // // roll (x-axis rotation)
    // double t0 = +2.0 * (q0 * q1 + q2 * q3);
    // double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
    // double roll = atan2(t0, t1) * 180.0 / PI;

    // pitch (y-axis rotation)
    double t2 = +2.0 * (q0 * q2 - q3 * q1);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    pitch = asin(t2) * 180.0 / PI;

    // // yaw (z-axis rotation)
    // double t3 = +2.0 * (q0 * q3 + q1 * q2);
    // double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
    // double yaw = atan2(t3, t4) * 180.0 / PI;
  }

  return pitch;
}

void updateBiasStoreSum(biasStore *store)  // Update the bias store checksum
{
  int32_t sum = store->header;
  sum += store->biasGyroX;
  sum += store->biasGyroY;
  sum += store->biasGyroZ;
  sum += store->biasAccelX;
  sum += store->biasAccelY;
  sum += store->biasAccelZ;
  sum += store->biasCPassX;
  sum += store->biasCPassY;
  sum += store->biasCPassZ;
  store->sum = sum;
}

bool isBiasStoreValid(biasStore *store)  // Returns true if the header and checksum are valid
{
  int32_t sum = store->header;

  if (sum != 0x42)
    return false;

  sum += store->biasGyroX;
  sum += store->biasGyroY;
  sum += store->biasGyroZ;
  sum += store->biasAccelX;
  sum += store->biasAccelY;
  sum += store->biasAccelZ;
  sum += store->biasCPassX;
  sum += store->biasCPassY;
  sum += store->biasCPassZ;

  return (store->sum == sum);
}

bool loadBiasStore() {
  biasStore store;
  EEPROM.get(EEPROM_BIAS_STORE, store);
  bool success = isBiasStoreValid(&store);
  if (success) {
    success &= (icm.setBiasGyroX(store.biasGyroX) == ICM_20948_Stat_Ok);
    success &= (icm.setBiasGyroY(store.biasGyroY) == ICM_20948_Stat_Ok);
    success &= (icm.setBiasGyroZ(store.biasGyroZ) == ICM_20948_Stat_Ok);
    success &= (icm.setBiasAccelX(store.biasAccelX) == ICM_20948_Stat_Ok);
    success &= (icm.setBiasAccelY(store.biasAccelY) == ICM_20948_Stat_Ok);
    success &= (icm.setBiasAccelZ(store.biasAccelZ) == ICM_20948_Stat_Ok);
    success &= (icm.setBiasCPassX(store.biasCPassX) == ICM_20948_Stat_Ok);
    success &= (icm.setBiasCPassY(store.biasCPassY) == ICM_20948_Stat_Ok);
    success &= (icm.setBiasCPassZ(store.biasCPassZ) == ICM_20948_Stat_Ok);
  }
  return success;
}

bool saveBiasStore() {
  biasStore store;

  bool success = (icm.getBiasGyroX(&store.biasGyroX) == ICM_20948_Stat_Ok);
  success &= (icm.getBiasGyroY(&store.biasGyroY) == ICM_20948_Stat_Ok);
  success &= (icm.getBiasGyroZ(&store.biasGyroZ) == ICM_20948_Stat_Ok);
  success &= (icm.getBiasAccelX(&store.biasAccelX) == ICM_20948_Stat_Ok);
  success &= (icm.getBiasAccelY(&store.biasAccelY) == ICM_20948_Stat_Ok);
  success &= (icm.getBiasAccelZ(&store.biasAccelZ) == ICM_20948_Stat_Ok);
  success &= (icm.getBiasCPassX(&store.biasCPassX) == ICM_20948_Stat_Ok);
  success &= (icm.getBiasCPassY(&store.biasCPassY) == ICM_20948_Stat_Ok);
  success &= (icm.getBiasCPassZ(&store.biasCPassZ) == ICM_20948_Stat_Ok);

  updateBiasStoreSum(&store);

  if (success) {
    EEPROM.put(EEPROM_BIAS_STORE, store);
    EEPROM.get(EEPROM_BIAS_STORE, store);
    success &= isBiasStoreValid(&store);
  }
  return success;
}