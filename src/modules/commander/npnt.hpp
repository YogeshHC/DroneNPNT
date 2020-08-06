/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   npnt.h
 * Author: Tanguturi Sai Sudheer
 *
 * Created on 23 September, 2019, 7:55 PM
 */

#ifndef NPNT_H
#define NPNT_H

//#include <fstream.h>

#include <time.h>
#include <string.h>
#include <systemlib/mavlink_log.h>
#include "pa.h"

#ifdef __cplusplus
extern "C" {
#endif

    namespace npnt{


    // const std::string SIGNATURE_VALUE_NODE[] = { "Signature","SignatureValue"};
    const int SIGN_VALUE_NL = 2,UADETAILS_NL = 3,FLIGHT_PURPOSE_NL = 3,PAYLOAD_DETAILS_NL = 3,FLIGHT_PARAMETERS_NL = 3,COORDINATES_NL = 4;
    const int X509CERT_NL = 4;
    const int STR_L_MAX = 25;

    const char SIGNATURE_VALUE_NODE[SIGN_VALUE_NL][STR_L_MAX] = {"Signature","SignatureValue"};
    // const std::string UADETAILS_NODE[] = {"Permission","FlightDetails","UADetails"};
    const char UADETAILS_NODE[UADETAILS_NL][STR_L_MAX] = {"Permission","FlightDetails","UADetails"};
    const char FLIGHT_PURPOSE_NODE[FLIGHT_PARAMETERS_NL][STR_L_MAX] = {"Permission","FlightDetails","FlightPurpose"};
    const char PAYLOAD_DETAILS_NODE[PAYLOAD_DETAILS_NL][STR_L_MAX] = {"Permission","FlightDetails","PayloadDetails"};
    const char FLIGHT_PARAMETERS_NODE[FLIGHT_PARAMETERS_NL][STR_L_MAX] = {"Permission","FlightDetails","FlightParameters"};
    const char COORDINATES_NODE[COORDINATES_NL][STR_L_MAX] = {"Permission","FlightDetails","FlightParameters","Coordinates"};
    const char X509CERTIFICATE_NODE[X509CERT_NL][STR_L_MAX] = {"Signature","KeyInfo","X509Data","X509Certificate"};

    const char PA_ID[] = "permissionArtifactId";

    const char UIN_NUMBER_PARAMETER[] = "uinNo";
    const char PAYLOAD_DETAILS_PARAMETER[] = "payloadDetails";
    const char PAYLOAD_WEIGHT_PARAMETER[] = "payloadWeight";
    const char FLIGHT_PURPOSE_DESC_PARAMETER[] = "shortDesc";
    const char FLIGHT_STRT_TIME_PARAMETER[] = "flightStartTime";
    const char FLIGHT_END_TIME_PARAMETER[] = "flightEndTime";

    typedef struct hardware_info_s {
        uint32_t mag_dev_id;
        uint32_t accel_dev_id;
        uint32_t gps_dev_id;
        uint32_t rc_dev_id;
        uint32_t gyro_dev_id;
        uint32_t baro_dev_id;
    } hard_info_t;

    bool isValidPA(orb_advert_t *mavlink_log_pub,pa::perm_a* art);

    bool isValidDrone(orb_advert_t *mavlink_log_pub,const char* droneID,const char* deviceID);

    bool isValidTime(orb_advert_t *mavlink_log_pub,struct tm* startTime, struct tm* endTime);

    bool isInGeoFence(orb_advert_t *mavlink_log_pub,pa::latlong* latlongs,uint8_t size,float lati ,float longi);

    void parsePermissionArtifat(orb_advert_t *mavlink_log_pub,pa::permissionArtifat& permArt,char* fp);

    int signAndBundleLog(orb_advert_t *mavlink_log_pub,const char* logFileName,const char* signature,const char* signedLogFile);

    bool isHardwareTampered(hard_info_t & hrd_info,hard_info_t & connected_devs);



    }

#ifdef __cplusplus
}
#endif

#endif /* NPNT_H */

