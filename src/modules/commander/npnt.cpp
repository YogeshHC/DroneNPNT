/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   npnt.cpp
 * Author: Tanguturi Sai Sudheer
 *
 * Created on 23 September, 2019
 * Modified on 5th Nov, 2019
 */

#include "npnt.hpp"
#include "ezxml/ezxml.h"
#include <parameters/param.h>
#include <ctime>

// #include <sstream>
// #include <iostream>
// #include <fstream>


#include <fcntl.h>
#include <px4_posix.h>
#include <string.h>
#include <errno.h>


#include "pa.h"
#include <string.h>
#include <time.h>
#include <stdio.h>
#include "health_flag_helper.h"
#include "rc_check.h"

#include <math.h>



/**
 * Declaration of constants and global variables from here
 */

/**
 * Declarations of methods fro here...
 */


#include "ezxml/ezxml.h"

ezxml_t getXmlNode(ezxml_t parentNode, const char tags[][npnt::STR_L_MAX], int size);

char* getXmlTagValue(ezxml_t node, const char tags[][npnt::STR_L_MAX], int size);

char* getXmlAttribValue(ezxml_t node, const char ptags[][npnt::STR_L_MAX], const char* attName, int size);

uint8_t getGeoFence(orb_advert_t *mavlink_log_pub,ezxml_t node,  pa::latlong* vctr);

struct tm* getTime();

bool isValidTimeLine(struct tm* time);

int8_t dateTimeStrToUnixTime(const char* dt_string, struct tm* date_time);

int timediff(struct tm* t1, struct tm* t2);

void printPermissionArtifat(orb_advert_t *mavlink_log_pub,pa::permissionArtifat& pa);



/**
 * Implementations start from here...
 *
 */

/**
 *
 * @param node in which value is to be retrieved
 * @param tags to be found
 * @return string value from the node for the given tag if found else empty string
 */


ezxml_t getXmlNode(ezxml_t parentNode, const char tags[][25], int size) {
#ifdef _EZXML_H
    ezxml_t tempNode;
    tempNode = parentNode;
    if(tags == nullptr){
        return tempNode;
    }
    for (int i = 0; i < size && tempNode != 0; i++) {
        tempNode = ezxml_child(tempNode,tags[i]);
    }
    return tempNode;
#endif
    return NULL;
}


char* getXmlTagValue(ezxml_t node, const char tags[][npnt::STR_L_MAX], int size) {
    ezxml_t temp = (getXmlNode(node, tags, size));
    char * ret = (char*)temp->txt;
    return ret;
}

char* getXmlAttribValue(ezxml_t node, const char ptags[][npnt::STR_L_MAX], const char* attName, int size) {
    ezxml_t tempNode = getXmlNode(node, ptags, size);
    char* ret = (char *) ezxml_attr(tempNode,attName);
    return ret;
}

uint8_t getGeoFence(orb_advert_t *mavlink_log_pub,ezxml_t node,  pa::latlong* vctr) {
    ezxml_t nd = ezxml_child(node,"Coordinate");
    ezxml_t cNode = nd;
    uint8_t size = 0;
    for (int i = 0;cNode; cNode = (ezxml_idx(nd,++i))) {
        pa::latlong pll;
        if(cNode == NULL) break;
        const char* latitude = ezxml_attr(cNode,"latitude");
        const char* longitude = ezxml_attr(cNode,"longitude");
        if (latitude != NULL && longitude != NULL){
            pll.lati = atof(latitude);
        pll.longi = atof(longitude);
        // mavlink_log_critical(mavlink_log_pub,"Tag :");
        //vctr.push_back(pll);
        *(vctr+i) = pll;
        vctr = (pa::latlong*) realloc(vctr, (i+2) * sizeof(pa::latlong));
        size++;
        // mavlink_log_critical(mavlink_log_pub,"Tag :");
        }

    }
    // mavlink_log_critical(mavlink_log_pub,"Tag :");
    return size;
}

struct tm* getTime() {
    time_t rawtime;
    struct tm * timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    return timeinfo;
}

bool isValidTimeLine(tm* time) {
    return true;
    // if (strcmp(time->tm_zone, getTime()->tm_zone) == 0) {
    //     return true;
    // } else {
    //     return false;
    // }
}

/**
 * This method only does the basic checks like null values.
 * @param art permission artifat to be validated.
 * @return true if valid permission artifat
 */

bool npnt::isValidPA(orb_advert_t *mavlink_log_pub,pa::perm_a* art) {
    // bool valid = (art->signature != nullptr) && ((strcmp(art->signature, "") != 0));
    bool valid = true;
    // if(valid){
    //     mavlink_log_critical(mavlink_log_pub,"Valid signature found in artifat %s",art->signature);
    // }else{
    //     mavlink_log_critical(mavlink_log_pub,"InValid signature found in artifat");
    // }
    valid &= ((art->endTime) != nullptr);
    if(valid){
        mavlink_log_info(mavlink_log_pub,"Valid end time found in artifat");
    }else{
        mavlink_log_critical(mavlink_log_pub,"InValid end time found in artifat");
    }
    valid &= ((art->startTime) != nullptr);
    if(valid){
        mavlink_log_info(mavlink_log_pub,"Valid start time found in artifat");
    }else{
        mavlink_log_critical(mavlink_log_pub,"InValid start time found in artifat");
    }
    valid &= (art->geoFenceSize > 2);
    if(valid){
        mavlink_log_info(mavlink_log_pub,"Valid geofence found in artifat");
    }else{
        mavlink_log_critical(mavlink_log_pub,"InValid geofence found in artifat");
        mavlink_log_critical(mavlink_log_pub,"GeoFenceSize : %d",(int)art->geoFenceSize);
    }
    valid &= ((strcmp(art->uinNo,"") != 0));

    if(valid){
        mavlink_log_info(mavlink_log_pub,"Valid uinno found in artifat");
    }else{
        mavlink_log_critical(mavlink_log_pub,"InValid UinNo found in artifat %s",art->uinNo);
        mavlink_log_critical(mavlink_log_pub,"InValid uinno");
    }
    return valid;
}

/**
 * Checks if the permission artifat is intended for the current device or not
 * @param droneID from the permission artifat
 * @param deviceID hardware device's id.
 * @return true if valid device false if not.
 */

bool npnt::isValidDrone(orb_advert_t *mavlink_log_pub,const char* droneID, const char* deviceID) {
    if (!droneID || !deviceID) {
        return false;
    }
    if (*droneID == '\0' || *deviceID == '\0') {
        return false;
    }
    // return (strcmp(droneID, deviceID) == 0);
    return true;
}

/**
 *
 * @param dt_string to be converted to unix time
 * @param date_time tm struct which is to be filled with data
 * @return 0 on success and -1 on failure
 */

int8_t dateTimeStrToUnixTime(const char* dt_string, struct tm* date_time) {
    char data[5] = {};
    if (strlen(dt_string) != 32) {
        return -1;
    }
    if (!date_time) {
        return -1;
    }
    memset(date_time, 0, sizeof (struct tm));

    //read year
    memcpy(data, dt_string, 4);
    data[4] = '\0';
    date_time->tm_year = atoi(data) - 1900;
    //read month
    memcpy(data, &dt_string[5], 2);
    data[2] = '\0';
    date_time->tm_mon = atoi(data);
    //read day
    memcpy(data, &dt_string[8], 2);
    data[2] = '\0';
    date_time->tm_mday = atoi(data);
    //read hour
    memcpy(data, &dt_string[11], 2);
    data[2] = '\0';
    date_time->tm_hour = atoi(data); //also apply IST to UTC offset
    //read minute
    memcpy(data, &dt_string[14], 2); //also apply IST to UTC offset
    data[2] = '\0';
    date_time->tm_min = atoi(data);
    //read second
    memcpy(data, &dt_string[17], 2);
    data[2] = '\0';
    date_time->tm_sec = atoi(data);

    return 0;
}

/**
 *
 * @param startTime
 * @param endTime
 * @return if current time is valid for takeoff
 */
bool npnt::isValidTime(orb_advert_t *mavlink_log_pub,struct tm* startTime, struct tm* endTime) {
    // Check for valid time line of start and end times

    struct tm* nowTime = getTime();
    nowTime->tm_mon++;
    bool validStart = (timediff(startTime, nowTime) == -1);
    bool validEnd = (timediff(nowTime, endTime) == -1);
    //mavlink_log_critical(mavlink_log_pub,(validStart ? "Valid start. " : "Start error. ") << (validEnd ? "Valid end. " : "End error. "));
    // mavlink_log_critical(mavlink_log_pub,"%d|%d|%d",startTime->tm_mday,startTime->tm_mon,startTime->tm_year);
    PX4_INFO("%d|%d|%d",startTime->tm_mday,startTime->tm_mon,startTime->tm_year);
    return validStart && validEnd;

}

/**
 *
 * @param t1
 * @param t2
 * @return 0 if same, +1 if t1 > t2 and -1 if t2 > t1
 */
int timediff(struct tm* t1, struct tm* t2) {
    long result = 0;
    if (t1->tm_year > t2->tm_year) {
        goto t1;
    } else if (t1->tm_year == t2->tm_year) {
        if (t1->tm_mon > t2->tm_mon) {
            goto t1;
        } else if (t1->tm_mon == t2->tm_mon) {
            if (t1->tm_mday > t2->tm_mday) {
                goto t1;
            } else if (t1->tm_mday == t2->tm_mday) {
                if (t1->tm_hour > t2->tm_hour) {
                    goto t1;
                } else if (t1->tm_hour == t2->tm_hour) {
                    if (t1->tm_min > t2->tm_min) {
                        goto t1;
                    } else if (t1->tm_min == t2->tm_min) {
                        if (t1->tm_sec > t2->tm_sec) {
                            goto t1;
                        } else if (t1->tm_sec == t2->tm_sec) {
                            goto eq;
                        } else {
                            goto t2;
                        }
                    } else {
                        goto t2;
                    }
                } else {
                    goto t2;
                }
            } else {
                goto t2;
            }
        } else {
            goto t2;
        }
    } else if (t1->tm_year < t2->tm_year) {
        goto t2;
    }

t1:
    {
        result = 1;
        return result;}

t2:
    {
        result = -1;
        return result;}

eq:
    {
        result = 0;
        return result;}

}

/**
 * Based on Even odd method.
 * @param latlongs fence of geo latlongs
 * @param lati
 * @param longi
 * @return true if lat longs are inside the fence
 */

bool npnt::isInGeoFence(orb_advert_t *mavlink_log_pub,pa::latlong* latlongs,uint8_t size, float lati, float longi) {
    int i, j;
    bool c = false;
  int nvert = size;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
      pa::latlong ith = latlongs[i];
      pa::latlong jth = latlongs[j];
        if (((ith.longi>longi) != (jth.longi>longi)) &&
	        (lati < (jth.lati-ith.lati) * (longi-ith.longi) / (jth.longi-ith.longi) + ith.lati) ) {
            c = !c;
        }
  }
  return c;
}

/**
 *
 * @param permArt permission artifat object to be used.
 * @param file xml file to be parsed
 */
void npnt::parsePermissionArtifat(orb_advert_t *mavlink_log_pub,pa::permissionArtifat& permArt, char* file) {

    if (file == nullptr) { // File name supplied is null
//        std::cout << "Invalid file!!." << std::endl;
        mavlink_log_critical(mavlink_log_pub,"Unable to find file %s",file);
        return;
    } else {
        //try{
        // Initializing rapid xml document for xml parsing
        // permArt.signature = (char*)"";
        // permArt.flightPurpose = (char*)"";
        permArt.uinNo = (char*)"";
        // permArt.x_certificate = (char *) "";
        permArt.name = (char *) "";
        ezxml_t root_node;

            // mavlink_log_critical(mavlink_log_pub,"Found file %s",file);

            ezxml_t some_dt  = ezxml_parse_file(file);

            root_node = some_dt;
            //        Assigning signature
            int fd =    open("/tst.txt", O_CREAT | O_WRONLY, PX4_O_MODE_666);
            size_t wsize = write(fd,"Hello",5);
            PX4_WARN("%d",(int)(wsize));
            close(fd);
            if(some_dt == NULL){
                mavlink_log_critical(mavlink_log_pub,"Unable to parse file %s",file);

                return;
            }else{
                PX4_INFO("Parsed file successfully : %s",file);
            }
            permArt.name = (char *) ezxml_attr(root_node,npnt::PA_ID);
            // mavlink_log_critical(mavlink_log_pub,"Permission artefact ID obtained : %s",permArt.name);
            PX4_WARN("Permission artefact ID obtained : %s",permArt.name);
            // permArt.signature = (char *)getXmlTagValue(root_node, npnt::SIGNATURE_VALUE_NODE, npnt::SIGN_VALUE_NL);
            //      mavlink_log_critical(mavlink_log_pub,"Signature obtained : %s",permArt.signature);

            // permArt.x_certificate = (char *) getXmlTagValue(root_node,npnt::X509CERTIFICATE_NODE,npnt::X509CERT_NL);
            //     mavlink_log_critical(mavlink_log_pub,"X509Certificate : %s",permArt.x_certificate);

            //        Assigning UIN Number
            char* uin = (char *)getXmlAttribValue(root_node, npnt::UADETAILS_NODE,
                    npnt::UIN_NUMBER_PARAMETER, npnt::UADETAILS_NL);
            permArt.uinNo = uin;
            // mavlink_log_critical(mavlink_log_pub,"Uin obtained : %s",uin);

            //        Assigning payload details
            // permArt.payloadDetails = getXmlAttribValue(root_node, npnt::PAYLOAD_DETAILS_NODE,
            //         npnt::PAYLOAD_DETAILS_PARAMETER, npnt::PAYLOAD_DETAILS_NL);
            // //        std::cout << LOG_TAG << "Payload loaded" << std::endl;
            // mavlink_log_critical(mavlink_log_pub,"Payload details obtained : %s",permArt.payloadDetails);
            // //        Assigning payload weight
            // permArt.payloadWeight = getXmlAttribValue(root_node, npnt::PAYLOAD_DETAILS_NODE,
            //         npnt::PAYLOAD_WEIGHT_PARAMETER, npnt::PAYLOAD_DETAILS_NL);
            // //        std::cout << LOG_TAG << "Payload Weight calculated" << std::endl;
            // mavlink_log_critical(mavlink_log_pub,"Payload weight obtained : %s",permArt.payloadWeight);
            // //        Assigning purpose of flight
            // char* purpose = getXmlAttribValue(root_node, npnt::FLIGHT_PURPOSE_NODE,
            //         npnt::FLIGHT_PURPOSE_DESC_PARAMETER, npnt::FLIGHT_PURPOSE_NL);
            // permArt.flightPurpose = purpose;
            // //        std::cout << LOG_TAG << "Purpose determined : " << purpose << std::endl;
            // mavlink_log_critical(mavlink_log_pub,"Flight purpose obtained : %s",permArt.flightPurpose);
            //        Assigning geofence coordinates
            ezxml_t  coordinatesNode = getXmlNode(root_node, npnt::COORDINATES_NODE, npnt::COORDINATES_NL);
            pa::latlong* fence = (pa::latlong*) malloc(1 * sizeof(pa::latlong));
            // mavlink_log_critical(mavlink_log_pub,"Geofence node obtained : %s",file);
            uint8_t size = getGeoFence(mavlink_log_pub,coordinatesNode, fence);
            //        std::cout << fence[0].lati << " || " << fence[0].longi << std::endl;
            permArt.geoFence = fence;
            permArt.geoFenceSize = size;
            // mavlink_log_critical(mavlink_log_pub,"Geofence obtained : %s",file);
            // mavlink_log_critical(mavlink_log_pub,"Geofence size : %d",(int)size);

            //        Assigning start time of flight
            const char* strtTime = getXmlAttribValue(root_node, npnt::FLIGHT_PARAMETERS_NODE, npnt::FLIGHT_STRT_TIME_PARAMETER, npnt::FLIGHT_PARAMETERS_NL);
            mavlink_log_info(mavlink_log_pub,"Strt time : %s",strtTime);
            struct tm* startTimeTm = new tm();
            //        const char* TIME_FORMAT = "%Y-%m-%dT%H:%M:%s.";
            // mavlink_log_critical(mavlink_log_pub,"Strt time : %s",fistle);
            permArt.startTime = startTimeTm;
            dateTimeStrToUnixTime(strtTime, startTimeTm); //2019-09-20T19:36:26.849417+05:30

            // mavlink_log_critical(mavlink_log_pub,"Strt time obtained : %d %d %d",permArt.startTime->tm_hour,permArt.startTime->tm_min,permArt.startTime->tm_sec);

            //        Assigning end time of flight
            const char* endTime = getXmlAttribValue(root_node, npnt::FLIGHT_PARAMETERS_NODE, npnt::FLIGHT_END_TIME_PARAMETER, npnt::FLIGHT_PARAMETERS_NL);
            struct tm* endTimeTm = new tm();
            dateTimeStrToUnixTime(endTime, endTimeTm);
            permArt.endTime = endTimeTm;
            // mavlink_log_critical(mavlink_log_pub,"End time obtained : %d %d %d",permArt.endTime->tm_hour,permArt.endTime->tm_min,permArt.endTime->tm_sec);

            //printPermissionArtifat(mavlink_log_pub, permArt);
        // }
        // catch(rapidxml::parse_error e){

        // }
    }

}

/**
 *
 * @param pa permission artifat to be printed to console
 */

void printPermissionArtifat(orb_advert_t *mavlink_log_pub,pa::permissionArtifat& pa) {
//     std::string paString = "Permisssion Artifat: \n";

//     paString.append("Drone Id: ");
//     paString.append(pa.uinNo);

//     paString.append("\nPurpose of flight:");
//     paString.append(pa.flightPurpose);

//     paString.append("\nPayload details:");
//     paString.append(pa.payloadDetails);

//     paString.append("\nPayload weight");
//     paString.append(pa.payloadWeight);

//     paString.append("\nSignature Provided:");
//     paString.append(pa.signature);

//     paString.append("\nGeoFence:");
//     for(uint8_t i = 0; i < pa.geoFenceSize; i++){
//         paString.append("\n\tLatLongs:");
//         paString.append("\n\t\tLatitude:");
//         std::stringstream ss;
//         ss << paString << pa.geoFence[i].lati;
// //        paString.append((pa.geoFence[i].lati));
//         paString = ss.str();
//         paString.append("\n\t\tLongitude:");
// //        paString += (pa.geoFence[i].longi);
//         ss << paString << pa.geoFence[i].longi;
//         paString = ss.str();
//     }
//     //mavlink_log_critical(mavlink_log_pub,paString.c_str());
}

/**
 * Add digital signature to the log file and sign it here..
 * @param logFileName to be used.
 * @param signature to be added to the new log file.
 * @param signedLogFile path of the new log file.
 * @return -1 if error 0 if success
 */
int npnt::signAndBundleLog(orb_advert_t *mavlink_log_pub,const char* logFileName,const char* signature,const char* signedLogFile){
    // To be implemented
    if(logFileName==nullptr){
        return -1;
    }
    // std::ifstream logFile(logFileName);
    // // std::vector<char> buffer((std::istreambuf_iterator<char>(logFile)), std::istreambuf_iterator<char>());
    // // buffer.push_back('\0');
    // logFile.close();
    return 0;
}


bool npnt::isHardwareTampered(hard_info_t &hrd_info, hard_info_t &connected_devs)
{
    return ((hrd_info.accel_dev_id == connected_devs.accel_dev_id) &&
            (hrd_info.baro_dev_id == connected_devs.baro_dev_id) &&
            (hrd_info.gyro_dev_id == connected_devs.gyro_dev_id) &&
            (hrd_info.mag_dev_id == connected_devs.mag_dev_id));
            // (hrd_info.gps_dev_id == connected_devs.gps_dev_id) &&
            // (hrd_info.rc_dev_id == connected_devs.rc_dev_id));
}


