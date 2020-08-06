/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   pa.h
 * Author: Tanguturi Sai Sudheer
 *
 * Created on 24 September, 2019, 11:36 AM
 */

#ifndef PA_H
#define PA_H

#include <string.h>
#include <ctime>
//#include <vector>

#ifdef __cplusplus
extern "C" {
#endif
namespace pa {

    typedef struct ll {
        float lati;
        float longi;
    } latlong;

    typedef struct permissionArtifat {
        tm* startTime;
        tm* endTime;
        latlong* geoFence;
        char* uinNo;
        // char* flightPurpose;
        // char* payloadDetails;
        // char* payloadWeight;
        // char* signature;
        // char* x_certificate;
        char* name;
        uint8_t geoFenceSize;
    } perm_a;
}


#ifdef __cplusplus
}
#endif

#endif /* PA_H */

