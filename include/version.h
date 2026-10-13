#ifndef HIGHTORQUE_VERSION_H
#define HIGHTORQUE_VERSION_H





#define VER_COMBINE(major, minor, patch)    ((uint32_t)(((major) << 16) | ((minor) << 8) | (patch)))
#define VER_GET_MAJOR(version)              ((uint8_t)(((version) >> 16) & 0xFF))
#define VER_GET_MINOR(version)              ((uint8_t)(((version) >> 8) & 0xFF))
#define VER_GET_PATCH(version)              ((uint8_t)(((version) >> 0) & 0xFF))

#define HIGHTORQUE_ROBOT_VERSION_MAJOR      6
#define HIGHTORQUE_ROBOT_VERSION_MINOR      0
#define HIGHTORQUE_ROBOT_VERSION_PATCH      3

#define HIGHTORQUE_ROBOT_VERSION \
    VER_COMBINE(HIGHTORQUE_ROBOT_VERSION_MAJOR, \
                HIGHTORQUE_ROBOT_VERSION_MINOR, \
                HIGHTORQUE_ROBOT_VERSION_PATCH)  // 版本号


#endif
